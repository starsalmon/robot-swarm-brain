#!/usr/bin/env python3
"""Rover's reactive explorer: smooth, curving, never stop-and-stare. Pure Python (no ROS) so it can be replayed offline.

A vector-field-histogram style steer (Borenstein & Koren's VFH idea, the same one Nav2/ROS local planners build on),
cut down to Rover's sensors:
  front L8 8 columns (col 0 = LEFT, 45 deg FOV) + VL53 front / front_left (+45) / front_right (-45) / left / right.
Each reading is a ray. Forward speed scales with the clearance straight ahead; turn rate steers toward the most
open direction and away from near walls on either side, with a slow wander so long runs curve instead of
drawing straight lines. Blocked ahead: pivot toward the open side until clear (no stopping). Bump or wheel
stall: short reverse, then pivot away, then carry on.

Speeds are usable-power fractions (same units as /cmd_vel to the T-Display). Positive w = turn left.
"""
import math, random

E = lambda k, d: type(d)(__import__("os").environ.get(k, d))


class Explorer:
    def __init__(s):
        s.CRUISE = E("EXPLORE_CRUISE", 0.18)     # was 0.12
        s.VMIN = E("EXPLORE_VMIN", 0.08)         # slowest forward creep near things
        s.WMAX = E("EXPLORE_WMAX", 0.12)         # arc turn cap (was 0.05/0.06 = straight lines)
        s.PIVOT_W = E("EXPLORE_PIVOT_W", 0.14)   # in-place turn when blocked (0.06 was the floor to pivot on carpet)
        s.STOP = E("EXPLORE_STOP_M", 0.25)       # closer than this ahead: pivot, no forward
        s.SLOW = E("EXPLORE_SLOW_M", 0.90)       # full speed beyond this
        s.SIDE = E("EXPLORE_SIDE_M", 0.30)       # side wall closer than this: steer off it
        s.STALE = E("EXPLORE_STALE_S", 1.0)
        s.rays = {}                               # name -> (bearing rad, range m, t)
        s.cols = None; s.t_cols = -1e9
        s.v = 0.0; s.w = 0.0; s.wander = 0.0; s.pivot = 0.0; s.rec = None; s.t_last = None

    # ---- inputs ----
    def on_cols(s, cols, now):
        s.cols = [x if x > 0.02 else 4.0 for x in cols]; s.t_cols = now

    def on_range(s, name, bearing_deg, r, now):
        if r is None or r != r: r = 4.0
        s.rays[name] = (math.radians(bearing_deg), 4.0 if (r <= 0.02 or r > 4.0) else r, now)

    def on_bump(s, now, why="bump"):
        if s.rec is None or now - s.rec["t"] > 1.5:
            s.rec = dict(ph="back", t=now, why=why, dir=s._open_side(now) or random.choice((-1.0, 1.0)))

    # ---- helpers ----
    def _all_rays(s, now):
        out = []
        if s.cols and now - s.t_cols < s.STALE:
            n = len(s.cols)
            for i, x in enumerate(s.cols):
                out.append((math.radians(((n - 1) / 2 - i) * 45.0 / n), x))
        for b, r, t in s.rays.values():
            if now - t < s.STALE: out.append((b, r))
        return out

    def _open_side(s, now):
        rays = s._all_rays(now)
        L = [r for b, r in rays if b > 0.05]; R = [r for b, r in rays if b < -0.05]
        if not L or not R: return None
        mL, mR = sum(L) / len(L), sum(R) / len(R)
        if abs(mL - mR) < 0.05: return None
        return 1.0 if mL > mR else -1.0

    # ---- control ----
    def step(s, now):
        dt = 0.05 if s.t_last is None else max(0.01, min(0.2, now - s.t_last)); s.t_last = now
        rays = s._all_rays(now)
        l8_ok = s.cols is not None and now - s.t_cols < s.STALE
        fr = s.rays.get("front"); front_ok = fr is not None and now - fr[2] < s.STALE
        if not l8_ok and not front_ok:
            s.v = s.w = 0.0; s.rec = None
            return 0.0, 0.0, "explore: no fresh ToF, stopped"
        ahead = min([r for b, r in rays if abs(b) <= math.radians(16)] or [4.0])
        # recovery after bump / stall: reverse 0.6 s, pivot away 0.9 s, resume
        if s.rec is not None:
            e = s.rec; age = now - e["t"]
            if e["ph"] == "back":
                if age < 0.6: return s._out(-0.14, 0.0, f"recover ({e['why']}): reverse", dt, hard=True)
                e.update(ph="turn", t=now); age = 0.0
            if e["ph"] == "turn":
                if age < 0.9 or ahead < s.STOP + 0.1:
                    if age < 3.0: return s._out(0.0, e["dir"] * s.PIVOT_W, f"recover ({e['why']}): pivot {'left' if e['dir'] > 0 else 'right'}", dt, hard=True)
                s.rec = None
        # blocked ahead: pivot toward the open side (committed until clear; hysteresis)
        if ahead < s.STOP or (s.pivot and ahead < s.STOP + 0.20):
            if not s.pivot: s.pivot = s._open_side(now) or 1.0
            return s._out(0.0, s.pivot * s.PIVOT_W, f"explore: blocked {ahead:.2f} m, pivot {'left' if s.pivot > 0 else 'right'}", dt, hard=True)
        s.pivot = 0.0
        # forward speed from clearance ahead (and the +-45 diagonals, at half weight)
        diag = min([r for b, r in rays if math.radians(16) < abs(b) <= math.radians(50)] or [4.0])
        clear = min(ahead, 0.5 * (diag + ahead) if diag < ahead else ahead)
        k = max(0.0, min(1.0, (clear - s.STOP) / (s.SLOW - s.STOP)))
        v = s.VMIN + (s.CRUISE - s.VMIN) * k
        # steering: pull toward open directions, push from near ones (each ray weighted by closeness)
        pull = 0.0; wsum = 0.0
        for b, r in rays:
            if abs(b) > math.radians(100): continue
            g = min(r, 2.0)
            pull += g * math.sin(b) * math.cos(b / 2); wsum += g
        steer = pull / max(wsum, 1e-3)                                    # -1..1 toward openness
        push = 0.0
        for nm, sg in (("left", -1.0), ("right", 1.0)):
            ry = s.rays.get(nm)
            if ry and now - ry[2] < s.STALE and ry[1] < s.SIDE:
                push += sg * (s.SIDE - ry[1]) / s.SIDE                    # wall on the left -> turn right
        urgency = 1.0 - k                                                 # steer harder when it gets tight
        if urgency > 0.3 and abs(steer) < 0.08:                           # dead-on wall: pick a side early, keep it
            if not getattr(s, "bias", 0.0): s.bias = s._open_side(now) or (1.0 if s.wander >= 0 else -1.0)
            steer += 0.35 * s.bias * urgency
        elif urgency < 0.1:
            s.bias = 0.0
        # slow wander (Ornstein-Uhlenbeck) so open space is covered in arcs, not straight lines
        s.wander += -s.wander * 0.15 * dt + random.gauss(0.0, 0.35) * math.sqrt(dt)
        s.wander = max(-1.0, min(1.0, s.wander))
        w = s.WMAX * max(-1.0, min(1.0, (1.2 + 2.5 * urgency) * steer + 1.5 * push + 1.0 * s.wander * k))
        return s._out(v, w, f"explore: ahead {ahead:.2f} m, v {v:.2f}, steer {w:+.2f}", dt)

    def _out(s, v, w, why, dt, hard=False):
        if hard:
            s.v, s.w = v, w
        else:                                                             # smooth: ~0.4 s to full speed / full turn
            s.v += max(-0.6 * dt, min(0.5 * dt, v - s.v))
            s.w += max(-0.6 * dt, min(0.6 * dt, w - s.w))
        return s.v, s.w, why
