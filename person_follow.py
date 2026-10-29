#!/usr/bin/env python3
"""Rover person following: YOLO11 person detector + ByteTrack -> steer on bearing, hold ~1.0 m.

Live:    python3 person_follow.py                 (dry-run by default: publishes /rover/follow/cmd only)
         FOLLOW_DRY_RUN=0 -> publishes /cmd_vel + /rover/session (only via follow.sh, which sets allow_drive
         and pauses rover-brain, with the same cleanup as safe_drive.sh)
Offline: python3 person_follow.py --bag /captures/<bag> [--out /captures/follow_x] (dry-run, annotated jpg/mp4)

Target: locks the nearest track (largest bbox bottom) once seen in >=3 frames, keeps its ID. Lost >1 s -> stop;
relocks only after a fresh lock. Bearing from undistorted bbox centre (camera intrinsics). Distance: L8 grid zones
under the bbox (cols mirrored: grid col 0 = image right; floor rows 5-7 excluded), fallback ground-plane from the
bbox bottom (feet) when the feet are in frame, else bbox-height. All stop checks on a 20 Hz timer: target lost >1 s,
L8 front/cols min < stop_dist (0.38 m, 2-frame debounce; slow-down from 0.6 m), stale L8/image (>0.5 s), dead-man (max run time, optional heartbeat), Ctrl-C.
Speeds are usable-power fractions like rover_safe_drive.py (cap 0.25). Forward only, except the obstacle back-off: obstacle-stopped > backoff_after s -> slow
reverse ~backoff_m (odom/time-limited) then turn toward the last target bearing. Never drives forward while lost.
"""
import argparse, json, math, os, sys, time
import numpy as np, cv2

E = lambda k, d: type(d)(os.environ.get(k, d))
CAP = 0.25


def load_cal(path):
    import yaml
    c = yaml.safe_load(open(path))
    K = np.array(c["camera_matrix"]["data"], float).reshape(3, 3)
    D = np.array(c["distortion_coefficients"]["data"], float)
    return K, D, c["image_width"], c["image_height"]


class Follower:
    """Detector + tracker + control law, shared by the live node and offline bag replay."""

    def __init__(self, a):
        from ultralytics import YOLO
        self.a = a
        self.model = YOLO(a.model)
        self.K, self.D, self.W, self.H = load_cal(a.cal)
        self.l8 = None; self.t_l8 = 0.0; self.t_safe = 0.0; self.l8cols_min = None; self.front = None
        self.odom = None; self.t_obs = None; self.n_low = 0; self.bo = None; self.bearing = 0.0
        self.t_front = -1e9; self.bdot = 0.0; self.t_b = None; self.rear = None; self.t_rear = -1e9; self.servo_req = None   # rear sonar (Range, NaN = no echo) + servo request
        self.target = None; self.t_seen = -1e9; self.cand = {}; self.dist = None; self.dsrc = "-"
        self.dl8 = self.dpl = None; self.plsrc = '-'
        self.cmd = (0.0, 0.0); self.raw = (0.0, 0.0); self.t_raw = -1e9; self.t_ctl = 0.0; self.why = "searching"; self.dets = []; self.t_img = 0.0

    # ---- geometry ----
    def undist(self, u, v):
        p = cv2.undistortPoints(np.array([[[u, v]]], np.float32), self.K, self.D)[0, 0]
        return float(p[0]), float(p[1])          # normalised x (right +), y (down +)

    def l8_dist(self, x1, x2):
        if self.l8 is None or self.t_img - self.t_l8 > 0.5:
            return None
        xa, _ = self.undist(x1, self.K[1, 2]); xb, _ = self.undist(x2, self.K[1, 2])
        a1, a2 = math.degrees(math.atan(xa)), math.degrees(math.atan(xb))
        half = self.a.l8_fov / 2; zw = self.a.l8_fov / 8
        vals = []
        for j in range(8):                       # j = image-left-to-right zone
            lo, hi = -half + j * zw, -half + (j + 1) * zw
            if hi < a1 or lo > a2:
                continue
            col = 7 - j                          # mirrored
            for r in range(0, 5):                # rows 5-7 see the floor
                z = self.l8[r, col]
                if 0.05 < z < 4.0:
                    vals.append(z)
        if not vals:
            return None
        vals.sort()
        return float(np.mean(vals[:max(1, len(vals) // 3)]))   # near third = the person, not the wall behind

    def plane_dist(self, x1, y1, x2, y2):
        if y2 < self.H - 0.01 * self.H:                       # feet in frame -> ground plane
            _, yn = self.undist((x1 + x2) / 2, y2)
            ang = math.radians(self.a.pitch_deg) + math.atan(yn)
            if ang > 0.02:
                return self.a.cam_h / math.tan(ang), "feet"
        h = y2 - y1                               # whole body: ~1.7 m person
        if y1 > 0.01 * self.H and h > 0.05 * self.H:
            return 1.7 * self.K[1, 1] / h, "bbox"
        return None, "-"

    # ---- per frame ----
    def on_image(self, img, now):
        self.t_img = now
        if img.shape[1] != self.W:               # older QVGA bags: scale the XGA intrinsics
            k = img.shape[1] / self.W
            self.K = self.K.copy(); self.K[:2] *= k; self.W, self.H = img.shape[1], img.shape[0]
        r = self.model.track(img, persist=True, classes=[0], conf=self.a.conf, imgsz=self.a.imgsz,
                             tracker="bytetrack.yaml", half=True, verbose=False)[0]
        self.dets = []
        if r.boxes is not None and r.boxes.id is not None:
            for b, i, c in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.id.int().cpu().numpy(), r.boxes.conf.cpu().numpy()):
                self.dets.append((int(i), *map(float, b), float(c)))
        ids = {d[0]: d for d in self.dets}
        if self.target is None or (now - self.t_seen > self.a.lost_s and self.target not in ids):
            if self.target is not None and now - self.t_seen > self.a.lost_s:
                self.target = None
            for i in ids: self.cand[i] = self.cand.get(i, 0) + 1
            ok = [d for d in self.dets if self.cand.get(d[0], 0) >= 3]
            if ok:
                self.target = max(ok, key=lambda d: d[4])[0]   # nearest = lowest bbox bottom
                self.t_seen = now
        if self.target is not None and self.target not in ids and self.dets and now - self.t_seen < 3.0:
            # ByteTrack re-IDs the same person after blur/occlusion: hand over to the detection nearest the last
            # bearing (and of similar size) instead of waiting lost_s and then searching away from him
            def cost(d):
                xn, _ = self.undist((d[1] + d[3]) / 2, (d[2] + d[4]) / 2)
                return abs(math.atan(xn - self.a.lens_off) - self.bearing)
            best = min(self.dets, key=cost)
            if cost(best) < math.radians(25):
                self.n_reid = getattr(self, "n_reid", 0) + 1; self.target = best[0]
        if self.target in ids:
            _, x1, y1, x2, y2, _ = ids[self.target]
            self.t_seen = now
            xn, _ = self.undist((x1 + x2) / 2, (y1 + y2) / 2)
            nb = math.atan(xn - self.a.lens_off / 1.0)   # rad, + = right of lens axis
            if getattr(self, "t_b", None) is not None and 0.02 < now - self.t_b < 0.5:
                self.bdot = 0.6 * self.bdot + 0.4 * (nb - self.bearing) / (now - self.t_b)
            else:
                self.bdot = 0.0
            self.bearing = nb; self.t_b = now
            self.yaw_b = self.yaw_now()
            yl = getattr(self, 'yaw_hist', None)
            if yl:                                    # frame was captured ~lat_s ago: use yaw from then
                tt = now - self.a.lat_s; old = [y for t_, y in yl if t_ <= tt]
                if old: self.yaw_b = old[-1]
            d = self.l8_dist(x1, x2); self.dsrc = "L8"
            self.dl8, (self.dpl, self.plsrc) = d, self.plane_dist(x1, y1, x2, y2)
            if d is None:
                d, self.dsrc = self.dpl, self.plsrc
            self.dist = d
        return r

    # ---- 20 Hz control ----
    def yaw_now(self): return self.yaw_fn() if getattr(self, "yaw_fn", None) else 0.0
    def yaw_rate(self): return self.rate_fn() if getattr(self, "rate_fn", None) else 0.0

    def control(self, now):
        """Continuous smooth follow (20 Hz). v = kv * (dist - target), no deadband, smoothly -> 0 at the target;
        w = PD on bearing (deadband, D term covers the ~0.4 s yaw coast), low-passed, capped, arcs with v.
        Brief target loss (< lost_s) decays the last command; > search_after s: slow search turn toward the
        last bearing for search_max_s. Hard stops only: obstacle < stop_dist (slow-down from slow_from), stale L8."""
        a = self.a
        safe = min([v for v in (self.front, self.l8cols_min) if v is not None] or [9.0])
        l8_stale = now - self.t_safe > a.stale_s                 # receive-time based (no header stamps)
        front_ok = now - self.t_front <= a.stale_s and self.front is not None
        if a.need_l8 and l8_stale and not front_ok:
            return self.hard("stale L8")
        if l8_stale: safe = self.front if front_ok else safe      # L8 stale, front ToF fresh: rely on it, slow
        if self.bo is not None:                          # obstacle back-off in progress
            r = self.backoff(now)
            if r is not None: return r
        self.n_low = self.n_low + 1 if safe < a.stop_dist else 0
        if self.n_low >= a.obs_frames:
            if self.t_obs is None: self.t_obs = now
            if a.backoff and now - self.t_obs > a.backoff_after:
                self.t_obs = None; self.n_low = 0
                self.bo = dict(ph="aim", t=now, xy=self.odom, sgn=-1.0 if self.bearing > 0 else 1.0)
                return self.backoff(now)
            return self.hard(f"L8 {safe:.2f} < {a.stop_dist}")
        self.t_obs = None
        brake = max(0.0, min(1.0, (safe - a.stop_dist) / max(0.01, a.slow_from - a.stop_dist)))
        age = now - self.t_seen
        fresh = self.target is not None and age <= self.a.fresh_s and now - self.t_img <= 0.5
        if fresh:
            # latency + coast compensation: bearing now = bearing at frame + yaw turned since (CCW +, target drifts
            # right); then add the ~coast_s of yaw still to come at the current rate
            yr = self.yaw_rate()
            dyaw = math.atan2(math.sin(self.yaw_now() - self.yaw_b), math.cos(self.yaw_now() - self.yaw_b))
            b = self.bearing + dyaw + yr * a.coast_s; db = math.radians(a.deadband_deg)
            e = 0.0 if abs(b) < db else b - math.copysign(db, b)
            e *= min(1.0, abs(e) / math.radians(15)) if abs(e) < math.radians(15) else 1.0   # softer near centre
            w = -(a.kw * e)                                    # + bearing = right -> negative w (turn right)
            w = max(-a.wmax, min(a.wmax, w))
            if w != 0.0 and getattr(self, "w_sign", 0) and math.copysign(1, w) != self.w_sign and now - self.t_wsign < a.rev_hold_s:
                w = 0.0                                        # no left-right-left within rev_hold_s
            if abs(w) > 0.01:
                if math.copysign(1, w) != getattr(self, "w_sign", 0): self.t_wsign = now
                self.w_sign = math.copysign(1, w)
            v = 0.0
            if self.dist is not None:
                v = max(0.0, min(a.vmax, a.kv * (self.dist - a.target_m)))
                v *= max(0.5, 1.0 - abs(b) / math.radians(45))     # ease off a little while off-axis (still arcs)
            self.raw = (v, w); why = "follow"
        elif self.t_seen > 0 and age <= a.lost_s:                 # brief loss: decay last command smoothly
            k = max(0.0, 1.0 - age / a.lost_s)
            v, w = self.raw[0] * k, self.raw[1] * k; why = f"bridge {age:.1f}s"
        elif self.t_seen > 0 and a.search_after <= age <= a.search_after + a.search_max_s:
            v, w = 0.0, (-1.0 if self.bearing > 0 else 1.0) * a.search_w; why = "search turn"
        else:
            v, w = 0.0, 0.0; why = "searching" if self.target is None else "waiting"
        v *= brake
        if l8_stale:
            v = min(v, a.stale_vmax); why += " (L8 stale, front ok)"
        dt = max(0.0, min(0.2, now - self.t_ctl)); self.t_ctl = now
        al = min(1.0, dt / max(1e-3, a.w_tau))                    # low-pass the turn
        w = self.cmd[1] + al * (w - self.cmd[1])
        pv, pw = self.cmd
        v = pv + max(-a.dv * dt * 2, min(a.dv * dt, v - pv))      # accel dv/s, decel 2x
        w = pw + max(-a.dw * dt, min(a.dw * dt, w - pw))
        if abs(v) < 0.005: v = 0.0
        if abs(w) < 0.005: w = 0.0
        return self.set(v, w, why)

    def backoff(self, now):
        """Phase rev: reverse at backoff_v until odom moved backoff_m or backoff_max_s. Phase turn: turn toward the
        last bearing at backoff_w for backoff_turn_s or until the target is seen again. Returns None when done."""
        a, b = self.a, self.bo; el = now - b["t"]
        if b["ph"] == "aim":                             # point the rear sonar straight back, let it settle
            self.servo_req = a.sonar_back_deg
            if el < a.sonar_settle_s:
                return self.set(0.0, 0.0, "backoff aim sonar")
            ok, why = self.rear_ok(now, a.rear_start_m)
            if not ok:
                b.update(ph="turn", t=now); el = 0.0; b["skip"] = why
            else:
                b.update(ph="rev", t=now); el = 0.0
        if b["ph"] == "rev":
            ok, why = self.rear_ok(now, a.rear_stop_m)
            if not ok:
                b.update(ph="turn", t=now, skip=why); el = 0.0
        if b["ph"] == "rev":
            moved = 0.0 if (b["xy"] is None or self.odom is None) else math.hypot(self.odom[0] - b["xy"][0], self.odom[1] - b["xy"][1])
            if moved < a.backoff_m and el < a.backoff_max_s:
                return self.set(-a.backoff_v, 0.0, f"backoff rev {moved:.2f}m")
            b.update(ph="turn", t=now); el = 0.0
        if el < a.backoff_turn_s and not (self.target is not None and now - self.t_seen < 0.15):
            return self.set(0.0, b["sgn"] * a.backoff_w, "backoff turn" + (f" (no rev: {b['skip']})" if b.get("skip") else ""))
        self.bo = None; self.raw = (0.0, 0.0); self.cmd = (0.0, 0.0)
        return None

    def rear_ok(self, now, lim):
        """Rear sonar gate for reversing: fresh (< rear_stale_s), a real echo, and >= lim metres."""
        if not self.a.need_rear: return True, ""
        if now - self.t_rear > self.a.rear_stale_s or self.rear is None: return False, "rear stale"
        if not (self.rear == self.rear) or self.rear <= 0: return False, "rear no echo"
        if self.rear < lim: return False, f"rear {self.rear:.2f} < {lim}"
        return True, ""

    def hard(self, why):
        self.raw = (0.0, 0.0)
        return self.set(0.0, 0.0, why)

    def set(self, v, w, why):
        self.cmd = (round(max(-CAP, min(v, CAP)), 3), round(max(-CAP, min(CAP, w)), 3)); self.why = why
        return self.cmd

    def annotate(self, img):
        o = img.copy()
        for i, x1, y1, x2, y2, c in self.dets:
            col = (0, 0, 255) if i == self.target else (0, 200, 0)
            cv2.rectangle(o, (int(x1), int(y1)), (int(x2), int(y2)), col, 3)
            cv2.putText(o, f"id{i} {c:.2f}", (int(x1), max(20, int(y1) - 8)), 0, 0.8, col, 2)
        d = "-" if self.dist is None else f"{self.dist:.2f}m({self.dsrc})"
        s = f"tgt {self.target} d {d}  v {self.cmd[0]:.2f} w {self.cmd[1]:+.2f}  {self.why}"
        cv2.rectangle(o, (0, 0), (o.shape[1], 36), (0, 0, 0), -1)
        cv2.putText(o, s + ("  DRY-RUN" if self.a.dry_run else ""), (8, 26), 0, 0.8, (255, 255, 255), 2)
        return o


def args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag"); ap.add_argument("--out", default="/captures/follow_out")
    ap.add_argument("--every", type=int, default=1, help="offline: process every Nth frame")
    ap.add_argument("--model", default=E("FOLLOW_MODEL", "/weights/yolo11s.pt"))
    ap.add_argument("--imgsz", type=int, default=E("FOLLOW_IMGSZ", 640))
    ap.add_argument("--conf", type=float, default=E("FOLLOW_CONF", 0.35))
    ap.add_argument("--cal", default=E("FOLLOW_CAL", "/opt/fleet/config/rover_cam_info_xga.yaml"))
    ap.add_argument("--target-m", type=float, default=E("FOLLOW_TARGET_M", 1.0))
    ap.add_argument("--vmax", type=float, default=E("FOLLOW_VMAX", 0.12)); ap.add_argument("--vmin", type=float, default=0.05)
    ap.add_argument("--kv", type=float, default=0.20); ap.add_argument("--hold-m", type=float, default=0.05)
    ap.add_argument("--dv", type=float, default=0.15, help="usable/s accel"); ap.add_argument("--dw", type=float, default=0.30)
    ap.add_argument("--wmax", type=float, default=E("FOLLOW_WMAX", 0.18)); ap.add_argument("--wmin", type=float, default=0.10)
    ap.add_argument("--kw", type=float, default=0.22); ap.add_argument("--deadband-deg", type=float, default=8.0)
    ap.add_argument("--stop-dist", type=float, default=E("FOLLOW_STOP_DIST", 0.38)); ap.add_argument("--slow-from", type=float, default=E("FOLLOW_SLOW_FROM", 0.60))
    ap.add_argument("--obs-frames", type=int, default=2, help="consecutive low readings before an obstacle stop")
    ap.add_argument("--backoff", type=int, default=E("FOLLOW_BACKOFF", 1)); ap.add_argument("--backoff-after", type=float, default=2.0)
    ap.add_argument("--backoff-m", type=float, default=0.10); ap.add_argument("--backoff-v", type=float, default=0.06)
    ap.add_argument("--backoff-max-s", type=float, default=3.0); ap.add_argument("--backoff-w", type=float, default=0.12)
    ap.add_argument("--backoff-turn-s", type=float, default=1.5)
    ap.add_argument("--need-rear", type=int, default=E("FOLLOW_NEED_REAR", 1), help="reverse only with fresh rear sonar")
    ap.add_argument("--rear-start-m", type=float, default=E("FOLLOW_REAR_START_M", 0.30)); ap.add_argument("--rear-stop-m", type=float, default=E("FOLLOW_REAR_STOP_M", 0.20))
    ap.add_argument("--rear-stale-s", type=float, default=0.5); ap.add_argument("--sonar-back-deg", type=float, default=E("FOLLOW_SONAR_BACK_DEG", 90.0), help="/rover/servo/angle logical deg that points the sonar straight back (90 = centre)")
    ap.add_argument("--sonar-settle-s", type=float, default=0.4)
    ap.add_argument("--lost-s", type=float, default=1.5)
    ap.add_argument("--stale-s", type=float, default=E("FOLLOW_STALE_S", 1.0), help="L8/front data older than this (receive time) = stale")
    ap.add_argument("--stale-vmax", type=float, default=0.05, help="forward cap while L8 stale but front ToF fresh")
    ap.add_argument("--lat-s", type=float, default=E("FOLLOW_LAT_S", 0.15), help="camera+inference latency")
    ap.add_argument("--coast-s", type=float, default=E("FOLLOW_COAST_S", 0.35), help="yaw coast after a turn stops")
    ap.add_argument("--rev-hold-s", type=float, default=0.6)
    ap.add_argument("--fresh-s", type=float, default=E("FOLLOW_FRESH_S", 0.35), help="max detection age to steer on")
    ap.add_argument("--kd", type=float, default=0.12, help="turn D gain on bearing rate (s), covers ~0.4 s yaw coast")
    ap.add_argument("--w-tau", type=float, default=0.25, help="turn low-pass time constant s")
    ap.add_argument("--search-after", type=float, default=3.0); ap.add_argument("--search-max-s", type=float, default=6.0)
    ap.add_argument("--search-w", type=float, default=0.08)
    ap.add_argument("--max-s", type=float, default=E("FOLLOW_MAX_S", 60.0), help="dead-man: max run time (0 = run until stopped)")
    ap.add_argument("--heartbeat", type=int, default=E("FOLLOW_HEARTBEAT", 0),
                    help="1 = also require /rover/follow/heartbeat (Bool true) within 1.0 s")
    ap.add_argument("--l8-fov", type=float, default=45.0); ap.add_argument("--need-l8", type=int, default=1, help="require fresh /rover/tof/l8_cols (safety)")
    ap.add_argument("--cam-h", type=float, default=0.106); ap.add_argument("--pitch-deg", type=float, default=E("FOLLOW_PITCH_DEG", 4.0))
    ap.add_argument("--lens-off", type=float, default=0.0)
    ap.add_argument("--dry-run", type=int, default=E("FOLLOW_DRY_RUN", 1))
    a = ap.parse_args()
    if a.vmax > CAP or a.wmax > CAP:
        raise SystemExit("speed above the 25% usable cap")
    if a.bag: a.dry_run = 1
    return a


def offline(a):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    f = Follower(a); os.makedirs(a.out, exist_ok=True)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    vw = None; n = 0; rows = []; tdet = []; nxt = None
    while r.has_next():
        t, d, ts = r.read_next(); now = ts * 1e-9
        if t == "/rover/tof/l8_grid":
            m = deserialize_message(d, ty[t]); f.l8 = np.array(m.data, float).reshape(8, 8) / 1000.0; f.t_l8 = now
        elif t == "/rover/tof/l8_cols":
            m = deserialize_message(d, ty[t]); f.l8cols_min = min([x for x in m.data if x > 0.02] or [9.0]); f.t_safe = now
        elif t == "/rover/tof/front":
            rg = deserialize_message(d, ty[t]).range; f.front = rg if rg > 0.02 else None; f.t_front = now
        elif t == "/rover/sonar/range":
            f.rear = deserialize_message(d, ty[t]).range; f.t_rear = now
        elif t == "/rover/odom/wheel":
            m = deserialize_message(d, ty[t]); f.odom = (m.pose.pose.position.x, m.pose.pose.position.y)
        elif t == "/rover/camera/image_raw/compressed":
            n += 1
            if n % a.every: continue
            img = cv2.imdecode(np.frombuffer(deserialize_message(d, ty[t]).data, np.uint8), 1)
            t0 = time.perf_counter(); f.on_image(img, now); tdet.append(time.perf_counter() - t0)
            f.control(now)
            rows.append(dict(t=round(now, 3), n=len(f.dets), ids=[x[0] for x in f.dets], tgt=f.target,
                             dist=None if f.dist is None else round(f.dist, 3), src=f.dsrc, l8=f.dl8, pl=f.dpl, pls=f.plsrc, v=f.cmd[0], w=f.cmd[1], why=f.why))
            o = f.annotate(img)
            if vw is None:
                vw = cv2.VideoWriter(os.path.join(a.out, "annot.avi"), cv2.VideoWriter_fourcc(*"MJPG"), 15 / a.every, (o.shape[1] // 2, o.shape[0] // 2))
            vw.write(cv2.resize(o, (o.shape[1] // 2, o.shape[0] // 2)))
            if f.dets and len([x for x in rows if x["n"]]) % 20 == 1:
                cv2.imwrite(os.path.join(a.out, f"frame_{n:05d}.jpg"), o)
    if vw: vw.release()
    json.dump(rows, open(os.path.join(a.out, "log.json"), "w"))
    td = sorted(tdet[5:]) or [0]
    print(f"frames {len(rows)}  with person {sum(1 for x in rows if x['n'])}  ids {sorted({i for x in rows for i in x['ids']})}"
          f"  det+track ms median {1000 * td[len(td) // 2]:.1f} p95 {1000 * td[int(len(td) * .95)]:.1f}")


def live(a):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import CompressedImage, Range
    from std_msgs.msg import Bool, Float32, Float32MultiArray, String, UInt16MultiArray

    class N(Node):
        def __init__(s):
            super().__init__("person_follow")
            s.f = Follower(a); s.img = None; s.t0 = time.monotonic(); s.hb = 0.0; s.done = None
            s.create_subscription(CompressedImage, "/rover/camera/image_raw/compressed", s.on_img, qos_profile_sensor_data)
            s.create_subscription(UInt16MultiArray, "/rover/tof/l8_grid", s.on_grid, 10)
            s.create_subscription(Float32MultiArray, "/rover/tof/l8_cols", s.on_cols, 10)
            s.create_subscription(Range, "/rover/tof/front", lambda m: (setattr(s.f, "front", m.range if m.range > 0.02 else None), setattr(s.f, "t_front", time.monotonic())), 10)
            from nav_msgs.msg import Odometry
            from std_msgs.msg import Float32
            s.create_subscription(Range, "/rover/sonar/range", lambda m: (setattr(s.f, "rear", m.range), setattr(s.f, "t_rear", time.monotonic())), qos_profile_sensor_data)
            s.pservo = s.create_publisher(Float32, "/rover/servo/angle", 10) if not a.dry_run else None; s.t_servo = 0.0
            s.create_subscription(Odometry, "/rover/odom/wheel", lambda m: setattr(s.f, "odom", (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
            s.create_subscription(Bool, "/rover/follow/heartbeat", lambda m: setattr(s, "hb", time.monotonic() if m.data else 0.0), 10)
            s.pcmd = s.create_publisher(Twist, "/rover/follow/cmd", 10)
            s.pst = s.create_publisher(String, "/rover/follow/status", 10)
            s.pdbg = s.create_publisher(CompressedImage, "/rover/follow/debug/compressed", 2)
            if not a.dry_run:
                s.cmd = s.create_publisher(Twist, "/cmd_vel", 10); s.ses = s.create_publisher(Bool, "/rover/session", 10)
            s.f.model.predict(np.zeros((768, 1024, 3), np.uint8), classes=[0], half=True, verbose=False)  # warm-up
            s.create_timer(0.05, s.step); s.create_timer(0.02, s.detect)
            s.get_logger().info(f"person_follow up, dry_run={a.dry_run} model={a.model}")

        def on_img(s, m): s.img = m
        def on_cols(s, m): s.f.l8cols_min = min([x for x in m.data if x > 0.02] or [9.0]); s.f.t_safe = time.monotonic()
        def on_grid(s, m): s.f.l8 = np.array(m.data, float).reshape(8, 8) / 1000.0; s.f.t_l8 = time.monotonic()

        def detect(s):
            m, s.img = s.img, None
            if m is None: return
            img = cv2.imdecode(np.frombuffer(m.data, np.uint8), 1)
            s.f.on_image(img, time.monotonic())
            if s.pdbg.get_subscription_count():
                o = CompressedImage(); o.format = "jpeg"; o.header = m.header
                o.data = cv2.imencode(".jpg", cv2.resize(s.f.annotate(img), (512, 384)))[1].tobytes(); s.pdbg.publish(o)

        def step(s):
            now = time.monotonic()
            v, w = s.f.control(now)
            if a.max_s > 0 and now - s.t0 > a.max_s: v, w, s.f.why = 0, 0, "dead-man: max run time"; s.done = s.done or now
            elif a.heartbeat and now - s.hb > 1.0: v, w, s.f.why = 0, 0, "dead-man: no heartbeat"
            if s.f.bo is not None and s.f.servo_req is not None and s.pservo is not None and now - s.t_servo > 0.5:
                sm = Float32(); sm.data = float(s.f.servo_req); s.pservo.publish(sm); s.t_servo = now   # hold, no sweep
            if s.f.bo is None: s.f.servo_req = None
            tw = Twist(); tw.linear.x = float(v); tw.angular.z = float(w)
            s.pcmd.publish(tw)
            st = String(); st.data = json.dumps(dict(t=time.time(), tgt=s.f.target, ndet=len(s.f.dets), seen_age=round(time.monotonic()-s.f.t_seen,2), dist=s.f.dist, src=s.f.dsrc, l8=s.f.dl8, plane=s.f.dpl, plsrc=s.f.plsrc, bearing_deg=round(math.degrees(getattr(s.f,"bearing",0.0)),1), v=v, w=w, why=s.f.why, dry=a.dry_run))
            s.pst.publish(st)
            if not a.dry_run:
                b = Bool(); b.data = True                  # session stays ON for the whole run (no per-move toggling)
                s.cmd.publish(tw); s.ses.publish(b)

        def stop_robot(s):
            if a.dry_run: return
            z = Twist(); f = Bool(); f.data = False
            for _ in range(15):
                s.cmd.publish(z); s.ses.publish(f); time.sleep(0.02)

    rclpy.init(); n = N()
    try:
        while rclpy.ok() and not (n.done and time.monotonic() - n.done > 1.0):
            rclpy.spin_once(n, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        n.stop_robot(); n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    a = args()
    offline(a) if a.bag else live(a)
