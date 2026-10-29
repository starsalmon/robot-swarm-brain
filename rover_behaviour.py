#!/usr/bin/env python3
"""Rover's one always-on behaviour: EXPLORE until a person is seen, FOLLOW them, back to EXPLORE when lost.

Runs as the compose service rover-behaviour (restart unless-stopped). It is the ONLY publisher of /cmd_vel and
/rover/session. rover-brain (ROVER_EXPLORE_ARBITER=1) does the explore thinking and publishes its suggestion on
/rover/explore/cmd_vel; this node switches its session on/off via /rover/behaviour/explore_active.

States
  EXPLORE  rover-brain explore command (forward clamped to 0 when an obstacle is < stop_dist; stale L8 -> stop)
  ORANGE   a solid orange blob held in view. Creep in, using the front L8, and stop at 5 cm.
           Then a short rising tune on the drive-board speaker. Back to EXPLORE once the orange is gone.
  FOLLOW   entered when the tracker holds a person for >= enter_s (0.5 s), and no orange is claimed.
           person_follow.Follower control: smooth P/PD tracking, obstacle slow 0.6 / stop 0.38,
           sonar-gated back-off, stale-L8 fallback, brief-loss decay, search turn toward the last bearing from 3 s.
           Back to EXPLORE when the person has been lost > exit_s (5 s).
Safety master switch: /fleet/allow_drive (from HA / MQTT lab/robots/allow_drive, retained). OFF = the node runs
fully (status + /rover/behaviour/cmd show what it would do) but publishes no motion; on the ON->OFF edge it sends
zero /cmd_vel and session false. The board's own 0.4 s command timeout still applies.
Status: /rover/behaviour/status (JSON), would-be command: /rover/behaviour/cmd.
"""
import json, math, os, sys, time
from collections import deque

sys.argv = [sys.argv[0]] + sys.argv[1:]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import person_follow as PF

E = PF.E


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage, Range
    from std_msgs.msg import Bool, Float32, Float32MultiArray, String, UInt8, UInt16MultiArray, UInt32

    a = PF.args()
    a.dry_run = 0
    enter_s = E("BEHAVIOUR_ENTER_S", 0.5); exit_s = E("BEHAVIOUR_EXIT_S", 5.0)
    TAG_M = E("TAG_SIZE_M", 0.08)                      # tag36h11 black square edge (tags_80mm.pdf)
    CRUISE = E("EXPLORE_ESCAPE_V", 0.12); ESC_W = E("EXPLORE_ESCAPE_W", 0.10)
    GAP = int(E("EXPLORE_GAP", 1)); EWMAX = E("EXPLORE_WMAX", 0.05)   # own gap explorer; turn-rate cap
    STEP = math.radians(E("EXPLORE_STEP_DEG", 40)); PAUSE = E("EXPLORE_PAUSE_S", 0.7)
    CLEAR = E("EXPLORE_CLEAR_M", 0.8); GAPD = E("EXPLORE_GAP_DEPTH_M", 0.5); LOOK_S = E("EXPLORE_LOOK_EVERY_S", 30.0)
    PB = 36                                            # polar free-space bins (10 deg, world frame)
    NB = 12                                            # heading bins (30 deg)
    ARU = PF.cv2.aruco
    TDICT = ARU.getPredefinedDictionary(ARU.DICT_APRILTAG_36h11); TPAR = ARU.DetectorParameters_create()
    h = TAG_M / 2; TOBJ = PF.np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], PF.np.float32)

    class B(Node):
        def __init__(s):
            super().__init__("rover_behaviour")
            s.f = PF.Follower(a); s.img = None
            s.orange_blob = None; s.orange_t = 0.0; s.orange_since = None; s.orange_miss = None
            s.arrived = False; s.celebrated = False; s.song = None; s.song_i = 0
            s.ov = 0.0; s.ow = 0.0
            s.go = False
            s.room = None; s.room_t = 0.0; s.heading = None; s.turn_sign = 1.0
            s.flipped = False; s.err_track = None; s.room_arrived = False; s.last_oh_log = 0.0
            s.prev_r = None; s.prev_rt = 0.0
            s.state = "EXPLORE"; s.t_state = time.monotonic(); s.allow = False; s.t_allow = 0.0
            s.exp = (0.0, 0.0); s.t_exp = 0.0; s.tgt_since = None; s.cmd = (0.0, 0.0); s.trans = []
            # novelty / revisit memory
            s.pose = None; s.yaw = None; s.path = 0.0; s.hist = deque(); s.bins = [0.0] * NB; s.t_bins = time.monotonic()
            s.tags = {}; s.tag_last = {}; s.tag_vis = []; s.nimg = 0
            s.esc = None; s.t_esc_end = -1e9; s.esc_reason = ""
            s.pol = [None] * PB; s.ex = dict(ph="cruise", t=time.monotonic(), turned=0.0, y0=None, dir=1.0, hd=0.0)
            s.pan = 90.0; s.cols = []; s.tn = {}; s.tseq = []; s.tlast_id = None; s.avoid = []; s.good = []; s.yaw_off = 0.0; s.t_tagplan = -1e9; s.t_tagpause = -1e9
            f = s.f
            s.yrate = 0.0; s.t_yaw = 0.0; f.yaw_hist = deque(); f.yaw_fn = lambda: s.yaw or 0.0; f.rate_fn = lambda: s.yrate
            s.create_subscription(CompressedImage, "/rover/camera/image_raw/compressed", lambda m: setattr(s, "img", m), qos_profile_sensor_data)
            s.create_subscription(UInt16MultiArray, "/rover/tof/l8_grid", s.on_grid, 10)
            s.create_subscription(Float32MultiArray, "/rover/tof/l8_cols", s.on_cols, 10)
            s.create_subscription(Range, "/rover/tof/front", lambda m: (setattr(f, "front", m.range if m.range > 0.02 else None), setattr(f, "t_front", time.monotonic())), 10)
            s.create_subscription(Range, "/rover/sonar/range", lambda m: (setattr(f, "rear", m.range), setattr(f, "t_rear", time.monotonic()), s.mark(math.pi + math.radians(s.pan - 90.0), m.range)), qos_profile_sensor_data)
            s.create_subscription(Float32, "/rover/sonar/pan_deg", lambda m: setattr(s, "pan", float(m.data)), 10)
            for tp, b in (("left", 90), ("right", -90), ("front_left", 45), ("front_right", -45), ("front", 0)):
                s.create_subscription(Range, f"/rover/tof/{tp}", lambda m, b=b: s.mark(math.radians(b), m.range), 10)
            s.create_subscription(Odometry, "/rover/odom/wheel", s.on_odom, 10)
            s.ptag = s.create_publisher(String, "/rover/apriltags", 10)
            s.pspk = s.create_publisher(UInt32, "/rover/speaker/code", 10); s.t_snd = 0.0; s.t_person_snd = -1e9
            s.create_subscription(Twist, "/rover/explore/cmd_vel", s.on_exp, 10)
            s.create_subscription(Bool, "/fleet/allow_drive", s.on_allow, 10)
            s.create_subscription(Bool, "/rover/button", s.on_button, 10)
            s.create_subscription(UInt8, "/rover/button_event", s.on_button_event, 10)
            s.create_subscription(Float32MultiArray, "/room/overhead", s.on_room, 10)
            s.pcmd = s.create_publisher(Twist, "/cmd_vel", 10)
            s.pses = s.create_publisher(Bool, "/rover/session", 10)
            s.pservo = s.create_publisher(Float32, "/rover/servo/angle", 10); s.t_servo = 0.0
            s.pexp = s.create_publisher(Bool, "/rover/behaviour/explore_active", 10)
            s.pwould = s.create_publisher(Twist, "/rover/behaviour/cmd", 10)
            s.pst = s.create_publisher(String, "/rover/behaviour/status", 10)
            f.model.predict(PF.np.zeros((768, 1024, 3), PF.np.uint8), classes=[0], half=True, verbose=False)
            s.create_timer(0.05, s.step); s.create_timer(0.02, s.detect)
            s.get_logger().info("rover_behaviour up: Go button gates motion, overhead camera steers to the orange")

        def on_cols(s, m):
            if list(m.data) == s.cols:                      # bit-identical frame = frozen L8 (real frames jitter by mm): not fresh
                s.n_frozen = getattr(s, "n_frozen", 0) + 1
                if s.n_frozen in (20, 200): s.get_logger().warn(f"L8 frozen: {s.n_frozen} identical l8_cols frames in a row -> treating as stale")
                return
            s.n_frozen = 0
            s.f.l8cols_min = min([x for x in m.data if x > 0.02] or [9.0]); s.f.t_safe = time.monotonic()
            n = len(m.data)
            for i, x in enumerate(m.data):                   # col 0 = LEFT (matches /rover/tof/front_left) -> positive bearing
                s.mark(math.radians(((n - 1) / 2 - i) * a.l8_fov / n), x)
            s.cols = list(m.data)

        def mark(s, rel, r):
            """Polar free-distance map in world frame (odom yaw); readings older than 30 s are ignored."""
            if s.yaw is None or r is None or not (r == r): return
            r = 4.0 if (r <= 0.02 or r > 4.0) else r
            k = int(((s.yaw + rel) % (2 * math.pi)) / (2 * math.pi) * PB) % PB
            now = time.monotonic(); o = s.pol[k]
            s.pol[k] = (r if o is None or now - o[1] > 1.0 else min(r, 0.5 * (o[0] + r)) , now)

        def pick_gap(s, now):
            """Widest + deepest opening, discounted by headings travelled recently. Returns (heading, score, why)."""
            d = [p[0] if p is not None and now - p[1] < 30 else None for p in s.pol]
            known = sum(x is not None for x in d)
            if known < PB // 4: return None
            best = None
            for k in range(PB):
                win = [d[(k + j) % PB] for j in (-2, -1, 0, 1, 2)]   # +-20 deg: room for Rover's width
                if any(x is None for x in win): continue
                depth = min(win)
                if depth < GAPD: continue
                width = 0
                for side in (1, -1):
                    j = 1
                    while j < PB // 2 and d[(k + side * j) % PB] is not None and d[(k + side * j) % PB] >= 0.7: j += 1
                    width += j - 1
                trav = s.bins[int(k * NB / PB)] / (max(s.bins) + 0.3)
                sc = min(depth, 3.0) + 0.04 * width - 1.2 * trav + s.tag_bias((k + 0.5) * 2 * math.pi / PB, now)
                if best is None or sc > best[1]: best = ((k + 0.5) * 2 * math.pi / PB, sc, f"gap {(k+0.5)*10:.0f}deg depth {depth:.1f}m width {width*10}deg trav {trav:.2f}")
            return best

        def gap_cmd(s, now, safe):
            """Drive-first explorer (explore2): cruise forward while the CENTRE of the L8 (+ sonar) is open; when blocked,
            rotate continuously (no stop-and-scan pauses, no servo) toward the more open side, watching the L8 live, and
            leave as soon as the centre sees >= CLEAR. After ~200 deg without a clear view, turn to the deepest known gap
            and drive. Forward segments are committed: a minimum cruise time before any re-plan except a real block."""
            e = s.ex; y = s.yaw
            if y is None: return 0.0, 0.0, "gap: no odom yaw"
            if e["ph"] not in ("cruise", "rot", "pause", "turn"): e.update(ph="cruise", t=now)
            if e.get("y0") is None: e["y0"] = y
            cl = s.cols if len(s.cols) == 8 else None
            ctr = min([x for x in (cl[2:6] if cl else []) if x > 0.02] or [safe])
            if s.f.front is not None: ctr = min(ctr, s.f.front)
            lft = min(cl[:4]) if cl else 0.0; rgt = min(cl[4:]) if cl else 0.0
            def start_rot(why):
                e.update(ph="rot", t=now, y0=y, turned=0.0, ly=y, dir=1.0 if lft >= rgt else -1.0, why=why, nclr=0)
                return 0.0, 0.0, f"gap: rotate ({why})"
            if e["ph"] == "cruise":
                if ctr < a.slow_from or safe < a.stop_dist: return start_rot(f"front {ctr:.2f}m")
                v = CRUISE if ctr > CLEAR else CRUISE * 0.7
                w = 0.0
                if cl:                                      # steer gently away from the nearer side while driving
                    if min(lft, rgt) < 0.6 and abs(lft - rgt) > 0.2: w = 0.03 if lft > rgt else -0.03
                    elif abs(lft - rgt) > 0.8 and min(lft, rgt) < 1.2: w = 0.02 if lft > rgt else -0.02
                return v, w, f"gap: cruise (front {ctr:.2f}m)"
            if e["ph"] == "pause":                          # short settle before a planned turn (tag escape); no servo
                if now - e["t"] < 0.3: return 0.0, 0.0, f"gap: pause ({e.get('nx')})"
                e.update(ph=e.get("nx", "cruise"), t=now, y0=y)
            if e["ph"] == "rot":
                e["turned"] += abs(math.atan2(math.sin(y - e["ly"]), math.cos(y - e["ly"]))); e["ly"] = y
                e["nclr"] = e["nclr"] + 1 if ctr >= CLEAR else 0
                if e["nclr"] >= 3 and e["turned"] > math.radians(15):     # ~0.15 s of a clear centre: go
                    e.update(ph="cruise", t=now); return 0.0, 0.0, f"gap: clear {ctr:.1f}m -> cruise"
                if e["turned"] >= math.radians(200) or now - e["t"] > 15:
                    g = s.pick_gap(now)
                    if g is None:
                        dd = [(p[0] if p and now - p[1] < 30 else 0.0) for p in s.pol]; k = max(range(PB), key=lambda k: dd[k])
                        g = ((k + 0.5) * 2 * math.pi / PB, 0.0, f"deepest {dd[k]:.1f}m")
                    s.get_logger().info(f"GAP -> {g[2]} score {g[1]:.2f}")
                    e.update(hd=g[0], gw=g[2], ph="turn", t=now, y0=y); return 0.0, 0.0, "gap: chosen"
                return 0.0, e["dir"] * EWMAX, f"gap: rotate {math.degrees(e['turned']):.0f}deg ({e['why']}, front {ctr:.2f}m)"
            if e["ph"] == "turn":
                err = math.atan2(math.sin(e["hd"] - y), math.cos(e["hd"] - y))
                if abs(err) < math.radians(12) + abs(s.yrate) * a.coast_s or now - e["t"] > 12:
                    e.update(ph="cruise", t=now); return 0.0, 0.0, "gap: aligned -> cruise"
                return 0.0, math.copysign(EWMAX, err), f"gap: turn to {math.degrees(e['hd'] % (2*math.pi)):.0f}deg ({e['gw']})"
            e.update(ph="cruise", t=now); return 0.0, 0.0, "gap: reset"

        def on_grid(s, m): s.f.l8 = PF.np.array(m.data, float).reshape(8, 8) / 1000.0; s.f.t_l8 = time.monotonic()
        def on_odom(s, m):
            p = m.pose.pose; x, y = p.position.x, p.position.y; q = p.orientation
            s.f.odom = (x, y); yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            yaw = yaw + s.yaw_off
            now = time.monotonic()
            if s.pose is not None:
                d = math.hypot(x - s.pose[0], y - s.pose[1])
                if d < 0.5:                                   # ignore odom resets
                    s.path += d
                    k = math.exp(-(now - s.t_bins) / 180.0); s.t_bins = now     # decaying heading histogram
                    s.bins = [b * k for b in s.bins]; s.bins[int(((yaw % (2 * math.pi)) / (2 * math.pi)) * NB) % NB] += d
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            if s.yaw is not None and now - s.t_yaw > 1e-3:
                s.yrate = 0.6 * s.yrate + 0.4 * math.atan2(math.sin(yaw - s.yaw), math.cos(yaw - s.yaw)) / (now - s.t_yaw)
            s.t_yaw = now; s.f.yaw_hist.append((now, yaw))
            while len(s.f.yaw_hist) > 40: s.f.yaw_hist.popleft()
            s.pose = (x, y); s.yaw = yaw
            s.hist.append((now, x, y, s.path))
            while s.hist and now - s.hist[0][0] > 90: s.hist.popleft()

        def sound(s, hz1, hz2=0, reps=1, len_code=3, now=None, gap=0.8):
            """T-Display speaker (/rover/speaker/code): hz1 | hz2<<12 | reps<<24 | len<<28 (len ms = 40 + 20*len_code)."""
            now = now or time.monotonic()
            if now - s.t_snd < gap: return False
            m = UInt32(); m.data = (int(hz1) & 0xFFF) | ((int(hz2) & 0xFFF) << 12) | ((reps & 0xF) << 24) | ((len_code & 0xF) << 28)
            s.pspk.publish(m); s.t_snd = now; return True

        def tags_detect(s, img, now):
            g = PF.cv2.cvtColor(img, PF.cv2.COLOR_BGR2GRAY)
            c, ids, _ = ARU.detectMarkers(g, TDICT, parameters=TPAR)
            out = []
            if ids is not None:
                for cc, i in zip(c, ids.flatten()):
                    ok, rv, tv = PF.cv2.solvePnP(TOBJ, cc.reshape(4, 2).astype(PF.np.float32), s.f.K, s.f.D, flags=PF.cv2.SOLVEPNP_IPPE_SQUARE)
                    if not ok: continue
                    x, y, z = (float(v) for v in tv.flatten())
                    i = int(i); out.append(dict(id=i, x=round(x, 3), y=round(y, 3), z=round(z, 3), dist=round(math.hypot(x, z), 2), bearing_deg=round(math.degrees(math.atan2(x, z)), 1)))
                    if now - s.tag_last.get(i, -1e9) > 3.0:          # a new sighting of tag i
                        s.tags.setdefault(i, deque()).append(now)
                        s.tag_seen(i, x, z, rv, now)
                    if now - s.tag_last.get(i, -1e9) > 10.0:         # first seen / re-seen after 10 s: its own tone
                        if s.sound(600 + 80 * (i % 20), 0, 1 + i // 20, 4, now): s.get_logger().info(f"tag {i} sound")
                    s.tag_last[i] = now
            for q in s.tags.values():
                while q and now - q[0] > 180: q.popleft()
            s.tag_vis = out
            st = String(); st.data = json.dumps(dict(t=time.time(), tags=out, sightings={str(k): len(v) for k, v in s.tags.items() if v})); s.ptag.publish(st)

        def tag_seen(s, i, x, z, rv, now):
            """Tag navigation: visit counts, tag->tag graph, heading anchoring, and plan changes."""
            if s.yaw is None: return
            R, _ = PF.cv2.Rodrigues(rv); nx, nz = float(R[0, 2]), float(R[2, 2])
            rel_n = math.atan2(-nx, nz); hd_tag = s.yaw + math.atan2(-x, z)
            t = s.tn.setdefault(i, dict(n=0, first=now, nyaw=None, from_={}))
            t["n"] += 1; t["last"] = now
            if s.tlast_id is not None and s.tlast_id != i:
                s.tseq.append((s.tlast_id, i)); s.tseq = s.tseq[-200:]
                t["from_"][s.tlast_id] = t["from_"].get(s.tlast_id, 0) + 1
            s.tlast_id = i
            if math.hypot(x, z) < 2.5:                               # heading anchor (tag normal is fixed in the world)
                if t["nyaw"] is None: t["nyaw"] = s.yaw + rel_n
                else:
                    err = math.atan2(math.sin(t["nyaw"] - (s.yaw + rel_n)), math.cos(t["nyaw"] - (s.yaw + rel_n)))
                    if abs(err) < math.radians(45):
                        s.yaw_off += 0.5 * err; s.yaw += 0.5 * err
                        if abs(err) > math.radians(5): s.get_logger().info(f"TAG -> tag {i} heading anchor: corrected yaw {math.degrees(0.5*err):+.0f} deg")
            recent = len(s.tags.get(i, ()))
            most = max(s.tn, key=lambda k: s.tn[k]["n"])
            if s.state != "EXPLORE": return
            if recent >= 2 or (most == i and t["n"] >= 2):
                s.avoid.append((hd_tag, now))
                away = hd_tag + math.pi
                g = s.pick_gap(now)
                hd = g[0] if g is not None and abs(math.atan2(math.sin(g[0] - hd_tag), math.cos(g[0] - hd_tag))) > math.radians(70) else away
                s.ex.update(ph="pause", nx="turn", t=now, hd=hd, gw=f"away from tag {i}", y0=s.yaw); s.t_tagplan = now
                s.get_logger().info(f"TAG -> tag {i} visited ({recent} in 3 min, total {t['n']}): been here, turn away to {math.degrees(hd % (2*math.pi)):.0f} deg")
            else:
                s.good.append((s.yaw, now))
                s.get_logger().info(f"TAG -> tag {i} {'new' if t['n'] == 1 else 'rarely seen'} (total {t['n']}): good direction, keep heading {math.degrees(s.yaw % (2*math.pi)):.0f} deg")

        def tag_bias(s, hd, now):
            s.avoid = [a_ for a_ in s.avoid if now - a_[1] < 180]; s.good = [g_ for g_ in s.good if now - g_[1] < 180]
            d = lambda a1: abs(math.atan2(math.sin(hd - a1), math.cos(hd - a1)))
            return sum(-1.5 * math.exp(-(d(a_) / 0.6) ** 2) for a_, _ in s.avoid) + sum(0.6 * math.exp(-(d(g_) / 0.6) ** 2) for g_, _ in s.good)

        def loiter(s, now):
            """Return a reason to escape, or ''."""
            if now - s.t_esc_end < 20: return ""
            rev = [i for i, q in s.tags.items() if len(q) >= 3 and now - q[-3] < 120]
            if rev: return f"revisit tag {rev[0]} x{len(s.tags[rev[0]])}"
            if s.hist and now - s.hist[0][0] > 60:
                old = [h for h in s.hist if now - h[0] >= 60]
                o = old[-1]; travelled = s.path - o[3]
                xs = [h[1] for h in s.hist if h[0] >= o[0]]; ys = [h[2] for h in s.hist if h[0] >= o[0]]
                span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
                if travelled > 2.0 and span < 2.2: return f"loiter: {travelled:.1f} m driven in 60 s within {span:.2f} m"
            return ""

        def escape_cmd(s, now, safe):
            e = s.esc
            if s.yaw is None: e["ph"] = "done"; return 0.0, 0.0
            err = math.atan2(math.sin(e["hd"] - s.yaw), math.cos(e["hd"] - s.yaw))
            if e["ph"] == "turn":
                if abs(err) < math.radians(15) or now - e["t"] > 8: e.update(ph="run", t=now)
                else: return 0.0, math.copysign(max(0.08, min(ESC_W, 0.3 * abs(err))), err)
            if e["ph"] == "run":
                if now - e["t"] > 10 or safe < a.slow_from: e["ph"] = "done"; return 0.0, 0.0
                return CRUISE, max(-0.05, min(0.05, 0.4 * err))
            return 0.0, 0.0

        def on_exp(s, m): s.exp = (m.linear.x, m.angular.z); s.t_exp = time.monotonic()

        def on_allow(s, m):
            new = bool(m.data); s.t_allow = time.monotonic()
            if s.allow and not new:
                s.go = False
                s.halt("allow_drive OFF")
            if new != s.allow: s.get_logger().info(f"allow_drive={new}")
            s.allow = new

        def on_button(s, m):
            if not m.data:
                return
            if not s.allow:
                s.get_logger().info("Go ignored — allow_drive is off")
                return
            s.go = not s.go
            s.heading = None; s.flipped = False; s.room_arrived = False
            s.celebrated = False; s.song = None; s.prev_r = None
            s.get_logger().info("Go tap -> " + ("START" if s.go else "STOP"))
            if not s.go:
                s.halt("Go tap")

        def on_button_event(s, m):
            if int(m.data) == 3:
                s.go = False
                s.heading = None
                s.halt("E-stop")

        def on_room(s, m):
            if len(m.data) >= 7:
                s.room = [float(x) for x in m.data[:7]]
                s.room_t = time.monotonic()

        def _note_err(s, now, err, turning):
            ae = abs(err)
            if s.err_track is None:
                s.err_track = (now, ae)
                return
            t0, a0 = s.err_track
            if not turning:
                s.err_track = (now, ae)
                return
            if now - t0 > 0.8:
                if ae > a0 + 0.4 and not s.flipped:
                    s.turn_sign *= -1
                    s.flipped = True
                    s.heading = None
                    s.get_logger().info("overhead: flipped turn direction")
                s.err_track = (now, ae)

        def overhead_cmd(s, now):
            """Steer from the broom camera. Image motion tells us which way he is facing."""
            rx, ry, rw, rh, ox, oy, orad = s.room
            if rx < 1 or ox < 1:
                s.ov = s.ow = 0.0
                who = "rover and orange" if rx < 1 and ox < 1 else ("rover" if rx < 1 else "orange")
                return 0.0, 0.0, f"overhead: can't see the {who}"
            dist = math.hypot(ox - rx, oy - ry)
            # Stop while there is still a gap, about a rover-width. Closer than
            # that and he drives through the fruit before the tune starts.
            stop_px = 0.95 * max(rw, rh)
            nose = s.orange_dist(now)
            close = dist <= stop_px or (nose is not None and nose <= 0.18 and dist < stop_px * 2.2)
            if s.room_arrived or close:
                s.room_arrived = True
                s.celebrate(now)
                s.pump_song(now)
                s.ov = s.ow = 0.0
                s.heading = None
                return 0.0, 0.0, "overhead: at the orange — playing"
            vx = vy = 0.0
            if s.prev_r is not None and now - s.prev_rt < 0.45:
                vx, vy = rx - s.prev_r[0], ry - s.prev_r[1]
            s.prev_r = (rx, ry)
            s.prev_rt = now
            if math.hypot(vx, vy) > 3.0:
                s.heading = (vx, vy)
            if s.heading is None:
                tv, tw = 0.07, 0.0
                why = f"overhead: learning which way he's facing, {dist:.0f}px"
            else:
                hx, hy = s.heading
                dx, dy = ox - rx, oy - ry
                err = math.atan2(hx * dy - hy * dx, hx * dx + hy * dy) * s.turn_sign
                turning = abs(err) > 0.55
                s._note_err(now, err, turning)
                if turning:
                    tv, tw = 0.0, max(-0.08, min(0.08, 0.14 * err))
                    if abs(err) > 1.3:
                        s.heading = None
                    why = f"overhead: turning {math.degrees(err):+.0f} deg, {dist:.0f}px"
                else:
                    tv = 0.10 if dist > stop_px * 2.2 else 0.06
                    tw = max(-0.06, min(0.06, 0.12 * err))
                    why = f"overhead: to the orange, {dist:.0f}px, {math.degrees(err):+.0f} deg"
            s.ov += max(-0.008, min(0.008, tv - s.ov))
            s.ow += max(-0.01, min(0.01, tw - s.ow))
            if now - s.last_oh_log > 1.0:
                s.last_oh_log = now
                s.get_logger().info(why)
            return s.ov, s.ow, why

        def halt(s, why):
            z = Twist(); b = Bool(); b.data = False
            for _ in range(10): s.pcmd.publish(z); s.pses.publish(b)
            s.get_logger().info(f"halt: {why}")

        def find_orange(s, img):
            """Largest saturated orange blob. err > 0 means the blob is right of centre."""
            hsv = PF.cv2.cvtColor(img, PF.cv2.COLOR_BGR2HSV)
            mask = PF.cv2.inRange(hsv, (4, 120, 80), (22, 255, 255))
            mask = PF.cv2.morphologyEx(mask, PF.cv2.MORPH_OPEN, PF.np.ones((5, 5), PF.np.uint8))
            cnts, _ = PF.cv2.findContours(mask, PF.cv2.RETR_EXTERNAL, PF.cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                return None
            c = max(cnts, key=PF.cv2.contourArea)
            area = float(PF.cv2.contourArea(c))
            h, w = img.shape[:2]
            if area < 0.004 * w * h:
                return None
            x, y, bw, bh = PF.cv2.boundingRect(c)
            if bw * bh < 1 or area / float(bw * bh) < 0.35:
                return None
            if bw > 0.85 * w and bh > 0.45 * h:
                return None
            m = PF.cv2.moments(c)
            if m["m00"] < 1:
                return None
            cx = m["m10"] / m["m00"]
            return {"err": (cx / w - 0.5) * 2.0, "area": area}

        def orange_dist(s, now):
            """Nearest fresh front range, metres. None if the L8 picture is stale."""
            dist = None
            if s.cols and len(s.cols) == 8 and now - s.f.t_safe <= 0.6:
                mid = [x for x in s.cols[2:6] if x > 0.02]
                if mid:
                    dist = min(mid)
            if s.f.front is not None and now - s.f.t_front <= 0.6:
                dist = s.f.front if dist is None else min(dist, s.f.front)
            return dist

        def celebrate(s, now):
            if s.celebrated:
                return
            s.celebrated = True
            # Major fanfare, two notes per speaker code. ~14 codes x 0.42 s is a bit over 5 s.
            s.song = [
                (523, 659), (784, 1046), (1319, 1568), (1046, 1319),
                (784, 1046), (659, 880), (1046, 1319), (1568, 1319),
                (523, 784), (1046, 1319), (1568, 2093), (1568, 1319),
                (1046, 784), (1319, 1568), (1046, 523),
            ]
            s.song_i = 0
            s.get_logger().info("orange: 5 cm, playing the tune")

        def pump_song(s, now):
            if not s.song or s.song_i >= len(s.song):
                return
            hz1, hz2 = s.song[s.song_i]
            if s.sound(hz1, hz2, 1, 8, now, gap=0.42):
                s.song_i += 1

        def orange_cmd(s, now):
            blob = s.orange_blob if now - s.orange_t < 0.45 else None
            dist = s.orange_dist(now)
            err = blob["err"] if blob else 0.0
            arrived = dist is not None and dist <= 0.05
            if arrived:
                s.arrived = True
                s.celebrate(now)
                tv, tw = 0.0, 0.0
                why = f"orange: stopped at {dist:.2f} m"
            elif dist is None:
                tv = 0.0
                tw = 0.0 if blob is None else max(-0.10, min(0.10, 0.12 * err))
                why = "orange: no range" if blob is None else "orange: lining up, waiting for range"
            elif blob is not None and abs(err) > 0.50:
                tv, tw = 0.0, math.copysign(0.08, err)
                why = f"orange: turning, {dist:.2f} m"
            else:
                tv = 0.10 if dist > 0.35 else (0.06 if dist > 0.15 else 0.04)
                tw = max(-0.05, min(0.05, 0.08 * err)) if blob else 0.0
                why = f"orange: creeping, {dist:.2f} m to go toward 0.05"
            s.ov += max(-0.006, min(0.006, tv - s.ov))
            s.ow += max(-0.008, min(0.008, tw - s.ow))
            if arrived:
                s.ov = s.ow = 0.0
            s.pump_song(now)
            return s.ov, s.ow, why

        def detect(s):
            m, s.img = s.img, None
            if m is None: return
            img = PF.cv2.imdecode(PF.np.frombuffer(m.data, PF.np.uint8), 1)
            now = time.monotonic(); s.f.on_image(img, now); s.nimg += 1
            hit = s.find_orange(img)
            s.orange_blob = hit
            if hit:
                s.orange_t = now
            if s.nimg % 2 == 0: s.tags_detect(img, now)

        def goto(s, st, why):
            if st == s.state: return
            s.get_logger().info(f"{s.state} -> {st} ({why})"); s.trans.append((round(time.time(), 1), s.state, st, why))
            s.state = st; s.t_state = time.monotonic()
            if st == "ORANGE":
                s.arrived = False; s.celebrated = False; s.song = None; s.song_i = 0
                s.ov = s.ow = 0.0; s.esc = None
            if st == "FOLLOW":
                # start from rest: explore's turn rate must not carry over (the ~0.4 s yaw coast swung the person
                # out of frame, so every FOLLOW timed out without tracking)
                f = s.f; f.cmd = (0.0, 0.0); f.raw = (0.0, 0.0); f.bo = None; f.t_obs = None; f.n_low = 0; s.esc = None
                now = time.monotonic()
                if now - s.t_person_snd > 5.0 and s.sound(1800, 2400, 2, 1, now, gap=0.0): s.t_person_snd = now

        def step(s):
            now = time.monotonic(); f = s.f
            if now - s.t_allow > 3.0 and s.allow:                 # allow_drive heartbeat lost -> fail safe
                s.halt("allow_drive stale"); s.allow = False
            # Orange wins over explore and follow while a solid blob stays in frame.
            blob = s.orange_blob if now - s.orange_t < 0.45 else None
            if blob:
                if s.orange_since is None: s.orange_since = now
            elif s.state != "ORANGE":
                s.orange_since = None
            if s.state != "ORANGE" and s.orange_since is not None and now - s.orange_since >= 0.45:
                s.goto("ORANGE", "orange held in view")
            if s.state == "ORANGE":
                if blob is None:
                    if s.orange_miss is None: s.orange_miss = now
                    hold = 3.0 if s.arrived else 1.0
                    if now - s.orange_miss > hold:
                        s.goto("EXPLORE", "orange done" if s.arrived else "orange lost")
                else:
                    s.orange_miss = None
            # person stability for FOLLOW entry
            seen = f.target is not None and now - f.t_seen < 0.3
            if seen:
                if s.tgt_since is None: s.tgt_since = now
            else:
                s.tgt_since = None
            if s.state == "FOLLOW":
                s.goto("EXPLORE", "person follow paused — orange has the wheels")
            elif s.state == "FOLLOW" and f.bo is None and now - f.t_seen > exit_s:
                s.goto("EXPLORE", f"person lost {now - f.t_seen:.1f}s")
            if s.state == "ORANGE":
                v, w, why = s.orange_cmd(now)
            elif s.state == "FOLLOW":
                s.esc = None
                v, w = f.control(now); why = f.why
            else:
                safe = min([x for x in (f.front, f.l8cols_min) if x is not None] or [9.0])
                l8_stale = now - f.t_safe > a.stale_s
                v, w = s.exp if now - s.t_exp < 0.5 else (0.0, 0.0)
                why = "explore" if now - s.t_exp < 0.5 else "explore cmd stale"
                if GAP:
                    v, w, why = s.gap_cmd(now, safe)
                    if s.tag_vis and now - s.t_tagpause > 6.0: s.t_tagpause = now
                    if now - s.t_tagpause < 1.5 and s.ex["ph"] not in ("cruise", "turn", "rot") and now - s.t_tagplan > 2.0:
                        v, w, why = 0.0, 0.0, f"tag {[t['id'] for t in s.tag_vis]} seen: hold"
                    if s.tgt_since is not None: v, w, why = 0.0, 0.0, "person? hold still"
                elif s.esc is None:
                    r = s.loiter(now)
                    if r:
                        cur = int(((s.yaw or 0.0) % (2 * math.pi)) / (2 * math.pi) * NB) % NB
                        score = [s.bins[k] + (0.5 if min((k - cur) % NB, (cur - k) % NB) <= 1 and "tag" in r else 0.0) for k in range(NB)]
                        k = min(range(NB), key=lambda k: (score[k], -min((k - cur) % NB, (cur - k) % NB)))
                        s.esc = dict(ph="turn", t=now, hd=(k + 0.5) * 2 * math.pi / NB); s.esc_reason = r
                        s.get_logger().info(f"ESCAPE ({r}) -> heading {math.degrees(s.esc['hd']):.0f} deg")
                        if "tag" in r:
                            for q in s.tags.values(): q.clear()
                if not GAP and s.esc is not None:
                    ev, ew = s.escape_cmd(now, safe)
                    if s.esc["ph"] == "done": s.esc = None; s.t_esc_end = now
                    else: v, w, why = ev, ew, f"escape {s.esc['ph']} ({s.esc_reason})"
                if now - f.t_img > 2.0: why += f" (camera offline {min(99, now - f.t_img):.0f}s: no person detection)"
                w = max(-EWMAX, min(EWMAX, w))
                if a.need_l8 and l8_stale: v, w, why = 0.0, 0.0, "stale L8"
                elif safe < a.stop_dist and v > 0: v, why = 0.0, f"explore: obstacle {safe:.2f} < {a.stop_dist} (turn only)"
                f.cmd = (v, w); f.raw = (v, w); f.t_ctl = now
            if not s.go:
                v, w, why = 0.0, 0.0, "stopped — press Go"
            elif now - s.room_t < 2.0:
                v, w, why = s.overhead_cmd(now)
            s.cmd = (float(v), float(w))
            e = Bool(); e.data = s.state == "EXPLORE"; s.pexp.publish(e)
            tw = Twist(); tw.linear.x, tw.angular.z = s.cmd
            s.pwould.publish(tw)
            b = Bool(); b.data = bool(s.allow and s.go)
            if s.allow and s.go:
                s.pcmd.publish(tw)
                if f.bo is not None and f.servo_req is not None and now - s.t_servo > 0.5:
                    sm = Float32(); sm.data = float(f.servo_req); s.pservo.publish(sm); s.t_servo = now
            else:
                s.pcmd.publish(Twist())
            s.pses.publish(b)
            if f.bo is None: f.servo_req = None
            st = String(); st.data = json.dumps(dict(t=time.time(), state=s.state, allow=s.allow, v=round(s.cmd[0], 3), w=round(s.cmd[1], 3),
                        why=why, tgt=f.target, ndet=len(f.dets), dist=None if f.dist is None else round(f.dist, 2),
                        bearing_deg=round(math.degrees(f.bearing), 1), reid=getattr(f, "n_reid", 0), age=round(min(9.0, now - f.t_seen), 2), exp=[round(x, 3) for x in s.exp],
                        cam_age=round(min(99.0, now - f.t_img), 1), tags=[t["id"] for t in s.tag_vis],
                        gap=s.ex['ph'], tagvis={str(k): v['n'] for k, v in s.tn.items()}, yaw_off=round(math.degrees(s.yaw_off), 1), yaw_deg=None if s.yaw is None else round(math.degrees(s.yaw)), path_m=round(s.path, 2)))
            s.pst.publish(st)

    rclpy.init(); n = B()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.allow: n.halt("shutdown")
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
