#!/usr/bin/env python3
"""Rover's one always-on behaviour. Priority, re-checked 20 times a second:

  1. ORANGE   a real orange (the fruit: YOLO11 COCO class 49, held in >= 4 of the last 6 frames) is seen by ANY
              camera: Rover's own camera (steer on its bearing, L8 for range, stop at ~5 cm and play the tune), or
              the overhead LifeCam (room_eye.py, same detector + vote, steers Rover there from above).
              Obstacle safety still applies on the way.
  2. FOLLOW   a person held in view >= BEHAVIOUR_ENTER_S: person_follow.Follower (YOLO person + ByteTrack, smooth
              PD steer, holds ~1 m), a bit faster than explore. Back to EXPLORE when lost > BEHAVIOUR_EXIT_S.
  W. WHISTLE  a whistle (AI cam Goertzel detector -> rover_speech.py -> /rover/whistle) means "come here": for
              WHISTLE_S seconds oranges are ignored, a confirmed person goes straight to FOLLOW, and with nobody in
              view Rover turns on the spot (SEEK) looking for one. Losing the person inside the window -> SEEK again.
  3. EXPLORE  rover_explore.Explorer: continuous curving steer from the front L8 + VL53s, speed scales with the
              room ahead, pivots toward the open side when blocked, bump/stall = reverse + pivot, then carry on.

Only publisher of /cmd_vel and /rover/session. Gated by /fleet/allow_drive (HA) and the Go button.
Status: /rover/behaviour/status (JSON), would-be command: /rover/behaviour/cmd.
"""
import json, math, os, sys, time
from collections import deque

sys.argv = [sys.argv[0]] + sys.argv[1:]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import person_follow as PF
from orange_vote import OrangeVote
from rover_explore import Explorer
from cam_gate import CamGate
from rover_guard_escape import unstick_explore
from rover_tof_forward import has_trusted_forward_coverage
from rover_tof_trust import TofTrust, range_msg_stamp_sec

E = PF.E


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage, LaserScan, Range
    from std_msgs.msg import Bool, Float32, Float32MultiArray, String, UInt8, UInt16MultiArray, UInt32

    a = PF.args()
    a.dry_run = 0
    enter_s = E("BEHAVIOUR_ENTER_S", 0.5); exit_s = E("BEHAVIOUR_EXIT_S", 5.0)
    TAG_M = E("TAG_SIZE_M", 0.08)
    NB = 12
    ORANGE_V = E("ORANGE_V", 0.16)
    WHISTLE_S = E("WHISTLE_S", 45.0)                   # how long a whistle keeps Rover looking for / following you
    NEED_WHISTLE = E("FOLLOW_NEED_WHISTLE", 1)        # 1 = FOLLOW only starts within WHISTLE_S of a whistle (Cain 2026-11-10)
    FOLLOW_MAX_S = E("FOLLOW_MAX_S", 120.0)            # whistle-started FOLLOW ends after this, back to EXPLORE
    GUARD_M = E("GUARD_M", 0.20)                        # hard obstacle stop distance for every mode
    # Pro S3 STL-19P lidar (/rover/scan_sync). base_link -> lidar_link yaw -90 deg (scan angle 0 = starboard).
    LIDAR_YAW = math.radians(E("LIDAR_YAW_DEG", -90.0))
    LIDAR_SELF_M = E("LIDAR_SELF_M", 0.12)             # returns closer than this to the lidar centre = Rover itself
    LIDAR_NOSE_X = E("LIDAR_NOSE_X", 0.12)             # lidar centre -> front bumper, m
    LIDAR_HALF_W = E("LIDAR_HALF_W", 0.13)             # corridor half-width that counts as "ahead"
    LIDAR_GUARD_M = E("LIDAR_GUARD_M", 0.15)           # hard stop: lidar obstacle this close to the nose, in the corridor
    LIDAR_BODY_MARGIN = E("LIDAR_BODY_MARGIN", 0.02)   # corridor starts this far behind the nose (x > nose - margin)
    LIDAR_BODY_INNER_W = E("LIDAR_BODY_INNER_W", 0.11) # beside-body band |y| >= this (and x < nose) is ignored
    # Wedge detection (2026-11-13): commanding drive/turn but wheel odom does not move = wheel against a leg / under a table.
    WEDGE_S = E("WEDGE_S", 2.5)                        # commanded motion this long with no odom motion -> wedged
    WEDGE_MOVE_M = E("WEDGE_MOVE_M", 0.015)
    WEDGE_YAW = math.radians(E("WEDGE_YAW_DEG", 4.0))
    WEDGE_REAR_CLEAR_M = E("WEDGE_REAR_CLEAR_M", 0.30)  # rear sonar must show at least this to reverse out
    WEDGE_MAX_TRIES = int(E("WEDGE_MAX_TRIES", 2))       # escapes within WEDGE_WINDOW_S before stopping motors
    WEDGE_WINDOW_S = E("WEDGE_WINDOW_S", 60.0)
    REAR_FRESH_S = E("REAR_FRESH_S", 1.2)               # sonar is ~3 Hz; 0.6 s often read as "unknown"
    WEDGE_RESET_PATH_M = E("WEDGE_RESET_PATH_M", 0.40)  # drove this far since a wedge -> it no longer counts
    REAR_GUARD_M = E("REAR_GUARD_M", 0.20)             # never command reverse with fresh rear sonar closer than this (or stale)
    # 2026-11-13 15:40: Cain - reversing drags a wheel into skirting/furniture. NO_REVERSE=1 (default) turns every
    # reverse into a stop/pivot; a wedge while pivoting escapes FORWARD when the front is clear, never backwards.
    NO_REVERSE = E("NO_REVERSE", 1.0) >= 0.5
    WEDGE_FWD_CLEAR_M = E("WEDGE_FWD_CLEAR_M", 0.30)   # front (L8/ToF and lidar) must show this to escape forward
    # 2026-11-13 16:37: no pivot-in-place with a side ToF this tight (the tail swings into the skirting; the rear
    # sonar cannot see it). Arc forward away from that wall if the front is clear, else hold still.
    TAIL_SIDE_M = E("TAIL_SIDE_M", 0.18)
    TAIL_FWD_M = E("TAIL_FWD_M", 0.28)
    V_FLOOR = E("EXPLORE_V_FLOOR", 0.095)              # slowest forward command that actually turns the wheels on carpet
    WEDGE_FWD_V = E("WEDGE_FWD_V", 0.13)
    PIVOT_OUT_FRONT_M = E("PIVOT_OUT_FRONT_M", 0.0)    # nose clearance needed to turn on the spot instead of the forbidden reverse (0 = even touching)
    CORNER_X_M = E("CORNER_X_M", 0.15); CORNER_Y_M = E("CORNER_Y_M", 0.08); CORNER_STOP_M = E("CORNER_STOP_M", 0.06)
    if NO_REVERSE: WEDGE_MAX_TRIES = int(E("WEDGE_MAX_TRIES_NOREV", 3))
    LIDAR_OFF_FLAG = "/opt/fleet/maps/lidar/house_run/lidar_guard.off"   # touch to disable the lidar guard live
    SEEK_W = E("WHISTLE_SEEK_W", 0.14)                 # turn-on-the-spot speed while looking for the whistler                     # approach speed while far
    ARU = PF.cv2.aruco
    TDICT = ARU.getPredefinedDictionary(ARU.DICT_APRILTAG_36h11); TPAR = ARU.DetectorParameters_create()
    h = TAG_M / 2; TOBJ = PF.np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], PF.np.float32)

    class B(Node):
        def __init__(s):
            super().__init__("rover_behaviour")
            s.f = PF.Follower(a); s.img = None; s.x = Explorer(); s.vote = OrangeVote(need=4, window=6); s.gate = CamGate(enter_s=enter_s)
            s.orange_blob = None; s.orange_t = -1e9; s.orange_miss = None
            s.arrived = False; s.celebrated = False; s.song = None; s.song_i = 0
            s.ov = 0.0; s.ow = 0.0; s.go = False
            s.room = None; s.room_t = -1e9; s.heading = None; s.turn_sign = 1.0
            s.flipped = False; s.err_track = None; s.room_arrived = False; s.last_oh_log = 0.0
            s.prev_r = None; s.prev_rt = 0.0
            s.state = "EXPLORE"; s.t_state = time.monotonic(); s.allow = False; s.t_allow = 0.0
            s.tgt_since = None; s.cmd = (0.0, 0.0); s.trans = []
            s.pose = None; s.yaw = None; s.path = 0.0; s.hist = deque(); s.bins = [0.0] * 12; s.t_bins = time.monotonic()
            s.tags = {}; s.tag_last = {}; s.tag_vis = []; s.nimg = 0; s.yaw_off = 0.0; s.cols = []
            f = s.f
            s.yrate = 0.0; s.t_yaw = 0.0; f.yaw_hist = deque(); f.yaw_fn = lambda: s.yaw or 0.0; f.rate_fn = lambda: s.yrate
            s.create_subscription(CompressedImage, "/rover/camera/image_raw/compressed", lambda m: setattr(s, "img", m), qos_profile_sensor_data)
            s.create_subscription(UInt16MultiArray, "/rover/tof/l8_grid", s.on_grid, 10)
            s.create_subscription(Float32MultiArray, "/rover/tof/l8_cols", s.on_cols, 10)
            s.create_subscription(Range, "/rover/tof/front", s.on_front, 10)
            s.create_subscription(Range, "/rover/sonar/range", lambda m: (setattr(f, "rear", m.range), setattr(f, "t_rear", time.monotonic())), qos_profile_sensor_data)
            for tp, b in (("left", 90), ("right", -90), ("front_left", 45), ("front_right", -45)):
                s.create_subscription(Range, f"/rover/tof/{tp}", lambda m, tp=tp, b=b: s.on_tof(tp, b, m), 10)
            s.create_subscription(Bool, "/rover/bump", lambda m: m.data and s.x.on_bump(time.monotonic(), "bump"), 10)
            s.create_subscription(Bool, "/rover/stall", lambda m: m.data and s.x.on_bump(time.monotonic(), "stall"), 10)
            s.create_subscription(Odometry, "/rover/odom/wheel", s.on_odom, 10)
            # /rover/odom/wheel heading never changes (yaw stuck ~-65 deg all day): take turn evidence from the EKF (IMU) yaw.
            s.fyaw = None; s.t_fyaw = -1e9
            s.create_subscription(Odometry, "/odometry/filtered", s.on_fodom, 10)
            # map-based explore (rover_frontier.py) + lidar forward clearance. Both optional: silence = old behaviour.
            s.corner = {1: (9.0, -1e9), -1: (9.0, -1e9)}; s.pivot_sgn = (0.0, -1e9)
            s.lidar_ahead = None; s.t_lidar = -1e9; s.lidar_off = False; s.t_lidar_flag = -1e9; s.hint = None; s.t_hint = -1e9
            s.create_subscription(Float32MultiArray, "/rover/explore/goal_hint", s.on_hint, 10)
            s.create_subscription(LaserScan, "/rover/scan_sync", s.on_scan, qos_profile_sensor_data)
            s.ptag = s.create_publisher(String, "/rover/apriltags", 10)
            s.pspk = s.create_publisher(UInt32, "/rover/speaker/code", 10); s.t_snd = 0.0; s.t_person_snd = -1e9
            s.create_subscription(Bool, "/fleet/allow_drive", s.on_allow, 10)
            s.create_subscription(Bool, "/rover/button", s.on_button, 10)
            s.t_whistle = -1e9
            s.side = {}; s.t_cols = -1e9; s.tof_trust = TofTrust(); s.t_guard_log = 0.0; s.guard_hits = 0
            s.wd = None; s.wedge_esc = None; s.wedge_hist = deque(); s.wedged = None; s.t_odom = -1e9
            s.create_subscription(Bool, "/rover/whistle", s.on_whistle, 10)
            s.create_subscription(UInt8, "/rover/button_event", s.on_button_event, 10)
            s.create_subscription(Float32MultiArray, "/room/overhead", s.on_room, 10)
            s.pcmd = s.create_publisher(Twist, "/cmd_vel", 10)
            s.pses = s.create_publisher(Bool, "/rover/session", 10)
            s.pservo = s.create_publisher(Float32, "/rover/servo/angle", 10); s.t_servo = 0.0
            s.pexp = s.create_publisher(Bool, "/rover/behaviour/explore_active", 10)
            s.pwould = s.create_publisher(Twist, "/rover/behaviour/cmd", 10)
            s.pst = s.create_publisher(String, "/rover/behaviour/status", 10)
            f.model.predict(PF.np.zeros((768, 1024, 3), PF.np.uint8), classes=[0, 49], half=True, verbose=False)
            s.create_timer(0.05, s.step); s.create_timer(0.02, s.detect)
            s.get_logger().info("rover_behaviour up: ORANGE (any camera) > FOLLOW person > EXPLORE (smooth). Go button gates motion")

        # ---- sensors ----
        def on_cols(s, m):
            if list(m.data) == s.cols:                      # bit-identical frame = frozen L8: not fresh
                s.n_frozen = getattr(s, "n_frozen", 0) + 1
                if s.n_frozen in (20, 200): s.get_logger().warn(f"L8 frozen: {s.n_frozen} identical l8_cols frames -> stale")
                return
            s.n_frozen = 0; now = time.monotonic()
            s.f.l8cols_min = min([x for x in m.data if x > 0.02] or [9.0]); s.f.t_safe = now
            s.cols = list(m.data); s.t_cols = now; s.x.on_cols(s.cols, now)

        def on_front(s, m):
            now = time.monotonic()
            s.tof_trust.update("front", m.range, s.path, range_msg_stamp_sec(m), now)
            sus = s.tof_trust.suspect_republish("front")
            s.f.front = m.range if m.range > 0.02 and not sus else None
            s.f.t_front = now
            s.on_side("front", m.range)
            s.x.on_range("front", 0, float("nan") if sus else m.range, now)

        def on_tof(s, name, bearing, m):
            now = time.monotonic()
            s.tof_trust.update(name, m.range, s.path, range_msg_stamp_sec(m), now)
            sus = s.tof_trust.suspect_republish(name)
            diag_sus = any(s.tof_trust.suspect_republish(n) for n in ("front_left", "front_right"))
            drop_explore = sus or (name in ("front_left", "front_right") and diag_sus)
            s.on_side(name, m.range)
            if name in ("front_left", "front_right") and diag_sus:
                for dn, b in (("front_left", 45), ("front_right", -45)):
                    s.x.on_range(dn, b, float("nan"), now)
            else:
                s.x.on_range(name, bearing, float("nan") if drop_explore else m.range, now)

        def on_side(s, name, r):
            s.side[name] = (r if r is not None and r > 0.02 else None, time.monotonic())

        def guard(s, now, v, w, why):
            """Hard obstacle stop for EVERY mode (FOLLOW, SEEK spin, EXPLORE, ORANGE, overhead): no forward motion with
            anything inside GUARD_M on the L8 centre columns / front ToF, and no turn toward a front-corner or side ToF
            reading inside GUARD_M. Stale front sensing (L8 and front ToF both older than 0.6 s) also blocks forward."""
            ok = lambda x: x is not None and math.isfinite(x) and x > 0.02       # NaN / 0 / -1 = no valid range
            l8_vals = [x for x in (s.cols[2:6] if len(s.cols) == 8 and now - s.f.t_safe < 0.6 else []) if ok(x)]
            front_fresh = ok(s.f.front) and now - s.f.t_front < 0.6
            ahead = min(l8_vals + ([s.f.front] if front_fresh else []) or [9.0])
            l8_fresh = bool(l8_vals)                                  # fresh AND at least one valid centre zone
            def near(n):
                r, t = s.side.get(n, (None, -1e9))
                return r is not None and now - t < 0.6 and r < GUARD_M
            def usable(nm, _fwd):
                return s.tof_trust.usable_in_explore(nm, True)
            fwd_trusted = has_trusted_forward_coverage(s.cols, s.t_cols, s.x.rays, now, 0.6, usable)
            hit = []
            if v > 0 and not fwd_trusted:
                v = 0.0; hit.append("no trusted forward coverage")
            elif v > 0 and ahead < GUARD_M: v = 0.0; hit.append(f"ahead {ahead:.2f} m")
            elif (v > 0 and not s.lidar_off and s.lidar_ahead is not None and now - s.t_lidar < 0.5
                  and s.lidar_ahead < LIDAR_GUARD_M):
                v = 0.0; hit.append(f"lidar ahead {s.lidar_ahead:.2f} m")
            elif v > 0 and not (l8_fresh or front_fresh): v = 0.0; hit.append("no valid front range (L8 centre + front ToF)")
            if w > 0 and (near("front_left") or near("left")): w = 0.0; hit.append("left side close")
            if w < 0 and (near("front_right") or near("right")): w = 0.0; hit.append("right side close")
            if v > 0 and abs(w) > 0.02 and not s.lidar_off:
                cl, tl = s.corner[1]; cr, tr = s.corner[-1]
                cl = cl if now - tl < 0.5 else 9.0; cr = cr if now - tr < 0.5 else 9.0
                if min(cl, cr) < CORNER_STOP_M and v > 0:
                    # jamb/edge right at a nose corner: crawl straight only if the other corner is clear, never arc
                    if max(cl, cr) < CORNER_STOP_M: v = 0.0
                    else: v = min(v, 0.08)
                    w = 0.0; hit.append(f"corner L {cl:.2f} R {cr:.2f}: no arc")
            if hit:
                s.guard_hits += 1; why += " | GUARD: " + ", ".join(hit)
            s.g_ahead = ahead
            return v, w, why

        def on_hint(s, m):
            d = list(m.data)
            if len(d) >= 4:
                now = time.monotonic(); s.hint = d; s.t_hint = now
                s.x.on_goal(d[0], d[1], int(round(d[3])), now)

        def on_scan(s, m):
            now = time.monotonic()
            if now - s.t_lidar_flag > 1.0:
                s.t_lidar_flag = now; s.lidar_off = os.path.exists(LIDAR_OFF_FLAG)
            r = PF.np.asarray(m.ranges, dtype=float)
            if r.size == 0: return
            a = m.angle_min + PF.np.arange(r.size) * m.angle_increment + LIDAR_YAW
            ok = PF.np.isfinite(r) & (r > max(LIDAR_SELF_M, m.range_min)) & (r < m.range_max)
            if not ok.any(): return
            x = r * PF.np.cos(a); y = r * PF.np.sin(a)
            # 2026-11-13: only points at/after the nose count as ahead. Returns beside the body (x < nose, e.g. the
            # self-return blob at +60..+100 deg, r 0.13-0.20 m) made ahead=0 -> lidar guard had to be switched off.
            # Inside the body length (x < nose) only count points well inside the body width (an overhang over Rover,
            # e.g. a couch lip); points at the body edge there are beside Rover / its own parts, not ahead.
            body = (x < LIDAR_NOSE_X - LIDAR_BODY_MARGIN) & (PF.np.abs(y) >= LIDAR_BODY_INNER_W)
            cor = ok & (x > 0) & (PF.np.abs(y) < LIDAR_HALF_W) & ~body
            ahead = float(x[cor].min() - LIDAR_NOSE_X) if cor.any() else 4.0
            ab = PF.np.abs(PF.np.arctan2(PF.np.sin(a), PF.np.cos(a)))
            dg = ok & (ab > math.radians(16)) & (ab <= math.radians(50))
            diag = float(r[dg].min() - LIDAR_NOSE_X) if dg.any() else 4.0
            s.lidar_ahead = max(0.0, ahead); s.t_lidar = now
            # 2026-11-13 17:40: front-corner bands just outside the corridor. A door jamb edge there is missed by the
            # L8/ToF and by the corridor test, and an arcing turn swings the nose corner straight into it.
            band = ok & (x > LIDAR_NOSE_X) & (x < LIDAR_NOSE_X + CORNER_X_M) & (PF.np.abs(y) >= LIDAR_HALF_W - 0.02) & (PF.np.abs(y) < LIDAR_HALF_W + CORNER_Y_M)
            dist = PF.np.hypot(PF.np.maximum(x - LIDAR_NOSE_X, 0.0), PF.np.maximum(PF.np.abs(y) - LIDAR_HALF_W, 0.0))
            for sg in (1, -1):
                sel = band & ((y > 0) if sg > 0 else (y < 0))
                s.corner[sg] = (float(dist[sel].min()) if sel.any() else 9.0, now)
            if not s.lidar_off:
                s.x.on_lidar(max(0.0, ahead), max(0.0, diag), now)

        def on_grid(s, m): s.f.l8 = PF.np.array(m.data, float).reshape(8, 8) / 1000.0; s.f.t_l8 = time.monotonic()

        def on_room(s, m):
            if len(m.data) >= 8:                               # 8th field = Rover track confirmed by motion (room_eye/rover_track.py)
                s.room = [float(x) for x in m.data[:7]]
                if m.data[7] >= 1 and s.room[0] >= 1 and s.room[4] >= 1:   # only a CONFIRMED Rover + a voted orange counts
                    s.room_t = time.monotonic()

        def on_fodom(s, m):
            q = m.pose.pose.orientation
            s.fyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)); s.t_fyaw = time.monotonic()

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
            s.pose = (x, y); s.yaw = yaw; s.t_odom = now
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
                        pass
                    if now - s.tag_last.get(i, -1e9) > 10.0:         # first seen / re-seen after 10 s: its own tone
                        if s.sound(600 + 80 * (i % 20), 0, 1 + i // 20, 4, now): s.get_logger().info(f"tag {i} sound")
                    s.tag_last[i] = now
            for q in s.tags.values():
                while q and now - q[0] > 180: q.popleft()
            s.tag_vis = out
            st = String(); st.data = json.dumps(dict(t=time.time(), tags=out, sightings={str(k): len(v) for k, v in s.tags.items() if v})); s.ptag.publish(st)

        def on_allow(s, m):
            new = bool(m.data); s.t_allow = time.monotonic()
            if s.allow and not new:
                s.go = False
                s.halt("allow_drive OFF")
            if new != s.allow: s.get_logger().info(f"allow_drive={new}")
            s.allow = new

        def on_whistle(s, m):
            if m.data:
                s.t_whistle = time.monotonic(); s.get_logger().info("whistle: looking for a person to follow")

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
            s.wd = None; s.wedge_esc = None
            if s.go: s.wedged = None; s.wedge_hist.clear()
            if not s.go:
                s.halt("Go tap")

        def on_button_event(s, m):
            if int(m.data) == 3:
                s.go = False
                s.heading = None
                s.halt("E-stop")

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

        # ---- orange ----
        def orange_cmd(s, now):
            """Rover's own camera sees the orange: turn onto it and drive in, slowing with the L8 range; stop ~5 cm."""
            blob = s.orange_blob if now - s.orange_t < 0.5 else None
            dist = s.orange_dist(now)
            err = blob["err"] if blob else 0.0
            if dist is not None and dist <= 0.05 or s.arrived:
                s.arrived = True; s.celebrate(now); s.pump_song(now); s.ov = s.ow = 0.0
                return 0.0, 0.0, "orange: arrived, playing the tune"
            if blob is None:
                tv, tw, why = 0.0, 0.0, "orange: lost from camera, holding"
            elif abs(err) > 0.45:
                tv, tw, why = 0.0, math.copysign(0.12, -err), f"orange: turning onto it ({err:+.2f})"
            else:
                d = dist if dist is not None else 9.0
                tv = ORANGE_V if d > 0.6 else (0.11 if d > 0.25 else 0.07)
                tw = max(-0.10, min(0.10, -0.20 * err))
                why = f"orange: driving in, {'?' if dist is None else f'{dist:.2f}'} m"
            s.ov += max(-0.02, min(0.012, tv - s.ov)); s.ow += max(-0.02, min(0.02, tw - s.ow))
            return s.ov, s.ow, why

        def celebrate(s, now):
            if s.celebrated: return
            s.celebrated = True
            s.song = [(523, 659), (784, 1046), (1319, 1568), (1046, 1319), (784, 1046), (659, 880), (1046, 1319), (1568, 1319),
                      (523, 784), (1046, 1319), (1568, 2093), (1568, 1319), (1046, 784), (1319, 1568), (1046, 523)]
            s.song_i = 0; s.get_logger().info("orange: reached, playing the tune")

        def pump_song(s, now):
            if not s.song or s.song_i >= len(s.song): return
            hz1, hz2 = s.song[s.song_i]
            if s.sound(hz1, hz2, 1, 8, now, gap=0.42): s.song_i += 1

        # ---- vision ----
        def detect(s):
            m, s.img = s.img, None
            if m is None: return
            img = PF.cv2.imdecode(PF.np.frombuffer(m.data, PF.np.uint8), 1)
            now = time.monotonic(); s.f.on_image(img, now); s.nimg += 1
            s.gate.on_frame(now, s.f.target is not None and s.f.t_seen >= now - 1e-6)
            hit = s.vote.update(s.f.fruit, img.shape[1], img.shape[0])
            if hit:
                s.orange_blob = hit; s.orange_t = now
            if s.nimg % 3 == 0: s.tags_detect(img, now)

        def goto(s, st, why):
            if st == s.state: return
            s.get_logger().info(f"{s.state} -> {st} ({why})"); s.trans.append((round(time.time(), 1), s.state, st, why))
            s.state = st; s.t_state = time.monotonic()
            if st == "ORANGE":
                s.arrived = False; s.celebrated = False; s.song = None; s.song_i = 0; s.ov = s.ow = 0.0
                s.room_arrived = False; s.heading = None; s.flipped = False; s.prev_r = None
            if st == "FOLLOW":
                f = s.f; f.cmd = (0.0, 0.0); f.raw = (0.0, 0.0); f.bo = None; f.t_obs = None; f.n_low = 0
                now = time.monotonic()
                if now - s.t_person_snd > 5.0 and s.sound(1800, 2400, 2, 1, now, gap=0.0): s.t_person_snd = now

        def wedge(s, now, v, w, why):
            """Commanded drive/turn for WEDGE_S with wheel odom not moving = wedged (wheel on a table leg, body under
            furniture where lidar/ToF still see open floor). Escape once: reverse if rear sonar is clearly open,
            else turn the OTHER way. Repeated wedge inside WEDGE_WINDOW_S -> stop motors (go=False) instead of grinding."""
            rear = s.f.rear if getattr(s.f, "rear", None) is not None and now - getattr(s.f, "t_rear", -1e9) < REAR_FRESH_S else None
            e = s.wedge_esc
            if e is not None:                                   # running an escape manoeuvre
                age = now - e["t"]
                for dur, ev, ew, tag in e["plan"]:
                    if age < dur:
                        if ev < 0 and (rear is None or rear < WEDGE_REAR_CLEAR_M): ev = 0.0   # rear closed in: no reverse
                        if ev > 0: ev, _, why = s.guard(now, ev, 0.0, why)
                        return ev, ew, f"{why} | WEDGE escape: {tag}"
                    age -= dur
                s.wedge_esc = None; s.wd = None
                return v, w, why
            cv, cw = s.cmd
            active = abs(cv) >= 0.05 or abs(cw) >= 0.08
            fy = s.fyaw if now - s.t_fyaw < 0.5 else None
            if abs(cv) < 0.05 and fy is None:                    # pure turn but no working yaw source: cannot judge
                s.wd = None; return v, w, why
            if not active or s.pose is None:
                s.wd = None; return v, w, why
            if s.wd is None:
                s.wd = (now, s.pose, fy, cw); return v, w, why
            t0, p0, y0, w0 = s.wd
            if now - t0 < WEDGE_S:
                return v, w, why
            moved = math.hypot(s.pose[0] - p0[0], s.pose[1] - p0[1])
            dyaw = abs(math.atan2(math.sin(fy - y0), math.cos(fy - y0))) if (fy is not None and y0 is not None) else 0.0
            if abs(cv) < 0.05 and (fy is None or y0 is None):
                s.wd = None; return v, w, why
            if moved >= WEDGE_MOVE_M or dyaw >= WEDGE_YAW:
                s.wd = (now, s.pose, fy, cw); return v, w, why
            # wedged
            while s.wedge_hist and (now - s.wedge_hist[0][0] > WEDGE_WINDOW_S or s.path - s.wedge_hist[0][1] > WEDGE_RESET_PATH_M):
                s.wedge_hist.popleft()
            s.wedge_hist.append((now, s.path)); s.wd = None
            n = len(s.wedge_hist)
            odom_age = now - s.t_odom
            msg = (f"WEDGED #{n}: cmd v={cv:.2f} w={cw:.2f} for {WEDGE_S:.1f}s, odom moved {moved*100:.1f} cm / {math.degrees(dyaw):.1f} deg "
                   f"(odom age {odom_age:.1f}s), rear={None if rear is None else round(rear, 2)}")
            rear_clear = rear is not None and rear >= WEDGE_REAR_CLEAR_M and not NO_REVERSE
            if n > WEDGE_MAX_TRIES or (n >= 2 and not rear_clear and not NO_REVERSE):   # rear blocked: one other-way turn only, then stop
                s.wedged = msg + " -> motors stopped"
                s.go = False; s.halt(s.wedged); s.get_logger().warn(s.wedged)
                return 0.0, 0.0, s.wedged
            turn = -1.0 if (cw if abs(cw) > 1e-3 else w0) > 0 else 1.0     # the OTHER way from what was being commanded
            if abs(cw) < 1e-3 and abs(w0) < 1e-3:
                turn = s.x._open_side(now) or 1.0
            pw = s.x.PIVOT_W
            la = s.lidar_ahead if (s.lidar_ahead is not None and now - s.t_lidar < 0.5) else None
            fwd_clear = getattr(s, "g_ahead", 0.0) >= WEDGE_FWD_CLEAR_M and (la is None or la >= WEDGE_FWD_CLEAR_M)
            if NO_REVERSE and fwd_clear:
                plan = [(1.2, WEDGE_FWD_V, 0.0, f"forward escape (ahead {s.g_ahead:.2f} m, lidar {'-' if la is None else f'{la:.2f}'} m)"),
                        (1.0, 0.0, turn * pw, "turn " + ("left" if turn > 0 else "right"))]
            elif NO_REVERSE:
                plan = [(1.5, 0.0, turn * pw, "front closed, no reverse -> turn other way " + ("left" if turn > 0 else "right"))]
            elif rear_clear:
                plan = [(1.0, -0.14, 0.0, f"reverse (rear {rear:.2f} m clear)"), (1.2, 0.0, turn * pw, "turn " + ("left" if turn > 0 else "right"))]
            else:
                plan = [(1.5, 0.0, turn * pw, f"rear {'unknown' if rear is None else f'{rear:.2f} m'} -> no reverse, turn other way " + ("left" if turn > 0 else "right"))]
            s.wedge_esc = dict(t=now, plan=plan)
            s.get_logger().warn(msg + " -> " + plan[0][3])
            return v, w, f"{why} | {msg}"

        def safety(s, now, v, w, why):
            """Shared obstacle clamp for ORANGE/overhead: never drive forward into something closer than 0.20 m
            unless that something is the orange Rover's own camera is centred on."""
            ahead = min([x for x in (s.cols[2:6] if len(s.cols) == 8 and now - s.f.t_safe < 0.6 else []) if x > 0.02] or [9.0])
            if s.f.front is not None and now - s.f.t_front < 0.6: ahead = min(ahead, s.f.front)
            mine = now - s.orange_t < 0.5 and abs(s.orange_blob["err"]) < 0.35
            if v > 0 and ahead < 0.20 and not mine:
                return s.x.step(now)[:2] + (f"{why} — obstacle {ahead:.2f} m, avoiding",)
            return v, w, why

        # ---- 20 Hz decision ----
        def step(s):
            now = time.monotonic(); f = s.f
            if now - s.t_allow > 3.0 and s.allow:
                s.halt("allow_drive stale"); s.allow = False
            cam_orange = now - s.orange_t < 0.5
            room_orange = now - s.room_t < 1.0
            whistle = now - s.t_whistle < WHISTLE_S
            if whistle: cam_orange = room_orange = False             # "come here" beats the orange
            if cam_orange or room_orange:
                s.orange_miss = None
                s.goto("ORANGE", "orange seen by " + ("Rover camera" if cam_orange else "overhead LifeCam"))
            elif s.state == "ORANGE":
                if s.orange_miss is None: s.orange_miss = now
                if now - s.orange_miss > (3.0 if s.arrived else 1.5):
                    s.goto("EXPLORE", "orange done" if s.arrived else "orange lost")
            cam_live = s.gate.live(now)
            if not cam_live and (f.target is not None or f.dets or f.fruit):
                f.target = None; f.dets = []; f.fruit = []            # nothing from a silent camera counts
            cand, confirmed = s.gate.person(now)
            s.tgt_since = (s.tgt_since or now) if cand else None
            lost = "SEEK" if whistle else "EXPLORE"
            if whistle and s.state in ("EXPLORE", "ORANGE"):
                s.goto("SEEK", "whistle")
            was_follow = s.state == "FOLLOW"
            if NEED_WHISTLE and not whistle and s.state != "FOLLOW":
                cand, confirmed = False, False                    # no whistle: people are ignored
                s.tgt_since = None
            if NEED_WHISTLE: lost = "EXPLORE"                      # lost after a whistle follow: explore, wait for next whistle
            if s.state in ("EXPLORE", "SEEK") and confirmed:
                s.goto("FOLLOW", "person in view" + (" after whistle" if whistle else ""))
            elif s.state == "SEEK" and not whistle:
                s.goto("EXPLORE", "whistle window over, nobody found")
            elif s.state == "FOLLOW" and NEED_WHISTLE and now - s.t_state > FOLLOW_MAX_S:
                s.goto("EXPLORE", f"follow timeout {FOLLOW_MAX_S:.0f}s")
            elif s.state == "FOLLOW" and not cam_live:
                s.goto(lost, "camera stale")
            elif s.state == "FOLLOW" and now - f.t_seen > exit_s and (f.bo is None or now - f.t_seen > exit_s + 4.0):
                s.goto(lost, f"person lost {now - f.t_seen:.1f}s")
            if NEED_WHISTLE and was_follow and s.state != "FOLLOW":
                s.t_whistle = -1e9; s.tgt_since = None             # follow over: ignore people until the next whistle
            if s.state == "ORANGE":
                if cam_orange or s.arrived:
                    v, w, why = s.orange_cmd(now)
                elif room_orange:
                    v, w, why = s.overhead_cmd(now)
                else:
                    v, w, why = 0.0, 0.0, "orange: briefly lost, holding"
                v, w, why = s.safety(now, v, w, why)
            elif s.state == "FOLLOW":
                v, w = f.control(now); why = "follow: " + f.why
            elif s.state == "SEEK":
                v, w = 0.0, SEEK_W * s.turn_sign
                why = f"whistle: turning to find you ({WHISTLE_S - (now - s.t_whistle):.0f}s left)"
                if s.tgt_since is not None: w *= 0.4; why += " (person? confirming)"
                if not cam_live: why += " (camera offline)"
            else:
                v, w, why = s.x.step(now)
                if s.tgt_since is not None:                       # person maybe: ease off while the tracker confirms
                    v *= 0.6; why += " (person? confirming)"
                if now - f.t_img > 2.0: why += f" (camera offline {min(99, now - f.t_img):.0f}s)"
            v_req, w_req = v, w
            v, w, why = s.guard(now, v, w, why)                   # no mode bypasses the obstacle guard
            if s.state == "EXPLORE" and s.allow and s.go:
                rear = s.f.rear if getattr(s.f, "rear", None) is not None and now - getattr(s.f, "t_rear", -1e9) < REAR_FRESH_S else None
                v2, w2, tag = unstick_explore(
                    v, w, v_req, w_req, s.side, now, getattr(s, "g_ahead", 9.0),
                    guard_m=GUARD_M, pivot_w=s.x.PIVOT_W, rear_m=rear, rear_fresh=rear is not None,
                )
                if tag:
                    v, w, why = v2, w2, f"{why} | {tag}"
                    if "pivot anyway" in tag:
                        # Keep the escape turn; still block any forward motion into something close.
                        v, _, why = s.guard(now, v, 0.0, why)
                    else:
                        v, w, why = s.guard(now, v, w, why)
            if NO_REVERSE and v < 0 and s.state == "EXPLORE" and s.allow and s.go:
                # 2026-11-13 17:30: nose ~0.19 m off a wall with a front corner ToF just inside GUARD_M made the turn
                # guard veto both pivots, unstick asked for a reverse, NO_REVERSE held v=w=0 for 90 s and the
                # supervisor called it stuck. If both body sides are clear (the tail has room) and nothing is
                # touching the nose, turn on the spot toward the more open side instead of sitting there.
                def _fr(nm):
                    r, t = s.side.get(nm, (None, -1e9))
                    return r if (r is not None and math.isfinite(r) and r > 0.02 and now - t < 0.6) else None
                sl, sr = _fr("left"), _fr("right")
                fvals = [r for r in (_fr("front"), _fr("front_left"), _fr("front_right")) if r is not None]
                ga = getattr(s, "g_ahead", None)
                if ga is not None: fvals.append(ga)
                fmin = min(fvals) if fvals else 0.0
                if (sl is None or sl >= TAIL_SIDE_M) and (sr is None or sr >= TAIL_SIDE_M) and fmin >= PIVOT_OUT_FRONT_M:
                    L = min([r for r in (sl, _fr("front_left")) if r is not None] or [0.5])
                    R = min([r for r in (sr, _fr("front_right")) if r is not None] or [0.5])
                    ps, pt = s.pivot_sgn
                    if ps and now - pt < 2.5: sgn = ps                # keep the chosen side; no left/right dithering
                    else: sgn = 1.0 if L > R else -1.0
                    s.pivot_sgn = (sgn, now)
                    v, w = 0.0, sgn * s.x.PIVOT_W
                    why += f" | NO_REVERSE: pivot out {'left' if sgn > 0 else 'right'} (sides L {L:.2f} R {R:.2f}, nose {fmin:.2f})"
            if s.state == "EXPLORE" and s.allow and s.go:
                v, w, why = s.wedge(now, v, w, why)
            if s.state == "EXPLORE" and s.go and 0.005 < v < V_FLOOR:
                # 2026-11-13 17:52: v 0.05-0.07 stalls the motors on the bedroom carpet (wheel odom 0 cm, SLAM pose
                # frozen -> WEDGED x3). Any forward creep that passed the guards gets at least V_FLOOR.
                v = V_FLOOR
            if abs(v) < 0.03 and abs(w) > 0.05 and s.go:          # pivot in place: check the tail
                def _sd(nm):
                    r, t = s.side.get(nm, (None, -1e9))
                    return r if (r is not None and math.isfinite(r) and now - t < 0.6) else None
                sl, sr = _sd("left"), _sd("right")
                tight = [(nm, r) for nm, r in (("left", sl), ("right", sr)) if r is not None and r < TAIL_SIDE_M]
                if tight:
                    nm, r = min(tight, key=lambda x: x[1])
                    la = s.lidar_ahead if (s.lidar_ahead is not None and now - s.t_lidar < 0.5) else None
                    fwd = getattr(s, "g_ahead", 0.0)
                    if fwd >= TAIL_FWD_M and (la is None or la >= TAIL_FWD_M):
                        v, w = 0.08, (-1.0 if nm == "left" else 1.0) * 0.10      # roll forward, nose away from the tight wall
                        v, w, why = s.guard(now, v, w, why + f" | TAIL: {nm} {r:.2f} m, arc out forward instead of pivot")
                    else:
                        v, w = 0.0, 0.0; why += f" | TAIL: {nm} {r:.2f} m and front {fwd:.2f} m, hold (no pivot)"
            if v < 0 and NO_REVERSE:                             # no reversing at all, and do NOT swap it for a pivot:
                v = 0.0; w = 0.0; why += " | NO_REVERSE: hold (no reverse, no pivot)"   # pivots swing the tail into skirting (16:15)
            if v < 0:                                            # rear guard for EVERY reverse (plant behind Rover)
                rr = s.f.rear if getattr(s.f, "rear", None) is not None and now - getattr(s.f, "t_rear", -1e9) < REAR_FRESH_S else None
                if rr is None or not math.isfinite(rr) or rr < REAR_GUARD_M:
                    v = 0.0; why += f" | REAR GUARD: {'stale' if rr is None else f'{rr:.2f} m'}, no reverse"
            if not s.go:
                v, w, why = 0.0, 0.0, "stopped — press Go" + (f" ({s.wedged})" if s.wedged else "")
            s.cmd = (float(v), float(w))
            if now - s.t_guard_log >= 1.0:
                s.t_guard_log = now
                sd = {k: (None if r is None or now - t > 0.6 else round(r, 2)) for k, (r, t) in s.side.items()}
                tr = {k: v for k, v in s.tof_trust.snapshot(s.path).items() if k in sd}
                s.get_logger().info(f"[1Hz] mode={s.state} v={s.cmd[0]:.3f} w={s.cmd[1]:.3f} l8_min={s.g_ahead:.2f} side={sd} tof_diag={tr} "
                                    f"allow={s.allow} go={s.go} guard_hits={s.guard_hits} det={getattr(s.f, 'det_src', '-')}({getattr(getattr(s.f, 'det', None), 'why', 'off')}) why={why}")
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
            ob = s.orange_blob if cam_orange else None
            st = String(); st.data = json.dumps(dict(t=time.time(), state=s.state, allow=s.allow, go=s.go, v=round(s.cmd[0], 3), w=round(s.cmd[1], 3),
                        why=why, tgt=f.target, ndet=len(f.dets), nfruit=len(f.fruit), orange_cam=ob, orange_room=room_orange,
                        dist=None if f.dist is None else round(f.dist, 2), bearing_deg=round(math.degrees(f.bearing), 1),
                        age=round(min(9.0, now - f.t_seen), 2), cam_age=round(min(99.0, now - f.t_img), 1), cam_live=cam_live, whistle=whistle, tags=[t["id"] for t in s.tag_vis],
                        yaw_deg=None if s.yaw is None else round(math.degrees(s.yaw)), path_m=round(s.path, 2),
                        map_goal=None if s.hint is None or now - s.t_hint > 1.5 else dict(bearing_deg=round(math.degrees(s.hint[0]), 1), look_m=round(s.hint[1], 2), path_m=round(s.hint[2], 2), state=int(round(s.hint[3]))),
                        lidar_ahead=None if s.lidar_ahead is None or now - s.t_lidar > 0.5 else round(s.lidar_ahead, 2), lidar_guard_off=s.lidar_off))
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
