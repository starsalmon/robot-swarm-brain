#!/usr/bin/env python3
"""Rover's one always-on behaviour. Priority, re-checked 20 times a second:

  1. ORANGE   a real orange (the fruit: YOLO11 COCO class 49, held in >= 4 of the last 6 frames) is seen by ANY
              camera: Rover's own camera (steer on its bearing, L8 for range, stop at ~5 cm and play the tune), or
              the overhead LifeCam (room_eye.py, same detector + vote, steers Rover there from above).
              Obstacle safety still applies on the way.
  2. FOLLOW   a person held in view >= BEHAVIOUR_ENTER_S: person_follow.Follower (YOLO person + ByteTrack, smooth
              PD steer, holds ~1 m), a bit faster than explore. Back to EXPLORE when lost > BEHAVIOUR_EXIT_S.
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
    TAG_M = E("TAG_SIZE_M", 0.08)
    NB = 12
    ORANGE_V = E("ORANGE_V", 0.16)                     # approach speed while far
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
                s.create_subscription(Range, f"/rover/tof/{tp}", lambda m, tp=tp, b=b: s.x.on_range(tp, b, m.range, time.monotonic()), 10)
            s.create_subscription(Bool, "/rover/bump", lambda m: m.data and s.x.on_bump(time.monotonic(), "bump"), 10)
            s.create_subscription(Bool, "/rover/stall", lambda m: m.data and s.x.on_bump(time.monotonic(), "stall"), 10)
            s.create_subscription(Odometry, "/rover/odom/wheel", s.on_odom, 10)
            s.ptag = s.create_publisher(String, "/rover/apriltags", 10)
            s.pspk = s.create_publisher(UInt32, "/rover/speaker/code", 10); s.t_snd = 0.0; s.t_person_snd = -1e9
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
            s.cols = list(m.data); s.x.on_cols(s.cols, now)

        def on_front(s, m):
            now = time.monotonic()
            s.f.front = m.range if m.range > 0.02 else None; s.f.t_front = now
            s.x.on_range("front", 0, m.range, now)

        def on_grid(s, m): s.f.l8 = PF.np.array(m.data, float).reshape(8, 8) / 1000.0; s.f.t_l8 = time.monotonic()

        def on_room(s, m):
            if len(m.data) >= 8:                               # 8th field = Rover track confirmed by motion (room_eye/rover_track.py)
                s.room = [float(x) for x in m.data[:7]]
                if m.data[7] >= 1 and s.room[0] >= 1 and s.room[4] >= 1:   # only a CONFIRMED Rover + a voted orange counts
                    s.room_t = time.monotonic()

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
            if s.state == "EXPLORE" and confirmed:
                s.goto("FOLLOW", "person in view")
            elif s.state == "FOLLOW" and not cam_live:
                s.goto("EXPLORE", "camera stale")
            elif s.state == "FOLLOW" and now - f.t_seen > exit_s and (f.bo is None or now - f.t_seen > exit_s + 4.0):
                s.goto("EXPLORE", f"person lost {now - f.t_seen:.1f}s")
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
            else:
                v, w, why = s.x.step(now)
                if s.tgt_since is not None:                       # person maybe: ease off while the tracker confirms
                    v *= 0.6; why += " (person? confirming)"
                if now - f.t_img > 2.0: why += f" (camera offline {min(99, now - f.t_img):.0f}s)"
            if not s.go:
                v, w, why = 0.0, 0.0, "stopped — press Go"
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
            ob = s.orange_blob if cam_orange else None
            st = String(); st.data = json.dumps(dict(t=time.time(), state=s.state, allow=s.allow, go=s.go, v=round(s.cmd[0], 3), w=round(s.cmd[1], 3),
                        why=why, tgt=f.target, ndet=len(f.dets), nfruit=len(f.fruit), orange_cam=ob, orange_room=room_orange,
                        dist=None if f.dist is None else round(f.dist, 2), bearing_deg=round(math.degrees(f.bearing), 1),
                        age=round(min(9.0, now - f.t_seen), 2), cam_age=round(min(99.0, now - f.t_img), 1), cam_live=cam_live, tags=[t["id"] for t in s.tag_vis],
                        yaw_deg=None if s.yaw is None else round(math.degrees(s.yaw)), path_m=round(s.path, 2)))
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
