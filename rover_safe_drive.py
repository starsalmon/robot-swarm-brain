#!/usr/bin/env python3
"""Closed-loop short forward test drive for Rover. Run via safe_drive.sh (handles allow_drive/brain/bag).

  python3 rover_safe_drive.py --ticks 60 --clear 0.6 [--vmax 0.10] [--timeout 3.0]

All stop checks run on a fixed 20 Hz timer; callbacks only store the latest values (no image work).
Stops on: tick target (either wheel), L8 min < --stop-dist (0.30), timeout, tick stall (>0.8 s with no
new ticks after 1.2 s), stale sensors (L8 or ticks older than 0.5 s), or Ctrl-C. Always sends zero
cmd_vel + session false x15 on exit. Refuses to start if L8 min (front + all columns) < --clear.
Turns (--turn-deg): Rover coasts after the stop (~0.34 s x yaw rate, ~35-40 deg at 0.12 cmd, bag 2026-10-21),
so the turn is cut when yaw + rate*--coast-tau reaches the target, then it waits for the gyro to settle and,
if |error| > --corr-tol, runs up to --corr-max short slow correction turns with the same predictive stop.
"""
import argparse, json, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32MultiArray, UInt32

M_PER_TICK = math.pi * 0.095 / 60


class SafeDrive(Node):
    def __init__(self, a):
        super().__init__("rover_safe_drive")
        self.a = a
        self.v = {"l8": None, "cols": None, "tl": None, "tr": None}
        self.t = {k: 0.0 for k in self.v}
        def st(k, f):
            return lambda m: (self.v.__setitem__(k, f(m)), self.t.__setitem__(k, time.monotonic()))
        self.create_subscription(Range, "/rover/tof/front", st("l8", lambda m: m.range), 10)
        self.create_subscription(Float32MultiArray, "/rover/tof/l8_cols",
                                 st("cols", lambda m: min([x for x in m.data if x > 0.02] or [9.0])), 10)
        self.create_subscription(UInt32, "/rover/wheel/left_ticks", st("tl", lambda m: m.data), 10)
        self.create_subscription(UInt32, "/rover/wheel/right_ticks", st("tr", lambda m: m.data), 10)
        self.yaw = 0.0; self.gz_last = None; self.rate = 0.0
        self.ncorr = 0; self.goal = 0.0; self.seg0 = 0.0; self.t_settle = 0.0; self.t_quiet = None
        self.create_subscription(Imu, "/imu/data", self.on_imu, 50)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ses = self.create_publisher(Bool, "/rover/session", 10)
        self.state, self.why, self.log = "wait", "", []
        self.t_start = time.monotonic()
        self.timer = self.create_timer(0.05, self.step)   # 20 Hz control + safety loop

    def on_imu(self, m):
        now = time.monotonic()
        if self.gz_last is not None and self.state in ("drive", "settle", "corr"):
            self.yaw += m.angular_velocity.z * (now - self.gz_last)
        self.rate = 0.6 * self.rate + 0.4 * m.angular_velocity.z
        self.gz_last = now

    def fresh(self, now):
        return all(self.v[k] is not None and now - self.t[k] < 0.5 for k in self.v)

    def l8min(self):
        return min(self.v["l8"], self.v["cols"])

    def step(self):
        now = time.monotonic()
        if self.state == "wait":
            if now - self.t_start > 5.0:
                self.finish("SKIP: sensors not fresh after 5 s"); return
            if not self.fresh(now):
                return
            if self.l8min() < self.a.clear:
                self.finish(f"SKIP: start clearance {self.l8min():.2f} m < {self.a.clear} m"); return
            if self.a.turn_deg == 0:
                room = self.l8min() - self.a.stop_dist - 0.10   # 0.10 m coast margin
                cap = int(max(0.0, room) / M_PER_TICK)
                if cap < self.a.ticks:
                    self.get_logger().warn(f"tick target {self.a.ticks} -> {cap} (L8 clearance {self.l8min():.2f} m)")
                    self.a.ticks = cap
                if self.a.ticks < 10:
                    self.finish("SKIP: not enough clearance for >=10 ticks"); return
            self.yaw = 0.0; self.goal = math.radians(self.a.turn_deg); self.seg0 = 0.0
            self.t0 = now; self.tick0 = (self.v["tl"], self.v["tr"]); self.last_move = (0, now)
            self.start = dict(l8=self.v["l8"], cols=self.v["cols"], tl=self.v["tl"], tr=self.v["tr"])
            self.state = "drive"
        if self.state == "settle":
            self.settle_step(now); return
        if self.state not in ("drive", "corr"):
            return
        el = now - self.t0
        dl, dr = self.v["tl"] - self.tick0[0], self.v["tr"] - self.tick0[1]
        if dl + dr != self.last_move[0]:
            self.last_move = (dl + dr, now)
        why = None
        if not self.fresh(now): why = "stale sensor data"
        elif dl >= self.a.ticks or dr >= self.a.ticks: why = f"{self.a.ticks} ticks"
        elif self.a.turn_deg and self.pred_reached():
            self.cmd.publish(Twist()); self.state = "settle"; self.t_settle = now; self.t_quiet = None
            self.log.append(dict(t=round(el, 3), ev="cut", yaw=round(math.degrees(self.yaw), 2), rate=round(self.rate, 3)))
            return
        elif self.a.turn_deg and max(dl, dr) > 2.0 * abs(self.a.turn_deg) / 90 * 21: why = "turn tick backstop"
        elif self.a.turn_deg == 0 and self.l8min() < self.a.stop_dist: why = f"L8 min {self.l8min():.2f} < {self.a.stop_dist}"
        elif el > self.a.timeout: why = f"{self.a.timeout} s timeout"
        elif el > 1.2 and now - self.last_move[1] > 0.8 and now - self.t_settle > 1.2: why = "tick stall"
        if why:
            self.finish(why); return
        tw = Twist()
        if self.a.turn_deg:
            if self.state == "corr":
                w = self.a.corr_w
                d = self.corr_dir
                tw.angular.z = w * d
            else:
                w = min(self.a.wmax, 0.10 + (self.a.wmax - 0.10) * el / 0.8)
                tw.angular.z = w if self.a.turn_deg > 0 else -w
        else:
            tw.linear.x = min(self.a.vmax, 0.05 + (self.a.vmax - 0.05) * el / 0.8)
        b = Bool(); b.data = True
        self.cmd.publish(tw); self.ses.publish(b)
        self.log.append(dict(t=round(el, 3), v=tw.linear.x, w=tw.angular.z, yaw=round(math.degrees(self.yaw), 2), dl=dl, dr=dr, l8=self.v["l8"], l8min=self.l8min()))

    def pred_reached(self):
        """Predictive cut: the segment's turned angle plus the expected coast reaches the segment goal."""
        seg_goal = self.seg_goal if self.state == "corr" else self.goal
        done = (self.yaw - self.seg0) + self.rate * self.a.coast_tau
        return done * math.copysign(1, seg_goal) >= abs(seg_goal)

    def settle_step(self, now):
        self.cmd.publish(Twist())
        if not self.fresh(now):
            self.finish("stale sensor data (settle)"); return
        if now - self.t0 > self.a.timeout + 3.0:
            self.finish("timeout (settle)"); return
        if abs(self.rate) < 0.05:
            self.t_quiet = self.t_quiet or now
        else:
            self.t_quiet = None
        if not (self.t_quiet and now - self.t_quiet > 0.3) and now - self.t_settle < self.a.settle_s:
            return
        err = self.goal - self.yaw
        self.log.append(dict(t=round(now - self.t0, 3), ev="settled", yaw=round(math.degrees(self.yaw), 2), err=round(math.degrees(err), 2), ncorr=self.ncorr))
        if abs(math.degrees(err)) > self.a.corr_tol and self.ncorr < self.a.corr_max:
            self.ncorr += 1; self.state = "corr"; self.seg0 = self.yaw; self.seg_goal = err
            self.corr_dir = 1.0 if err > 0 else -1.0; self.last_move = (self.last_move[0], now); self.t_settle = now
            return
        self.finish(f"IMU turn {math.degrees(self.yaw):+.1f} deg (err {math.degrees(err):+.1f}, {self.ncorr} corr)")

    def finish(self, why):
        self.stop_robot()
        self.state, self.why = "done", why
        self.t_done = time.monotonic()

    def stop_robot(self):
        z = Twist(); f = Bool(); f.data = False
        for _ in range(15):
            self.cmd.publish(z); self.ses.publish(f); time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ticks", type=int, default=60); ap.add_argument("--clear", type=float, default=0.6)
    ap.add_argument("--stop-dist", type=float, default=0.30); ap.add_argument("--vmax", type=float, default=0.10)
    ap.add_argument("--timeout", type=float, default=3.0)
    ap.add_argument("--turn-deg", type=float, default=0.0, help="in-place turn (+ = CCW/left) stopped by IMU yaw")
    ap.add_argument("--wmax", type=float, default=0.25); ap.add_argument("--log", default="/tmp/safe_drive.json")
    ap.add_argument("--coast-tau", type=float, default=0.40, help="s; predicted coast = yaw rate x tau")
    ap.add_argument("--settle-s", type=float, default=1.5, help="max wait for gyro to settle after a cut")
    ap.add_argument("--corr-tol", type=float, default=5.0); ap.add_argument("--corr-max", type=int, default=0)
    ap.add_argument("--corr-w", type=float, default=0.10, help="correction turn command (slowest that moves)")
    a = ap.parse_args()
    if a.vmax > 0.25 or a.wmax > 0.40: raise SystemExit("speed above the usable cap")
    rclpy.init(); n = SafeDrive(a)
    try:
        while rclpy.ok() and not (n.state == "done" and time.monotonic() - n.t_done > 2.0):
            rclpy.spin_once(n, timeout_sec=0.01)
    except KeyboardInterrupt:
        n.why = n.why or "Ctrl-C"
    finally:
        n.stop_robot()
    s = getattr(n, "start", None)
    print(f"result: {n.why}")
    if s:
        dl, dr = n.v["tl"] - s["tl"], n.v["tr"] - s["tr"]
        print(f"ticks L{dl:+d} R{dr:+d} -> {(dl + dr) / 2 * M_PER_TICK:.3f} m; L8 {s['l8']:.3f} -> {n.v['l8']:.3f} "
              f"(d {s['l8'] - n.v['l8']:.3f}); IMU yaw {math.degrees(n.yaw):+.1f} deg; max loop dt {max([b['t'] - a_['t'] for a_, b in zip(n.log, n.log[1:])] or [0]):.3f} s")
    json.dump(dict(why=n.why, start=s, log=n.log), open(a.log, "w"))
    n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
