#!/usr/bin/env python3
"""Closed-loop short forward test drive for Rover. Run via safe_drive.sh (handles allow_drive/brain/bag).

  python3 rover_safe_drive.py --ticks 60 --clear 0.6 [--vmax 0.10] [--timeout 3.0]

All stop checks run on a fixed 20 Hz timer; callbacks only store the latest values (no image work).
Stops on: tick target (either wheel), L8 min < --stop-dist (0.30), timeout, tick stall (>0.8 s with no
new ticks after 1.2 s), stale sensors (L8 or ticks older than 0.5 s), or Ctrl-C. Always sends zero
cmd_vel + session false x15 on exit. Refuses to start if L8 min (front + all columns) < --clear.
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
        self.yaw = 0.0; self.gz_last = None
        self.create_subscription(Imu, "/imu/data", self.on_imu, 50)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ses = self.create_publisher(Bool, "/rover/session", 10)
        self.state, self.why, self.log = "wait", "", []
        self.t_start = time.monotonic()
        self.timer = self.create_timer(0.05, self.step)   # 20 Hz control + safety loop

    def on_imu(self, m):
        now = time.monotonic()
        if self.gz_last is not None and self.state == "drive":
            self.yaw += m.angular_velocity.z * (now - self.gz_last)
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
            self.yaw = 0.0
            self.t0 = now; self.tick0 = (self.v["tl"], self.v["tr"]); self.last_move = (0, now)
            self.start = dict(l8=self.v["l8"], cols=self.v["cols"], tl=self.v["tl"], tr=self.v["tr"])
            self.state = "drive"
        if self.state != "drive":
            return
        el = now - self.t0
        dl, dr = self.v["tl"] - self.tick0[0], self.v["tr"] - self.tick0[1]
        if dl + dr != self.last_move[0]:
            self.last_move = (dl + dr, now)
        why = None
        if not self.fresh(now): why = "stale sensor data"
        elif dl >= self.a.ticks or dr >= self.a.ticks: why = f"{self.a.ticks} ticks"
        elif self.a.turn_deg and abs(math.degrees(self.yaw)) >= abs(self.a.turn_deg): why = f"IMU turn {math.degrees(self.yaw):+.1f} deg"
        elif self.a.turn_deg and max(dl, dr) > 2.0 * abs(self.a.turn_deg) / 90 * 21: why = "turn tick backstop"
        elif self.a.turn_deg == 0 and self.l8min() < self.a.stop_dist: why = f"L8 min {self.l8min():.2f} < {self.a.stop_dist}"
        elif el > self.a.timeout: why = f"{self.a.timeout} s timeout"
        elif el > 1.2 and now - self.last_move[1] > 0.8: why = "tick stall"
        if why:
            self.finish(why); return
        tw = Twist()
        if self.a.turn_deg:
            w = min(self.a.wmax, 0.10 + (self.a.wmax - 0.10) * el / 0.8)
            tw.angular.z = w if self.a.turn_deg > 0 else -w
        else:
            tw.linear.x = min(self.a.vmax, 0.05 + (self.a.vmax - 0.05) * el / 0.8)
        b = Bool(); b.data = True
        self.cmd.publish(tw); self.ses.publish(b)
        self.log.append(dict(t=round(el, 3), v=tw.linear.x, w=tw.angular.z, yaw=round(math.degrees(self.yaw), 2), dl=dl, dr=dr, l8=self.v["l8"], l8min=self.l8min()))

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
