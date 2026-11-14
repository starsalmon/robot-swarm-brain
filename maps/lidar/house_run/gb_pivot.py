#!/usr/bin/env python3
"""One short pivot-in-place pulse (no reverse, no forward). W>0 = left. Stops when the nose is clear."""
import math, sys, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Range, Imu
W = float(sys.argv[1]) if len(sys.argv) > 1 else 0.14
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 0.6
rclpy.init(); n = Node("grok_pivot_pulse"); st = {}
for nm in ("front", "front_left", "front_right", "left", "right"):
    n.create_subscription(Range, f"/rover/tof/{nm}", lambda m, nm=nm: st.__setitem__(nm, m.range), 10)
n.create_subscription(Imu, "/imu/data", lambda m: st.__setitem__("gz", m.angular_velocity.z), 10)
cmd = n.create_publisher(Twist, "/cmd_vel", 10); ses = n.create_publisher(Bool, "/rover/session", 10)
t0 = time.time()
while time.time() - t0 < 2: rclpy.spin_once(n, timeout_sec=0.05)
fmt = lambda: {k: (round(v, 2) if isinstance(v, float) else v) for k, v in st.items()}
print("before", fmt(), flush=True)
side = st.get("right" if W < 0 else "left")
if side is not None and math.isfinite(side) and 0.02 < side < 0.15:
    print("ABORT: tail side tight", flush=True); sys.exit(2)
tw = Twist(); tw.angular.z = W; on = Bool(data=True); t0 = time.time(); why = "timeout"; yaw = 0.0; tp = t0
while time.time() - t0 < DUR:
    ses.publish(on); cmd.publish(tw); rclpy.spin_once(n, timeout_sec=0.03)
    now = time.time(); yaw += (st.get("gz") or 0.0) * (now - tp); tp = now
    f, fl, fr = st.get("front", 0), st.get("front_left", 0), st.get("front_right", 0)
    if time.time() - t0 > 0.15 and f > 0.25 and fl > 0.15 and fr > 0.15: why = "nose clear"; break
for _ in range(10): cmd.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
ses.publish(Bool(data=False))
t1 = time.time()
while time.time() - t1 < 1.0: rclpy.spin_once(n, timeout_sec=0.05)
print(f"{why}; turned ~{math.degrees(yaw):.0f} deg; after", fmt(), flush=True)
