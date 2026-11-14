#!/usr/bin/env python3
"""One short FORWARD pulse (no reverse, no turn). Checks the lidar corridor first and aborts if blocked."""
import math, sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt32
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
V = float(sys.argv[1]) if len(sys.argv) > 1 else 0.12
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
LY = -math.pi / 2; NOSE = 0.12; HALF = 0.13
rclpy.init(); n = Node("grok_fwd_pulse"); st = {"ahead": None}
def on_scan(m):
    best = 9.0
    for k, r in enumerate(m.ranges):
        if not math.isfinite(r) or r < 0.12: continue
        a = m.angle_min + k * m.angle_increment + LY
        x, y = r * math.cos(a), r * math.sin(a)
        if x > NOSE - 0.02 and abs(y) < HALF: best = min(best, x - NOSE)
    st["ahead"] = best; st["t_scan"] = time.time()
n.create_subscription(LaserScan, "/rover/scan_sync", on_scan, qos_profile_sensor_data)
n.create_subscription(UInt32, "/rover/wheel/left_ticks", lambda m: st.__setitem__("tl", m.data), qos_profile_sensor_data)
n.create_subscription(UInt32, "/rover/wheel/right_ticks", lambda m: st.__setitem__("tr", m.data), qos_profile_sensor_data)
n.create_subscription(Range, "/rover/tof/front", lambda m: st.__setitem__("front", m.range), 10)
n.create_subscription(Odometry, "/odometry/filtered", lambda m: st.__setitem__("od", (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
cmd = n.create_publisher(Twist, "/cmd_vel", 10); ses = n.create_publisher(Bool, "/rover/session", 10)
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.05)
a0 = st["ahead"]; tl0, tr0, od0 = st.get("tl"), st.get("tr"), st.get("od")
print(f"before: lidar corridor ahead={a0} tof_front={st.get('front')} ticks=({tl0},{tr0}) odom={od0}", flush=True)
if a0 is None or a0 < 0.30:
    print("ABORT: front not clear by lidar", flush=True); sys.exit(2)
tw = Twist(); tw.linear.x = min(V, 0.16); on = Bool(data=True); t0 = time.time(); why = "done"
while time.time() - t0 < DUR:
    ses.publish(on); cmd.publish(tw)
    rclpy.spin_once(n, timeout_sec=0.04)
    if st["ahead"] is not None and st["ahead"] < 0.18: why = f"stopped early: lidar ahead {st['ahead']:.2f}"; break
for _ in range(10):
    cmd.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
ses.publish(Bool(data=False))
t1 = time.time()
while time.time() - t1 < 1.5: rclpy.spin_once(n, timeout_sec=0.05)
od1 = st.get("od")
print(f"{why}; after: lidar ahead={st['ahead']:.2f} tof_front={st.get('front')} ticks=({st.get('tl')},{st.get('tr')}) dticks=({(st.get('tl') or 0)-(tl0 or 0)},{(st.get('tr') or 0)-(tr0 or 0)}) odom_moved={math.hypot(od1[0]-od0[0], od1[1]-od0[1]) if od0 and od1 else None}", flush=True)
