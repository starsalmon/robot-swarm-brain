import time, math, json, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range, LaserScan, CompressedImage
from std_msgs.msg import UInt32, Float32, Bool, String, UInt8
from geometry_msgs.msg import Twist
import tf2_ros
DUR = 15.0
rclpy.init(); n = Node("grok_still"); D = {}
def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
def rec(k, v): D.setdefault(k, []).append((round(time.time()-T0, 2), v))
T0 = time.time()
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
S = qos_profile_sensor_data
n.create_subscription(UInt32, "/rover/wheel/left_ticks", lambda m: rec("tickL", m.data), S)
n.create_subscription(UInt32, "/rover/wheel/right_ticks", lambda m: rec("tickR", m.data), S)
for t in ["/cmd_vel", "/rover/behaviour/cmd", "/rover/explore/cmd_vel"]:
    n.create_subscription(Twist, t, lambda m, t=t: rec(t, (round(m.linear.x, 3), round(m.angular.z, 3))), 10)
for t in ["/imu/data", "/rover/imu/stamped"]:
    n.create_subscription(Imu, t, lambda m, t=t: rec(t, (round(m.angular_velocity.x, 4), round(m.angular_velocity.y, 4), round(m.angular_velocity.z, 4), round(yaw(m.orientation), 2), round(m.linear_acceleration.z, 2), m.header.stamp.sec + m.header.stamp.nanosec*1e-9)), S)
for t in ["/odometry/filtered", "/rover/odom/wheel"]:
    n.create_subscription(Odometry, t, lambda m, t=t: rec(t, (round(m.pose.pose.position.x, 3), round(m.pose.pose.position.y, 3), round(yaw(m.pose.pose.orientation), 2), round(m.twist.twist.linear.x, 4), round(m.twist.twist.angular.z, 4))), 10)
for t in ["/rover/tof/front", "/rover/tof/left", "/rover/tof/right", "/rover/tof/front_left", "/rover/tof/front_right", "/rover/sonar/range"]:
    n.create_subscription(Range, t, lambda m, t=t: rec(t, m.range if math.isfinite(m.range) else "nan"), S)
for t in ["/rover/scan", "/rover/scan_sync"]:
    n.create_subscription(LaserScan, t, lambda m, t=t: rec(t, (round(time.time() - (m.header.stamp.sec + m.header.stamp.nanosec*1e-9), 3), sum(1 for r in m.ranges if math.isfinite(r) and r > 0.05))), S)
for t in ["/rover/battery/voltage"]:
    n.create_subscription(Float32, t, lambda m, t=t: rec(t, round(m.data, 3)), 10)
for t in ["/rover/session", "/fleet/allow_drive", "/rover/stall", "/rover/bump", "/rover/explore/session", "/rover/behaviour/explore_active"]:
    n.create_subscription(Bool, t, lambda m, t=t: rec(t, m.data), QoSProfile(depth=5))
for t in ["/rover/behaviour/status", "/rover/drive_diag"]:
    n.create_subscription(String, t, lambda m, t=t: rec(t, m.data[:400]), 10)
n.create_subscription(Bool, "/rover/odom/imu_ok", lambda m: rec("imu_ok", m.data), 10)
n.create_subscription(UInt32, "/rover/heartbeat", lambda m: rec("hb", m.data), 10)
n.create_subscription(UInt8, "/rover/drive_mode", lambda m: rec("drive_mode", m.data), 10)
n.create_subscription(CompressedImage, "/rover/camera/image_raw/compressed", lambda m: D.setdefault("img", bytes(m.data)), S)
tl = 0
while time.time() - T0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.02)
    if time.time() - tl > 1.0:
        tl = time.time()
        for parent in ["map", "odom"]:
            try:
                tr = buf.lookup_transform(parent, "base_link", rclpy.time.Time())
                rec("tf_"+parent, (round(tr.transform.translation.x, 3), round(tr.transform.translation.y, 3), round(yaw(tr.transform.rotation), 2)))
            except Exception as e: rec("tf_"+parent, str(e)[:60])
img = D.pop("img", None)
if img: open("/tmp/grok_cam_post.jpg", "wb").write(img)
json.dump(D, open("/tmp/grok_still_post.json", "w"))
print("img", len(img) if img else None)
for k, v in D.items():
    if k in ("/rover/behaviour/status", "/rover/drive_diag"):
        print(k, len(v), v[-1]); continue
    print(k, "n", len(v), "first", v[0][1], "last", v[-1][1])
