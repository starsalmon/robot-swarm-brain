#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from sensor_msgs.msg import Range, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry

rclpy.init()
n = Node("house_map_preflight")
got = {}

def take(name):
    def cb(msg):
        got[name] = msg
    return cb

n.create_subscription(String, "/rover/behaviour/status", take("status"), 10)
n.create_subscription(Bool, "/fleet/allow_drive", take("allow"), 10)
n.create_subscription(Bool, "/rover/session", take("session"), 10)
n.create_subscription(Bool, "/rover/behaviour/explore_active", take("explore_active"), 10)
n.create_subscription(Range, "/rover/tof/front", take("front"), 10)
n.create_subscription(Range, "/rover/tof/front_left", take("fl"), 10)
n.create_subscription(Range, "/rover/tof/front_right", take("fr"), 10)
n.create_subscription(Range, "/rover/tof/left", take("left"), 10)
n.create_subscription(Range, "/rover/tof/right", take("right"), 10)
n.create_subscription(Float32MultiArray, "/rover/tof/l8_cols", take("l8"), 10)
n.create_subscription(Float32MultiArray, "/rover/tof/lidar", take("lidar"), 10)
n.create_subscription(Float32MultiArray, "/rover/tof/status", take("tofstat"), 10)
n.create_subscription(Float32, "/rover/battery/voltage", take("batt"), 10)
n.create_subscription(LaserScan, "/rover/scan", take("scan"), qos_profile_sensor_data)
n.create_subscription(Twist, "/cmd_vel", take("cmd"), 10)
n.create_subscription(OccupancyGrid, "/map", take("map"), 10)
n.create_subscription(Odometry, "/odometry/filtered", take("odom"), 10)
n.create_subscription(Bool, "/rover/heartbeat", take("hb"), 10)

t0 = time.time()
while time.time() - t0 < 5.0:
    rclpy.spin_once(n, timeout_sec=0.1)

def rng(k):
    m = got.get(k)
    return None if m is None else round(m.range, 3)

print("allow", None if "allow" not in got else got["allow"].data)
print("session", None if "session" not in got else got["session"].data)
print("explore_active", None if "explore_active" not in got else got["explore_active"].data)
if "status" in got:
    print("status", got["status"].data)
print("tof front/fl/fr/left/right", rng("front"), rng("fl"), rng("fr"), rng("left"), rng("right"))
if "l8" in got:
    print("l8_cols", [round(x, 3) for x in got["l8"].data])
if "lidar" in got:
    print("lidar", list(got["lidar"].data))
if "tofstat" in got:
    print("tofstat", list(got["tofstat"].data))
print("batt", None if "batt" not in got else got["batt"].data)
if "scan" in got:
    s = got["scan"]
    valid = [r for r in s.ranges if s.range_min < r < s.range_max]
    mn = min(valid) if valid else None
    mx = max(valid) if valid else None
    print(f"scan n={len(s.ranges)} valid={len(valid)} min={mn} max={mx}")
if "cmd" in got:
    print("cmd_vel", got["cmd"].linear.x, got["cmd"].angular.z)
else:
    print("cmd_vel none in window")
if "map" in got:
    m = got["map"]
    print(f"map {m.info.width}x{m.info.height} res={m.info.resolution} origin=({m.info.origin.position.x:.2f},{m.info.origin.position.y:.2f})")
if "odom" in got:
    p = got["odom"].pose.pose.position
    print(f"odom pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})")
print("keys", sorted(got))
n.destroy_node()
rclpy.shutdown()
