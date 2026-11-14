#!/usr/bin/env python3
"""Read-only preflight for the morning house run. Prints one JSON line. Never publishes anything."""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, Float32, String

WAIT = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0


def main():
    rclpy.init()
    n = Node("house_explore_preflight")
    g = {}
    t = {}

    def rec(k, f=lambda m: m):
        def cb(m):
            g[k] = f(m)
            t[k] = time.time()
        return cb

    q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST, depth=1)
    n.create_subscription(Bool, "/fleet/allow_drive", rec("allow", lambda m: bool(m.data)), 10)
    n.create_subscription(Float32, "/rover/battery/voltage", rec("batt", lambda m: round(float(m.data), 2)), 10)
    n.create_subscription(String, "/rover/behaviour/status", rec("status", lambda m: json.loads(m.data)), 10)
    def _mpu(m):
        s = m.data
        i = s.find(" mpu=")
        return int(s[i + 5]) if i >= 0 and s[i + 5:i + 6].isdigit() else None
    n.create_subscription(String, "/rover/drive_diag", rec("imu_mpu", _mpu), 10)
    n.create_subscription(Bool, "/rover/odom/imu_ok", rec("imu_ok", lambda m: bool(m.data)), 10)
    n.create_subscription(Range, "/rover/tof/front", rec("front", lambda m: round(float(m.range), 3)), 10)
    n.create_subscription(LaserScan, "/rover/scan", rec("scan", lambda m: len(m.ranges)), qos_profile_sensor_data)
    n.create_subscription(LaserScan, "/rover/scan_sync", rec("scan_sync", lambda m: len(m.ranges)), qos_profile_sensor_data)
    n.create_subscription(OccupancyGrid, "/map", rec("map", lambda m: dict(w=m.info.width, h=m.info.height, res=m.info.resolution,
                          free=sum(1 for v in m.data if v == 0), occ=sum(1 for v in m.data if v >= 50))), q)
    t0 = time.time()
    while time.time() - t0 < WAIT:
        rclpy.spin_once(n, timeout_sec=0.1)
    st = g.get("status") or {}
    out = dict(
        allow=g.get("allow"), batt=g.get("batt"), tof_front=g.get("front"),
        scan_live="scan" in g, scan_sync_live="scan_sync" in g, map=g.get("map"),
        behaviour_live="status" in g, go=st.get("go"), state=st.get("state"), why=st.get("why"),
        lidar_ahead=st.get("lidar_ahead"), lidar_guard_off=st.get("lidar_guard_off"),
        behaviour_has_map_explore="lidar_ahead" in st,
        imu_mpu=g.get("imu_mpu"), imu_ok=g.get("imu_ok"),
    )
    print(json.dumps(out))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
