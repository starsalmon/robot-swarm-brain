#!/usr/bin/env python3
"""One-shot: is /rover/scan or /rover/scan_sync actually publishing?"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

TIMEOUT = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0


def main():
    rclpy.init()
    n = Node("house_lidar_confirm")
    got = {"ok": False, "topic": None}

    def rec(name):
        def cb(_msg):
            got["ok"] = True
            got["topic"] = name
        return cb

    n.create_subscription(LaserScan, "/rover/scan_sync", rec("scan_sync"), qos_profile_sensor_data)
    n.create_subscription(LaserScan, "/rover/scan", rec("scan"), qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < TIMEOUT and not got["ok"]:
        rclpy.spin_once(n, timeout_sec=0.1)
    print("ALIVE" if got["ok"] else "DEAD", got["topic"] or "", flush=True)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if got["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
