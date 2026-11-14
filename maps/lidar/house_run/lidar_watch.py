#!/usr/bin/env python3
"""Independent lidar publication check. Subscribes to /rover/scan_sync and
/rover/scan only — never goes through house_status_node or TF."""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

OUT = Path(os.environ.get("HOUSE_RUN_DIR", "/maps/lidar/house_run"))
HEALTH = OUT / "lidar_health.json"
LOG = OUT / "lidar_watch.log"


def wlog(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    try:
        with LOG.open("a") as f:
            f.write(line + "\n")
    except Exception:
        pass


class LidarWatch(Node):
    def __init__(self):
        super().__init__("house_lidar_watch")
        self.last = {}
        self.create_subscription(LaserScan, "/rover/scan_sync", self.cb_sync, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/rover/scan", self.cb_raw, qos_profile_sensor_data)
        self.create_timer(0.3, self.flush)
        wlog("lidar_watch started")

    def _rec(self, name, msg):
        valid = sum(1 for r in msg.ranges if msg.range_min < r < msg.range_max)
        self.last[name] = {
            "wall_t": time.time(),
            "header_stamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            "n": len(msg.ranges),
            "valid": int(valid),
            "frame": msg.header.frame_id,
        }

    def cb_sync(self, msg):
        self._rec("scan_sync", msg)

    def cb_raw(self, msg):
        self._rec("scan", msg)

    def flush(self):
        try:
            self._flush()
        except Exception as e:
            wlog(f"flush error: {e}")

    def _flush(self):
        OUT.mkdir(parents=True, exist_ok=True)
        now = time.time()
        ages = {}
        for name, rec in self.last.items():
            ages[name] = now - rec["wall_t"]
        doc = {
            "wall_t": now,
            "topics": self.last,
            "ages": ages,
            "preferred": "scan_sync" if "scan_sync" in self.last else ("scan" if "scan" in self.last else None),
        }
        tmp = HEALTH.with_suffix(".tmp")
        tmp.write_text(json.dumps(doc))
        tmp.replace(HEALTH)


def main():
    rclpy.init()
    node = LidarWatch()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
