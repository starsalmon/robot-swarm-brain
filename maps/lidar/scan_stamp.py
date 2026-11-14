#!/usr/bin/env python3
"""Republish /rover/scan with the NUC clock.

The Pro S3 stamps LaserScan with its own clock (uptime seconds), while
odom->base_link is on the brain clock, so slam_toolbox would otherwise drop
every scan as 'timestamp earlier than the transform cache'.
"""
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

HEALTH = Path("/maps/lidar/house_run/lidar_health.json")


class ScanStamp(Node):
    def __init__(self):
        super().__init__("rover_scan_stamp")
        self.pub = self.create_publisher(LaserScan, "/rover/scan_sync", 10)
        self.create_subscription(LaserScan, "/rover/scan", self._cb, 10)
        self.create_timer(0.3, self._flush_health)
        self._n = 0
        self._last = None
        self._last_n = 0
        self._ranges = 0

    def _cb(self, msg: LaserScan) -> None:
        # Hot path: no file I/O. Only restamp, publish, and remember last receipt.
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self._n += 1
        self._last = time.time()
        self._ranges = len(msg.ranges)
        if self._n == 1 or self._n % 50 == 0:
            self.get_logger().info(f"restamped {self._n} scans -> /rover/scan_sync")

    def _flush_health(self) -> None:
        if self._last is None:
            return
        try:
            HEALTH.parent.mkdir(parents=True, exist_ok=True)
            now = time.time()
            doc = {
                "wall_t": now,
                "source": "scan_stamp",
                "n": self._n,
                "topics": {
                    "scan": {"wall_t": self._last, "n": self._ranges},
                    "scan_sync": {"wall_t": self._last, "n": self._ranges},
                },
            }
            tmp = HEALTH.with_suffix(".tmp")
            tmp.write_text(json.dumps(doc))
            tmp.replace(HEALTH)
            if self._n == self._last_n and now - self._last > 2.0:
                self.get_logger().warn(f"no new scans for {now - self._last:.1f}s (n={self._n})")
            self._last_n = self._n
        except Exception as e:
            self.get_logger().warn(f"health write failed: {e}")


def main() -> None:
    rclpy.init()
    node = ScanStamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
