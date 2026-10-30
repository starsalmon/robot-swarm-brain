#!/usr/bin/env python3
"""Rover light bridge: poll the AI Cam /status (LTR-308 ambient light sensor) and publish to ROS 2.

Publishes (every ROVER_LIGHT_PERIOD s, only while /status answers)
  /rover/light/lux    std_msgs/Float32  ALS lux as reported by the cam (IR LEDs included)
  /rover/light/night  std_msgs/Bool     True when the cam's ALS auto switch is in its night profile
                                        (IR LEDs on, night exposure, night gain ceiling)

Env
  ROVER_STATUS_URL    default http://192.168.3.128/status
  ROVER_LIGHT_PERIOD  default 1.0 (s)

Read-only: only GETs /status, never changes camera settings.
"""
import json
import os
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

URL = os.environ.get("ROVER_STATUS_URL", "http://192.168.3.128/status")
PERIOD = float(os.environ.get("ROVER_LIGHT_PERIOD", "1.0"))


class LightBridge(Node):
    def __init__(self):
        super().__init__("rover_light_bridge")
        self.pub_lux = self.create_publisher(Float32, "/rover/light/lux", 10)
        self.pub_night = self.create_publisher(Bool, "/rover/light/night", 10)
        self.ok = None
        self.create_timer(PERIOD, self.tick)

    def tick(self):
        try:
            with urllib.request.urlopen(URL, timeout=min(PERIOD, 2.0)) as r:
                d = json.loads(r.read())
        except Exception as e:  # noqa: BLE001 - cam may be rebooting / offline
            if self.ok is not False:
                self.get_logger().warn(f"{URL}: {e}")
            self.ok = False
            return
        if self.ok is not True:
            self.get_logger().info(f"{URL} ok (als_ok={d.get('als_ok')})")
        self.ok = True
        if d.get("als_ok") == 1 and d.get("lux", -1) >= 0:
            self.pub_lux.publish(Float32(data=float(d["lux"])))
        self.pub_night.publish(Bool(data=d.get("profile") == "night" or d.get("night") == 1))


def main():
    rclpy.init()
    node = LightBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
