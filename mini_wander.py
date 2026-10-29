#!/usr/bin/env python3
"""Simple wander brain for c3-mini-bot — runs on dockerhost with ROS 2 Jazzy."""
from __future__ import annotations

import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float32

from fleet_steering import WanderConfig, WanderState


class MiniWander(Node):
    TICK = 0.05

    def __init__(self) -> None:
        ns = os.environ.get('BOT_NS', 'bot1').strip('/')
        pan_center = float(os.environ.get('PAN_CENTER', '90'))
        super().__init__('mini_wander')
        self.pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        self._wander = WanderState.from_config(WanderConfig.from_env(pan_center=pan_center))
        self._pan_deg = pan_center
        self.create_subscription(Range, f'/{ns}/sonar/range', self._on_range, 10)
        self.create_subscription(Imu, f'/{ns}/imu', self._on_imu, 10)
        self.create_subscription(Float32, f'/{ns}/sonar/pan_deg', self._on_pan, 10)
        self.create_timer(self.TICK, self._tick)
        self.get_logger().info(f'mini_wander: /{ns}/sonar/range → /{ns}/cmd_vel')

    def _on_range(self, msg: Range) -> None:
        self._wander.update_range(float(msg.range), self._pan_deg)

    def _on_imu(self, msg: Imu) -> None:
        self._wander.update_imu(float(msg.angular_velocity.z))

    def _on_pan(self, msg: Float32) -> None:
        self._pan_deg = float(msg.data)
        self._wander._pan_deg = self._pan_deg

    def _tick(self) -> None:
        msg = Twist()
        self._wander.fill_twist(msg)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MiniWander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for _ in range(5):
            node.pub.publish(stop)
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
