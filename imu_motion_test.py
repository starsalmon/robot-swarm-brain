#!/usr/bin/env python3
"""Drive straight or rotate while logging IMU integration vs cmd_vel."""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _ns() -> str:
    import os

    return os.environ.get('BOT_NS', 'bot1').strip('/') or 'bot1'


class ImuMotionTest(Node):
    def __init__(self) -> None:
        super().__init__('imu_motion_test')
        ns = _ns()
        self._pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        self._imu_sub = self.create_subscription(Imu, f'/{ns}/imu', self._on_imu, 10)
        self._yaw = 0.0
        self._imu_last: float | None = None
        self._gz_samples: list[float] = []

    def _on_imu(self, msg: Imu) -> None:
        now = time.monotonic()
        gz = float(msg.angular_velocity.z)
        self._gz_samples.append(gz)
        if self._imu_last is not None:
            dt = now - self._imu_last
            if 0.0 < dt < 0.2:
                self._yaw += gz * dt
        self._imu_last = now

    def stop(self) -> None:
        self._pub.publish(Twist())

    def stream(self, lin: float, ang: float, duration_s: float, rate_hz: float = 25.0) -> None:
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        period = 1.0 / rate_hz
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=period)

    def run_straight(self, lin: float, duration_s: float) -> None:
        self.get_logger().info(f'straight: linear.x={lin:.2f} for {duration_s:.1f}s')
        self._yaw = 0.0
        self._gz_samples.clear()
        self._imu_last = None
        self.stream(lin, 0.0, duration_s)
        self.stop()
        drift = math.degrees(self._yaw)
        gz_mean = sum(self._gz_samples) / len(self._gz_samples) if self._gz_samples else 0.0
        self.get_logger().info(
            f'done: IMU yaw drift {drift:+.1f}°  gz_mean={gz_mean:+.4f} rad/s  '
            f'samples={len(self._gz_samples)}'
        )

    def run_rotate_deg(self, ang_vel: float, target_deg: float, timeout_s: float = 12.0) -> None:
        sign = 1.0 if ang_vel >= 0 else -1.0
        target_rad = math.radians(abs(target_deg)) * sign
        self.get_logger().info(
            f'rotate: angular.z={ang_vel:+.2f} target {target_deg:+.0f}° (timeout {timeout_s:.0f}s)'
        )
        self._yaw = 0.0
        self._gz_samples.clear()
        self._imu_last = None
        t0 = time.monotonic()
        msg = Twist()
        msg.angular.z = ang_vel
        period = 0.04
        while time.monotonic() - t0 < timeout_s:
            if abs(self._yaw) >= abs(target_rad) * 0.92:
                break
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=period)
        self.stop()
        actual = math.degrees(self._yaw)
        err = actual - target_deg
        self.get_logger().info(
            f'done: IMU turned {actual:+.1f}° (target {target_deg:+.0f}°, err {err:+.1f}°)'
        )


def main() -> int:
    ap = argparse.ArgumentParser(description='IMU motion bench (pauses brain externally)')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p_st = sub.add_parser('straight', help='drive forward, measure yaw drift')
    p_st.add_argument('linear', type=float, nargs='?', default=0.15)
    p_st.add_argument('seconds', type=float, nargs='?', default=4.0)

    p_rot = sub.add_parser('rotate', help='spin in place to target degrees')
    p_rot.add_argument('angular', type=float, nargs='?', default=0.35)
    p_rot.add_argument('degrees', type=float, nargs='?', default=90.0)

    args = ap.parse_args()
    rclpy.init()
    node = ImuMotionTest()
    try:
        time.sleep(0.3)
        if args.cmd == 'straight':
            node.run_straight(args.linear, args.seconds)
        else:
            node.run_rotate_deg(args.angular, args.degrees)
    finally:
        node.stop()
        time.sleep(0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
