#!/usr/bin/env python3
"""Wheel-tick distance + IMU yaw → /odom and tf. Range sensors → /scan.

Heading comes from the gyro. Wheel ticks are distance along that heading.
Straight driving stays straight because the heading hold already proved that.
"""
from __future__ import annotations

import math
import os

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from rover_wheel_odom import WheelOdometry
from rover_qos import TELEMETRY_QOS


def _yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class RoverNavSensors(Node):
    def __init__(self) -> None:
        super().__init__('rover_nav_sensors')
        self._wheels = WheelOdometry()
        self._gyro_sign = float(os.environ.get('ROVER_IMU_GYRO_SIGN', '1'))
        self._yaw = 0.0
        self._yaw_ready = False
        self._last_imu = None
        self._dist = 0.0
        self._x = 0.0
        self._y = 0.0
        self._left = float('nan')
        self._right = float('nan')
        self._sonar = float('nan')
        self._cols = [float('nan')] * 8

        self.create_subscription(Imu, '/imu/data', self._on_imu, TELEMETRY_QOS)
        from std_msgs.msg import UInt32
        self.create_subscription(UInt32, '/rover/wheel/left_ticks', self._on_left, TELEMETRY_QOS)
        self.create_subscription(UInt32, '/rover/wheel/right_ticks', self._on_right, TELEMETRY_QOS)
        self.create_subscription(Range, '/rover/tof/left', self._on_tof_l, qos_profile_sensor_data)
        self.create_subscription(Range, '/rover/tof/right', self._on_tof_r, qos_profile_sensor_data)
        self.create_subscription(Range, '/rover/sonar/range', self._on_sonar, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/rover/tof/l8_cols', self._on_cols, qos_profile_sensor_data)

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self._tf = TransformBroadcaster(self)
        self.create_timer(0.05, self._publish_odom)
        self.create_timer(0.2, self._publish_scan)
        self.get_logger().info(f'nav sensors up — {self._wheels.config_summary()}')

    def _on_imu(self, msg: Imu) -> None:
        now = self.get_clock().now()
        gz = self._gyro_sign * float(msg.angular_velocity.z)
        if self._last_imu is not None:
            dt = (now - self._last_imu).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                self._yaw += gz * dt
        self._last_imu = now
        self._yaw_ready = True
        self._gz = gz

    def _on_left(self, msg) -> None:
        self._advance(msg)

    def _on_right(self, msg) -> None:
        self._advance(msg, right=True)

    def _advance(self, msg, right: bool = False) -> None:
        before = self._wheels.snapshot().distance_m
        if right:
            self._wheels.on_right(int(msg.data))
        else:
            self._wheels.on_left(int(msg.data))
        after = self._wheels.snapshot().distance_m
        delta = after - before
        if delta > 0.0 and self._yaw_ready:
            self._x += delta * math.cos(self._yaw)
            self._y += delta * math.sin(self._yaw)
            self._dist = after

    def _on_tof_l(self, msg: Range) -> None:
        self._left = float(msg.range)

    def _on_tof_r(self, msg: Range) -> None:
        self._right = float(msg.range)

    def _on_sonar(self, msg: Range) -> None:
        self._sonar = float(msg.range)

    def _on_cols(self, msg: Float32MultiArray) -> None:
        vals = [float(v) for v in msg.data[:8]]
        while len(vals) < 8:
            vals.append(float('nan'))
        self._cols = vals

    def _publish_odom(self) -> None:
        now = self.get_clock().now().to_msg()
        half = 0.5 * self._yaw
        qz = math.sin(half)
        qw = math.cos(half)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        snap = self._wheels.snapshot()
        speed = 0.5 * (snap.left_rate + snap.right_rate) * (math.pi * 0.05 / 6.0)
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = getattr(self, '_gz', 0.0)
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf.sendTransform(tf)

    def _put(self, ranges: list[float], angle: float, dist: float) -> None:
        if not math.isfinite(dist) or dist <= 0.03 or dist > 4.0:
            return
        i = int(round((angle + math.pi) / self._inc))
        if 0 <= i < len(ranges):
            ranges[i] = dist if not math.isfinite(ranges[i]) else min(ranges[i], dist)

    def _publish_scan(self) -> None:
        n = 180
        self._inc = 2.0 * math.pi / n
        ranges = [float('inf')] * n
        # 0 rad = forward, +left. L8 columns 0..7 span about ±22°.
        for i, dist in enumerate(self._cols):
            ang = (3.5 - i) / 3.5 * math.radians(22.0)
            self._put(ranges, ang, dist)
        self._put(ranges, math.pi / 2.0, self._left)
        self._put(ranges, -math.pi / 2.0, self._right)
        self._put(ranges, math.pi, self._sonar)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self._inc
        scan.range_min = 0.05
        scan.range_max = 4.0
        scan.ranges = ranges
        self._scan_pub.publish(scan)


def main() -> None:
    rclpy.init()
    node = RoverNavSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
