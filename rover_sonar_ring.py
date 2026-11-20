#!/usr/bin/env python3
"""Front sonar → WS2812 ring: distance colour + sweep beam (forward arc)."""
from __future__ import annotations

import math
import os
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

from rover_ring import set_mode as ring_set_mode
from rover_ring import sonar_frame
from rover_scan_map import front_pan_to_led


def _ring_count() -> int:
    return int(os.environ.get('ROVER_RING_COUNT', '8'))


def _pan_mirror() -> bool:
    return os.environ.get('ROVER_SONAR_PAN_MIRROR', '0').strip().lower() in (
        '1',
        'yes',
        'true',
    )


def _pan_center() -> float:
    return float(os.environ.get('ROVER_SONAR_PAN_CENTER', '90'))


def _hold_sec() -> float:
    return float(os.environ.get('ROVER_SONAR_RING_HOLD_SEC', '4.0'))


def _m_to_cm(distance_m: float) -> int:
    if distance_m <= 0.0 or math.isnan(distance_m):
        return 255
    return max(0, min(254, int(distance_m * 100.0)))


class SonarRingNode(Node):
    def __init__(self) -> None:
        super().__init__('rover_sonar_ring')
        self._n = _ring_count()
        self._bins = [255] * self._n
        self._sweep = 0
        self._pan_deg = _pan_center()
        self._mirror = _pan_mirror()
        self._center = _pan_center()
        self._hold_until = 0.0
        self._scanning = False
        self.create_subscription(Range, '/rover/sonar/range', self._on_range, qos_profile_sensor_data)
        self.create_subscription(Float32, '/rover/sonar/pan_deg', self._on_pan, 10)
        self.create_timer(0.05, self._tick)
        ring_set_mode('sonar')
        self.get_logger().info(
            f'Front sonar ring on {self._n} LEDs (mirror={self._mirror})'
        )

    def _led_for_pan(self, pan_deg: float) -> int:
        return front_pan_to_led(
            pan_deg,
            self._n,
            center=self._center,
            mirror=self._mirror,
        )

    def _update_scan_state(self) -> None:
        off_center = abs(self._pan_deg - self._center) > 10.0
        now = time.monotonic()
        if off_center:
            self._scanning = True
            self._hold_until = now + _hold_sec()
        elif self._scanning and now < self._hold_until:
            return
        else:
            self._scanning = False

    def _on_pan(self, msg: Float32) -> None:
        self._pan_deg = float(msg.data)
        self._sweep = self._led_for_pan(self._pan_deg)
        self._update_scan_state()

    def _on_range(self, msg: Range) -> None:
        if math.isnan(msg.range):
            return
        led = self._led_for_pan(self._pan_deg)
        self._bins[led] = _m_to_cm(float(msg.range))
        self._sweep = led
        self._hold_until = time.monotonic() + _hold_sec()

    def _tick(self) -> None:
        self._update_scan_state()
        if not self._scanning:
            for i, d in enumerate(self._bins):
                if d < 255:
                    self._bins[i] = min(255, d + 1)
        sonar_frame(self._bins, self._sweep)


def main() -> None:
    rclpy.init()
    node = SonarRingNode()
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
