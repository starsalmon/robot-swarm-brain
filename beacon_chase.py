#!/usr/bin/env python3
"""Chase a 38 kHz IR beacon — dual TSOP or single TSOP on a pan servo."""
from __future__ import annotations

import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from fleet_steering import IrSteerConfig, steer_ir_beacon


class BeaconChase(Node):
    TICK = 0.05
    SIGNAL_HOLD = 1.4

    def __init__(self) -> None:
        ns = os.environ.get('BOT_NS', 'bot1').strip('/')
        pan_center = float(os.environ.get('PAN_CENTER', '90'))
        self.SIGNAL_HOLD = float(os.environ.get('IR_SIGNAL_HOLD', self.SIGNAL_HOLD))
        super().__init__('beacon_chase')
        self._cmd = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        self._detected = False
        self._pan_deg = pan_center
        self._last_seen = 0.0
        self._dual_ir = os.environ.get('IR_DUAL', '0') == '1'
        self._left = False
        self._ir_cfg = IrSteerConfig(pan_center=pan_center)
        self.create_subscription(Bool, f'/{ns}/ir/detected', self._on_detected, 10)
        self.create_subscription(Float32, f'/{ns}/sonar/pan_deg', self._on_pan, 10)
        self.create_timer(self.TICK, self._tick)
        mode = 'dual TSOP' if self._dual_ir else 'pan-scan (body still until IR in view)'
        self.get_logger().info(f'beacon_chase: {mode} hold={self.SIGNAL_HOLD}s')

    def _on_detected(self, msg: Bool) -> None:
        self._detected = bool(msg.data)
        if self._detected:
            self._last_seen = time.monotonic()

    def _on_pan(self, msg: Float32) -> None:
        self._pan_deg = float(msg.data)

    def _signal_recent(self) -> bool:
        return (time.monotonic() - self._last_seen) < self.SIGNAL_HOLD

    def _tick_dual(self, msg: Twist) -> None:
        if self._left and self._detected:
            msg.linear.x = self._ir_cfg.cruise
            msg.angular.z = 0.0
        elif self._left:
            msg.linear.x = self._ir_cfg.cruise * 0.5
            msg.angular.z = 0.18
        elif self._detected:
            msg.linear.x = self._ir_cfg.cruise * 0.5
            msg.angular.z = -0.18
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.12

    def _tick_pan_scan(self, msg: Twist) -> None:
        if not self._signal_recent():
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            return
        steer_ir_beacon(
            msg,
            pan_deg=self._pan_deg,
            ir_now=self._detected,
            cfg=self._ir_cfg,
        )

    def _tick(self) -> None:
        msg = Twist()
        if self._dual_ir:
            self._tick_dual(msg)
        else:
            self._tick_pan_scan(msg)
        self._cmd.publish(msg)


def main() -> None:
    rclpy.init()
    node = BeaconChase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for _ in range(5):
            node._cmd.publish(stop)
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
