#!/usr/bin/env python3
"""Wheel tick stall cal — publishes straight cruise cmd_vel (ang=0), samples ticks.

Does NOT sample free explore wander. Prefer stopping rover-brain during the run
so explore/HA-idle zeros do not fight the straight drive publisher.
"""
from __future__ import annotations

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32

sys.path.insert(0, '/opt/fleet')
from rover_qos import CMD_VEL_QOS

BTN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
COLLECT_SEC = float(os.environ.get('ROVER_WHEEL_CAL_SEC', '20'))
SAMPLE_DT = 0.2
# Sustained straight cruise — below typical ROVER_AUTO_MAX_LINEAR default (0.16).
CRUISE_LIN = float(os.environ.get('ROVER_WHEEL_CAL_LIN', '0.14'))
PUB_DT = 0.05  # 20 Hz — keep cmd_vel fresh for ESP
# Accept only near-zero angular (evidence of straight).
ANG_ACCEPT = float(os.environ.get('ROVER_WHEEL_CAL_ANG_MAX', '0.02'))


class WheelTickCal(Node):
    def __init__(self) -> None:
        super().__init__('wheel_tick_cal')
        self.btn_pub = self.create_publisher(Bool, '/rover/button', BTN_QOS)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self._l: int | None = None
        self._r: int | None = None
        self._last_lin = 0.0
        self._last_ang = 0.0
        self._session = False
        self._pairs: list[tuple[float, float, float]] = []  # lin, rate, ang
        self._ang_seen: list[float] = []
        self._sample_combined: int | None = None
        self._sample_t: float | None = None
        self._driving = False
        self.create_subscription(UInt32, '/rover/wheel/left_ticks', self._on_l, 10)
        self.create_subscription(UInt32, '/rover/wheel/right_ticks', self._on_r, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, CMD_VEL_QOS)
        self.create_subscription(Bool, '/rover/session', self._on_session, BTN_QOS)
        self.create_timer(SAMPLE_DT, self._sample)
        self.create_timer(PUB_DT, self._publish_drive)

    def _on_l(self, msg: UInt32) -> None:
        self._l = int(msg.data)

    def _on_r(self, msg: UInt32) -> None:
        self._r = int(msg.data)

    def _on_cmd(self, msg: Twist) -> None:
        self._last_lin = float(msg.linear.x)
        self._last_ang = float(msg.angular.z)
        if self._driving:
            self._ang_seen.append(self._last_ang)

    def _on_session(self, msg: Bool) -> None:
        self._session = bool(msg.data)

    def _combined(self) -> int | None:
        if self._l is None or self._r is None:
            return None
        return self._l + self._r

    def _publish_drive(self) -> None:
        if not self._driving:
            return
        msg = Twist()
        msg.linear.x = CRUISE_LIN
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def _stop_drive(self, bursts: int = 10) -> None:
        self._driving = False
        msg = Twist()
        for _ in range(bursts):
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _sample(self) -> None:
        if not self._driving:
            return
        cur = self._combined()
        if cur is None:
            return
        now = time.monotonic()
        if self._sample_combined is None or self._sample_t is None:
            self._sample_combined = cur
            self._sample_t = now
            return
        dt = now - self._sample_t
        if dt < SAMPLE_DT * 0.8:
            return
        delta = cur - self._sample_combined
        rate = delta / dt if dt > 0.0 else 0.0
        lin = self._last_lin
        ang = self._last_ang
        # Straight cruise only — reject steer / reverse / no motion.
        if lin > 0.05 and abs(ang) <= ANG_ACCEPT and delta > 0:
            self._pairs.append((lin, rate, ang))
        self._sample_combined = cur
        self._sample_t = now

    def _tap_go(self) -> None:
        btn = Bool()
        btn.data = True
        self.btn_pub.publish(btn)
        rclpy.spin_once(self, timeout_sec=0.15)

    def _wait_session(self, want: bool, timeout: float) -> bool:
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._session == want:
                return True
        return self._session == want


def main() -> int:
    rclpy.init()
    node = WheelTickCal()
    try:
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._l is not None and node._r is not None:
                break

        if node._session:
            node.get_logger().warn(
                'Session active (explore may fight). Prefer: docker stop rover-brain first.'
            )
            # End session so brain hard-stops before we take over cmd_vel.
            node._tap_go()
            node._wait_session(False, 3.0)

        node.get_logger().info(
            f'Straight cruise lin={CRUISE_LIN:.3f} ang=0 for {COLLECT_SEC:.0f}s '
            f'(accept |ang|<={ANG_ACCEPT:.3f})'
        )
        node._driving = True
        # Warm-up: let lin settle before first sample ratio.
        t_warm = time.monotonic() + 1.0
        while time.monotonic() < t_warm:
            rclpy.spin_once(node, timeout_sec=0.05)
        node._sample_combined = None
        node._sample_t = None
        node._pairs.clear()

        t_done = time.monotonic() + COLLECT_SEC
        while time.monotonic() < t_done:
            rclpy.spin_once(node, timeout_sec=0.05)

        node._stop_drive()
        node.get_logger().info('Stopped cmd_vel')

        if len(node._pairs) < 8:
            node.get_logger().error(
                f'Only {len(node._pairs)} straight samples — wheels must spin; '
                f'ensure rover-brain is stopped and space is clear'
            )
            return 1

        ratios = [rate / lin for lin, rate, _ang in node._pairs if lin > 0.05]
        angs = [ang for _lin, _rate, ang in node._pairs]
        if len(ratios) < 8:
            node.get_logger().error('Not enough straight samples')
            return 1

        mean = sum(ratios) / len(ratios)
        var = sum((r - mean) ** 2 for r in ratios) / len(ratios)
        std = math.sqrt(var)
        abs_angs = [abs(a) for a in angs]
        mean_abs_ang = sum(abs_angs) / len(abs_angs)
        max_abs_ang = max(abs_angs)
        seen = node._ang_seen
        seen_max = max((abs(a) for a in seen), default=0.0)
        seen_mean = (sum(abs(a) for a in seen) / len(seen)) if seen else 0.0

        out_path = '/tmp/wheel_stall.env'
        with open(out_path, 'w') as f:
            f.write('# Wheel tick stall cal — combined ticks/s per unit cmd magnitude\n')
            f.write(f'ROVER_WHEEL_TICKS_PER_CMD={mean:.3f}\n')
            f.write(f'ROVER_WHEEL_CAL_SAMPLES={len(ratios)}\n')
        node.get_logger().info(
            f'Cal: ticks/cmd={mean:.2f} ±{std:.2f} from {len(ratios)} samples → {out_path}'
        )
        print(f'ROVER_WHEEL_TICKS_PER_CMD={mean:.3f}')
        print(f'ROVER_WHEEL_CAL_SAMPLES={len(ratios)}')
        print(f'ROVER_WHEEL_CAL_STDEV={std:.3f}')
        print(f'ROVER_WHEEL_CAL_LIN={CRUISE_LIN:.3f}')
        print(
            f'STRAIGHT_EVIDENCE accepted_mean_abs_ang={mean_abs_ang:.4f} '
            f'accepted_max_abs_ang={max_abs_ang:.4f} '
            f'cmd_vel_mean_abs_ang={seen_mean:.4f} '
            f'cmd_vel_max_abs_ang={seen_max:.4f}'
        )
        return 0
    finally:
        try:
            node._stop_drive()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
