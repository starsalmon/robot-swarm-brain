#!/usr/bin/env python3
"""Monitor wheel encoder ticks from ESP (/rover/wheel/*_ticks)."""
from __future__ import annotations

import argparse
import os
import sys
import time

DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, DIR)


def main() -> int:
    ap = argparse.ArgumentParser(description='Wheel sensor tick monitor')
    ap.add_argument('--secs', type=float, default=30.0, help='Run duration (0 = forever)')
    ap.add_argument(
        '--strips',
        type=int,
        default=int(os.environ.get('ROVER_WHEEL_STRIPS', '6')),
        help='Foil strips per wheel revolution (for rev estimate)',
    )
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import UInt32
    from rover_qos import TELEMETRY_QOS

    class _Mon(Node):
        def __init__(self) -> None:
            super().__init__('wheel_tick_monitor')
            self.left = 0
            self.right = 0
            self._t0 = time.monotonic()
            self.create_subscription(UInt32, '/rover/wheel/left_ticks', self._on_l, TELEMETRY_QOS)
            self.create_subscription(UInt32, '/rover/wheel/right_ticks', self._on_r, TELEMETRY_QOS)

        def _on_l(self, msg: UInt32) -> None:
            self.left = int(msg.data)

        def _on_r(self, msg: UInt32) -> None:
            self.right = int(msg.data)

    rclpy.init()
    node = _Mon()
    strips = max(1, args.strips)
    print(
        f'Watching /rover/wheel/left_ticks and /rover/wheel/right_ticks '
        f'({strips} strips/rev). Roll the wheels or drive slowly.',
        flush=True,
    )
    print('L_ticks  R_ticks  L_rev  R_rev  L_rate  R_rate', flush=True)
    last_l = last_r = 0
    last_ts = time.monotonic()
    end = time.monotonic() + args.secs if args.secs > 0 else None

    try:
        while rclpy.ok() and (end is None or time.monotonic() < end):
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_ts < 0.5:
                continue
            dt = now - last_ts
            dl = node.left - last_l
            dr = node.right - last_r
            l_rate = dl / dt
            r_rate = dr / dt
            print(
                f'{node.left:7d}  {node.right:7d}  '
                f'{node.left / strips:5.2f}  {node.right / strips:5.2f}  '
                f'{l_rate:6.1f}  {r_rate:6.1f}',
                flush=True,
            )
            last_l, last_r, last_ts = node.left, node.right, now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(
        f'\nFinal: L={node.left} R={node.right} ticks '
        f'({node.left / strips:.2f} / {node.right / strips:.2f} rev)',
        flush=True,
    )
    print('No odometry in drive stack yet — this is raw tick telemetry only.', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
