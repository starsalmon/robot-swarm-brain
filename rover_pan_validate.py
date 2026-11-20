#!/usr/bin/env python3
"""Validate front sonar pan aim vs range.

Firmware convention (rover_scan_map.front_pan_to_bearing):
  pan 0° = robot RIGHT, 90° = FORWARD, 180° = robot LEFT.

Modes:
  --full-sweep   90° → 180° → 0° → 90° dwell (~4s). Use when hugging left wall.
  --hold         cruise ±22° glance while session on (open hallway only).
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32

from rover_qos import CMD_VEL_QOS
from rover_twist import set_twist


def _bucket(pan_deg: float, center: float) -> str:
    if pan_deg < center - 5.0:
        return 'left'
    if pan_deg > center + 5.0:
        return 'right'
    return 'center'


def _median_near(bins: dict[int, list[float]], target: float, tol: float = 15.0) -> float:
    vals: list[float] = []
    for deg, samples in bins.items():
        if abs(deg - target) <= tol:
            vals.extend(samples)
    return statistics.median(vals) if vals else float('nan')


class PanValidate(Node):
    def __init__(
        self,
        *,
        wall: str,
        center: float,
        creep_lin: float,
        duration_s: float,
        mode: str,
    ) -> None:
        super().__init__('rover_pan_validate')
        self._wall = wall
        self._center = center
        self._creep_lin = creep_lin
        self._duration_s = duration_s
        self._mode = mode
        self._t0 = time.monotonic()
        self._samples: dict[str, list[float]] = defaultdict(list)
        self._pan_bins: dict[int, list[float]] = defaultdict(list)
        self._pan_deg = center
        self._cal_sent = False
        self._pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self._session_pub = self.create_publisher(Bool, '/rover/session', 10)
        self._cal_pub = self.create_publisher(Bool, '/rover/sonar/cal_sweep', 10)
        self.create_subscription(Range, '/rover/sonar/range', self._on_range, qos_profile_sensor_data)
        self.create_subscription(Float32, '/rover/sonar/pan_deg', self._on_pan, 10)
        self.create_timer(0.2, self._publish_session)
        if mode == 'creep':
            self.create_timer(0.05, self._publish_drive)
        elif mode == 'full-sweep':
            self.create_timer(0.1, self._maybe_trigger_cal)
        elif mode == 'hold':
            self.create_timer(0.05, self._publish_stop_only)
        self.create_timer(2.0, self._status)
        self.get_logger().info(
            f'pan validate wall={wall} mode={mode} duration={duration_s}s'
        )

    def _on_pan(self, msg: Float32) -> None:
        self._pan_deg = float(msg.data)

    def _on_range(self, msg: Range) -> None:
        rng = float(msg.range)
        if rng <= 0.02 or rng > 3.5:
            return
        bin_deg = int(round(self._pan_deg / 5.0) * 5)
        self._pan_bins[bin_deg].append(rng)
        bucket = _bucket(self._pan_deg, self._center)
        if bucket != 'center':
            self._samples[bucket].append(rng)

    def _publish_session(self) -> None:
        if time.monotonic() - self._t0 >= self._duration_s:
            off = Bool()
            off.data = False
            self._session_pub.publish(off)
            cal_off = Bool()
            cal_off.data = False
            self._cal_pub.publish(cal_off)
            return
        on = Bool()
        on.data = True
        self._session_pub.publish(on)

    def _maybe_trigger_cal(self) -> None:
        # Firmware runs full sweep automatically on session start (Go).
        pass

    def _publish_drive(self) -> None:
        if time.monotonic() - self._t0 >= self._duration_s:
            self._publish_stop()
            return
        msg = Twist()
        set_twist(msg, self._creep_lin, 0.0)
        self._pub.publish(msg)

    def _publish_stop_only(self) -> None:
        self._publish_stop()

    def _publish_stop(self) -> None:
        msg = Twist()
        set_twist(msg, 0.0, 0.0)
        self._pub.publish(msg)

    def _status(self) -> None:
        self.get_logger().info(f'collecting… pan={self._pan_deg:.0f}° bins={len(self._pan_bins)}')

    def done(self) -> bool:
        return time.monotonic() - self._t0 >= self._duration_s

    def _report_full_sweep(self) -> int:
        print()
        print('=== Sonar FULL pan sweep validation ===')
        print(f'  wall on robot: {self._wall.upper()}')
        if self._pan_bins:
            print('  pan histogram (5° bins, median range):')
            for b in sorted(self._pan_bins):
                vals = self._pan_bins[b]
                if len(vals) >= 2:
                    print(f'    {b:3d}°  {statistics.median(vals):.2f}m  n={len(vals)}')
        left_m = _median_near(self._pan_bins, 180.0)
        right_m = _median_near(self._pan_bins, 0.0)
        center_m = _median_near(self._pan_bins, 90.0)
        print(f'  median @ ~180° (LEFT):   {left_m:.2f} m')
        print(f'  median @ ~90° (FWD):     {center_m:.2f} m')
        print(f'  median @ ~0° (RIGHT):    {right_m:.2f} m')
        if any(map(lambda x: x != x, (left_m, right_m))):  # NaN check
            print('  FAIL — missing samples at 0° or 180° (needs new firmware cal_sweep topic)')
            return 1
        margin = 0.15
        if self._wall == 'left':
            ok = left_m + margin < right_m
            expect = f'LEFT 180° ({left_m:.2f}m) < RIGHT 0° ({right_m:.2f}m) − {margin}m'
        else:
            ok = right_m + margin < left_m
            expect = f'RIGHT 0° ({right_m:.2f}m) < LEFT 180° ({left_m:.2f}m) − {margin}m'
        if ok:
            print(f'  PASS — {expect}')
            return 0
        print(f'  FAIL — expected {expect}')
        if self._wall == 'left' and left_m >= right_m:
            print('  → Pan likely INVERTED. Try SONAR_PAN_INVERT=1 in platformio.ini.')
        return 1

    def report(self) -> int:
        self._publish_stop()
        off = Bool()
        off.data = False
        self._session_pub.publish(off)
        self._cal_pub.publish(off)
        if self._mode == 'full-sweep':
            return self._report_full_sweep()
        left = self._samples.get('left', [])
        right = self._samples.get('right', [])
        print()
        print('=== Sonar glance pan validation ===')
        print(f'  wall on robot: {self._wall.upper()}')
        print(f'  samples: left={len(left)}  right={len(right)}')
        if self._pan_bins:
            print('  pan histogram (5° bins, median range):')
            for b in sorted(self._pan_bins):
                vals = self._pan_bins[b]
                if len(vals) >= 3:
                    print(f'    {b:3d}°  {statistics.median(vals):.2f}m  n={len(vals)}')
        if len(left) < 5 or len(right) < 5:
            print('  FAIL — not enough glance samples (use --full-sweep near a wall).')
            return 1
        left_med = statistics.median(left)
        right_med = statistics.median(right)
        print(f'  median range  LEFT pan:  {left_med:.2f} m')
        print(f'  median range  RIGHT pan: {right_med:.2f} m')
        margin = 0.12
        if self._wall == 'left':
            ok = left_med + margin < right_med
            expect = f'left ({left_med:.2f}) < right ({right_med:.2f}) − {margin:.2f} m'
        else:
            ok = right_med + margin < left_med
            expect = f'right ({right_med:.2f}) < left ({left_med:.2f}) − {margin:.2f} m'
        if ok:
            print(f'  PASS — {expect}')
            return 0
        print(f'  FAIL — expected {expect}')
        return 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--wall', choices=('left', 'right'), default='left')
    ap.add_argument('--center', type=float, default=90.0)
    ap.add_argument('--duration', type=float, default=None)
    ap.add_argument('--full-sweep', action='store_true', help='90→180→0→90 cal sweep (default)')
    ap.add_argument('--hold', action='store_true', help='cruise ±22° glance only')
    ap.add_argument('--creep', action='store_true', help='slow forward + glance')
    ap.add_argument('--creep-lin', type=float, default=0.12)
    args = ap.parse_args()
    modes = [args.full_sweep, args.hold, args.creep]
    if sum(modes) > 1:
        ap.error('pick one mode')
    if args.full_sweep or not (args.hold or args.creep):
        mode = 'full-sweep'
        duration = args.duration if args.duration is not None else 8.0
    elif args.hold:
        mode = 'hold'
        duration = args.duration if args.duration is not None else 45.0
    else:
        mode = 'creep'
        duration = args.duration if args.duration is not None else 45.0

    rclpy.init()
    node = PanValidate(
        wall=args.wall,
        center=args.center,
        creep_lin=args.creep_lin,
        duration_s=duration,
        mode=mode,
    )
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.3)
        return node.report()
    except KeyboardInterrupt:
        return node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
