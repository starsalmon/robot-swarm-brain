#!/usr/bin/env python3
"""Lightweight /cmd_vel + /imu/data logger for Pi Zero (rosbag2 flush hangs)."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

from rover_qos import CMD_VEL_QOS, TELEMETRY_QOS
from rover_twist import wire_to_logical


class TuneLogger(Node):
    def __init__(self, out_dir: Path, duration: float):
        super().__init__('tune_logger')
        self.out_dir = out_dir
        self.deadline = time.monotonic() + duration
        self.duration = duration
        self.cmd_path = out_dir / 'cmd_vel.jsonl'
        self.imu_path = out_dir / 'imu.jsonl'
        self._cmd_f = self.cmd_path.open('w', encoding='utf-8')
        self._imu_f = self.imu_path.open('w', encoding='utf-8')
        self.cmd_n = 0
        self.imu_n = 0
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, CMD_VEL_QOS)
        self.create_subscription(Imu, '/imu/data', self._on_imu, TELEMETRY_QOS)
        self.create_timer(0.2, self._check_done)

    def _write(self, fp, row: dict) -> None:
        fp.write(json.dumps(row, separators=(',', ':')) + '\n')

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd_n += 1
        lin_w = float(msg.linear.x)
        ang_w = float(msg.angular.z)
        lin, ang = wire_to_logical(lin_w, ang_w)
        self._write(
            self._cmd_f,
            {
                't': time.monotonic(),
                'lin': lin_w,
                'ang': ang_w,
                'lin_logical': lin,
                'ang_logical': ang,
            },
        )

    def _on_imu(self, msg: Imu) -> None:
        self.imu_n += 1
        self._write(
            self._imu_f,
            {
                't': time.monotonic(),
                'gz': float(msg.angular_velocity.z),
            },
        )

    def _check_done(self) -> None:
        if time.monotonic() >= self.deadline:
            raise SystemExit(0)

    def close(self) -> None:
        self._cmd_f.close()
        self._imu_f.close()
        meta = {
            'cmd_samples': self.cmd_n,
            'imu_samples': self.imu_n,
            'duration_sec': self.duration,
        }
        (self.out_dir / 'meta.json').write_text(json.dumps(meta, indent=2))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--output', type=Path, required=True)
    ap.add_argument('-d', '--duration', type=float, default=25.0)
    args = ap.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = TuneLogger(args.output, args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

    if node.imu_n < 5:
        print('WARN: very few IMU samples — check MPU / agent', file=sys.stderr)
        return 1
    print(f'Logged cmd={node.cmd_n} imu={node.imu_n} → {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
