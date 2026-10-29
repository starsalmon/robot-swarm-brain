#!/usr/bin/env python3
"""Static sensor health sample for Rover (Phase 1 explore overhaul).

Samples live topics for ~30–60s and prints rates / NaN% / basic stats.
Does NOT publish cmd_vel, does NOT require Go / session, does NOT flash or
change explore. Safe to run while rover-brain is idle or exploring.

Usage (on brain, inside a ROS container with /opt/fleet mounted):
  source /opt/ros/jazzy/setup.bash
  python3 /opt/fleet/cal/sensor_verify.py            # default 45s
  python3 /opt/fleet/cal/sensor_verify.py --seconds 60

Env:
  SENSOR_VERIFY_SEC  override default sample window (seconds)
"""
from __future__ import annotations

import argparse
import math
import os
import statistics
import time
from collections import defaultdict
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32MultiArray, UInt32

DEFAULT_SEC = float(os.environ.get("SENSOR_VERIFY_SEC", "45"))

SENSOR_QOS = qos_profile_sensor_data
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    durability=DurabilityPolicy.VOLATILE,
)

# Floor checklist distances (mm) for Cain's tape-measure wall setup.
# Side VL53L0X-class: max_range ~1.3 m. Front L8: max_range ~4.0 m.
FLOOR_DISTANCES_MM = (200, 500, 1000)
HOLD_STILL_SEC = 20
IMU_HOLD_STILL_SEC = 15
IMU_TURN_HOLD_SEC = 10


def _is_nan(x: float) -> bool:
    return isinstance(x, float) and math.isnan(x)


def _pct(n: int, d: int) -> float:
    return 100.0 * n / d if d else 0.0


def _mean(xs: list[float]) -> float | None:
    return statistics.mean(xs) if xs else None


def _stdev(xs: list[float]) -> float | None:
    return statistics.stdev(xs) if len(xs) >= 2 else (0.0 if xs else None)


def _fmt(x: float | None, digits: int = 3) -> str:
    if x is None:
        return "n/a"
    if _is_nan(x):
        return "nan"
    return f"{x:.{digits}f}"


class SensorVerify(Node):
    def __init__(self) -> None:
        super().__init__("sensor_verify")
        self.t0 = time.monotonic()
        # topic -> list of (t_mono, payload)
        self.range_vals: dict[str, list[float]] = defaultdict(list)
        self.l8_rows: list[list[float]] = []
        self.imu_gyro_z: list[float] = []
        self.imu_acc: list[tuple[float, float, float]] = []
        self.tick_l: list[int] = []
        self.tick_r: list[int] = []
        self.bump: list[bool] = []
        self.stall: list[bool] = []
        self.counts: dict[str, int] = defaultdict(int)
        self._seen_stamp: dict[str, set[tuple[int, int]]] = defaultdict(set)

        range_topics = [
            "/rover/tof/left",
            "/rover/tof/right",
            "/rover/tof/front",
            "/rover/tof/front_left",
            "/rover/tof/front_right",
        ]
        for topic in range_topics:
            self.create_subscription(
                Range, topic, lambda m, t=topic: self._on_range(t, m), SENSOR_QOS
            )

        self.create_subscription(
            Float32MultiArray, "/rover/tof/l8_cols", self._on_l8, SENSOR_QOS
        )
        self.create_subscription(
            UInt32, "/rover/wheel/left_ticks", self._on_tick_l, SENSOR_QOS
        )
        self.create_subscription(
            UInt32, "/rover/wheel/right_ticks", self._on_tick_r, SENSOR_QOS
        )

        # IMU / bump / stall: subscribe both QoS profiles; dedupe by header stamp
        # (Bool has no stamp — dedupe by (count, value, time-bucket) is weak, so
        # just accept both and report; dual QoS rarely double-fires for Bool).
        for qos in (SENSOR_QOS, RELIABLE_QOS):
            self.create_subscription(Imu, "/imu/data", self._on_imu, qos)
            self.create_subscription(Bool, "/rover/bump", self._on_bump, qos)
            self.create_subscription(Bool, "/rover/stall", self._on_stall, qos)

    def _dedupe_header(self, key: str, stamp) -> bool:
        """Return True if this stamp is new (should process)."""
        sig = (int(stamp.sec), int(stamp.nanosec))
        if sig in self._seen_stamp[key]:
            return False
        # Bound memory
        bucket = self._seen_stamp[key]
        if len(bucket) > 5000:
            bucket.clear()
        bucket.add(sig)
        return True

    def _on_range(self, topic: str, msg: Range) -> None:
        if not self._dedupe_header(topic, msg.header.stamp):
            return
        self.counts[topic] += 1
        self.range_vals[topic].append(float(msg.range))

    def _on_l8(self, msg: Float32MultiArray) -> None:
        self.counts["/rover/tof/l8_cols"] += 1
        self.l8_rows.append([float(x) for x in msg.data])

    def _on_imu(self, msg: Imu) -> None:
        if not self._dedupe_header("/imu/data", msg.header.stamp):
            return
        self.counts["/imu/data"] += 1
        self.imu_gyro_z.append(float(msg.angular_velocity.z))
        self.imu_acc.append(
            (
                float(msg.linear_acceleration.x),
                float(msg.linear_acceleration.y),
                float(msg.linear_acceleration.z),
            )
        )

    def _on_tick_l(self, msg: UInt32) -> None:
        self.counts["/rover/wheel/left_ticks"] += 1
        self.tick_l.append(int(msg.data))

    def _on_tick_r(self, msg: UInt32) -> None:
        self.counts["/rover/wheel/right_ticks"] += 1
        self.tick_r.append(int(msg.data))

    def _on_bump(self, msg: Bool) -> None:
        self.counts["/rover/bump"] += 1
        self.bump.append(bool(msg.data))

    def _on_stall(self, msg: Bool) -> None:
        self.counts["/rover/stall"] += 1
        self.stall.append(bool(msg.data))


def _report_range(topic: str, vals: list[float], elapsed: float) -> dict[str, Any]:
    n = len(vals)
    nan_n = sum(1 for v in vals if _is_nan(v))
    valid = [v for v in vals if not _is_nan(v)]
    hz = n / elapsed if elapsed > 0 else 0.0
    status = "OK"
    if n == 0:
        status = "DEAD"
    elif nan_n == n:
        status = "ALL_NAN"
    elif _pct(nan_n, n) > 50:
        status = "WARN_NAN"
    elif hz < 1.0:
        status = "WARN_RATE"
    return {
        "topic": topic,
        "n": n,
        "hz": hz,
        "nan_pct": _pct(nan_n, n),
        "min": min(valid) if valid else None,
        "median": statistics.median(valid) if valid else None,
        "max": max(valid) if valid else None,
        "status": status,
    }


def _report_l8(rows: list[list[float]], elapsed: float) -> dict[str, Any]:
    n = len(rows)
    hz = n / elapsed if elapsed > 0 else 0.0
    # Expect 8 columns; pad/truncate safely
    cols = 8
    col_vals: list[list[float]] = [[] for _ in range(cols)]
    nan_cells = 0
    total_cells = 0
    for row in rows:
        for i in range(cols):
            total_cells += 1
            if i >= len(row) or _is_nan(row[i]):
                nan_cells += 1
            else:
                col_vals[i].append(row[i])
    col_med = [_mean(c) for c in col_vals]
    status = "OK"
    if n == 0:
        status = "DEAD"
    elif total_cells and _pct(nan_cells, total_cells) > 80:
        status = "WARN_NAN"
    elif hz < 1.0:
        status = "WARN_RATE"
    return {
        "topic": "/rover/tof/l8_cols",
        "n": n,
        "hz": hz,
        "nan_pct": _pct(nan_cells, total_cells),
        "col_median_m": col_med,
        "status": status,
    }


def print_report(node: SensorVerify, elapsed: float) -> None:
    print()
    print("=" * 72)
    print(f"SENSOR VERIFY  window={elapsed:.1f}s  (static; no cmd_vel / no Go)")
    print("=" * 72)

    rows: list[dict[str, Any]] = []
    for topic in [
        "/rover/tof/left",
        "/rover/tof/right",
        "/rover/tof/front",
        "/rover/tof/front_left",
        "/rover/tof/front_right",
    ]:
        rows.append(_report_range(topic, node.range_vals.get(topic, []), elapsed))

    l8 = _report_l8(node.l8_rows, elapsed)
    print(
        f"{'topic':<28} {'n':>6} {'Hz':>7} {'NaN%':>7}  "
        f"{'min':>8} {'med':>8} {'max':>8}  status"
    )
    print("-" * 72)
    for r in rows:
        print(
            f"{r['topic']:<28} {r['n']:6d} {r['hz']:7.2f} {r['nan_pct']:6.1f}%  "
            f"{_fmt(r['min'], 3):>8} {_fmt(r['median'], 3):>8} {_fmt(r['max'], 3):>8}  "
            f"{r['status']}"
        )

    col_s = " ".join(
        _fmt(v, 2) if v is not None else "nan" for v in l8["col_median_m"]
    )
    print(
        f"{l8['topic']:<28} {l8['n']:6d} {l8['hz']:7.2f} {l8['nan_pct']:6.1f}%  "
        f"cols_med_m=[{col_s}]  {l8['status']}"
    )

    # IMU
    n_imu = node.counts["/imu/data"]
    imu_hz = n_imu / elapsed if elapsed else 0.0
    if n_imu == 0:
        print(
            f"{'/imu/data':<28} {0:6d} {0.0:7.2f} {'n/a':>7}  "
            f"DEAD (publisher discovered but no samples)"
        )
        imu_status = "DEAD"
        gz_mean = gz_std = mag_mean = None
    else:
        gz_mean = _mean(node.imu_gyro_z)
        gz_std = _stdev(node.imu_gyro_z)
        mags = [math.sqrt(x * x + y * y + z * z) for x, y, z in node.imu_acc]
        mag_mean = _mean(mags)
        imu_status = "OK"
        if gz_mean is not None and abs(gz_mean) > 0.05:
            imu_status = "WARN_BIAS"
        print(
            f"{'/imu/data':<28} {n_imu:6d} {imu_hz:7.2f} {'n/a':>7}  "
            f"gz_mean={_fmt(gz_mean, 4)} rad/s  gz_std={_fmt(gz_std, 4)}  "
            f"|a|={_fmt(mag_mean, 2)}  {imu_status}"
        )

    # Wheel ticks
    for name, series in (
        ("/rover/wheel/left_ticks", node.tick_l),
        ("/rover/wheel/right_ticks", node.tick_r),
    ):
        n = len(series)
        hz = n / elapsed if elapsed else 0.0
        if n == 0:
            print(f"{name:<28} {0:6d} {0.0:7.2f} {'n/a':>7}  DEAD")
        else:
            delta = series[-1] - series[0]
            print(
                f"{name:<28} {n:6d} {hz:7.2f} {'n/a':>7}  "
                f"start={series[0]} end={series[-1]} delta={delta}  "
                f"{'OK' if hz >= 1.0 else 'WARN_RATE'}"
            )

    # Bump / stall
    for name, series in (("/rover/bump", node.bump), ("/rover/stall", node.stall)):
        n = len(series)
        hz = n / elapsed if elapsed else 0.0
        if n == 0:
            print(
                f"{name:<28} {0:6d} {0.0:7.2f} {'n/a':>7}  "
                f"DEAD (no samples — not just idle-false)"
            )
        else:
            true_n = sum(1 for v in series if v)
            print(
                f"{name:<28} {n:6d} {hz:7.2f} {'n/a':>7}  "
                f"true={true_n}/{n} ({_pct(true_n, n):.1f}%) last={series[-1]}  OK"
            )

    print("-" * 72)
    dead = [r["topic"] for r in rows if r["status"] == "DEAD"]
    if l8["status"] == "DEAD":
        dead.append(l8["topic"])
    if imu_status == "DEAD":
        dead.append("/imu/data")
    if not node.tick_l:
        dead.append("/rover/wheel/left_ticks")
    if not node.tick_r:
        dead.append("/rover/wheel/right_ticks")
    if not node.bump:
        dead.append("/rover/bump")
    if not node.stall:
        dead.append("/rover/stall")
    if dead:
        print(f"DEAD topics: {', '.join(dead)}")
    else:
        print("All watched topics produced samples.")
    print()
    print_floor_checklist()


def print_floor_checklist() -> None:
    dists = "/".join(str(d) for d in FLOOR_DISTANCES_MM)
    print("=" * 72)
    print("FLOOR CHECKLIST (Cain — tape measure + wall; no Go needed)")
    print("=" * 72)
    print(
        f"1. Clear a flat floor against a plain vertical wall. Mark tape at "
        f"{dists} mm from the wall face."
    )
    print(
        "2. Power Rover, wait until ToF topics are live (this script OK on "
        "L/R/front*). Do NOT press Go — keep session idle / motors stopped."
    )
    print(
        "3. LEFT wall pass: point Rover so LEFT ToF faces the wall, body "
        "square. Place front bumper / chassis reference on each mark "
        f"({dists} mm). Hold still {HOLD_STILL_SEC}s per mark while "
        "sensor_verify (or bag) runs. Record median /rover/tof/left."
    )
    print(
        "4. RIGHT wall pass: same for RIGHT ToF on the same marks. Record "
        "/rover/tof/right."
    )
    print(
        "5. FRONT pass: square the FRONT / L8 toward the wall on the same "
        f"marks ({dists} mm). Hold still {HOLD_STILL_SEC}s each. Record "
        "/rover/tof/front, front_left, front_right, and l8_cols medians."
    )
    print(
        f"6. IMU still: park level, no hand contact {IMU_HOLD_STILL_SEC}s — "
        "note gyro-z bias (want |gz| < ~0.05 rad/s)."
    )
    print(
        f"7. IMU 90° hand-turn: after the still sample, carefully yaw Rover "
        f"~90° by hand on the spot, then hold still {IMU_TURN_HOLD_SEC}s. "
        "Confirms gyro sign / integration later — do this ONLY after the "
        "static still sample, and still without Go / cmd_vel."
    )
    print(
        "8. Optional bump: gentle press bumper once during a sample window "
        "to confirm /rover/bump toggles (if topic is live)."
    )
    print(
        "Distances rationale: 200 mm near-field, 500 mm mid, 1000 mm far "
        "(within side ~1.3 m max; front L8 max ~4 m)."
    )
    print("=" * 72)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--seconds",
        type=float,
        default=DEFAULT_SEC,
        help=f"sample window seconds (default {DEFAULT_SEC:g})",
    )
    args = ap.parse_args()
    sec = max(5.0, float(args.seconds))

    print(f"sensor_verify: sampling {sec:g}s (static, no drive)…")
    rclpy.init()
    node = SensorVerify()
    t_end = time.monotonic() + sec
    try:
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("\n(interrupted)")
    elapsed = time.monotonic() - node.t0
    try:
        print_report(node, elapsed)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
