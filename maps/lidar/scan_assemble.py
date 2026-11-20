#!/usr/bin/env python3
"""Rebuild /rover/scan from the Pro S3's best-effort /rover/scan_slice.

micro-ROS cannot send a best-effort message larger than one datagram, and a
full STL-19P LaserScan is ~4 kB, so the Pro S3 sends each revolution as
SLICES LaserScans (lego-rover-esp32 include/scan_slices.h). Contract:

* every slice of a revolution has the same header.stamp (grouping key);
* slice k covers bins [k*B, (k+1)*B), B = len(ranges), and its angle_min is
  FULL_ANGLE_MIN + k*B*angle_increment, which is how k is recovered;
* angle_increment, time_increment, scan_time, range_min/max and frame_id are
  the full revolution's values.

The rebuilt /rover/scan has the same frame, stamp, angles and 500 bins as the
scan the Pro S3 used to publish, so rover_scan_stamp (/rover/scan_sync),
slam_toolbox, rover-behaviour and explore see no change.

Missing slice policy: a revolution with at least MIN_SLICES (default 3 of 4)
is published with NaN in the missing wedge (ranges and intensities). NaN is
what the firmware already writes for an empty bin, and slam_toolbox and the
range consumers already skip it. Fewer slices: the revolution is dropped, so
a mostly-empty scan never reaches the mapper. A revolution is closed when all
slices arrive, when a slice of a newer revolution arrives (slices are sent
back to back, so the missing one is not coming), or after TIMEOUT_S.

Subscriber is BEST_EFFORT (matches the publisher). /rover/scan is published
RELIABLE like the old agent-side publisher, which serves both reliable and
best-effort subscribers.
"""
from __future__ import annotations

import math
import os
import time
from collections import OrderedDict
from dataclasses import dataclass, field

FULL_ANGLE_MIN = float(os.environ.get("SCAN_FULL_ANGLE_MIN", str(-math.pi)))
MIN_SLICES = int(os.environ.get("SCAN_MIN_SLICES", "3"))
TIMEOUT_S = float(os.environ.get("SCAN_ASSEMBLE_TIMEOUT_S", "0.25"))
NAN = float("nan")


@dataclass
class Slice:
    stamp: tuple  # (sec, nanosec)
    frame_id: str
    angle_min: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: list
    intensities: list


@dataclass
class Full:
    stamp: tuple
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: list
    intensities: list
    slices: int
    n_slices: int


@dataclass
class _Pending:
    t0: float
    n_slices: int
    bins: int
    parts: dict = field(default_factory=dict)


def slice_index(s: Slice, full_angle_min: float = FULL_ANGLE_MIN):
    """(k, n_slices) from a slice's geometry, or None when it does not fit the contract."""
    b = len(s.ranges)
    if b == 0 or not (s.angle_increment > 0.0):
        return None
    if s.intensities and len(s.intensities) != b:
        return None
    width = b * s.angle_increment
    n_f = (2.0 * math.pi) / width
    n = int(round(n_f))
    k_f = (s.angle_min - full_angle_min) / width
    k = int(round(k_f))
    if n < 1 or abs(n_f - n) > 0.01 or abs(k_f - k) > 0.01 or not (0 <= k < n):
        return None
    return k, n


class Assembler:
    """Pure grouping logic (no ROS) so it can be unit tested."""

    def __init__(self, min_slices: int = MIN_SLICES, timeout_s: float = TIMEOUT_S,
                 full_angle_min: float = FULL_ANGLE_MIN):
        self.min_slices = min_slices
        self.timeout_s = timeout_s
        self.full_angle_min = full_angle_min
        self.pending: "OrderedDict[tuple, _Pending]" = OrderedDict()
        self.closed: "OrderedDict[tuple, None]" = OrderedDict()
        self.stats = {"slices": 0, "bad": 0, "late": 0, "full": 0, "partial": 0, "dropped": 0}

    def add(self, s: Slice, now: float) -> list:
        """Add one slice; returns finished Full scans (oldest first)."""
        out = []
        self.stats["slices"] += 1
        idx = slice_index(s, self.full_angle_min)
        if idx is None:
            self.stats["bad"] += 1
            return out
        k, n = idx
        key = s.stamp
        if key in self.closed:
            self.stats["late"] += 1
            return out
        p = self.pending.get(key)
        if p is None:
            # A newer revolution started: older ones will not get more slices.
            for old in list(self.pending.keys()):
                r = self._close(old)
                if r is not None:
                    out.append(r)
            p = _Pending(t0=now, n_slices=n, bins=len(s.ranges))
            self.pending[key] = p
        if n != p.n_slices or len(s.ranges) != p.bins:
            self.stats["bad"] += 1
            return out
        p.parts[k] = s
        if len(p.parts) == p.n_slices:
            r = self._close(key)
            if r is not None:
                out.append(r)
        return out

    def expire(self, now: float) -> list:
        out = []
        for key, p in list(self.pending.items()):
            if now - p.t0 >= self.timeout_s:
                r = self._close(key)
                if r is not None:
                    out.append(r)
        return out

    def _close(self, key) -> "Full | None":
        p = self.pending.pop(key)
        self.closed[key] = None
        while len(self.closed) > 32:
            self.closed.popitem(last=False)
        have = len(p.parts)
        if have < min(self.min_slices, p.n_slices):
            self.stats["dropped"] += 1
            return None
        any_s = next(iter(p.parts.values()))
        ranges, intens = [], []
        with_int = all(len(s.intensities) == p.bins for s in p.parts.values())
        for k in range(p.n_slices):
            s = p.parts.get(k)
            if s is None:
                ranges.extend([NAN] * p.bins)
                intens.extend([NAN] * p.bins)
            else:
                ranges.extend(s.ranges)
                intens.extend(s.intensities if with_int else [NAN] * p.bins)
        total = p.bins * p.n_slices
        inc = any_s.angle_increment
        amin = self.full_angle_min
        self.stats["full" if have == p.n_slices else "partial"] += 1
        return Full(
            stamp=key, frame_id=any_s.frame_id, angle_min=amin,
            angle_max=float(amin + (total - 1) * inc), angle_increment=inc,
            time_increment=any_s.time_increment, scan_time=any_s.scan_time,
            range_min=any_s.range_min, range_max=any_s.range_max,
            ranges=ranges, intensities=intens if with_int else [],
            slices=have, n_slices=p.n_slices)


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan

    # Depth covers a 4-slice burst plus a late callback.
    slice_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=32,
                           durability=DurabilityPolicy.VOLATILE)
    scan_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10,
                          durability=DurabilityPolicy.VOLATILE)

    class ScanAssemble(Node):
        def __init__(self):
            super().__init__("rover_scan_assemble")
            self.asm = Assembler()
            self.pub = self.create_publisher(LaserScan, "/rover/scan", scan_qos)
            self.create_subscription(LaserScan, "/rover/scan_slice", self._cb, slice_qos)
            self.create_timer(0.05, self._tick)
            self.create_timer(30.0, self._report)
            self._last_report = dict(self.asm.stats)
            self._t_report = time.monotonic()
            self.get_logger().info(
                f"/rover/scan_slice -> /rover/scan (min {self.asm.min_slices} slices, "
                f"timeout {self.asm.timeout_s:.2f} s)")

        def _cb(self, m: LaserScan) -> None:
            s = Slice(stamp=(m.header.stamp.sec, m.header.stamp.nanosec),
                      frame_id=m.header.frame_id, angle_min=m.angle_min,
                      angle_increment=m.angle_increment, time_increment=m.time_increment,
                      scan_time=m.scan_time, range_min=m.range_min, range_max=m.range_max,
                      ranges=m.ranges, intensities=m.intensities)
            for f in self.asm.add(s, time.monotonic()):
                self._publish(f)

        def _tick(self) -> None:
            for f in self.asm.expire(time.monotonic()):
                self._publish(f)

        def _publish(self, f: Full) -> None:
            out = LaserScan()
            out.header.stamp.sec = f.stamp[0]
            out.header.stamp.nanosec = f.stamp[1]
            out.header.frame_id = f.frame_id
            out.angle_min = f.angle_min
            out.angle_max = f.angle_max
            out.angle_increment = f.angle_increment
            out.time_increment = f.time_increment
            out.scan_time = f.scan_time
            out.range_min = f.range_min
            out.range_max = f.range_max
            out.ranges = f.ranges
            out.intensities = f.intensities
            self.pub.publish(out)

        def _report(self) -> None:
            st = dict(self.asm.stats)
            dt = time.monotonic() - self._t_report
            d = {k: st[k] - self._last_report.get(k, 0) for k in st}
            hz = (d["full"] + d["partial"]) / dt if dt > 0 else 0.0
            self.get_logger().info(
                f"{hz:.1f} Hz out; last {dt:.0f}s: full {d['full']} partial {d['partial']} "
                f"dropped {d['dropped']} late {d['late']} bad {d['bad']} slices {d['slices']}")
            self._last_report = st
            self._t_report = time.monotonic()

    rclpy.init()
    node = ScanAssemble()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
