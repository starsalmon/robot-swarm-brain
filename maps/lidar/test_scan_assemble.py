#!/usr/bin/env python3
"""Unit test for scan_assemble.Assembler. No ROS needed: python3 test_scan_assemble.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scan_assemble import Assembler, Slice, slice_index  # noqa: E402

BINS, N = 125, 4
TOTAL = BINS * N
INC = float(2.0 * math.pi / TOTAL)  # as float64; firmware is float32, tolerated below
import struct
f32 = lambda x: struct.unpack("f", struct.pack("f", x))[0]


def mk(stamp, k, base=0.0):
    amin = f32(-math.pi + k * BINS * INC)
    return Slice(stamp=stamp, frame_id="lidar_link", angle_min=amin, angle_increment=f32(INC),
                 time_increment=f32(0.1 / TOTAL), scan_time=f32(0.1), range_min=0.03, range_max=12.0,
                 ranges=[base + k * BINS + j for j in range(BINS)],
                 intensities=[float(k)] * BINS)


fails = 0


def check(cond, msg):
    global fails
    if not cond:
        print("FAIL", msg)
        fails += 1


for k in range(N):
    check(slice_index(mk((1, 0), k)) == (k, N), f"index {k}")

# Complete revolution, in order.
a = Assembler()
out = []
for k in range(N):
    out += a.add(mk((10, 0), k), 0.0)
check(len(out) == 1 and out[0].slices == 4, "complete -> one scan")
f = out[0]
check(len(f.ranges) == TOTAL and f.ranges[0] == 0 and f.ranges[-1] == TOTAL - 1, "bins in order")
check(abs(f.angle_min + math.pi) < 1e-6, "angle_min -pi")
check(abs(f.angle_max - (-math.pi + (TOTAL - 1) * INC)) < 1e-4, "angle_max")
check(f.frame_id == "lidar_link" and f.stamp == (10, 0), "frame/stamp kept")

# Out of order still complete.
a = Assembler()
out = []
for k in (2, 0, 3, 1):
    out += a.add(mk((11, 0), k), 0.0)
check(len(out) == 1 and out[0].ranges[:3] == [0, 1, 2], "out of order")

# 3 of 4, closed by next revolution: NaN wedge only in slice 1.
a = Assembler()
out = []
for k in (0, 2, 3):
    out += a.add(mk((12, 0), k), 0.0)
check(out == [], "waits for missing slice")
out += a.add(mk((12, 100000000), 0), 0.1)
check(len(out) == 1 and out[0].slices == 3, "3/4 published on next rev")
r = out[0].ranges
check(all(math.isnan(x) for x in r[BINS:2 * BINS]), "wedge NaN")
check(not any(math.isnan(x) for x in r[:BINS] + r[2 * BINS:]), "rest intact")
check(all(math.isnan(x) for x in out[0].intensities[BINS:2 * BINS]), "wedge intensities NaN")

# 2 of 4 dropped (by timeout).
a = Assembler()
out = a.add(mk((13, 0), 0), 0.0) + a.add(mk((13, 0), 1), 0.0)
out += a.expire(0.1)
check(out == [], "not expired yet")
out += a.expire(0.3)
check(out == [] and a.stats["dropped"] == 1, "2/4 dropped")

# 3 of 4 by timeout.
a = Assembler()
for k in (0, 1, 3):
    a.add(mk((14, 0), k), 0.0)
out = a.expire(0.3)
check(len(out) == 1 and out[0].slices == 3, "3/4 by timeout")

# Late slice of a closed revolution is ignored, not a new revolution.
out = a.add(mk((14, 0), 2), 0.31)
check(out == [] and a.stats["late"] == 1 and not a.pending, "late ignored")

# Board reboot: stamp goes backwards, still treated as new revolution.
a = Assembler()
for k in range(N):
    a.add(mk((500, 0), k), 0.0)
out = []
for k in range(N):
    out += a.add(mk((3, 0), k), 1.0)
check(len(out) == 1 and out[0].stamp == (3, 0), "stamp backwards ok")

# Garbage geometry rejected.
bad = mk((15, 0), 0)
bad.angle_min += 0.3 * BINS * INC
check(a.add(bad, 2.0) == [] and a.stats["bad"] == 1, "bad angle rejected")

print("test_scan_assemble", "OK" if not fails else f"{fails} failure(s)")
sys.exit(1 if fails else 0)
