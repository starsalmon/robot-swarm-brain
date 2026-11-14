#!/usr/bin/env python3
"""Stationary check: does the live lidar scan line up with the existing SLAM map at Rover's current map pose?
Prints one JSON line: {"ok": bool, "match": fraction, ...}. Never drives.

Used by go_house_explore.sh to decide whether yesterday's map can be continued (Rover was not carried far /
odom did not jump) or whether slam_toolbox should start a fresh map.
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

WAIT = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
NEED = float(sys.argv[2]) if len(sys.argv) > 2 else 0.55
LIDAR_YAW = -math.pi / 2


def main():
    rclpy.init()
    n = Node("map_match_check")
    buf = Buffer()
    TransformListener(buf, n)
    got = {"scans": []}
    q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST, depth=1)
    n.create_subscription(OccupancyGrid, "/map", lambda m: got.__setitem__("map", m), q)
    n.create_subscription(LaserScan, "/rover/scan_sync", lambda m: got["scans"].append(m), qos_profile_sensor_data)
    t0 = time.time()
    pose = None
    while time.time() - t0 < WAIT:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform("map", "base_link", rclpy.time.Time(), Duration(seconds=0.0))
            q_ = tf.transform.rotation
            pose = (tf.transform.translation.x, tf.transform.translation.y,
                    math.atan2(2 * (q_.w * q_.z + q_.x * q_.y), 1 - 2 * (q_.y * q_.y + q_.z * q_.z)))
        except Exception:
            pass
        if "map" in got and pose is not None and len(got["scans"]) >= 10 and time.time() - t0 > 4:
            break
    out = {"ok": False, "pose": pose, "n_scans": len(got["scans"]), "have_map": "map" in got}
    if "map" not in got or pose is None or not got["scans"]:
        out["why"] = "missing " + ", ".join(k for k, v in (("map", "map" in got), ("pose(TF map->base_link)", pose is not None), ("scan", bool(got["scans"]))) if not v)
        print(json.dumps(out)); return 2
    m = got["map"]
    h, w, res = m.info.height, m.info.width, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    g = np.array(m.data, np.int8).reshape(h, w)
    occ = g >= 50
    pad = 2
    p = np.pad(occ, pad)
    near = np.zeros_like(occ)
    for dy in range(-pad, pad + 1):
        for dx in range(-pad, pad + 1):
            near |= p[pad + dy:pad + dy + h, pad + dx:pad + dx + w]
    known = g >= 0
    hits = tot = inside_known = 0
    for s in got["scans"][-10:]:
        r = np.asarray(s.ranges, float)
        a = s.angle_min + np.arange(r.size) * s.angle_increment + LIDAR_YAW + pose[2]
        ok = np.isfinite(r) & (r > 0.15) & (r < min(6.0, s.range_max))
        x = pose[0] + r[ok] * np.cos(a[ok]); y = pose[1] + r[ok] * np.sin(a[ok])
        ix = np.floor((x - ox) / res).astype(int); iy = np.floor((y - oy) / res).astype(int)
        inb = (ix >= 0) & (iy >= 0) & (ix < w) & (iy < h)
        ix, iy = ix[inb], iy[inb]
        kn = known[iy, ix]
        inside_known += int(kn.sum())
        hits += int(near[iy, ix][kn].sum())
        tot += int(ok.sum())
    match = hits / max(1, inside_known)
    out.update(match=round(match, 3), endpoints=tot, endpoints_in_known=inside_known,
               map_size_m=[round(w * res, 2), round(h * res, 2)], free=int((g == 0).sum()), occupied=int(occ.sum()))
    out["ok"] = inside_known >= 100 and match >= NEED
    out["why"] = "scan agrees with existing map" if out["ok"] else f"scan/map agreement {match:.2f} < {NEED} (Rover moved since the map, or odom reset)"
    print(json.dumps(out))
    n.destroy_node(); rclpy.shutdown()
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
