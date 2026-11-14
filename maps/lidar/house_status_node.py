#!/usr/bin/env python3
"""Live house-mapping telemetry. Writes live.json; does not drive."""
import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

OUT_DIR = Path(os.environ.get("HOUSE_RUN_DIR", "/maps/lidar/house_run"))
OUT_DIR.mkdir(parents=True, exist_ok=True)
LIVE = OUT_DIR / "live.json"
LOG = OUT_DIR / "status.log"


def now():
    return time.time()


class Status(Node):
    def __init__(self):
        super().__init__("house_map_status")
        self.t = {}
        self.val = {}
        self.map_stats = None
        self.map_seq = None
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        def rec(name, unwrap=None):
            def cb(msg):
                self.t[name] = now()
                self.val[name] = unwrap(msg) if unwrap else msg
            return cb

        self.create_subscription(String, "/rover/behaviour/status", rec("status", lambda m: m.data), 10)
        self.create_subscription(Bool, "/fleet/allow_drive", rec("allow", lambda m: bool(m.data)), 10)
        self.create_subscription(Bool, "/rover/session", rec("session", lambda m: bool(m.data)), 10)
        self.create_subscription(Bool, "/rover/behaviour/explore_active", rec("explore_active", lambda m: bool(m.data)), 10)
        self.create_subscription(Range, "/rover/tof/front", rec("front"), 10)
        self.create_subscription(Range, "/rover/tof/front_left", rec("fl"), 10)
        self.create_subscription(Range, "/rover/tof/front_right", rec("fr"), 10)
        self.create_subscription(Range, "/rover/tof/left", rec("left"), 10)
        self.create_subscription(Range, "/rover/tof/right", rec("right"), 10)
        self.create_subscription(Float32MultiArray, "/rover/tof/l8_cols", rec("l8", lambda m: list(m.data)), 10)
        self.create_subscription(Float32MultiArray, "/rover/tof/lidar", rec("lidar", lambda m: list(m.data)), 10)
        self.create_subscription(Float32, "/rover/battery/voltage", rec("batt", lambda m: float(m.data)), 10)
        self.create_subscription(LaserScan, "/rover/scan", rec("scan"), qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", rec("cmd"), 10)
        self.create_subscription(OccupancyGrid, "/map", rec("map"), map_qos)
        self.create_subscription(Odometry, "/odometry/filtered", rec("odom"), 10)
        self.create_subscription(Bool, "/rover/heartbeat", rec("hb", lambda m: bool(m.data)), 10)
        self.create_subscription(Bool, "/rover/bump", rec("bump", lambda m: bool(m.data)), 10)
        self.create_subscription(Bool, "/rover/stall", rec("stall", lambda m: bool(m.data)), 10)
        self.create_timer(0.4, self.flush)

    def age(self, name):
        t = self.t.get(name)
        return None if t is None else now() - t

    def rng(self, name):
        m = self.val.get(name)
        if m is None:
            return None
        return round(float(m.range), 3)

    def flush(self):
        st = {}
        raw = self.val.get("status")
        if raw:
            try:
                st = json.loads(raw)
            except Exception:
                st = {"raw": raw}

        scan = self.val.get("scan")
        scan_info = None
        if scan is not None:
            valid = [r for r in scan.ranges if scan.range_min < r < scan.range_max]
            scan_info = {
                "n": len(scan.ranges),
                "valid": len(valid),
                "min": None if not valid else round(min(valid), 3),
                "max": None if not valid else round(max(valid), 3),
            }

        mmap = self.val.get("map")
        map_info = self.map_stats
        if mmap is not None:
            sig = (mmap.info.width, mmap.info.height, mmap.info.map_load_time.sec, mmap.info.map_load_time.nanosec, len(mmap.data))
            if sig != self.map_seq:
                occ = free = unk = 0
                for v in mmap.data:
                    if v < 0:
                        unk += 1
                    elif v == 0:
                        free += 1
                    else:
                        occ += 1
                map_info = {
                    "w": mmap.info.width,
                    "h": mmap.info.height,
                    "res": mmap.info.resolution,
                    "origin": [mmap.info.origin.position.x, mmap.info.origin.position.y],
                    "occ": occ,
                    "free": free,
                    "unk": unk,
                    "size_m": [mmap.info.width * mmap.info.resolution, mmap.info.height * mmap.info.resolution],
                }
                self.map_stats = map_info
                self.map_seq = sig

        pose = None
        try:
            # timeout 0 so a missing TF cannot block the executor (and drop /rover/scan)
            tf = self.buf.lookup_transform("map", "base_link", rclpy.time.Time(), Duration(seconds=0.0))
            px = tf.transform.translation.x
            py = tf.transform.translation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w
            yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
            pose = {"x": px, "y": py, "yaw_deg": math.degrees(yaw)}
        except Exception:
            pass

        odom = self.val.get("odom")
        odom_info = None
        if odom is not None:
            p = odom.pose.pose.position
            odom_info = {"x": p.x, "y": p.y, "z": p.z}

        cmd = self.val.get("cmd")
        cmd_info = None
        if cmd is not None:
            cmd_info = {"v": cmd.linear.x, "w": cmd.angular.z}

        l8 = self.val.get("l8")
        l8_min = None if not l8 else min(l8)

        doc = {
            "wall_t": now(),
            "allow": self.val.get("allow"),
            "session": self.val.get("session"),
            "explore_active": self.val.get("explore_active"),
            "state": st.get("state"),
            "go": st.get("go"),
            "why": st.get("why"),
            "beh_v": st.get("v"),
            "beh_w": st.get("w"),
            "tof": {
                "front": self.rng("front"),
                "fl": self.rng("fl"),
                "fr": self.rng("fr"),
                "left": self.rng("left"),
                "right": self.rng("right"),
                "l8_min": None if l8_min is None else round(l8_min, 3),
                "l8": None if l8 is None else [round(x, 3) for x in l8],
            },
            "lidar": self.val.get("lidar"),
            "batt": self.val.get("batt"),
            "scan": scan_info,
            "cmd": cmd_info,
            "map": map_info,
            "pose": pose,
            "odom": odom_info,
            "bump": self.val.get("bump"),
            "stall": self.val.get("stall"),
            "ages": {
                "status": self.age("status"),
                "allow": self.age("allow"),
                "session": self.age("session"),
                "scan": self.age("scan"),
                "front": self.age("front"),
                "cmd": self.age("cmd"),
                "map": self.age("map"),
                "batt": self.age("batt"),
                "hb": self.age("hb"),
                "left": self.age("left"),
            },
        }
        tmp = LIVE.with_suffix(".tmp")
        tmp.write_text(json.dumps(doc))
        tmp.replace(LIVE)
        line = (
            f"{time.strftime('%H:%M:%S')} go={doc.get('go')} ses={doc.get('session')} "
            f"st={doc.get('state')} v={doc.get('beh_v')} w={doc.get('beh_w')} "
            f"why={doc.get('why')} front={doc['tof']['front']} left={doc['tof']['left']} "
            f"scan_age={doc['ages']['scan']} pose={doc.get('pose')} map_free={None if not map_info else map_info['free']}\n"
        )
        with LOG.open("a") as f:
            f.write(line)


def main():
    rclpy.init()
    node = Status()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
