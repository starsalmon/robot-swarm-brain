#!/usr/bin/env python3
"""Write a labeled PNG (and .txt) of the current /map. Usage: render_map_png.py /maps/lidar/NAME"""
import math
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from PIL import Image, ImageDraw

if len(sys.argv) < 2:
    print("usage: render_map_png.py /maps/lidar/NAME")
    raise SystemExit(2)

OUT_BASE = Path(sys.argv[1])
OUT_PNG = OUT_BASE.with_suffix(".png")
OUT_INFO = OUT_BASE.with_suffix(".txt")

PALETTE = {-1: (48, 48, 48), 0: (245, 245, 245)}


class MapShot(Node):
    def __init__(self):
        super().__init__("map_shot")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map = None
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)

    def _on_map(self, msg):
        self.map = msg


def cell_color(v):
    if v < 0:
        return PALETTE[-1]
    if v == 0:
        return PALETTE[0]
    g = int(30 + (100 - v) * 0.4)
    return (g, g, g)


def main():
    rclpy.init()
    node = MapShot()
    end = node.get_clock().now().nanoseconds + 15_000_000_000
    while rclpy.ok() and node.map is None and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.map is None:
        print("NO MAP")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    m = node.map
    w, h = m.info.width, m.info.height
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    data = list(m.data)
    img = Image.new("RGB", (w, h))
    pix = img.load()
    occ = free = unk = 0
    occ_xy = []
    for y in range(h):
        for x in range(w):
            v = data[y * w + x]
            if v < 0:
                unk += 1
            elif v == 0:
                free += 1
            else:
                occ += 1
                if v >= 50:
                    occ_xy.append((x, y))
            pix[x, h - 1 - y] = cell_color(v)

    scale = max(4, int(round(0.05 / res * 6)))
    vis = img.resize((w * scale, h * scale), Image.Resampling.NEAREST)
    draw = ImageDraw.Draw(vis)

    pose = None
    try:
        tf: TransformStamped = node.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        px = tf.transform.translation.x
        py = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        pose = (px, py, yaw)
        cx = (px - ox) / res * scale
        cy = (h - 1 - (py - oy) / res) * scale
        L = 0.4 / res * scale
        nx = cx + L * math.cos(yaw)
        ny = cy - L * math.sin(yaw)
        R = 0.25 / res * scale
        rx = cx + R * math.cos(yaw - math.pi / 2)
        ry = cy - R * math.sin(yaw - math.pi / 2)
        draw.line([(cx, cy), (nx, ny)], fill=(220, 40, 40), width=max(2, scale // 3))
        draw.line([(cx, cy), (rx, ry)], fill=(40, 90, 220), width=max(2, scale // 3))
        r = max(3, scale // 2)
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=(220, 40, 40))
    except Exception as e:
        print("TF lookup failed:", e)

    bar = 1.0 / res * scale
    margin = 8
    yb = vis.height - margin
    draw.line([(margin, yb), (margin + bar, yb)], fill=(255, 220, 0), width=3)
    draw.text((margin, yb - 16), "1 m", fill=(255, 220, 0))
    draw.text((margin, 4), "red=nose  blue=starboard", fill=(200, 200, 200))

    OUT_PNG.parent.mkdir(parents=True, exist_ok=True)
    vis.save(OUT_PNG)
    info = []
    info.append(f"map {w}x{h} cells  resolution={res:.3f} m")
    info.append(f"size {w*res:.2f} x {h*res:.2f} m")
    info.append(f"origin ({ox:.3f}, {oy:.3f})")
    info.append(f"cells occupied={occ} free={free} unknown={unk}")
    if pose:
        info.append(f"base_link in map: x={pose[0]:.3f} y={pose[1]:.3f} yaw_deg={math.degrees(pose[2]):.1f}")
    if occ_xy:
        xs = [p[0] for p in occ_xy]
        ys = [p[1] for p in occ_xy]
        info.append(f"occupied bbox cells x={min(xs)}..{max(xs)} y={min(ys)}..{max(ys)}")
        info.append(
            f"occupied bbox m: x={ox+min(xs)*res:.2f}..{ox+max(xs)*res:.2f} "
            f"y={oy+min(ys)*res:.2f}..{oy+max(ys)*res:.2f}"
        )
    text = "\n".join(info)
    OUT_INFO.write_text(text + "\n")
    print(text)
    print("wrote", OUT_PNG)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
