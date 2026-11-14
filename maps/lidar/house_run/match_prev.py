#!/usr/bin/env python3
"""Stationary: where does the live lidar scan fit in a saved map pgm? Prints JSON. Never drives."""
import json, math, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
base = sys.argv[1] if len(sys.argv) > 1 else "/maps/lidar/house_prev_20261113_0726"
LY = -math.pi / 2
def pgm(p):
    b = open(p, "rb").read(); parts = []; i = 0
    while len(parts) < 4:
        while b[i:i+1].isspace(): i += 1
        if b[i:i+1] == b"#":
            while b[i:i+1] != b"\n": i += 1
            continue
        j = i
        while not b[j:j+1].isspace(): j += 1
        parts.append(b[i:j]); i = j
    w, h = int(parts[1]), int(parts[2]); i += 1
    return np.frombuffer(b[i:i+w*h], np.uint8).reshape(h, w)
y = {}
for ln in open(base + ".yaml"):
    if ":" in ln: k, v = ln.split(":", 1); y[k.strip()] = v.strip()
res = float(y["resolution"]); ox, oy = [float(v) for v in y["origin"].strip("[]").split(",")[:2]]
img = pgm(base + ".pgm"); H, W = img.shape
occ = np.flipud(img < 50)            # row 0 = y min
free = np.flipud(img > 250)
# dilate occupied by 1 cell for tolerance
d = occ.copy()
for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,-1),(1,-1),(-1,1)):
    d |= np.roll(np.roll(occ, dy, 0), dx, 1)
rclpy.init(); n = Node("grok_match_prev"); scans = []
n.create_subscription(LaserScan, "/rover/scan_sync", lambda m: scans.append(m), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 4 and len(scans) < 5: rclpy.spin_once(n, timeout_sec=0.1)
if not scans: print(json.dumps({"ok": False, "err": "no scan"})); sys.exit(0)
pts = []
for m in scans[-3:]:
    for k, r in enumerate(m.ranges):
        if math.isfinite(r) and 0.12 < r < 8.0:
            a = m.angle_min + k * m.angle_increment + LY
            pts.append((r * math.cos(a), r * math.sin(a)))
P = np.array(pts)[::max(1, len(pts) // 240)]
fy, fx = np.nonzero(free)
sel = (fy % 2 == 0) & (fx % 2 == 0)
cx = ox + (fx[sel] + 0.5) * res; cy = oy + (fy[sel] + 0.5) * res
def score(xs, ys, th):
    c, s = math.cos(th), math.sin(th)
    px = P[:, 0] * c - P[:, 1] * s; py = P[:, 0] * s + P[:, 1] * c
    gx = ((xs[:, None] + px[None, :] - ox) / res).astype(int); gy = ((ys[:, None] + py[None, :] - oy) / res).astype(int)
    ok = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
    hit = np.zeros(gx.shape, bool); hit[ok] = d[gy[ok], gx[ok]]
    return hit.mean(1)
best = []
for th in np.radians(np.arange(0, 360, 3)):
    sc = score(cx, cy, th); k = np.argsort(sc)[-5:]
    for i in k: best.append((float(sc[i]), float(cx[i]), float(cy[i]), float(th)))
best.sort(reverse=True)
# refine top 10
ref = []
for s0, x0, y0, t0_ in best[:10]:
    xs, ys = np.meshgrid(np.arange(-0.1, 0.101, 0.025), np.arange(-0.1, 0.101, 0.025))
    xs = xs.ravel() + x0; ys = ys.ravel() + y0
    for dt in np.radians(np.arange(-3, 3.01, 0.5)):
        sc = score(xs, ys, t0_ + dt); i = int(np.argmax(sc)); ref.append((float(sc[i]), float(xs[i]), float(ys[i]), float(t0_ + dt)))
ref.sort(reverse=True)
b = ref[0]
alt = next((r for r in ref + best if math.hypot(r[1] - b[1], r[2] - b[2]) > 0.6 or abs(math.remainder(r[3] - b[3], 2*math.pi)) > 0.35), None)
out = dict(map=base, npts=len(P), best=dict(score=round(b[0], 3), x=round(b[1], 3), y=round(b[2], 3), th_deg=round(math.degrees(b[3]), 1)),
           alt=None if alt is None else dict(score=round(alt[0], 3), x=round(alt[1], 2), y=round(alt[2], 2), th_deg=round(math.degrees(alt[3]), 1)),
           free_cells=int(free.sum()), occ_cells=int(occ.sum()))
out["ok"] = b[0] >= 0.6 and (alt is None or alt[0] <= b[0] - 0.12)
print(json.dumps(out))
