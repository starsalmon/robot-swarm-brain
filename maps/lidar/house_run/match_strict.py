#!/usr/bin/env python3
"""Stationary strict global relocaliser: endpoint hits + free-space (ray) consistency. Never drives. Prints JSON."""
import json, math, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
base = sys.argv[1]
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
occ = np.flipud(img < 50); free = np.flipud(img > 250)
d = occ.copy()
for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,-1),(1,-1),(-1,1)):
    d |= np.roll(np.roll(occ, dy, 0), dx, 1)
rclpy.init(); n = Node("grok_match_strict"); scans = []
n.create_subscription(LaserScan, "/rover/scan_sync", lambda m: scans.append(m), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 8 and len(scans) < 12: rclpy.spin_once(n, timeout_sec=0.1)
if not scans: print(json.dumps({"ok": False, "err": "no scan"})); sys.exit(0)
ang = []; rng = []
for m in scans[-10:]:
    for k, r in enumerate(m.ranges):
        if math.isfinite(r) and 0.15 < r < 8.0:
            ang.append(m.angle_min + k * m.angle_increment + LY); rng.append(r)
ang = np.array(ang); rng = np.array(rng)
# bin by 1 deg, median range -> robust single sweep
bins = np.round(np.degrees(ang) % 360).astype(int) % 360
A = []; Rr = []
for b in np.unique(bins):
    A.append(math.radians(b)); Rr.append(float(np.median(rng[bins == b])))
A = np.array(A); Rr = np.array(Rr)
P = np.stack([Rr * np.cos(A), Rr * np.sin(A)], 1)
# ray samples (free space): every 0.1 m up to r-0.2
RS = []
for a, r in zip(A, Rr):
    for t in np.arange(0.2, r - 0.2, 0.1): RS.append((t * math.cos(a), t * math.sin(a)))
RS = np.array(RS) if RS else np.zeros((0, 2))
def lookup(grid, xs, ys, th, Q):
    c, s = math.cos(th), math.sin(th)
    px = Q[:, 0] * c - Q[:, 1] * s; py = Q[:, 0] * s + Q[:, 1] * c
    gx = ((xs[:, None] + px[None, :] - ox) / res).astype(int); gy = ((ys[:, None] + py[None, :] - oy) / res).astype(int)
    ok = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
    out = np.zeros(gx.shape, bool); out[ok] = grid[gy[ok], gx[ok]]
    return out
def score(xs, ys, th):
    hit = lookup(d, xs, ys, th, P).mean(1)
    viol = lookup(occ, xs, ys, th, RS).mean(1) if len(RS) else 0
    return hit - 3.0 * viol, hit, viol
fy, fx = np.nonzero(free); sel = (fy % 2 == 0) & (fx % 2 == 0)
cx = ox + (fx[sel] + 0.5) * res; cy = oy + (fy[sel] + 0.5) * res
cand = []
for th in np.radians(np.arange(0, 360, 2)):
    sc, _, _ = score(cx, cy, th); k = np.argsort(sc)[-6:]
    for i in k: cand.append((float(sc[i]), float(cx[i]), float(cy[i]), float(th)))
cand.sort(reverse=True)
ref = []
for s0, x0, y0, t_ in cand[:30]:
    xs, ys = np.meshgrid(np.arange(-0.12, 0.121, 0.02), np.arange(-0.12, 0.121, 0.02))
    xs = xs.ravel() + x0; ys = ys.ravel() + y0
    for dt in np.radians(np.arange(-3, 3.01, 0.5)):
        sc, hit, viol = score(xs, ys, t_ + dt); i = int(np.argmax(sc))
        ref.append((float(sc[i]), float(xs[i]), float(ys[i]), float(t_ + dt), float(hit[i]), float(viol[i] if len(RS) else 0)))
ref.sort(reverse=True)
dist = []
for r in ref:
    if all(math.hypot(r[1]-q[1], r[2]-q[2]) > 0.5 or abs(math.remainder(r[3]-q[3], 2*math.pi)) > 0.35 for q in dist): dist.append(r)
    if len(dist) >= 4: break
fmt = lambda r: dict(score=round(r[0],3), hit=round(r[4],3), viol=round(r[5],4), x=round(r[1],3), y=round(r[2],3), th_deg=round(math.degrees(r[3]) % 360,1))
b = dist[0]; a = dist[1] if len(dist) > 1 else None
ok = b[4] >= 0.75 and (a is None or a[0] <= b[0] - 0.10)
print(json.dumps(dict(nbeams=len(P), nray=len(RS), nscans=len(scans), top=[fmt(r) for r in dist], ok=ok)))
