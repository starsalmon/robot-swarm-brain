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

# endpoints landing on known-free cells far from walls = strong evidence against a pose
fd = free & ~d
def score2(xs, ys, th):
    hit = lookup(d, xs, ys, th, P).mean(1)
    endfree = lookup(fd, xs, ys, th, P).mean(1)
    viol = lookup(occ, xs, ys, th, RS).mean(1) if len(RS) else 0
    return hit - 3.0*viol - 2.0*endfree, hit, viol, endfree
# line through candidates along corridor
p0 = np.array([0.745, 5.183]); p1 = np.array([-0.695, 6.103]); u = (p1-p0)/np.linalg.norm(p1-p0); nrm = np.array([-u[1], u[0]])
prof = []
for s in np.arange(-1.5, 3.01, 0.1):
    best = None
    for lat in np.arange(-0.2, 0.201, 0.04):
        c = p0 + s*u + lat*nrm
        for thd in list(np.arange(326, 339, 1.0)) + list(np.arange(147, 160, 1.0)):
            sc, h, v, e = score2(np.array([c[0]]), np.array([c[1]]), math.radians(thd))
            r = (float(sc[0]), float(c[0]), float(c[1]), thd, float(h[0]), float(v[0]) if len(RS) else 0, float(e[0]))
            if best is None or r[0] > best[0]: best = r
    prof.append(best)
for r in prof: print("s=%5.2f score=%.3f hit=%.3f viol=%.3f endfree=%.3f x=%.2f y=%.2f th=%.0f" % (( (np.array(r[1:3])-p0)@u), r[0], r[4], r[5], r[6], r[1], r[2], r[3]))
