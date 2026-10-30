#!/usr/bin/env python3
"""Offline check of cam_gate.CamGate (the FOLLOW entry/exit freshness gate)."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cam_gate import CamGate

def sim(frames, until, dt=0.05):
    """frames: list of (t, person). Returns times at which person was confirmed, and live flags."""
    g = CamGate(enter_s=0.5); conf = []; live = []; fi = 0; t = 0.0
    while t <= until:
        while fi < len(frames) and frames[fi][0] <= t: g.on_frame(*frames[fi]); fi += 1
        live.append((round(t, 2), g.live(t))); c = g.person(t)[1]
        if c: conf.append(round(t, 2))
        t += dt
    return conf, live

# 1. the 22:36 case: a short burst of buffered frames (person in all) then the camera goes silent
conf, live = sim([(1.0 + 0.03 * i, True) for i in range(8)], 20)
print("burst then silent: confirmed at", conf[:3] or "never", "| live at t=5:", dict(live)[5.0])
assert not conf and not dict(live)[5.0]
# 2. camera silent from the start: never live, never a person
conf, live = sim([], 10); assert not conf and not any(l for _, l in live); print("no camera: never live, never a person")
# 3. live 10 fps camera, person throughout: confirmed after ~0.5 s
conf, _ = sim([(0.1 * i, True) for i in range(100)], 10); print("live person: first confirmed at", conf[0]); assert 0.5 <= conf[0] <= 0.8
# 4. live camera, one-frame flicker person every 2 s: never confirmed
conf, _ = sim([(0.1 * i, i % 20 == 0) for i in range(100)], 10); print("flicker person: confirmed", conf or "never"); assert not conf
# 5. live person, then camera dies at 5 s: confirmation ends within 0.55 s
conf, _ = sim([(0.1 * i, True) for i in range(51)], 10); print("camera dies at 5.0 s: last confirmed at", conf[-1]); assert conf[-1] <= 5.55
print("PASS")
