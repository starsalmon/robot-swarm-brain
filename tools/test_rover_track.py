#!/usr/bin/env python3
"""Offline check of rover_track.RoverTrack on a real LifeCam frame (python3 tools/test_rover_track.py frame.jpg).
  A: the frame as-is for 15 s with /cmd_vel 'moving' -> static blobs (tubs, parked Rover) must never be confirmed.
  B: a Rover-sized patch cut from the frame slides across the carpet while 'moving' -> confirmed, and tracked.
  C: then it stops and the cmd stops -> stays confirmed.  D: cmd says move but it stays put -> dropped within ~3 s."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2
from rover_track import RoverTrack
from room_eye import rover_candidates

base = cv2.imread(sys.argv[1]); H, W = base.shape[:2]
patch = base[194:278, 621:731].copy()                    # parked Rover in the 2026-10-30 frame
dt = 0.12; t = 0.0; tr = RoverTrack(); out = {}

def run(n, frame_fn, moving, tag):
    global t
    res = []
    for i in range(n):
        f = frame_fn(i); hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
        r = tr.update(f, rover_candidates(hsv, None), moving, t); t += dt
        res.append(r)
    got = [r for r in res if r]
    out[tag] = (len(got), n, None if not got else (round(got[-1][1]), round(got[-1][2])))
    return res

def place(x):
    f = base.copy(); x = int(x); y = 420
    f[y:y + patch.shape[0], x:x + patch.shape[1]] = patch
    return f

run(125, lambda i: base, True, "A static frame, cmd moving")
assert out["A static frame, cmd moving"][0] == 0, "confirmed a static blob"
tr = RoverTrack(); [tr.mog.apply(cv2.GaussianBlur(base, (5, 5), 0)) for _ in range(60)]   # learnt background
res = run(40, lambda i: place(300 + 6 * i), True, "B patch moving 6 px/frame")
first = next(i for i, r in enumerate(res) if r)
run(30, lambda i: place(300 + 6 * 39), False, "C stopped, cmd stopped")
res = run(40, lambda i: place(300 + 6 * 39), True, "D cmd moving, patch stays still")
drop = next((i for i, r in enumerate(res) if not r), None)
for k, v in out.items(): print(f"{k:34s} rover reported in {v[0]}/{v[1]} frames, last at {v[2]}")
print(f"B: confirmed after {first * dt:.2f} s;  D: dropped after {None if drop is None else round(drop * dt, 2)} s")
assert out["B patch moving 6 px/frame"][0] > 0 and out["C stopped, cmd stopped"][0] == 30 and drop is not None and drop * dt < 3.5
print("PASS")
