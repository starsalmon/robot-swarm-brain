#!/usr/bin/env python3
"""Replay recorded ToF topics from a rosbag through the new Explorer (rover_explore.py) and report what it would
command: forward speed, turn rate, how often it would pivot/stop. Offline, nothing is published.
Usage (in a ROS jazzy image): python3 tools/replay_explore.py /captures/<bag> [...]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rover_explore import Explorer

SIDES = {"/rover/tof/front": ("front", 0), "/rover/tof/front_left": ("front_left", 45), "/rover/tof/front_right": ("front_right", -45),
         "/rover/tof/left": ("left", 90), "/rover/tof/right": ("right", -90)}

for bag in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    sid = "mcap" if any(f.endswith(".mcap") for f in os.listdir(bag)) else "sqlite3"
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=sid), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    x = Explorer(); nxt = None; out = []; old = []
    while r.has_next():
        t, d, ts = r.read_next(); now = ts * 1e-9
        if t == "/rover/tof/l8_cols": x.on_cols(list(deserialize_message(d, ty[t]).data), now)
        elif t in SIDES: x.on_range(SIDES[t][0], SIDES[t][1], deserialize_message(d, ty[t]).range, now)
        elif t in ("/rover/bump", "/rover/stall") and deserialize_message(d, ty[t]).data: x.on_bump(now, t.split("/")[-1])
        elif t == "/cmd_vel":
            m = deserialize_message(d, ty[t]); old.append((m.linear.x, m.angular.z))
        if nxt is None: nxt = now
        while now >= nxt:
            out.append(x.step(nxt)); nxt += 0.05
    if not out: print(bag, "no data"); continue
    live = [o for o in out if "no fresh" not in o[2]]
    def stats(cmds, name):
        if not cmds: return f"{name}: none"
        v = [c[0] for c in cmds]; w = [abs(c[1]) for c in cmds]
        return (f"{name}: n={len(cmds)} mean v {sum(v)/len(v):.3f}  stopped(v<=0.01 & |w|<0.01) {100*sum(1 for c in cmds if c[0] <= 0.01 and abs(c[1]) < 0.01)/len(cmds):.0f}%"
                f"  arcing(v>0.05 & |w|>0.02) {100*sum(1 for c in cmds if c[0] > 0.05 and abs(c[1]) > 0.02)/len(cmds):.0f}%  mean|w| {sum(w)/len(w):.3f}")
    print(os.path.basename(bag.rstrip("/")), f"{len(out)*0.05:.0f}s")
    print("  new  " + stats([(o[0], o[1]) for o in live], "explore") + f"  pivot {100*sum('pivot' in o[2] for o in live)/max(1,len(live)):.0f}%  recover {sum('recover' in o[2] for o in live)}")
    print("  old  " + stats(old, "/cmd_vel as recorded"))
