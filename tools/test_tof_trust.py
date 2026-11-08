#!/usr/bin/env python3
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rover_explore import Explorer
from rover_tof_trust import TofTrust


def test_motion_lock_not_value_timeout():
    t = TofTrust(motion_m=0.12, val_eps=0.004)
    t.update("front_left", 1.218, 0.0, 1.0, 1.0)
    assert t.trusted("front_left", 0.05)
    assert t.trusted("front_left", 0.11)
    assert not t.trusted("front_left", 0.13)


def test_diagonal_locked_while_peers_move():
    t = TofTrust(motion_m=0.12, val_eps=0.004)
    t.update("front_left", 1.218, 0.0, 1.0, 1.0)
    t.update("front_right", 0.743, 0.0, 1.0, 1.0)
    t.update("left", 0.88, 0.0, 1.0, 1.0)
    t.update("left", 0.85, 0.08, 1.1, 1.1)
    assert not t.trusted("front_left", 0.2)
    assert not t.trusted("front_right", 0.2)
    assert t.trusted("left", 0.19)


def _iter_g6_merged(path: str):
    import bisect

    if not os.path.isfile(path):
        return None, None
    by_epoch = []
    paths = []
    for line in open(path):
        o = json.loads(line)
        by_epoch.append(o)
        if o["kind"] == "status" and o["data"].get("session"):
            paths.append((o["epoch"], float(o["data"]["path_m"])))
    by_epoch.sort(key=lambda x: x["epoch"])

    def path_at(epoch: float) -> float:
        if not paths:
            return 0.0
        xs = [p[0] for p in paths]
        i = bisect.bisect_right(xs, epoch) - 1
        return paths[max(0, i)][1]

    return by_epoch, path_at


def test_g6_jsonl_frozen_diagonals():
    path = "/home/ubuntu/.cursor/projects/workspace/uploads/rover-g6-baseline-1133_3af0.jsonl"
    merged, path_at = _iter_g6_merged(path)
    if merged is None:
        print("skip g6 jsonl (not in CI)")
        return
    t = TofTrust(motion_m=0.12, val_eps=0.004)
    ticks0 = ticks1 = None
    locked_fl = locked_fr = 0
    total_fl = 0
    for o in merged:
        if o["kind"] == "left_ticks":
            ticks0 = ticks0 or o["data"]
            ticks1 = o["data"]
    for o in merged:
        k = o["kind"]
        if k not in ("front_left", "front_right", "left", "right"):
            continue
        d = o["data"]
        r = d.get("range")
        if r != r:
            continue
        path_m = path_at(o["epoch"])
        t.update(k, r, path_m, float(d.get("stamp", 0)), float(o["epoch"]))
        if k == "front_left":
            total_fl += 1
            if not t.trusted("front_left", path_m):
                locked_fl += 1
        if k == "front_right" and not t.trusted("front_right", path_m):
            locked_fr += 1
    assert ticks1 and ticks0 and (ticks1 - ticks0) > 200
    assert total_fl > 50 and locked_fl > total_fl * 0.8, (locked_fl, total_fl)
    assert locked_fr > 50


def test_explore_drops_untrusted_rays():
    x = Explorer()
    t0 = 100.0
    cols = [1.0] * 8
    x.on_cols(cols, t0)
    x.on_range("front_right", -45, 0.743, t0)
    x.on_range("right", -90, 0.45, t0)
    x.on_range("left", 90, 0.85, t0)
    v, w, why = x.step(t0)
    assert "0.74" not in why or "0.45" in why or float(why.split("ahead")[1].split("m")[0]) < 0.7


def main():
    test_motion_lock_not_value_timeout()
    print("ok motion_lock")
    test_diagonal_locked_while_peers_move()
    print("ok peer cross-check")
    test_g6_jsonl_frozen_diagonals()
    print("ok g6 jsonl")
    test_explore_drops_untrusted_rays()
    print("ok explore side fallback")
    test_g6_jsonl_replay_ahead_changes()
    print("ok g6 replay ahead")
    print("PASS")


def test_g6_jsonl_replay_ahead_changes():
    """Replay G6 ToF+path: explore ahead should not stay pinned at 0.743 once diagonals dropped."""
    path = "/home/ubuntu/.cursor/projects/workspace/uploads/rover-g6-baseline-1133_3af0.jsonl"
    merged, path_at = _iter_g6_merged(path)
    if merged is None:
        return
    trust = TofTrust(motion_m=0.12, val_eps=0.004)
    x = Explorer()
    aheads = []
    t = 100.0
    for o in merged:
        if o["kind"] not in ("left", "right", "front_left", "front_right"):
            continue
        d = o["data"]
        r = d.get("range")
        name = o["kind"]
        path_m = path_at(o["epoch"])
        bearing = {"left": 90, "right": -90, "front_left": 45, "front_right": -45}[name]
        trust.update(name, r if r == r else None, path_m, float(d.get("stamp", 0)), t)
        if trust.trusted(name, path_m) and r == r and r > 0.02:
            x.on_range(name, bearing, r, t)
        else:
            x.on_range(name, bearing, float("nan"), t)
        x.on_cols([4.0] * 8, t)
        t += 0.02
        v, w, why = x.step(t)
        if "ahead" in why:
            try:
                a = float(why.split("ahead")[1].split("m")[0].strip())
                aheads.append(a)
            except ValueError:
                pass
    assert aheads
    uniq = len(set(round(a, 2) for a in aheads))
    assert uniq > 3, f"ahead stayed frozen: {uniq} unique of {len(aheads)}"
    assert min(aheads) < 0.65, f"min ahead {min(aheads)}"


if __name__ == "__main__":
    main()
