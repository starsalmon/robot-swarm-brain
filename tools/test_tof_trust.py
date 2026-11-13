#!/usr/bin/env python3
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rover_explore import Explorer
from rover_tof_forward import has_trusted_forward_coverage, trusted_forward_min
from rover_tof_trust import TofTrust


def test_republish_suspect_not_parallel_wall():
    t = TofTrust(stamp_advances=3)
    for i in range(5):
        t.update("left", 0.85 + 0.02 * i, 0.1 * i, float(i), float(i))
    t.update("front_left", 1.218, 0.4, 5.0, 5.0)
    t.update("front_left", 1.218, 0.5, 6.0, 6.0)
    t.update("front_left", 1.218, 0.6, 7.0, 7.0)
    t.update("front_left", 1.218, 0.7, 8.0, 8.0)
    assert not t.suspect_republish("left")
    assert t.suspect_republish("front_left")


def test_no_suspect_without_peer_change():
    t = TofTrust(stamp_advances=2)
    for i in range(4):
        t.update("front_left", 1.0, None, float(i), float(i))
    assert not t.suspect_republish("front_left")


def test_motion_diag_only_without_path():
    t = TofTrust()
    t.update("front_left", 1.0, None, 1.0, 1.0)
    assert not t.motion_suspect("front_left", None)
    t.update("front_left", 1.0, 0.5, 2.0, 2.0)
    assert not t.motion_suspect("front_left", None)


def test_stale_forward_clear_sides_no_forward():
    x = Explorer()
    t0 = 100.0
    x.on_cols([4.0] * 8, t0)
    x.on_range("front_right", -45, float("nan"), t0)
    x.on_range("front_left", 45, float("nan"), t0)
    x.on_range("right", -90, 0.55, t0)
    x.on_range("left", 90, 0.85, t0)
    v, w, why = x.step(t0)
    assert v == 0.0 and "no trusted forward" in why


def test_guard_near_uses_suspect_reading():
    """Suspected republish with near raw value must still count for guard (side store)."""
    # Behaviour integration covered in doc; here test side map path via explore pivot guard helpers.
    from rover_guard_escape import near_side
    side = {"front_left": (0.15, 100.0)}
    assert near_side(side, 100.0, ("front_left",))


def _iter_telem_merged(path: str):
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


def test_telem_jsonl_suspect_diagonals():
    path = os.environ.get(
        "ROVER_TOF_TELEM_JSONL",
        "/home/ubuntu/.cursor/projects/workspace/uploads/rover-g6-baseline-1133_3af0.jsonl",
    )
    merged, path_at = _iter_telem_merged(path)
    if merged is None:
        return
    t = TofTrust(stamp_advances=3)
    sus = 0
    n = 0
    for o in merged:
        if o["kind"] not in ("front_left", "front_right", "left", "right"):
            continue
        d = o["data"]
        r = d.get("range")
        if r != r:
            continue
        pm = path_at(o["epoch"])
        t.update(o["kind"], r, pm, float(d.get("stamp", 0)), float(o["epoch"]))
        if o["kind"] == "front_left":
            n += 1
            if t.suspect_republish("front_left"):
                sus += 1
    assert n > 50 and sus > n * 0.5


def test_telem_replay_sustained_no_trusted_forward():
    path = os.environ.get(
        "ROVER_TOF_TELEM_JSONL",
        "/home/ubuntu/.cursor/projects/workspace/uploads/rover-g6-baseline-1133_3af0.jsonl",
    )
    merged, path_at = _iter_telem_merged(path)
    if merged is None:
        return
    trust = TofTrust(stamp_advances=3)
    x = Explorer()
    t = 100.0
    tail_whys = []
    for o in merged:
        if o["kind"] not in ("left", "right", "front_left", "front_right"):
            continue
        d = o["data"]
        r = d.get("range")
        name = o["kind"]
        pm = path_at(o["epoch"])
        bearing = {"left": 90, "right": -90, "front_left": 45, "front_right": -45}[name]
        trust.update(name, r if r == r else None, pm, float(d.get("stamp", 0)), t)
        diag_sus = trust.suspect_republish("front_left") or trust.suspect_republish("front_right")
        drop = trust.suspect_republish(name) or (name in ("front_left", "front_right") and diag_sus)
        if name in ("front_left", "front_right") and diag_sus:
            for dn, b in (("front_left", 45), ("front_right", -45)):
                x.on_range(dn, b, float("nan"), t)
        else:
            use = r if r == r and r > 0.02 and not drop else float("nan")
            x.on_range(name, bearing, use, t)
        t += 0.02
        v, w, why = x.step(t)
        if diag_sus and pm > 3.5:
            tail_whys.append((v, why))
    assert tail_whys
    pinned = sum(1 for v, w in tail_whys if "ahead 0.74" in w and v > 0.05)
    assert pinned <= max(3, int(len(tail_whys) * 0.35)), f"pinned 0.74 cruise {pinned}/{len(tail_whys)}"
    assert any("no trusted forward" in w for _, w in tail_whys[-40:])


def test_forward_min_never_side_only():
    x = Explorer()
    t0 = 1.0
    x.on_range("right", -90, 0.4, t0)
    x.on_range("left", 90, 0.9, t0)
    assert not has_trusted_forward_coverage(None, -1e9, x.rays, t0, 1.0, lambda *_: True)
    m, src = trusted_forward_min(None, -1e9, x.rays, t0, 1.0, usable=lambda *_: True)
    assert m is None


def main():
    test_republish_suspect_not_parallel_wall()
    print("ok republish suspect")
    test_no_suspect_without_peer_change()
    print("ok parallel constant")
    test_motion_diag_only_without_path()
    print("ok motion diag only")
    test_stale_forward_clear_sides_no_forward()
    print("ok stale forward no v")
    test_guard_near_uses_suspect_reading()
    print("ok guard conservative")
    test_forward_min_never_side_only()
    print("ok no side-as-ahead")
    test_telem_jsonl_suspect_diagonals()
    print("ok telem suspect (skipped if no fixture)")
    test_telem_replay_sustained_no_trusted_forward()
    print("ok telem sustained forward gate (skipped if no fixture)")
    print("PASS")


if __name__ == "__main__":
    main()
