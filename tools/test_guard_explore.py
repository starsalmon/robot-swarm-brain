#!/usr/bin/env python3
"""Offline tests: pivot pick, guard unstick, explore step (deployed f7e1b6e baseline + unstick fix)."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rover_explore import Explorer
from rover_guard_escape import (
    GUARD_M_DEFAULT,
    apply_turn_guard,
    clear_side,
    near_side,
    pick_open_side,
    unstick_explore,
)


def _side(**kw):
    now = 100.0
    out = {}
    for k, r in kw.items():
        out[k] = (r, now)
    return out


def test_pick_open_left_blocked_right_clear():
    side = _side(front_left=0.08, front_right=0.60, right=0.62)
    assert pick_open_side(side, 100.0) == -1.0


def test_pick_open_only_right_fresh():
    side = _side(right=0.58)
    assert pick_open_side(side, 100.0) == -1.0


def test_pick_open_no_blind_default():
    side = _side()
    assert pick_open_side(side, 100.0) is None


def test_both_sides_blocked_no_pick():
    side = _side(front_left=0.10, left=0.12, front_right=0.11, right=0.09)
    assert pick_open_side(side, 100.0) is None


def test_guard_blocks_left_turn():
    side = _side(front_left=0.08)
    w, hit = apply_turn_guard(0.14, side, 100.0)
    assert w == 0.0 and "left" in hit[0]


def test_unstick_flip_right():
    side = _side(front_left=0.08, right=0.65, front_right=0.55)
    v, w, tag = unstick_explore(0.0, 0.0, 0.0, 0.14, side, 100.0, 0.08, pivot_w=0.14)
    assert w < 0 and tag and "right" in tag


def test_unstick_reverse_when_boxed():
    side = _side(front_left=0.08, front_right=0.09)
    v, w, tag = unstick_explore(0.0, 0.0, 0.0, 0.14, side, 100.0, 0.08, pivot_w=0.14)
    assert v < 0 and tag and "reverse" in tag


def test_unstick_no_reverse_if_rear_close():
    side = _side(front_left=0.08, front_right=0.09)
    v, w, tag = unstick_explore(
        0.0, 0.0, 0.0, 0.14, side, 100.0, 0.08, pivot_w=0.14, rear_m=0.12, rear_fresh=True,
    )
    assert v == 0.0 and w == 0.0


def test_explore_blocked_flip_not_deadlock():
    x = Explorer()
    t0 = 100.0
    cols = [0.08, 0.08, 0.08, 0.08, 0.10, 0.10, 0.30, 0.30]
    x.on_cols(cols, t0)
    x.on_range("front", 0, 0.08, t0)
    x.on_range("front_left", 45, 0.08, t0)
    x.on_range("right", -90, 0.62, t0)
    v, w, why = x.step(t0)
    assert abs(w) > 0.05 and w < 0, why
    assert "pivot right" in why


def test_explore_stale_stops():
    x = Explorer()
    v, w, why = x.step(1.0)
    assert v == 0 and w == 0 and "no fresh" in why


def test_go_off_semantics_doc():
    """Sanity: imported behaviour module still documents halt on Go toggle (parent verified E-stop 3)."""
    import pathlib

    text = pathlib.Path(__file__).resolve().parents[1].joinpath("rover_behaviour.py").read_text()
    assert 'if int(m.data) == 3' in text and 's.go = not s.go' in text


def main():
    tests = [
        test_pick_open_left_blocked_right_clear,
        test_pick_open_only_right_fresh,
        test_pick_open_no_blind_default,
        test_both_sides_blocked_no_pick,
        test_guard_blocks_left_turn,
        test_unstick_flip_right,
        test_unstick_reverse_when_boxed,
        test_unstick_no_reverse_if_rear_close,
        test_explore_blocked_flip_not_deadlock,
        test_explore_stale_stops,
        test_go_off_semantics_doc,
    ]
    for fn in tests:
        fn()
        print("ok", fn.__name__)
    print("PASS", len(tests))


if __name__ == "__main__":
    main()
