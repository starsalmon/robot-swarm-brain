#!/usr/bin/env python3
"""Obstacle guard helpers + EXPLORE unstick when pivot direction is vetoed.

Used by rover_behaviour.guard and rover_explore pivot choice. Pure functions for offline tests.
"""
from __future__ import annotations

import math
import os
from typing import Mapping, Sequence

SideMap = Mapping[str, tuple[float | None, float]]


def _e(k: str, d: float) -> float:
    try:
        return float(os.environ.get(k, d))
    except (TypeError, ValueError):
        return d


GUARD_M_DEFAULT = 0.20
FRESH_S_DEFAULT = 0.6
PIVOT_CLEAR_MARGIN_M = 0.03


def span_fresh(side: SideMap, now: float, names: Sequence[str], *, fresh_s: float = FRESH_S_DEFAULT) -> list[float]:
    vals: list[float] = []
    for n in names:
        r, t = side.get(n, (None, -1e9))
        if r is not None and math.isfinite(r) and r > 0.02 and now - t < fresh_s:
            vals.append(float(r))
    return vals


def near_side(
    side: SideMap,
    now: float,
    names: Sequence[str],
    *,
    guard_m: float = GUARD_M_DEFAULT,
    fresh_s: float = FRESH_S_DEFAULT,
) -> bool:
    vals = span_fresh(side, now, names, fresh_s=fresh_s)
    return bool(vals) and min(vals) < guard_m


def clear_side(
    side: SideMap,
    now: float,
    names: Sequence[str],
    *,
    guard_m: float = GUARD_M_DEFAULT,
    fresh_s: float = FRESH_S_DEFAULT,
    margin: float = PIVOT_CLEAR_MARGIN_M,
) -> bool:
    """Fresh reading on that wing and far enough to commit a pivot toward the opposite side."""
    vals = span_fresh(side, now, names, fresh_s=fresh_s)
    return bool(vals) and min(vals) >= guard_m + margin


def pick_open_side(
    side: SideMap,
    now: float,
    *,
    guard_m: float | None = None,
    fresh_s: float = FRESH_S_DEFAULT,
) -> float | None:
    """+1 = prefer pivot left, -1 = pivot right, None = no fresh basis (caller should reverse or hold)."""
    guard_m = GUARD_M_DEFAULT if guard_m is None else guard_m
    left_names = ("left", "front_left")
    right_names = ("right", "front_right")
    lvals = span_fresh(side, now, left_names, fresh_s=fresh_s)
    rvals = span_fresh(side, now, right_names, fresh_s=fresh_s)
    left_near = near_side(side, now, left_names, guard_m=guard_m, fresh_s=fresh_s)
    right_near = near_side(side, now, right_names, guard_m=guard_m, fresh_s=fresh_s)
    left_clear = clear_side(side, now, left_names, guard_m=guard_m, fresh_s=fresh_s)
    right_clear = clear_side(side, now, right_names, guard_m=guard_m, fresh_s=fresh_s)

    if lvals and rvals:
        mL, mR = sum(lvals) / len(lvals), sum(rvals) / len(rvals)
        if abs(mL - mR) < 0.05:
            if left_near and right_clear:
                return -1.0
            if right_near and left_clear:
                return 1.0
            return None
        pick = 1.0 if mL > mR else -1.0
        if pick > 0 and left_near and right_clear:
            return -1.0
        if pick < 0 and right_near and left_clear:
            return 1.0
        if pick > 0 and left_near:
            return -1.0 if right_clear else None
        if pick < 0 and right_near:
            return 1.0 if left_clear else None
        return pick
    if rvals and not lvals:
        if left_near:
            return -1.0 if right_clear else None
        return -1.0 if min(rvals) >= guard_m else None
    if lvals and not rvals:
        if right_near:
            return 1.0 if left_clear else None
        return 1.0 if min(lvals) >= guard_m else None
    return None


def apply_turn_guard(
    w: float,
    side: SideMap,
    now: float,
    *,
    guard_m: float = GUARD_M_DEFAULT,
    fresh_s: float = FRESH_S_DEFAULT,
) -> tuple[float, list[str]]:
    hit: list[str] = []
    out = w
    if w > 0 and near_side(side, now, ("front_left", "left"), guard_m=guard_m, fresh_s=fresh_s):
        out = 0.0
        hit.append("left side close")
    if w < 0 and near_side(side, now, ("front_right", "right"), guard_m=guard_m, fresh_s=fresh_s):
        out = 0.0
        hit.append("right side close")
    return out, hit


def unstick_explore(
    v: float,
    w: float,
    v_req: float,
    w_req: float,
    side: SideMap,
    now: float,
    ahead: float,
    *,
    guard_m: float = GUARD_M_DEFAULT,
    fresh_s: float = FRESH_S_DEFAULT,
    pivot_w: float = 0.14,
    reverse_v: float = -0.14,
    rear_m: float | None = None,
    rear_fresh: bool = False,
) -> tuple[float, float, str | None]:
    """After guard zeroed explore pivot: flip toward provably clear side, else short reverse if boxed ahead."""
    if abs(w_req) < 1e-6 and abs(v_req) < 1e-6:
        return v, w, None
    tag: str | None = None
    if w == 0.0 and abs(w_req) > 1e-6:
        want_left = w_req > 0
        if want_left and clear_side(side, now, ("right", "front_right"), guard_m=guard_m, fresh_s=fresh_s):
            w = -abs(pivot_w if abs(w_req) < 1e-3 else w_req)
            tag = "unstick: pivot right (left guard)"
        elif not want_left and clear_side(side, now, ("left", "front_left"), guard_m=guard_m, fresh_s=fresh_s):
            w = abs(pivot_w if abs(w_req) < 1e-3 else w_req)
            tag = "unstick: pivot left (right guard)"
    # Reverse whenever a requested pivot was fully zeroed, even if ToF/L8 "ahead"
    # is still outside guard_m (lidar can show a tighter blockage). Sitting still
    # while asking to turn is boxed-in.
    if v == 0.0 and w == 0.0 and abs(w_req) > 1e-6:
        rear_ok = rear_m is None or not rear_fresh or rear_m >= guard_m
        if rear_ok:
            v = reverse_v
            tag = (tag + "; reverse" if tag else "unstick: reverse (pivot boxed)")
        else:
            # Rear is actually close (a plant, a wall). Do not reverse into it;
            # take the requested pivot even if a front-corner ToF is a bit tight.
            w = (pivot_w if abs(w_req) < 1e-3 else abs(w_req)) * (1.0 if w_req > 0 else -1.0)
            tag = "unstick: pivot anyway (rear blocked)"
    return v, w, tag
