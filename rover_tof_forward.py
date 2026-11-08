#!/usr/bin/env python3
"""Minimum trusted forward ToF coverage for cruise (not lateral)."""
from __future__ import annotations

import math

from rover_tof_geometry import CRUISE_CONE_DEG, GUARD_L8_COLS, FORWARD_ARC_DEG, in_forward_arc


def l8_center_fresh(cols, t_cols, now, stale_s: float) -> list[float]:
    if not cols or len(cols) != 8 or now - t_cols > stale_s:
        return []
    return [float(x) for x in cols[GUARD_L8_COLS] if x > 0.02 and x < 4.0]


def trusted_forward_min(
    cols,
    t_cols,
    rays: dict,
    now,
    stale_s: float,
    *,
    usable,
) -> tuple[float | None, str]:
    """Return (min_clearance, source) or (None, reason). usable(name, in_forward) from TofTrust."""
    vals: list[float] = []
    for x in l8_center_fresh(cols, t_cols, now, stale_s):
        vals.append(x)
    if vals:
        return min(vals), "l8_center"
    for nm, bdeg in (("front", 0), ("front_left", 45), ("front_right", -45)):
        ry = rays.get(nm)
        if not ry or now - ry[2] > stale_s:
            continue
        b, r = ry[0], ry[1]
        if not (in_forward_arc(b, arc_deg=FORWARD_ARC_DEG)):
            continue
        if not usable(nm, True):
            continue
        if r > 0.02 and r < 4.0:
            vals.append(r)
    if vals:
        return min(vals), "tof_forward_arc"
    return None, "no_trusted_forward"


def has_trusted_forward_coverage(cols, t_cols, rays, now, stale_s, usable) -> bool:
    m, _ = trusted_forward_min(cols, t_cols, rays, now, stale_s, usable=usable)
    return m is not None
