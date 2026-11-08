#!/usr/bin/env python3
"""Mark ToF ranges untrusted when the robot moves but a channel's range does not change.

Uses wheel odom path distance (not "same value for N seconds"). Independent peers
(left/right) updating while a diagonal stays fixed is treated as motion-locked on the
diagonal — typical of republished cached readings with new header stamps.

Does not weaken GUARD_M: untrusted readings are withheld from steer and from
open-clearance decisions; guard still uses only fresh trusted near readings.
"""
from __future__ import annotations

import math
import os
from typing import Any

E = lambda k, d: float(os.environ.get(k, d))


MOTION_M = E("TOF_TRUST_MOTION_M", 0.12)
VAL_EPS = E("TOF_TRUST_VAL_EPS", 0.004)
DIAGONAL = frozenset({"front_left", "front_right"})
PEERS = {"front_left": ("left", "right"), "front_right": ("left", "right")}


def range_stamp_sec(msg: Any) -> float:
    try:
        st = msg.header.stamp
        return float(st.sec) + float(st.nanosec) * 1e-9
    except Exception:
        return 0.0


class TofTrust:
    def __init__(s, *, motion_m: float = MOTION_M, val_eps: float = VAL_EPS):
        s.motion_m = motion_m
        s.val_eps = val_eps
        s.ch: dict[str, dict[str, float]] = {}

    def _valid(s, r: float | None) -> bool:
        return r is not None and math.isfinite(r) and r > 0.02

    def update(s, name: str, r: float | None, path_m: float, acq_s: float, rx_s: float) -> None:
        if not s._valid(r):
            s.ch.pop(name, None)
            return
        r = float(r)
        st = s.ch.get(name)
        if st is None or abs(r - st["val"]) > s.val_eps:
            s.ch[name] = dict(val=r, path_at_val=path_m, acq=acq_s, rx=rx_s, updates=1)
            return
        st["acq"] = acq_s
        st["rx"] = rx_s
        st["updates"] += 1

    def motion_locked(s, name: str, path_m: float) -> bool:
        st = s.ch.get(name)
        if st is None:
            return True
        return (path_m - st["path_at_val"]) > s.motion_m

    def peers_changing(s, name: str, path_m: float) -> bool:
        for peer in PEERS.get(name, ()):
            pst = s.ch.get(peer)
            if pst is None:
                continue
            if (path_m - pst["path_at_val"]) <= s.motion_m * 0.5:
                return True
        return False

    def trusted(s, name: str, path_m: float) -> bool:
        if not s._valid(s.ch.get(name, {}).get("val")):
            return False
        if not s.motion_locked(name, path_m):
            return True
        if name in DIAGONAL and s.peers_changing(name, path_m):
            return False
        return not s.motion_locked(name, path_m)

    def snapshot(s, path_m: float) -> dict[str, bool]:
        return {n: s.trusted(n, path_m) for n in s.ch}
