#!/usr/bin/env python3
"""ToF message tracking: republish suspicion (control) and motion diagnostic (log only).

Message header.stamp is recorded as published stamp — not verified new acquisition unless
the Pro S3 firmware is inspected (lego-rover-esp32, um_pros3_tof_microros).

Control rules (conservative):
  - Suspected republish (stamps advance, value fixed, peer ranges changed) → exclude from
    forward cruise / VFH rays only. Does not clear guard near vetoes.
  - Motion path without value change → diagnostic flag only (parallel flat wall is valid).

Does not weaken GUARD_M: guard uses live side readings including suspected channels if near.
"""
from __future__ import annotations

import math
import os
from typing import Any

from rover_tof_geometry import DIAGONAL, LATERAL

E = lambda k, d: float(os.environ.get(k, d))

VAL_EPS = E("TOF_TRUST_VAL_EPS", 0.004)
STAMP_ADVANCES = int(E("TOF_REPUBLISH_STAMP_ADVANCES", 3))


def range_msg_stamp_sec(msg: Any) -> float:
    """ROS message header time (publisher-set; may be remeasure or republish)."""
    try:
        st = msg.header.stamp
        return float(st.sec) + float(st.nanosec) * 1e-9
    except Exception:
        return 0.0


PEERS = {
    "front_left": tuple(LATERAL | {"front"}),
    "front_right": tuple(LATERAL | {"front"}),
    "front": ("left", "right", "front_left", "front_right"),
    "left": ("right", "front_left", "front"),
    "right": ("left", "front_right", "front"),
}


class TofTrust:
    def __init__(s, *, val_eps: float = VAL_EPS, stamp_advances: int = STAMP_ADVANCES):
        s.val_eps = val_eps
        s.stamp_advances = max(2, stamp_advances)
        s.ch: dict[str, dict] = {}
        s._last_path_m = 0.0

    def _valid(s, r: float | None) -> bool:
        return r is not None and math.isfinite(r) and r > 0.02

    def update(s, name: str, r: float | None, path_m: float | None, msg_stamp: float, rx_s: float) -> None:
        s._last_path_m = path_m if path_m is not None else s._last_path_m
        if not s._valid(r):
            s.ch.pop(name, None)
            return
        r = float(r)
        st = s.ch.get(name)
        if st is None or abs(r - st["val"]) > s.val_eps:
            s.ch[name] = dict(
                val=r,
                val_changes=(1 if st is None else st["val_changes"] + 1),
                stamps_since_change=1,
                last_msg_stamp=msg_stamp,
                prev_msg_stamp=msg_stamp,
                rx=rx_s,
                path_at_val=s._last_path_m,
            )
            return
        st["last_msg_stamp"] = msg_stamp
        st["rx"] = rx_s
        if msg_stamp != st.get("prev_msg_stamp", msg_stamp):
            st["stamps_since_change"] = st.get("stamps_since_change", 1) + 1
        st["prev_msg_stamp"] = msg_stamp

    def motion_suspect(s, name: str, path_m: float | None) -> bool:
        """Diagnostic only — parallel drive along a flat wall can be constant range."""
        if path_m is None:
            return False
        st = s.ch.get(name)
        if not st:
            return False
        return (path_m - st.get("path_at_val", path_m)) > E("TOF_DIAG_MOTION_M", 0.12)

    def _peer_vals_changed(s, name: str) -> bool:
        st = s.ch.get(name)
        if not st:
            return False
        my_changes = st["val_changes"]
        for peer in PEERS.get(name, ()):
            pst = s.ch.get(peer)
            if pst and pst["val_changes"] > my_changes:
                return True
        return False

    def suspect_republish(s, name: str) -> bool:
        """Stamp advances with fixed value while peer channel(s) changed — withhold from forward rays."""
        st = s.ch.get(name)
        if not st:
            return True
        if st.get("stamps_since_change", 1) < s.stamp_advances:
            return False
        return s._peer_vals_changed(name)

    def usable_in_explore(s, name: str, bearing_in_forward: bool) -> bool:
        if name in LATERAL:
            return not s.suspect_republish(name)
        if bearing_in_forward or name in DIAGONAL or name == "front":
            return not s.suspect_republish(name)
        return True

    def snapshot(s, path_m: float | None) -> dict[str, dict[str, bool]]:
        out = {}
        for n in s.ch:
            out[n] = dict(
                suspect_repub=s.suspect_republish(n),
                motion_diag=s.motion_suspect(n, path_m),
            )
        return out
