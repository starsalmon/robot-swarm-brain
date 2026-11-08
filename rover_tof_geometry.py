#!/usr/bin/env python3
"""Rover ToF topic geometry (brain-side conventions). Publisher firmware: lego-rover-esp32 Pro S3 ToF board.

See lego-rover-esp32/WIRING.md (not vendored in this repo) and fleet rules: VL53L8CX front + L8 grid;
VL53L0X side and diagonal ranging on the second I2C bus. micro-ROS publishes sensor_msgs/Range + l8_cols.

Bearings (explore VFH, + = left / CCW from nose):
  front 0 deg | front_left +45 | front_right -45 | left +90 | right -90
  l8_cols: 8 zones, col 0 = LEFT, ~45 deg FOV total (each col ~5.6 deg); guard uses center cols 2:6.

Forward coverage for cruise (minimum trusted):
  at least one fresh range in the FORWARD_ARC (|bearing| <= FORWARD_ARC_DEG) from L8 centre columns
  and/or trusted nose beam (front), not from left/right side beams alone.

Lateral beams inform steering/pivot off walls only; they must not set forward cruise clearance.
"""
from __future__ import annotations

import math

FORWARD_ARC_DEG = 50.0
CRUISE_CONE_DEG = 16.0
GUARD_L8_COLS = slice(2, 6)

LATERAL = frozenset({"left", "right"})
FORWARD_BEAMS = frozenset({"front", "front_left", "front_right"})
DIAGONAL = frozenset({"front_left", "front_right"})


def bearing_deg(name: str) -> float:
    return {"front": 0.0, "front_left": 45.0, "front_right": -45.0, "left": 90.0, "right": -90.0}[name]


def in_forward_arc(bearing_rad: float, *, arc_deg: float = FORWARD_ARC_DEG) -> bool:
    return abs(bearing_rad) <= math.radians(arc_deg)


def in_cruise_cone(bearing_rad: float, *, cone_deg: float = CRUISE_CONE_DEG) -> bool:
    return abs(bearing_rad) <= math.radians(cone_deg)
