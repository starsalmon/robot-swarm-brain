#!/usr/bin/env python3
"""Host tests for the Jazz pitch-stabilizer math (no ROS, no camera)."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jazz_pitch_stab import (  # noqa: E402
    applied_pitch,
    load_K,
    pitch_from_imu_quat,
    pitch_homography,
    rx,
)


def jazz_rpy_to_quat(yaw, pitch, roll=0.0):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def test_identity_at_zero():
    K = load_K("/nonexistent")
    H = pitch_homography(0.0, K)
    assert np.allclose(H, np.eye(3), atol=1e-9)


def test_applied_gain_and_trim():
    assert abs(applied_pitch(0.2, 0.0, 1.0) - 0.2) < 1e-12
    assert abs(applied_pitch(0.2, 0.0, -1.0) + 0.2) < 1e-12
    assert abs(applied_pitch(0.0, 0.05, 1.0) - 0.05) < 1e-12


def test_small_pitch_moves_principal_point_vertically():
    # A nose-up pitch (+), Rx(+pitch) should send the principal point "up"
    # in OpenCV (negative y) — looking down to undo the nod.
    K = np.array([[800.0, 0.0, 512.0], [0.0, 800.0, 384.0], [0.0, 0.0, 1.0]])
    H = pitch_homography(0.1, K, pitch_gain=1.0)
    p = H @ np.array([512.0, 384.0, 1.0])
    p /= p[2]
    assert p[1] < 384.0, (p, "expected horizon/principal point to move up")
    assert abs(p[0] - 512.0) < 1.0


def test_quat_roundtrip_pitch():
    for p in (-0.8, -0.1, 0.0, 0.25, 1.2):
        qx, qy, qz, qw = jazz_rpy_to_quat(0.3, p, 0.0)
        got = pitch_from_imu_quat(qx, qy, qz, qw)
        assert abs(got - p) < 1e-6, (p, got)


def test_rx_orthonormal():
    R = rx(0.37)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert abs(np.linalg.det(R) - 1.0) < 1e-9


if __name__ == "__main__":
    test_identity_at_zero()
    test_applied_gain_and_trim()
    test_small_pitch_moves_principal_point_vertically()
    test_quat_roundtrip_pitch()
    test_rx_orthonormal()
    print("test_jazz_pitch_stab: OK")
