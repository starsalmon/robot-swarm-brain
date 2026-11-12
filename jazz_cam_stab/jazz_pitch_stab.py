"""Pitch-stabilize a forward-looking Jazz camera using IMU pitch (radians).

The Pi stream stays unrotated (CAM_ROT=0). This module virtually gimbals the
frame so a body nod does not tilt the view: a homography from the cam146
intrinsics and Rx(applied), where applied = gain * pitch + trim.

Jazz /jazz/pitch is std_msgs/Float32 in radians (same as the balance loop).
Positive pitch is nose-up / lying back. Default gain is +1: an OpenCV Rx(+pitch)
looks down and undoes a nose-up nod. Set pitch_gain=-1 if the horizon moves
the wrong way (mount sign).

The output is the same size as the input, with black borders (FOV/corner loss
grows with |pitch|). No zoom by default so a tag tracker keeps a stable
apparent scale. This is a plane-at-infinity warp, not a full 3D recon.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Optional

import numpy as np

DEFAULT_CALIB = "/calib/cam146.json"


def load_K(calib_path: str | Path | None = None) -> np.ndarray:
    path = Path(calib_path or DEFAULT_CALIB)
    if path.is_file():
        data = json.loads(path.read_text())
        if "camera_matrix" in data:
            return np.array(data["camera_matrix"], dtype=np.float64)
        return np.array(
            [[data["fx"], 0.0, data["cx"]], [0.0, data["fy"], data["cy"]], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    # Reasonable fallback for the 1024x768 IMX708 stream if calib is missing.
    return np.array([[878.0, 0.0, 511.0], [0.0, 881.0, 421.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def pitch_from_imu_quat(x: float, y: float, z: float, w: float) -> float:
    """Extract pitch (rad) from the quaternion jazzRpyToQuat(yaw, pitch, 0) publishes."""
    # Same Tait-Bryan convention as jazz_uros_cmd.h.
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


def rx(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def applied_pitch(pitch_rad: float, trim_rad: float = 0.0, pitch_gain: float = 1.0) -> float:
    return pitch_gain * float(pitch_rad) + float(trim_rad)


def pitch_homography(pitch_rad: float, K: np.ndarray, trim_rad: float = 0.0, pitch_gain: float = 1.0) -> np.ndarray:
    a = applied_pitch(pitch_rad, trim_rad, pitch_gain)
    Kinv = np.linalg.inv(K)
    return K @ rx(a) @ Kinv


class JazzPitchStabilizer:
    """Stateful warp used by the preview node and as a hook for the AprilTag tracker."""

    def __init__(
        self,
        calib_path: str | Path | None = None,
        trim_deg: float = 0.0,
        pitch_gain: float = 1.0,
    ):
        self.K = load_K(calib_path)
        self.trim_rad = math.radians(float(trim_deg))
        self.pitch_gain = float(pitch_gain)
        self.pitch_rad = 0.0
        self._H = None
        self._size = None  # (w, h)
        self._update_H()

    def set_trim_deg(self, trim_deg: float) -> None:
        self.trim_rad = math.radians(float(trim_deg))
        self._H = None

    def set_pitch(self, pitch_rad: float) -> None:
        self.pitch_rad = float(pitch_rad)
        self._H = None

    def applied_rad(self) -> float:
        return applied_pitch(self.pitch_rad, self.trim_rad, self.pitch_gain)

    def _update_H(self) -> np.ndarray:
        self._H = pitch_homography(self.pitch_rad, self.K, self.trim_rad, self.pitch_gain)
        return self._H

    def warp(self, frame, pitch_rad: Optional[float] = None):
        import cv2

        if pitch_rad is not None:
            self.set_pitch(pitch_rad)
        if frame is None or frame.size == 0:
            return frame
        h, w = frame.shape[:2]
        if self._H is None or self._size != (w, h):
            self._update_H()
            self._size = (w, h)
        return cv2.warpPerspective(
            frame,
            self._H,
            (w, h),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )


def stabilize_bgr(frame, pitch_rad: float, calib_path: str | Path | None = None,
                  trim_deg: float = 0.0, pitch_gain: float = 1.0):
    """One-shot hook: return a pitch-stabilized copy of a BGR frame."""
    stab = JazzPitchStabilizer(calib_path=calib_path, trim_deg=trim_deg, pitch_gain=pitch_gain)
    return stab.warp(frame, pitch_rad)
