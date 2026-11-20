#!/usr/bin/env python3
"""NUC-side Jazz camera pitch stabilizer.

Reads the Pi MJPEG stream (CAM_ROT=0) and /jazz/pitch (radians), warps each
frame with JazzPitchStabilizer, and serves a preview:

  http://<nuc>:8181/stream
  http://<nuc>:8181/jpg
  http://<nuc>:8181/health

Does not touch the Pi, Jazz firmware, AU, or /fleet/allow_drive.
"""
from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

# Allow `python3 jazz_cam_stab_node.py` from this directory or /opt/fleet.
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from jazz_pitch_stab import JazzPitchStabilizer, pitch_freshness, pitch_from_imu_quat  # noqa: E402

CAM_URL = os.environ.get("JAZZ_CAM_URL", "http://192.168.3.146:81/stream")
JPG_URL = os.environ.get("JAZZ_CAM_JPG", "http://192.168.3.146:81/jpg")
PORT = int(os.environ.get("JAZZ_STAB_PORT", "8181"))
CALIB = os.environ.get("JAZZ_CALIB", "/calib/cam146.json")
PITCH_TOPIC = os.environ.get("JAZZ_PITCH_TOPIC", "/jazz/pitch")
IMU_TOPIC = os.environ.get("JAZZ_IMU_TOPIC", "/jazz/imu/data")
TRIM_DEG = float(os.environ.get("JAZZ_STAB_TRIM_DEG", "0"))
PITCH_GAIN = float(os.environ.get("JAZZ_STAB_PITCH_GAIN", "1"))
STALE_S = float(os.environ.get("JAZZ_STAB_STALE_S", "0.5"))
HOLD_S = float(os.environ.get("JAZZ_STAB_HOLD_S", "2"))
HUD = os.environ.get("JAZZ_STAB_HUD", "1") not in ("0", "false", "no")

_lock = threading.Lock()
_state = {
    "pitch_rad": 0.0,
    "pitch_age_s": 1e9,
    "pitch_src": "none",
    "pitch_mode": "stale",
    "frame": None,
    "stab": None,
    "raw": None,
    "t_pitch": 0.0,
    "t_frame": 0.0,
    "ok": False,
    "err": "",
    "in_fps": 0.0,
    "out_size": (0, 0),
}


def _hud(img, pitch, applied, age, src, mode):
    if not HUD or img is None:
        return img
    out = img.copy()
    if mode == "hold":
        tag = "  HOLD (last pitch)"
    elif mode == "stale":
        tag = "  STALE passthrough"
    else:
        tag = "  live"
    lines = [
        "jazz cam stab  (IMU pitch, not vision)",
        f"pitch {math.degrees(pitch):+.1f} deg   applied {math.degrees(applied):+.1f} deg",
        f"src {src}  age {age:.2f}s" + tag,
    ]
    y = 22
    for ln in lines:
        cv2.putText(out, ln, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, ln, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1, cv2.LINE_AA)
        y += 22
    return out


def _grab_loop():
    stab = JazzPitchStabilizer(calib_path=CALIB, trim_deg=TRIM_DEG, pitch_gain=PITCH_GAIN)
    cap = None
    last_ok = time.time()
    frames = 0
    t_fps = time.time()
    while True:
        if cap is None or not cap.isOpened():
            if cap is not None:
                cap.release()
            cap = cv2.VideoCapture(CAM_URL)
            if not cap.isOpened():
                with _lock:
                    _state["err"] = f"cannot open {CAM_URL}"
                    _state["ok"] = False
                time.sleep(1.0)
                cap = None
                continue
        ok, raw = cap.read()
        if not ok or raw is None:
            # Fallback: pull a still from /jpg
            try:
                import urllib.request
                with urllib.request.urlopen(JPG_URL, timeout=3) as r:
                    buf = np.frombuffer(r.read(), dtype=np.uint8)
                raw = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                ok = raw is not None
            except Exception as e:
                ok, raw = False, None
                with _lock:
                    _state["err"] = f"frame grab failed: {type(e).__name__}"
            if not ok:
                cap.release()
                cap = None
                time.sleep(0.4)
                continue
        now = time.time()
        with _lock:
            pitch = _state["pitch_rad"]
            age = now - _state["t_pitch"] if _state["t_pitch"] else 1e9
            src = _state["pitch_src"]
        mode = pitch_freshness(age, STALE_S, HOLD_S)
        if mode == "stale":
            warped = raw
            applied = 0.0
        else:
            # live and HOLD both apply the last good pitch.
            warped = stab.warp(raw, pitch)
            applied = stab.applied_rad()
        vis = _hud(warped, pitch, applied, age, src, mode)
        frames += 1
        if now - t_fps >= 1.0:
            fps = frames / (now - t_fps)
            frames = 0
            t_fps = now
        else:
            fps = None
        with _lock:
            _state["frame"] = vis
            _state["raw"] = raw
            _state["stab"] = warped
            _state["t_frame"] = now
            _state["ok"] = True
            _state["err"] = ""
            _state["pitch_age_s"] = age
            _state["pitch_mode"] = mode
            _state["out_size"] = (int(vis.shape[1]), int(vis.shape[0]))
            if fps is not None:
                _state["in_fps"] = fps
        last_ok = now
        # If the stream stalls, reopen.
        if now - last_ok > 5:
            cap.release()
            cap = None


def _encode_jpg(img, quality=80) -> bytes:
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return b""
    return buf.tobytes()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def _send(self, code, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/health"):
            with _lock:
                s = dict(_state)
            body = json.dumps(
                {
                    "ok": s["ok"],
                    "err": s["err"],
                    "pitch_deg": math.degrees(s["pitch_rad"]),
                    "applied_deg": (
                        math.degrees(s["pitch_rad"]) * PITCH_GAIN + TRIM_DEG
                        if s.get("pitch_mode", "stale") != "stale"
                        else 0.0
                    ),
                    "pitch_age_s": s["pitch_age_s"],
                    "pitch_src": s["pitch_src"],
                    "pitch_mode": s.get("pitch_mode", "stale"),
                    "stale": s.get("pitch_mode", "stale") == "stale",
                    "hold": s.get("pitch_mode") == "hold",
                    "stale_s": STALE_S,
                    "hold_s": HOLD_S,
                    "fps": s["in_fps"],
                    "size": s["out_size"],
                    "cam_url": CAM_URL,
                    "pitch_topic": PITCH_TOPIC,
                    "trim_deg": TRIM_DEG,
                    "pitch_gain": PITCH_GAIN,
                }
            ).encode()
            self._send(200, body, "application/json")
            return
        if path in ("/jpg", "/capture"):
            with _lock:
                frame = _state["frame"]
            if frame is None:
                self._send(503, b"no frame", "text/plain")
                return
            self._send(200, _encode_jpg(frame), "image/jpeg")
            return
        if path == "/raw.jpg":
            with _lock:
                frame = _state["raw"]
            if frame is None:
                self._send(503, b"no frame", "text/plain")
                return
            self._send(200, _encode_jpg(frame), "image/jpeg")
            return
        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    with _lock:
                        frame = _state["frame"]
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    jpg = _encode_jpg(frame)
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode() + b"\r\n\r\n")
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    time.sleep(1 / 15)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return
        self._send(404, b"try /stream /jpg /health", "text/plain")


def _ros_thread():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32
    from sensor_msgs.msg import Imu

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

    class PitchNode(Node):
        def __init__(self):
            super().__init__("jazz_cam_stab")
            # Best-effort: matches reliable Jazz today and best-effort after the QoS flash.
            self.create_subscription(Float32, PITCH_TOPIC, self._on_pitch, qos)
            self.create_subscription(Imu, IMU_TOPIC, self._on_imu, qos)
            self.get_logger().info(
                f"pitch {PITCH_TOPIC} (rad)  cam {CAM_URL}  preview :{PORT}  gain {PITCH_GAIN} trim {TRIM_DEG}"
            )

        def _on_pitch(self, msg: Float32):
            with _lock:
                _state["pitch_rad"] = float(msg.data)
                _state["t_pitch"] = time.time()
                _state["pitch_src"] = PITCH_TOPIC

        def _on_imu(self, msg: Imu):
            # Fallback only if /jazz/pitch is stale.
            with _lock:
                age = time.time() - _state["t_pitch"] if _state["t_pitch"] else 1e9
                if age < STALE_S and _state["pitch_src"] == PITCH_TOPIC:
                    return
            q = msg.orientation
            p = pitch_from_imu_quat(q.x, q.y, q.z, q.w)
            with _lock:
                # Don't overwrite a fresh /jazz/pitch sample.
                age = time.time() - _state["t_pitch"] if _state["t_pitch"] else 1e9
                if age < STALE_S and _state["pitch_src"] == PITCH_TOPIC:
                    return
                _state["pitch_rad"] = p
                _state["t_pitch"] = time.time()
                _state["pitch_src"] = IMU_TOPIC

    rclpy.init()
    node = PitchNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    threading.Thread(target=_grab_loop, name="grab", daemon=True).start()
    threading.Thread(target=_ros_thread, name="ros", daemon=True).start()
    httpd = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"jazz_cam_stab preview http://0.0.0.0:{PORT}/stream  /jpg  /health", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
