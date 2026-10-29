#!/usr/bin/env python3
"""Closed-loop straight pulse holding ESP yaw from /imu/data (NOT DIAG UDP telem).

Why not DIAG telem: last furniture incident — DIAG samples lagged for seconds
(stuck ~7°) while ang saturated, then jumped to -103°; 25° abort fired late.

ESP already packs the same yaw into /imu/data orientation (~20–40 Hz) and gz
into angular_velocity.z. Hold on that ROS topic with freshness checks.

Safety (always):
  - wrap-safe yaw error in (-180, 180]
  - if IMU sample age > 150 ms → cmd zero + abort
  - if |yaw error| > 20° → cmd zero + abort
  - |angular| cap 0.08, both signs
  - optional DIAG jsonl cross-check (abort if fresh DIAG disagrees badly)

Requires --drive to publish motion. Without --drive: dry-run (no cmd_vel / session).

Usage (brain stopped so explore cannot fight):
  docker stop rover-brain
  docker run --rm --network host -v ... ros:jazzy-ros-base \\
    bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /pulse.py --drive'
  # or: python3 straight_yaw_hold_pulse.py --drive -d 14 -s 0.16
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, UInt32

DIAG_YAW_RE = re.compile(r"yaw=([-\d.]+)")


def wrap_deg(a: float) -> float:
    """Wrap degrees to (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    if a <= -180.0:
        a += 360.0
    return a


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_deg_from_quat(x: float, y: float, z: float, w: float) -> float | None:
    n = x * x + y * y + z * z + w * w
    if n < 0.25:
        return None
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


@dataclass
class YawSample:
    yaw_deg: float
    mono: float
    source: str  # "imu_quat" | "imu_integ"


class DiagCrossCheck:
    """Optional slow DIAG telem cross-check (not used for control)."""

    def __init__(self, telem_dir: str | None) -> None:
        self.enabled = bool(telem_dir)
        self.yaw: float | None = None
        self.mono: float | None = None
        self._path: Path | None = None
        self._pos = 0
        if not self.enabled:
            return
        root = Path(telem_dir)
        paths = sorted(root.glob("2026-10-*.jsonl")) or sorted(root.glob("*.jsonl"))
        if not paths:
            self.enabled = False
            return
        self._path = paths[-1]
        self._pos = self._path.stat().st_size

    def poll(self) -> None:
        if not self.enabled or self._path is None:
            return
        root = self._path.parent
        paths = sorted(root.glob("2026-10-*.jsonl")) or sorted(root.glob("*.jsonl"))
        if paths and paths[-1] != self._path:
            self._path = paths[-1]
            self._pos = 0
        with self._path.open("rb") as f:
            f.seek(self._pos)
            chunk = f.read()
            self._pos = f.tell()
        if not chunk:
            return
        now = time.monotonic()
        for line in chunk.decode("utf-8", "replace").splitlines():
            if not line.strip():
                continue
            try:
                o = json.loads(line)
            except json.JSONDecodeError:
                continue
            L = str(o.get("line", ""))
            if not L.startswith("DIAG "):
                continue
            m = DIAG_YAW_RE.search(L)
            if not m:
                continue
            self.yaw = float(m.group(1))
            self.mono = now


class StraightYawHold(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("straight_yaw_hold_pulse")
        self.args = args
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self.sess_pub = self.create_publisher(Bool, "/rover/session", 10)
        self.hb_pub = self.create_publisher(UInt32, "/rover/heartbeat", 10)
        self.wl = self.wr = None
        self._hb_n = 0

        self._imu_yaw_deg: float | None = None
        self._imu_mono: float | None = None
        self._integ_yaw_rad = 0.0
        self._gz_last_mono: float | None = None
        self._use_integ = args.yaw_source == "gyro"

        self.diag = DiagCrossCheck(args.telem_dir)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 20)
        self.create_subscription(UInt32, "/rover/wheel/left_ticks", lambda m: setattr(self, "wl", int(m.data)), 10)
        self.create_subscription(UInt32, "/rover/wheel/right_ticks", lambda m: setattr(self, "wr", int(m.data)), 10)

    def _on_imu(self, msg: Imu) -> None:
        now = time.monotonic()
        gz = float(msg.angular_velocity.z)
        if self._gz_last_mono is not None:
            dt = now - self._gz_last_mono
            if 0.0 < dt < 0.25:
                self._integ_yaw_rad = wrap_pi(self._integ_yaw_rad + gz * dt)
        self._gz_last_mono = now

        if self._use_integ:
            self._imu_yaw_deg = math.degrees(self._integ_yaw_rad)
            self._imu_mono = now
            return

        q = msg.orientation
        y = yaw_deg_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        if y is None:
            # fall back to integration for this sample
            self._imu_yaw_deg = math.degrees(self._integ_yaw_rad)
            self._imu_mono = now
            return
        self._imu_yaw_deg = y
        self._imu_mono = now

    def sample(self) -> YawSample | None:
        if self._imu_yaw_deg is None or self._imu_mono is None:
            return None
        src = "imu_integ" if self._use_integ else "imu_quat"
        return YawSample(self._imu_yaw_deg, self._imu_mono, src)

    def age_ms(self) -> float:
        if self._imu_mono is None:
            return 1e9
        return (time.monotonic() - self._imu_mono) * 1000.0

    def _session(self, on: bool) -> None:
        if not self.args.drive:
            return
        b = Bool()
        b.data = on
        for _ in range(8):
            self.sess_pub.publish(b)
            self._hb_n = (self._hb_n + 1) & 0x7FFFFFFF
            hb = UInt32()
            hb.data = self._hb_n | (0x80000000 if on else 0)
            self.hb_pub.publish(hb)
            time.sleep(0.05)
            rclpy.spin_once(self, timeout_sec=0.0)

    def zero(self, n: int = 25) -> None:
        if not self.args.drive:
            return
        msg = Twist()
        for _ in range(n):
            self.pub.publish(msg)
            time.sleep(0.04)
            rclpy.spin_once(self, timeout_sec=0.0)

    def run(self) -> dict:
        args = self.args
        # Wait for IMU
        t_wait = time.monotonic() + 5.0
        while time.monotonic() < t_wait and self.sample() is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.diag.poll()
        if self.sample() is None:
            raise RuntimeError("no /imu/data yaw yet")

        if args.drive:
            self._session(True)
            time.sleep(0.3)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            self.diag.poll()

        # Freeze hold target on current IMU yaw; reset integ-relative hold
        if self._use_integ:
            self._integ_yaw_rad = 0.0
            self._gz_last_mono = None
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
            target = 0.0
        else:
            target = float(self._imu_yaw_deg)  # type: ignore[arg-type]

        yaw0 = float(self._imu_yaw_deg)  # type: ignore[arg-type]
        wl0, wr0 = self.wl, self.wr
        rows: list[dict] = []
        ang_cmds: list[float] = []
        prev_yaw = None
        prev_mono = None
        aborted = False
        abort_reason = ""
        start_wall = datetime.now(timezone.utc).isoformat()
        start_mono = time.monotonic()
        period = 1.0 / args.rate
        deadline = start_mono + args.duration
        n = 0

        while time.monotonic() < deadline:
            # Drain queued IMU samples. timeout 0 often returns without a
            # callback, which starved this loop down to ~5 Hz on a 20 Hz topic.
            for _ in range(8):
                rclpy.spin_once(self, timeout_sec=0.0)
            self.diag.poll()
            samp = self.sample()
            age = self.age_ms()

            if samp is None or age > args.max_age_ms:
                aborted = True
                abort_reason = f"imu_age_ms={age:.0f}>{args.max_age_ms}"
                self.zero(30)
                break

            yaw_now = samp.yaw_deg
            err_deg = wrap_deg(yaw_now - target)

            if abs(err_deg) > args.abort_err_deg:
                aborted = True
                abort_reason = f"|yaw_err|={abs(err_deg):.1f}>{args.abort_err_deg}"
                self.zero(30)
                break

            # Cross-check DIAG if fresh (<400ms) — disagree → abort (telem lag alone won't trip this)
            if (
                self.diag.yaw is not None
                and self.diag.mono is not None
                and (time.monotonic() - self.diag.mono) * 1000.0 <= 400.0
            ):
                diag_err = wrap_deg(self.diag.yaw - (target if not self._use_integ else (yaw0 + math.degrees(0.0))))
                # For quat mode, DIAG and IMU quat are same ESP yaw — compare absolute
                if not self._use_integ:
                    disagree = abs(wrap_deg(self.diag.yaw - yaw_now))
                    if disagree > args.diag_disagree_deg:
                        aborted = True
                        abort_reason = f"diag_imu_disagree={disagree:.1f}>{args.diag_disagree_deg}"
                        self.zero(30)
                        break

            # Symmetric P: both signs. Open-loop +DIAG drift → negative ang when err>0
            # if +ang increases ESP yaw (ROS ENU). Flip with --ang-sign -1 if needed.
            err_rad = math.radians(err_deg)
            # Damping on yaw rate so a late P correction does not grow the weave.
            yaw_rate = 0.0
            if prev_yaw is not None and prev_mono is not None:
                dt = time.monotonic() - prev_mono
                if dt > 0.02:
                    yaw_rate = math.radians(wrap_deg(yaw_now - prev_yaw)) / dt
            prev_yaw = yaw_now
            prev_mono = time.monotonic()
            ang = args.ang_sign * (-args.kp * err_rad - args.kd * yaw_rate)
            if ang > args.ang_lim:
                ang = args.ang_lim
            elif ang < -args.ang_lim:
                ang = -args.ang_lim

            if args.drive:
                msg = Twist()
                msg.linear.x = float(args.linear)
                msg.angular.z = float(ang)
                self.pub.publish(msg)
                if n % 10 == 0:
                    b = Bool()
                    b.data = True
                    self.sess_pub.publish(b)
                    self._hb_n = (self._hb_n + 1) & 0x7FFFFFFF
                    hb = UInt32()
                    hb.data = self._hb_n | 0x80000000
                    self.hb_pub.publish(hb)

            ang_cmds.append(ang)
            rows.append(
                {
                    "t": round(time.monotonic() - start_mono, 3),
                    "wl": self.wl,
                    "wr": self.wr,
                    "cmd_lin": args.linear if args.drive else 0.0,
                    "cmd_ang": round(ang, 4),
                    "imu_yaw": round(yaw_now, 2),
                    "yaw_err_deg": round(err_deg, 3),
                    "imu_age_ms": round(age, 1),
                    "diag_yaw": None if self.diag.yaw is None else round(self.diag.yaw, 2),
                    "drive": int(args.drive),
                }
            )
            n += 1
            # Keep spinning while waiting so /imu/data (~20 Hz) is not stuck
            # behind time.sleep. The 2026-10-03 hall run only saw a new yaw
            # every ~200 ms and rode the 150 ms age limit.
            wake = start_mono + n * period
            while True:
                remain = wake - time.monotonic()
                if remain <= 0:
                    break
                rclpy.spin_once(self, timeout_sec=min(0.02, remain))

        stop_wall = datetime.now(timezone.utc).isoformat()
        yaw1 = float(self._imu_yaw_deg) if self._imu_yaw_deg is not None else yaw0
        self.zero(30)
        self._session(False)
        self.zero(8)

        summary = {
            "start_utc": start_wall,
            "stop_utc": stop_wall,
            "drive": bool(args.drive),
            "yaw_source": args.yaw_source,
            "hold_topic": "/imu/data",
            "hold_field": "orientation (ESP yaw)" if args.yaw_source == "quat" else "angular_velocity.z integrate",
            "planned_s": args.duration,
            "elapsed_s": rows[-1]["t"] if rows else 0.0,
            "linear": args.linear,
            "ang_lim": args.ang_lim,
            "kp": args.kp,
            "kd": args.kd,
            "ang_sign": args.ang_sign,
            "max_age_ms": args.max_age_ms,
            "abort_err_deg": args.abort_err_deg,
            "aborted": aborted,
            "abort_reason": abort_reason,
            "wl0": wl0,
            "wr0": wr0,
            "wl1": self.wl,
            "wr1": self.wr,
            "dL": None if wl0 is None or self.wl is None else self.wl - wl0,
            "dR": None if wr0 is None or self.wr is None else self.wr - wr0,
            "yaw0": round(yaw0, 2),
            "yaw1": round(yaw1, 2),
            "dyaw": round(wrap_deg(yaw1 - yaw0), 2),
            "ang_cmd_min": round(min(ang_cmds), 4) if ang_cmds else 0.0,
            "ang_cmd_max": round(max(ang_cmds), 4) if ang_cmds else 0.0,
            "ang_cmd_mean": round(sum(ang_cmds) / len(ang_cmds), 4) if ang_cmds else 0.0,
            "samples": len(rows),
            "note": "IMU hold; DIAG telem is cross-check only; require --drive to move",
        }
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        with (out / "motion.csv").open("w", newline="") as f:
            w = csv.DictWriter(
                f,
                fieldnames=[
                    "t",
                    "wl",
                    "wr",
                    "cmd_lin",
                    "cmd_ang",
                    "imu_yaw",
                    "yaw_err_deg",
                    "imu_age_ms",
                    "diag_yaw",
                    "drive",
                ],
            )
            w.writeheader()
            w.writerows(rows)
        with (out / "summary.json").open("w") as f:
            json.dump(summary, f, indent=2)
        print("SUMMARY", json.dumps(summary), flush=True)
        return summary


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Straight yaw-hold pulse on /imu/data")
    p.add_argument("--drive", action="store_true", help="Actually publish cmd_vel/session (default: dry-run)")
    p.add_argument("-d", "--duration", type=float, default=14.0)
    p.add_argument("-s", "--linear", type=float, default=0.16)
    p.add_argument("--ang-lim", type=float, default=0.08)
    p.add_argument("--kp", type=float, default=1.0)
    p.add_argument("--kd", type=float, default=0.0, help="yaw-rate damping, rad/s")
    p.add_argument("--ang-sign", type=int, choices=(-1, 1), default=1, help="ang = sign * (-kp*err). Hall 20:56 with -1 pinned left-fast and yaw ran further negative; do not use -1")
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--max-age-ms", type=float, default=150.0)
    p.add_argument("--abort-err-deg", type=float, default=20.0)
    p.add_argument("--yaw-source", choices=("quat", "gyro"), default="quat",
                   help="quat=ESP yaw in Imu.orientation (~20Hz); gyro=integrate gz")
    p.add_argument("--telem-dir", default="", help="Optional DIAG jsonl dir for cross-check")
    p.add_argument("--diag-disagree-deg", type=float, default=35.0)
    p.add_argument("--out", default="/tmp/pulse_out")
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    if not args.telem_dir:
        args.telem_dir = None
    rclpy.init()
    node = StraightYawHold(args)
    try:
        node.run()
    finally:
        try:
            node.zero(8)
            node._session(False)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
