#!/usr/bin/env python3
"""Log front sonar escape scans + parallel rear IR + turn outcome (JSONL)."""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32, UInt8

from rover_qos import CMD_VEL_QOS, TELEMETRY_QOS
from rover_radar import background_instance, clearest_gap_led, scan_hits
from rover_scan_map import led_to_bearing
from rover_twist import wire_to_logical


def _pan_center() -> float:
    return float(os.environ.get('ROVER_SONAR_PAN_CENTER', '90'))


def _ring_count() -> int:
    return int(os.environ.get('ROVER_RING_COUNT', '8'))


def _rear_scan_enabled() -> bool:
    return os.environ.get('ROVER_SONAR_REAR_SCAN', '1').strip().lower() not in (
        '0',
        'no',
        'false',
    )


class SonarEscapeRecorder(Node):
    def __init__(self, out_dir: Path) -> None:
        super().__init__('rover_sonar_escape_recorder')
        self.out_dir = out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.events_path = out_dir / 'escape_events.jsonl'
        self.stream_path = out_dir / 'stream.jsonl'
        self._events_fp = self.events_path.open('a', encoding='utf-8')
        self._stream_fp = self.stream_path.open('a', encoding='utf-8')
        self._lock = threading.Lock()
        self._last_stream_t = 0.0

        self._center = _pan_center()
        self._pan = self._center
        self._range_m = math.nan
        self._avoid = False
        self._yaw_deg = 0.0
        self._last_gz = 0.0
        self._last_imu_t: float | None = None

        self._scan_active = False
        self._scan_bins: list[dict] = []
        self._scan_started = 0.0
        self._rear_thread: threading.Thread | None = None
        self._rear_result: dict | None = None
        self._pending_follow: list[dict] = []
        self._event_seq = 0
        self._escape_open = False
        self._finalized_escape = False

        self._esp_turn = 0.0
        self._esp_best_deg = self._center
        self._esp_best_m = 0.0
        self._esp_event_id = 0

        self.create_subscription(Range, '/rover/sonar/range', self._on_range, 10)
        self.create_subscription(Float32, '/rover/sonar/pan_deg', self._on_pan, 10)
        self.create_subscription(Bool, '/rover/sonar/avoid_active', self._on_avoid, 10)
        self.create_subscription(UInt8, '/rover/sonar/escape_event', self._on_escape_event, 10)
        self.create_subscription(Float32, '/rover/sonar/escape_turn', self._on_escape_turn, 10)
        self.create_subscription(Float32, '/rover/sonar/escape_best_deg', self._on_escape_best_deg, 10)
        self.create_subscription(Float32, '/rover/sonar/escape_best_m', self._on_escape_best_m, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, TELEMETRY_QOS)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, CMD_VEL_QOS)
        self.create_timer(0.1, self._tick_followups)

        self.get_logger().info(f'Sonar escape recorder → {self.out_dir}')

    def _write(self, fp, row: dict) -> None:
        with self._lock:
            fp.write(json.dumps(row, separators=(',', ':')) + '\n')
            fp.flush()

    def _on_range(self, msg: Range) -> None:
        self._range_m = float(msg.range)
        self._stream_sample('range', {'range_m': self._range_m, 'pan_deg': self._pan})
        if self._scan_active and not math.isnan(self._range_m) and self._range_m > 0.0:
            self._scan_bins.append(
                {'t': time.monotonic(), 'pan_deg': round(self._pan, 1), 'range_m': round(self._range_m, 3)}
            )

    def _on_pan(self, msg: Float32) -> None:
        self._pan = float(msg.data)
        self._stream_sample('pan', {'pan_deg': self._pan})
        off = abs(self._pan - self._center)
        if off > 18.0 and not self._scan_active:
            self._begin_scan()
        if self._scan_active and off < 6.0 and (time.monotonic() - self._scan_started) > 1.0:
            self._scan_active = False

    def _on_avoid(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active and not self._avoid:
            self._escape_open = True
            self._finalized_escape = False
            self._begin_scan()
        elif not active and self._avoid and self._escape_open and not self._finalized_escape:
            if self._esp_event_id == 0:
                self._event_seq = (self._event_seq + 1) % 255
                self._esp_event_id = self._event_seq or 1
            self._finalize_event()
        self._avoid = active
        self._stream_sample('avoid', {'active': active})

    def _on_escape_turn(self, msg: Float32) -> None:
        self._esp_turn = float(msg.data)

    def _on_escape_best_deg(self, msg: Float32) -> None:
        self._esp_best_deg = float(msg.data)

    def _on_escape_best_m(self, msg: Float32) -> None:
        self._esp_best_m = float(msg.data)

    def _on_escape_event(self, msg: UInt8) -> None:
        eid = int(msg.data)
        if eid > 0:
            self._esp_event_id = eid
        if self._escape_open and not self._finalized_escape:
            self._finalize_event()

    def _on_imu(self, msg: Imu) -> None:
        gz = float(msg.angular_velocity.z)
        now = time.monotonic()
        if self._last_imu_t is not None:
            dt = now - self._last_imu_t
            if dt > 0.0:
                self._yaw_deg += math.degrees(gz * dt)
        self._last_imu_t = now
        self._last_gz = gz

    def _on_cmd(self, msg: Twist) -> None:
        lin, ang = wire_to_logical(float(msg.linear.x), float(msg.angular.z))
        self._stream_sample('cmd', {'lin': lin, 'ang': ang})

    def _stream_sample(self, kind: str, payload: dict) -> None:
        now = time.monotonic()
        if kind in ('range', 'pan') and not self._scan_active:
            if now - self._last_stream_t < 0.5:
                return
        self._last_stream_t = now
        row = {'t': now, 'kind': kind, **payload}
        self._write(self._stream_fp, row)

    def _begin_scan(self) -> None:
        if self._scan_active:
            return
        self._scan_active = True
        self._scan_bins = []
        self._scan_started = time.monotonic()
        self._rear_result = None
        if _rear_scan_enabled():
            self._start_rear_scan()

    def _start_rear_scan(self) -> None:
        if self._rear_thread is not None and self._rear_thread.is_alive():
            return

        def _run() -> None:
            bg = background_instance()
            if bg is not None:
                bg.pause()
            try:
                hits = scan_hits(update_ring=False, ping=False)
                n = _ring_count()
                gap_led = clearest_gap_led(hits) if hits else -1
                bearing = led_to_bearing(gap_led, n) if gap_led >= 0 else -1.0
                self._rear_result = {
                    'hits': hits,
                    'gap_led': gap_led,
                    'gap_bearing_deg': bearing,
                }
            except Exception as exc:
                self._rear_result = {'error': str(exc)}
            finally:
                if bg is not None:
                    bg.resume()

        self._rear_thread = threading.Thread(target=_run, name='sonar-rear-scan', daemon=True)
        self._rear_thread.start()

    def _finalize_event(self) -> None:
        if self._finalized_escape:
            return
        self._finalized_escape = True
        self._escape_open = False
        if self._rear_thread is not None and self._rear_thread.is_alive():
            self._rear_thread.join(timeout=6.0)

        front_bins = list(self._scan_bins)
        front_best = self._best_front_bin(front_bins)
        row = {
            't': time.monotonic(),
            'event_id': self._esp_event_id,
            'esp_turn_sign': self._esp_turn,
            'esp_best_deg': self._esp_best_deg,
            'esp_best_m': self._esp_best_m,
            'front_bins': front_bins,
            'front_best': front_best,
            'rear': self._rear_result,
            'yaw_deg_at_event': round(self._yaw_deg, 2),
            'pan_deg_at_event': round(self._pan, 1),
            'range_m_at_event': self._range_m,
        }
        self._write(self._events_fp, row)
        self.get_logger().info(
            f'escape #{self._esp_event_id}: esp_turn={self._esp_turn:+.0f} '
            f'best={self._esp_best_deg:.0f}° {self._esp_best_m:.2f}m '
            f'front_bins={len(front_bins)} rear={"ok" if self._rear_result else "none"}'
        )
        self._pending_follow.append(
            {
                't0': time.monotonic(),
                'event_id': self._esp_event_id,
                'yaw0': self._yaw_deg,
                'esp_turn': self._esp_turn,
                'front_best': front_best,
            }
        )
        self._scan_active = False
        self._scan_bins = []

    @staticmethod
    def _best_front_bin(bins: list[dict]) -> dict | None:
        best = None
        for b in bins:
            r = b.get('range_m', -1.0)
            if r <= 0.0:
                continue
            if best is None or r > best['range_m']:
                best = dict(b)
        return best

    def _tick_followups(self) -> None:
        now = time.monotonic()
        keep: list[dict] = []
        for item in self._pending_follow:
            age = now - item['t0']
            if age < 2.5:
                keep.append(item)
                continue
            dyaw = self._yaw_deg - item['yaw0']
            while dyaw > 180.0:
                dyaw -= 360.0
            while dyaw < -180.0:
                dyaw += 360.0
            turn_obs = 'left' if dyaw > 8 else 'right' if dyaw < -8 else 'straight'
            esp = 'left' if item['esp_turn'] > 0 else 'right' if item['esp_turn'] < 0 else 'straight'
            front_best = item.get('front_best')
            want = None
            if front_best:
                center = _pan_center()
                pan = front_best['pan_deg']
                want = 'left' if pan > center + 5 else 'right' if pan < center - 5 else 'straight'
            verdict = 'unknown'
            if want and turn_obs != 'straight':
                verdict = 'ok' if want == turn_obs else 'WRONG'
            row = {
                't': now,
                'event_id': item['event_id'],
                'followup_s': round(age, 2),
                'imu_yaw_delta_deg': round(dyaw, 1),
                'turn_observed': turn_obs,
                'esp_turn': esp,
                'front_best_want': want,
                'verdict': verdict,
            }
            self._write(self._events_fp, row)
            self.get_logger().info(
                f'followup #{item["event_id"]}: imu {turn_obs} esp {esp} want {want} → {verdict}'
            )
        self._pending_follow = keep

    def close(self) -> None:
        self._events_fp.close()
        self._stream_fp.close()


def main() -> None:
    out = Path(os.environ.get('ROVER_SONAR_RECORD_DIR', '')).expanduser()
    if not str(out):
        out = Path.home() / f'rover_sonar_escape_{time.strftime("%Y%m%d_%H%M%S")}'

    rclpy.init()
    node = SonarEscapeRecorder(out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'Recording saved under {out}')
    print(f'Analyze: python3 analyze_sonar_escape.py {out}')


if __name__ == '__main__':
    main()
