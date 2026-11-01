#!/usr/bin/env python3
"""Wander forward on sonar; chase rover IR beacon when TSOP sees it."""
from __future__ import annotations

import json
import math
import os
import socket
import threading
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32, UInt8

from fleet_meet_spin import MeetSpinController
from fleet_steering import IrSteerConfig, WanderConfig, WanderState, steer_ir_beacon


class ExploreThenChase(Node):
    TICK = 0.05

    def __init__(self) -> None:
        ns = os.environ.get('BOT_NS', 'bot1').strip('/')
        fleet_ids = {'rover': 0, 'bot0': 0, 'bot1': 1, 'bot2': 2, 'bot3': 3}
        fleet_id = int(os.environ.get('FLEET_IR_ID', fleet_ids.get(ns, 1)))
        pan_center = float(os.environ.get('PAN_CENTER', '90'))
        super().__init__('explore_then_chase')
        self._meet = MeetSpinController(fleet_id=fleet_id)
        self._meet_was_active = False
        self.pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        self.create_subscription(Range, f'/{ns}/sonar/range', self._on_range, qos_profile_sensor_data)
        self.create_subscription(Range, f'/{ns}/tof/rear', self._on_rear_tof, qos_profile_sensor_data)
        self.create_subscription(Imu, f'/{ns}/' + os.environ.get('IMU_TOPIC', 'imu').strip('/'), self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, f'/{ns}/ir/detected', self._on_ir, 10)
        self.create_subscription(UInt8, f'/{ns}/ir/peer', self._on_ir_peer, 10)
        self.create_subscription(Float32, f'/{ns}/sonar/pan_deg', self._on_pan, 10)
        # Side IR proximity: binary, trips at ~5 cm. Very-close only, NOT gap/range info.
        self._side = {'l': (False, 0.0), 'r': (False, 0.0)}
        self.create_subscription(Bool, f'/{ns}/side/left', lambda m: self._on_side('l', m), 10)
        self.create_subscription(Bool, f'/{ns}/side/right', lambda m: self._on_side('r', m), 10)
        self.SIDE_STEER = float(os.environ.get('SIDE_STEER', '0.5'))
        self.SIDE_SLOW = float(os.environ.get('SIDE_SLOW', '0.5'))
        self._side_log = ''
        # Per-bot hard gate on top of /fleet/allow_drive (0 = never drive).
        self._drive_enable = os.environ.get('DRIVE_ENABLE', '1').strip() not in ('0', 'false', 'off', 'no')
        self.IR_ON_TICKS = int(os.environ.get('CHASE_IR_ON_TICKS', '24'))
        self.IR_OFF_TICKS = int(os.environ.get('CHASE_IR_OFF_TICKS', '15'))
        self.IR_OFF_TICKS_CHASE = int(os.environ.get('CHASE_IR_OFF_TICKS_ACTIVE', '4'))
        self.CHASE_MAX_SEC = float(os.environ.get('CHASE_MAX_SEC', '10.0'))
        self.CHASE_COOLDOWN_SEC = float(os.environ.get('CHASE_COOLDOWN_SEC', '6.0'))
        wander_cfg = WanderConfig.from_env(pan_center=pan_center)
        self._wander = WanderState.from_config(wander_cfg)
        self._wander._heading.gyro_sign = float(os.environ.get('WANDER_GYRO_SIGN', '-1'))
        self._wander._heading.cal_s = float(os.environ.get('IMU_CAL_SEC', '2.5'))
        self._wander._heading.cal_min_samples = int(os.environ.get('IMU_CAL_MIN_SAMPLES', '50'))
        self._ir_cfg = IrSteerConfig(
            cruise=float(os.environ.get('CHASE_CRUISE', '0.24')),
            pan_center=pan_center,
            spin_rate=float(os.environ.get('CHASE_SPIN_RATE', '0.45')),
            align_creep=float(os.environ.get('CHASE_ALIGN_CREEP', '0.14')),
            pan_align_deg=float(os.environ.get('CHASE_PAN_ALIGN_DEG', '28')),
        )
        self._ir = False
        self._ir_on_streak = 0
        self._ir_off_streak = 0
        self._pan_deg = pan_center
        self._notify_host = os.environ.get('ROVER_NOTIFY_HOST', 'lego-rover.local')
        self._notify_port = int(os.environ.get('ROVER_NOTIFY_PORT', '4242'))
        self._mode = 'wander'
        self.CMD_ANG_SCALE = float(os.environ.get('CMD_ANG_SCALE', '1.0'))
        self.CMD_LIN_SCALE = float(os.environ.get('CMD_LIN_SCALE', '1.0'))
        # Person follow (camera, via detector /latest). Obstacle checks still apply: apply_sonar_stop + _gate_now.
        self.FOLLOW = os.environ.get('PERSON_FOLLOW', '0').strip().lower() in ('1', 'true', 'on', 'yes')
        self.FOLLOW_CAM = os.environ.get('FOLLOW_CAM', ns)
        self.FOLLOW_URL = os.environ.get('DET_URL', 'http://127.0.0.1:8765').rstrip('/')
        self.FOLLOW_CONF = float(os.environ.get('FOLLOW_CONF', '0.45'))
        self.FOLLOW_STOP_HFRAC = float(os.environ.get('FOLLOW_STOP_HFRAC', '0.60'))  # bbox h / frame h at ~1 m (tune)
        self.FOLLOW_CRUISE = float(os.environ.get('FOLLOW_CRUISE', '0.16'))
        self.FOLLOW_K_ANG = float(os.environ.get('FOLLOW_K_ANG', '1.2'))
        self.FOLLOW_MAX_ANG = float(os.environ.get('FOLLOW_MAX_ANG', '0.5'))
        self.FOLLOW_ALIGN = float(os.environ.get('FOLLOW_ALIGN', '0.25'))  # |x err| above this: turn only
        self.FOLLOW_LOST_S = float(os.environ.get('FOLLOW_LOST_S', '3.0'))
        self.FOLLOW_STALE_S = float(os.environ.get('FOLLOW_STALE_S', '1.0'))
        self._person = None          # (x_err -0.5..0.5, h_frac, t_mono)
        self._person_lock = threading.Lock()
        if self.FOLLOW:
            threading.Thread(target=self._det_loop, daemon=True).start()
        self._chase_started_at: float | None = None
        self._chase_cooldown_until = 0.0
        self._drive_log_at = 0.0
        self._drive_csv = os.environ.get('DRIVE_CSV', '').strip()
        self._csv_ready = False
        self._last_gz = 0.0
        self._imu_seen = False
        self._require_imu = os.environ.get('REQUIRE_IMU', '1').strip().lower() not in (
            '0',
            'false',
            'no',
            'off',
        )
        self._allow_drive = True
        self.create_subscription(Bool, '/fleet/allow_drive', self._on_allow_drive, 10)
        if not self._require_imu:
            self._wander._heading.force_ready()
        self._cal_log_at = 0.0
        self._gate_reason: str | None = None
        self._gate_log_at = 0.0
        self._cal_done_logged = False
        # Wheel shaping: firmware mixes wheel = lin -/+ ang (normalised, 1.0 = 400 us).
        self.MAX_FWD = float(os.environ.get("MAX_FWD", "0.18"))
        self.WHEEL_MIN = float(os.environ.get("WHEEL_MIN", "0.06"))
        # Spin watchdog
        self.WD_FWD_GZ = float(os.environ.get("WD_FWD_GZ", "0.8"))
        self.WD_FWD_S = float(os.environ.get("WD_FWD_S", "2.0"))
        self.WD_TURN_ANG = float(os.environ.get("WD_TURN_ANG", "0.15"))
        self.WD_TURN_GZ = float(os.environ.get("WD_TURN_GZ", "0.1"))
        self.WD_TURN_S = float(os.environ.get("WD_TURN_S", "1.5"))
        self.WD_HOLD_S = float(os.environ.get("WD_HOLD_S", "3.0"))
        self._wd_fwd_since = None
        self._wd_turn_since = None
        self._wd_hold_until = 0.0
        self.create_timer(self.TICK, self._tick)
        self.get_logger().info(
            f'explore_then_chase: /{ns} forward wander → chase on IR'
            + ('' if self._require_imu else ' (no IMU gate)')
        )

    def _det_loop(self) -> None:
        url = f'{self.FOLLOW_URL}/latest?cam={self.FOLLOW_CAM}'
        while rclpy.ok():
            try:
                with urllib.request.urlopen(url, timeout=1.0) as r:
                    d = json.load(r)
                w, h = float(d.get('w') or 0), float(d.get('h') or 0)
                age = time.time() - float(d.get('ts') or 0)  # PC clock
                best = None
                if w > 0 and h > 0 and age < self.FOLLOW_STALE_S:
                    for det in d.get('dets') or []:
                        if det.get('cls') != 'person' or float(det.get('conf', 0)) < self.FOLLOW_CONF:
                            continue
                        x1, y1, x2, y2 = det['bbox']
                        hf = (y2 - y1) / h
                        if best is None or hf > best[1]:
                            best = ((x1 + x2) / 2.0 / w - 0.5, hf)
                if best is not None:
                    with self._person_lock:
                        self._person = (best[0], best[1], time.monotonic())
            except Exception:
                pass
            time.sleep(0.1)

    def _person_now(self):
        with self._person_lock:
            p = self._person
        if p is None or time.monotonic() - p[2] > self.FOLLOW_LOST_S:
            return None, False
        return p, time.monotonic() - p[2] < 0.5   # (last seen, seen-fresh)

    def _on_range(self, msg: Range) -> None:
        # No-target (>= max_range) counts as clear only after 3 in a row and 0.5 s after a real reading < 0.3 m.
        r = float(msg.range); now = time.monotonic()
        mx = float(msg.max_range) if msg.max_range > 0 else 2.0
        if r >= mx - 1e-3:
            self._nt_streak = getattr(self, "_nt_streak", 0) + 1
            if self._nt_streak < 3 or now - getattr(self, "_last_close", -1e9) < 0.5:
                r = float("nan")
        else:
            self._nt_streak = 0
            if 0 < r < 0.3:
                self._last_close = now
        self._wander.update_range(r, self._pan_deg)

    def _on_rear_tof(self, msg: Range) -> None:
        self._wander.update_rear(float(msg.range))

    def _on_imu(self, msg: Imu) -> None:
        self._imu_seen = True
        self._last_gz = float(msg.angular_velocity.z)
        self._meet.on_imu_gz(self._last_gz, self.TICK)
        self._wander.update_imu(self._last_gz)

    def _on_ir_peer(self, msg: UInt8) -> None:
        self._meet.on_peer(int(msg.data))

    def _on_ir(self, msg: Bool) -> None:
        detected = bool(msg.data)
        if detected:
            self._ir_on_streak += 1
            self._ir_off_streak = 0
        else:
            self._ir_off_streak += 1
            self._ir_on_streak = 0

        prev = self._ir
        off_need = self.IR_OFF_TICKS_CHASE if self._ir else self.IR_OFF_TICKS
        if self._ir_on_streak >= self.IR_ON_TICKS:
            self._ir = True
        elif self._ir_off_streak >= off_need:
            self._ir = False

        if self._ir and not prev:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.sendto(
                        b'beacon_lock',
                        (self._notify_host, self._notify_port),
                    )
            except OSError as exc:
                self.get_logger().warn(f'beacon notification failed: {exc}')

    def _on_side(self, key: str, msg: Bool) -> None:
        self._side[key] = (bool(msg.data), time.monotonic())

    def _side_now(self, key: str) -> bool:
        hit, at = self._side[key]
        return hit and time.monotonic() - at < 0.5

    def _apply_side(self, msg: Twist) -> None:
        """Steer off a side wall; pick the open side for turns; slow in tight corridors."""
        left, right = self._side_now('l'), self._side_now('r')
        tag = ('L' if left else '-') + ('R' if right else '-')
        if tag != self._side_log:
            self._side_log = tag
            self.get_logger().info(f'side {tag}')
        if not (left or right):
            return
        if left and right:
            msg.linear.x *= self.SIDE_SLOW
            return
        away = -1.0 if left else 1.0  # +z turns left
        if abs(msg.linear.x) < 0.03 and abs(msg.angular.z) > 0.02:
            # Turning on the spot: turn towards the open side (gap choice).
            msg.angular.z = away * abs(msg.angular.z)
        elif msg.linear.x > 0.0:
            msg.angular.z += away * self.SIDE_STEER
            msg.linear.x *= self.SIDE_SLOW

    def _on_pan(self, msg: Float32) -> None:
        self._pan_deg = float(msg.data)
        self._wander._pan_deg = self._pan_deg

    def _on_allow_drive(self, msg: Bool) -> None:
        allow = bool(msg.data)
        if allow == self._allow_drive:
            return
        self._allow_drive = allow
        self.get_logger().info(f'allow_drive={allow}')

    def _publish(self, msg: Twist, mode: str) -> None:
        """Gate + publish (meet path)."""
        self._gate_now(msg, mode)
        self.pub.publish(msg)

    def _shape_wheels(self, msg: Twist) -> None:
        """Never raises forward. Avoids a stalled/reversing inside wheel on arcs."""
        lin = min(float(msg.linear.x), self.MAX_FWD)
        ang = float(msg.angular.z)
        m = self.WHEEL_MIN
        if lin > 0.0:
            if lin < m:
                lin = 0.0  # too slow to roll cleanly: drop forward (pivot or stop)
            elif abs(ang) > lin:
                lin = 0.0  # inside wheel would reverse: do a clean pivot instead
            elif lin - abs(ang) < m:
                # inside wheel below its minimum: straighten until it is at the minimum
                ang = math.copysign(max(0.0, lin - m), ang) if ang else 0.0
        if lin == 0.0 and ang != 0.0 and abs(ang) < m:
            ang = math.copysign(m, ang) if abs(ang) >= m / 2 else 0.0
        msg.linear.x = lin
        msg.angular.z = ang

    def _watchdog(self, msg: Twist, mode: str) -> None:
        now = time.monotonic()
        lin, ang, gz = float(msg.linear.x), float(msg.angular.z), abs(self._last_gz)
        if now < self._wd_hold_until:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            return
        fwd = lin > 0.0 and abs(ang) < 0.3 and gz > self.WD_FWD_GZ
        self._wd_fwd_since = (self._wd_fwd_since or now) if fwd else None
        turn = abs(lin) < 0.02 and abs(ang) >= self.WD_TURN_ANG and gz < self.WD_TURN_GZ
        self._wd_turn_since = (self._wd_turn_since or now) if turn else None
        why = None
        if self._wd_fwd_since and now - self._wd_fwd_since > self.WD_FWD_S:
            why = f"cmd mostly forward (lin={lin:+.2f} ang={ang:+.2f}) but IMU |gz|={gz:.2f}>{self.WD_FWD_GZ} for >{self.WD_FWD_S}s"
        elif self._wd_turn_since and now - self._wd_turn_since > self.WD_TURN_S:
            why = f"cmd turn ang={ang:+.2f} but IMU |gz|={gz:.2f}<{self.WD_TURN_GZ} for >{self.WD_TURN_S}s"
        if why:
            self.get_logger().error(f"spin watchdog [{mode}]: motion/IMU mismatch: {why}; zero for {self.WD_HOLD_S}s")
            self._wd_hold_until = now + self.WD_HOLD_S
            self._wd_fwd_since = self._wd_turn_since = None
            msg.linear.x = 0.0
            msg.angular.z = 0.0

    def _gate_now(self, msg: Twist, mode: str) -> None:
        """Forward gate for every mode (exactly one call per tick)."""
        zeroed = self._wander.gate_forward(msg) is not None
        self._shape_wheels(msg)
        self._watchdog(msg, mode)
        reason = self._wander.forward_block_reason()
        now = time.monotonic()
        cat = reason.split(' ')[0:3] if reason else None
        if cat != self._gate_reason or (reason and now >= self._gate_log_at):
            if reason:
                act = 'zeroed forward cmd' if zeroed else 'forward held at 0'
                self.get_logger().warn(f'forward gate [{mode}]: {act}, {reason}')
                self._gate_log_at = now + 2.0
            elif self._gate_reason:
                self.get_logger().info(f'forward gate [{mode}]: clear')
            self._gate_reason = cat

    def _tick(self) -> None:
        if not self._allow_drive or not self._drive_enable:
            self.pub.publish(Twist())
            return
        meet_cmd = self._meet.tick()
        if meet_cmd is not None:
            if not self._meet_was_active:
                self.get_logger().info(
                    f'meet dance — saw {self._meet.peer_label()}, spinning 2×360°'
                )
            self._meet_was_active = True
            msg = Twist()
            msg.linear.x, msg.angular.z = meet_cmd
            self._publish(msg, 'meet')
            return
        if self._meet_was_active:
            self._meet_was_active = False
            self.get_logger().info('meet dance done')
        now = time.monotonic()
        if self._require_imu and not self._imu_seen:
            if now >= self._gate_log_at:
                self._gate_log_at = now + 5.0
                self.get_logger().info('hold: no IMU yet (lin=0, ang=0)')
            self.pub.publish(Twist())
            return

        if self._require_imu and not self._wander.imu_ready():
            if now >= self._cal_log_at:
                self._cal_log_at = now + 1.0
                pct = int(self._wander.imu_cal_progress() * 100.0)
                self.get_logger().info(
                    f'IMU calibrating — hold still ({pct}%, '
                    f'{self._wander._heading.cal_s:.0f}s)'
                )
            self.pub.publish(Twist())
            return

        if not self._cal_done_logged:
            self._cal_done_logged = True
            self.get_logger().info('exploring')

        chasing = self._ir and time.monotonic() >= self._chase_cooldown_until
        if chasing and self._chase_started_at is None:
            self._chase_started_at = time.monotonic()
        if chasing and self._chase_started_at is not None:
            if time.monotonic() - self._chase_started_at >= self.CHASE_MAX_SEC:
                chasing = False
                self._ir = False
                self._ir_on_streak = 0
                self._ir_off_streak = self.IR_OFF_TICKS
                self._chase_started_at = None
                self._chase_cooldown_until = time.monotonic() + self.CHASE_COOLDOWN_SEC
                self.get_logger().info(
                    f'chase timeout ({self.CHASE_MAX_SEC:.0f}s) — back to wander'
                )
        if not chasing:
            self._chase_started_at = None

        person, person_fresh = (self._person_now() if (self.FOLLOW and not chasing) else (None, False))
        mode = 'chase' if chasing else ('follow' if person is not None else 'wander')
        if mode != self._mode:
            self._mode = mode
            self.get_logger().info(f'mode → {mode}')
            if mode != 'wander':
                self._wander._heading.release()
            else:
                self._wander._heading.reset_target()

        if chasing:
            msg = Twist()
            steer_ir_beacon(
                msg,
                pan_deg=self._pan_deg,
                ir_now=self._ir_on_streak >= max(2, self.IR_ON_TICKS // 4),
                cfg=self._ir_cfg,
            )
            self._wander.apply_sonar_stop(msg)
            self._wander._last_driving = msg.linear.x > 0.05
        elif mode == 'follow':
            msg = Twist()
            xe, hf, _ = person
            if person_fresh:
                msg.angular.z = max(-self.FOLLOW_MAX_ANG, min(self.FOLLOW_MAX_ANG, -self.FOLLOW_K_ANG * xe))
                if abs(xe) < self.FOLLOW_ALIGN:
                    msg.linear.x = self.FOLLOW_CRUISE * max(0.0, min(1.0, (self.FOLLOW_STOP_HFRAC - hf) / 0.25))
            # not fresh (briefly lost): hold still until FOLLOW_LOST_S, then wander
            self._wander.apply_sonar_stop(msg)   # obstacle stop ALWAYS (plus _gate_now below)
            self._wander._last_driving = msg.linear.x > 0.05
        else:
            msg = Twist()
            self._wander.fill_twist(msg)
            self._apply_side(msg)
            if self._wander.consume_burst_started():
                self.get_logger().info(
                    f'sprint burst {self._wander.burst_speed:.0%} for '
                    f'{self._wander.burst_duration_s:.1f}s'
                )

        self._gate_now(msg, mode)  # single gate call; logs/CSV show what is sent
        now = time.monotonic()
        if now >= self._drive_log_at:
            self._drive_log_at = now + 2.0
            rng = self._wander.debug_range_m()
            rng_s = 'nan' if math.isnan(rng) else f'{rng:.2f}'
            self.get_logger().info(
                f'drive lin={msg.linear.x:+.2f} ang={msg.angular.z:+.3f} '
                f'trim={self._wander._heading.trim_ang:+.3f} trap={self._wander.trap_level()} stalls={getattr(self._wander, "turn_stalls", 0)} '
                f'mode={mode} rng={rng_s}m pan={self._pan_deg:.0f}'
            )

        if self._drive_csv:
            rng = self._wander.debug_range_m()
            rng_s = '' if math.isnan(rng) else f'{rng:.3f}'
            if not self._csv_ready:
                with open(self._drive_csv, 'w', encoding='utf-8') as f:
                    f.write('t,lin,ang,gz,range_m,mode,pan,trap\n')
                self._csv_ready = True
            with open(self._drive_csv, 'a', encoding='utf-8') as f:
                f.write(
                    f'{time.time():.3f},{msg.linear.x:.4f},{msg.angular.z:.4f},'
                    f'{self._last_gz:.5f},{rng_s},{mode},{self._pan_deg:.0f},'
                    f'{self._wander.trap_level()}\n'
                )

        msg.angular.z *= self.CMD_ANG_SCALE  # firmware takes rad/s; brain ang is ~power
        msg.linear.x *= self.CMD_LIN_SCALE  # after all gates: scaling never turns a zero into motion
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ExploreThenChase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for _ in range(5):
            node.pub.publish(stop)
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
