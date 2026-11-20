#!/usr/bin/env python3
"""Autonomous rover: wander, cruise, escape on bump/stall."""
import math
import os
import random
import signal
import time
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, UInt32

from rover_qos import CMD_VEL_QOS, TELEMETRY_QOS
from rover_ir import close as ir_close
from rover_radar import BackgroundRadar, cruise_radar_enabled, pick_escape_spin
from rover_ring_ipc import emit as ring_emit
from rover_speaker import play as speaker_play
from rover_twist import set_twist
from rover_wheel_odom import WheelOdometry, wheel_odom_enabled

# Speeds are fractions of ROVER_AUTO_MAX_LINEAR (default 0.30) unless noted.
MIN_DRIVE_SEC = 3.0
BURST_PROB = 0.01
BURST_DURATION = (0.8, 1.2)

# ESP spin-in-place when |linear| < ~0.08 and |angular| > 0.02 — stay above this when steering.
MIN_LIN_FOR_STEER = 0.11

ESCAPE_REVERSE_SEC = 1.0
ESCAPE_ARC_LIN = 0.28
ESCAPE_ARC_ANG = 0.16
SPIN_DEG_MIN = 55
SPIN_DEG_MAX = 105

FRONT_AVOID_REVERSE_SEC = 1.0
FRONT_AVOID_STRAIGHT_SEC = 0.40
FRONT_AVOID_STEER = 0.12
FRONT_DRIVE_AWAY_SEC = 1.6
FRONT_DRIVE_AWAY_POWER = 0.30
FRONT_DRIVE_AWAY_STEER = 0.15
FRONT_CLEAR_HOLD_SEC = 0.40
FRONT_CLEAR_TIMEOUT_SEC = 2.5
FRONT_IR_COOLDOWN_SEC = 0.6
FRONT_STUCK_ESCALATE = 3

STARTUP_GRACE_SEC = 1.5
EVENT_COOLDOWN_SEC = 5.0
STRAIGHT_BLOCK_SEC = 10.0
MAX_ESCAPES_PER_MIN = 4
ESCAPE_FLOOD_PAUSE_SEC = 8.0


def _max_linear() -> float:
    return max(0.08, min(1.0, float(os.environ.get('ROVER_AUTO_MAX_LINEAR', '0.23'))))


def _burst_max() -> float:
    return max(_max_linear(), min(1.0, float(os.environ.get('ROVER_AUTO_BURST_MAX', '0.50'))))


def _reverse_power() -> float:
    """Reverse speed — slower than forward cruise (safer on castor / tight spaces)."""
    raw = os.environ.get('ROVER_AUTO_REVERSE_POWER', '').strip()
    if raw:
        return max(0.08, min(1.0, float(raw)))
    frac = float(os.environ.get('ROVER_AUTO_REVERSE_FRAC', '0.72'))
    return _max_linear() * max(0.25, min(1.0, frac))


def _max_reverse() -> float:
    """Cap for any negative linear command."""
    raw = os.environ.get('ROVER_AUTO_MAX_REVERSE', '').strip()
    if raw:
        return max(0.08, min(1.0, float(raw)))
    return _reverse_power()


def _burst_prob() -> float:
    return max(0.0, min(0.2, float(os.environ.get('ROVER_AUTO_BURST_PROB', str(BURST_PROB)))))


def _max_steer() -> float:
    """Max wander / arc steer — kept low so it curves, never spins on the spot."""
    raw = os.environ.get('ROVER_AUTO_MAX_ANGULAR', '').strip()
    if raw:
        return max(0.03, min(0.35, float(raw)))
    return max(0.05, _max_linear() * 0.28)


def _escape_spin_sign() -> float:
    return float(os.environ.get('ROVER_ESCAPE_SPIN_SIGN', '-1'))


def _imu_gyro_sign() -> float:
    return float(os.environ.get('ROVER_IMU_GYRO_SIGN', '1'))


def _front_ir_stop() -> bool:
    return os.environ.get('ROVER_IR_FRONT_STOP', '1').strip().lower() not in (
        '0',
        'no',
        'false',
    )


def _sonar_front() -> bool:
    return os.environ.get('ROVER_FRONT_TOF', '1').strip().lower() not in (
        '0',
        'no',
        'false',
    )


def _auto_tick() -> float:
    return max(0.01, min(0.1, float(os.environ.get('ROVER_AUTO_TICK', '0.02'))))


def _cap_linear(lin: float, *, burst: bool = False) -> float:
    if lin < 0:
        cap = _max_reverse()
    else:
        cap = _burst_max() if burst else _max_linear()
    return max(-cap, min(cap, lin))


def _cap_angular(ang: float) -> float:
    cap = _max_steer()
    return max(-cap, min(cap, ang))


def _arc_not_spin(lin: float, ang: float) -> tuple[float, float]:
    """ESP treats lin≈0 + steer as spin-in-place — always creep forward when steering."""
    if abs(ang) <= 0.02:
        return lin, ang
    if abs(lin) >= MIN_LIN_FOR_STEER:
        return lin, ang
    creep = MIN_LIN_FOR_STEER if lin >= 0 else -MIN_LIN_FOR_STEER
    if abs(lin) < 0.02:
        creep = MIN_LIN_FOR_STEER
    return creep, ang


class Mode(Enum):
    CRUISE = auto()
    BURST = auto()
    WANDER = auto()
    AVOID = auto()
    ESCAPE = auto()


class EscapePhase(Enum):
    REVERSE = auto()
    SCAN = auto()
    ARC = auto()


class AvoidPhase(Enum):
    REVERSE = auto()
    DRIVE_AWAY = auto()


class AutonomousExplore(Node):
    def __init__(self):
        super().__init__('autonomous_explore')
        self.TICK = _auto_tick()
        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self.create_subscription(Bool, '/rover/bump', self._on_bump, 10)
        self.create_subscription(Bool, '/rover/stall', self._on_stall, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, TELEMETRY_QOS)

        self._wheel: WheelOdometry | None = None
        if wheel_odom_enabled():
            self._wheel = WheelOdometry()
            self.create_subscription(
                UInt32, '/rover/wheel/left_ticks', self._on_wheel_left, TELEMETRY_QOS
            )
            self.create_subscription(
                UInt32, '/rover/wheel/right_ticks', self._on_wheel_right, TELEMETRY_QOS
            )

        self.mode = Mode.WANDER
        self.mode_until = time.monotonic() + MIN_DRIVE_SEC
        self._mode_dur = MIN_DRIVE_SEC
        self._cruise_speed = _max_linear() * 0.85
        self._cruise_curve = 0.05
        self.wander_ang = 0.06
        self.wander_lin = _max_linear() * 0.80

        self.escape_phase = EscapePhase.REVERSE
        self.avoid_steer_dir = 1.0
        self.avoid_phase = AvoidPhase.REVERSE
        self.avoid_phase_until = 0.0
        self.avoid_start = 0.0

        self.escape_backoff = -_reverse_power()
        self.arc_steer = ESCAPE_ARC_ANG
        self.arc_until = 0.0
        self.reverse_until = 0.0
        self.spin_dir = 1.0

        self._yaw_integrated = 0.0
        self._imu_last_mono: float | None = None
        self._imu_ok = False
        self._arc_yaw_start = 0.0

        self.straight_block_until = 0.0
        self._driving_linear = 0.0
        self._stuck_linear = 0.0
        self._start_time = time.monotonic()
        self._event_ignore_until = self._start_time + STARTUP_GRACE_SEC
        self._bump_pending = False
        self._stall_pending = False
        self._scan_done = False
        self._tick_count = 0
        self._ir_front_cooldown_until = 0.0
        self._ir_wait_clear = False
        self._front_clear_since: float | None = None
        self._ir_wait_clear_since: float | None = None
        self._front_stuck_retries = 0
        self._escape_times: list[float] = []
        self._imu_gyro_sign = _imu_gyro_sign()
        self._escape_spin_sign = _escape_spin_sign()
        self._bg_radar = BackgroundRadar.start() if cruise_radar_enabled() else None
        self._wheel_log_next = 0.0

        self._enter(Mode.WANDER, random.uniform(3.0, 6.0))
        self.create_timer(self.TICK, self._tick)

        cap = _max_linear()
        radar_note = 'cruise radar on' if self._bg_radar else 'cruise radar off'
        wheel_note = ''
        if self._wheel is not None:
            wheel_note = f', wheel odom log ({self._wheel.config_summary()})'
        self.get_logger().info(
            f'autonomous: cruise {cap:.0%}, burst {_burst_max():.0%} '
            f'({_burst_prob():.0%}), steer max {_max_steer():.2f}, '
            f'MPU bump+stall (ESP), {radar_note}{wheel_note}'
        )

    def _on_wheel_left(self, msg: UInt32) -> None:
        if self._wheel is not None:
            self._wheel.on_left(int(msg.data))

    def _on_wheel_right(self, msg: UInt32) -> None:
        if self._wheel is not None:
            self._wheel.on_right(int(msg.data))

    def _sync_bg_radar(self) -> None:
        if self._bg_radar is None:
            return
        if self.mode in (Mode.ESCAPE, Mode.AVOID):
            self._bg_radar.pause()
        else:
            self._bg_radar.resume()

    def _front_blocked(self) -> bool:
        try:
            from rover_ir import read_front

            return bool(read_front())
        except Exception:
            return False

    def _reversing_from_front(self) -> bool:
        """Allow drive commands only when explicitly backing away from front IR."""
        if self.mode == Mode.AVOID and self.avoid_phase == AvoidPhase.REVERSE:
            return True
        if self.mode == Mode.ESCAPE and self.escape_phase == EscapePhase.REVERSE:
            return True
        return False

    def _apply_front_stop(self, lin: float, ang: float, front_blocked: bool) -> tuple[float, float]:
        if not _front_ir_stop() or not front_blocked:
            return lin, ang
        if self._reversing_from_front():
            if lin > 0.0:
                lin = 0.0
            return lin, ang
        return 0.0, 0.0

    def _update_front_clear_gate(self, now: float) -> None:
        if not self._ir_wait_clear:
            return
        if self._front_blocked():
            self._front_clear_since = None
            if self._ir_wait_clear_since is None:
                self._ir_wait_clear_since = now
            elif now - self._ir_wait_clear_since >= FRONT_CLEAR_TIMEOUT_SEC:
                self._front_stuck_retries += 1
                if self._front_stuck_retries >= FRONT_STUCK_ESCALATE:
                    self._trigger_front_radar_escape(now)
                else:
                    self.get_logger().info(
                        'front still blocked — reverse+steer again'
                    )
                    self._ir_wait_clear = False
                    self._ir_wait_clear_since = None
                    self._ir_front_cooldown_until = 0.0
            return
        if self._ir_wait_clear_since is not None:
            self._ir_wait_clear_since = None
        if self._front_clear_since is None:
            self._front_clear_since = now
        elif now - self._front_clear_since >= FRONT_CLEAR_HOLD_SEC:
            self._ir_wait_clear = False
            self._front_clear_since = None
            self._ir_wait_clear_since = None
            self._front_stuck_retries = 0

    def _on_imu(self, msg: Imu):
        now = time.monotonic()
        if self._imu_last_mono is not None:
            dt = now - self._imu_last_mono
            if 0.0 < dt < 0.5:
                self._yaw_integrated += (
                    self._imu_gyro_sign * float(msg.angular_velocity.z) * dt
                )
        self._imu_last_mono = now
        self._imu_ok = True

    def _trigger_front_avoid(self, now: float) -> None:
        if self.mode in (Mode.ESCAPE, Mode.AVOID):
            return
        if now < self._ir_front_cooldown_until:
            return
        if now < self._event_ignore_until:
            return
        if self._ir_wait_clear:
            return

        self.avoid_steer_dir = random.choice([-1.0, 1.0])
        self.avoid_start = now
        self.avoid_phase = AvoidPhase.REVERSE
        self.avoid_phase_until = now + FRONT_AVOID_REVERSE_SEC
        self.mode = Mode.AVOID
        self.mode_until = now + FRONT_AVOID_REVERSE_SEC + FRONT_DRIVE_AWAY_SEC + 2.0
        self._mode_dur = FRONT_AVOID_REVERSE_SEC + FRONT_DRIVE_AWAY_SEC
        self._ir_front_cooldown_until = now + FRONT_IR_COOLDOWN_SEC
        speaker_play('front_ir')
        self.get_logger().info(
            f'front IR — reverse then arc '
            f'({"right" if self.avoid_steer_dir > 0 else "left"})'
        )

    def _finish_front_avoid(self, now: float) -> None:
        self._ir_wait_clear = True
        self._front_clear_since = None
        self._ir_wait_clear_since = now
        self._event_ignore_until = now + 1.0
        self._enter(Mode.CRUISE, random.uniform(4.0, 7.0))
        self.get_logger().info('front avoid done → cruise (wait for clear path)')

    def _trigger_front_radar_escape(self, now: float) -> None:
        """Front IR keep retrying — escalate to full radar escape."""
        self._ir_wait_clear = False
        self._ir_wait_clear_since = None
        self._front_stuck_retries = 0
        self._scan_done = False
        self.escape_phase = EscapePhase.REVERSE
        self.escape_backoff = -_reverse_power()
        self.reverse_until = now + ESCAPE_REVERSE_SEC
        self.mode = Mode.ESCAPE
        self._event_ignore_until = now + EVENT_COOLDOWN_SEC
        ring_emit('escape')
        self.get_logger().info('front stuck — reverse → radar arc')

    def _on_bump(self, msg: Bool):
        if msg.data:
            self._bump_pending = True

    def _on_stall(self, msg: Bool):
        if msg.data:
            self._stall_pending = True

    def _backoff_linear(self) -> float:
        rev = _reverse_power()
        if self._stuck_linear > 0.08:
            return -rev
        if self._stuck_linear < -0.08:
            return rev
        return -rev

    def _trigger_escape(self):
        now = time.monotonic()
        self._escape_times = [t for t in self._escape_times if now - t < 60.0]
        if len(self._escape_times) >= MAX_ESCAPES_PER_MIN:
            self.get_logger().warn(f'escape flood — pausing')
            self._bump_pending = False
            self._stall_pending = False
            self._event_ignore_until = now + ESCAPE_FLOOD_PAUSE_SEC
            return

        if not (self._bump_pending or self._stall_pending):
            return
        if self.mode == Mode.ESCAPE:
            return

        reason = 'bump+stall' if self._bump_pending and self._stall_pending else (
            'bump' if self._bump_pending else 'stall'
        )
        self._bump_pending = False
        self._stall_pending = False
        self._escape_times.append(now)
        self._stuck_linear = self._driving_linear
        self._scan_done = False
        self.escape_phase = EscapePhase.REVERSE
        self.escape_backoff = self._backoff_linear()
        self.reverse_until = now + ESCAPE_REVERSE_SEC
        self.mode = Mode.ESCAPE
        self._ir_wait_clear = False

        if reason.startswith('bump'):
            speaker_play('bump')
            ring_emit('bump')
        else:
            speaker_play('stall')
            ring_emit('stall')
        ring_emit('escape')
        self.get_logger().info(f'escape ({reason}): reverse → scan → arc')

    def _start_escape_arc(self, now: float, steer: float, duration: float):
        self.escape_phase = EscapePhase.ARC
        self.arc_steer = steer
        self.arc_until = now + duration
        self._arc_yaw_start = self._yaw_integrated
        self.mode_until = self.arc_until
        self._mode_dur = duration
        self._event_ignore_until = self.arc_until + EVENT_COOLDOWN_SEC

    def _run_escape_scan(self):
        self._scan_done = True
        steer = max(0.12, ESCAPE_ARC_ANG)
        now = time.monotonic()

        if _sonar_front():
            spin_deg = random.uniform(SPIN_DEG_MIN, SPIN_DEG_MAX)
            spin_dir = random.choice([-1.0, 1.0]) * self._escape_spin_sign
            bearing = -1.0
            if self._bg_radar is not None:
                hits, _ = self._bg_radar.get_hits()
                if hits and not all(hits):
                    try:
                        spin_dir, spin_deg, bearing = pick_escape_spin(
                            hits,
                            spin_min=SPIN_DEG_MIN,
                            spin_max=SPIN_DEG_MAX,
                        )
                    except Exception as exc:
                        self.get_logger().warn(
                            f'rear map escape failed ({exc}) — random arc'
                        )
            duration = max(1.0, min(2.5, spin_deg / 45.0))
            if bearing >= 0:
                self.get_logger().info(
                    f'escape arc (cached rear map) {bearing:.0f}° → '
                    f'{spin_deg:.0f}° ({"right" if spin_dir > 0 else "left"})'
                )
            else:
                self.get_logger().info(
                    f'escape arc (sonar front) {spin_deg:.0f}° '
                    f'({"right" if spin_dir > 0 else "left"})'
                )
            self._start_escape_arc(now, spin_dir * steer, duration)
            return

        try:
            spin_dir, spin_deg, bearing = pick_escape_spin(
                spin_min=SPIN_DEG_MIN,
                spin_max=SPIN_DEG_MAX,
            )
        except Exception as exc:
            self.get_logger().warn(f'radar scan failed ({exc}) — random arc')
            spin_deg = random.uniform(SPIN_DEG_MIN, SPIN_DEG_MAX)
            spin_dir = random.choice([-1.0, 1.0]) * self._escape_spin_sign
            bearing = -1.0

        duration = max(1.2, min(3.5, (spin_deg / 40.0) * 1.15))

        if bearing >= 0:
            self.get_logger().info(
                f'radar gap {bearing:.0f}° → arc {spin_deg:.0f}° '
                f'({"right" if spin_dir > 0 else "left"}) {duration:.1f}s'
            )
        else:
            self.get_logger().info(
                f'radar blocked — arc {spin_deg:.0f}° '
                f'({"right" if spin_dir > 0 else "left"})'
            )
        self._start_escape_arc(now, spin_dir * steer, duration)

    def _finish_escape_arc(self, now: float):
        self._event_ignore_until = now + EVENT_COOLDOWN_SEC
        self.straight_block_until = now + STRAIGHT_BLOCK_SEC
        ring_emit('auto')
        self._enter(Mode.CRUISE, random.uniform(5.0, 9.0))
        self.get_logger().info('escape done → cruise')

    def _enter(self, mode: Mode, duration: float):
        duration = max(duration, MIN_DRIVE_SEC)
        self.mode = mode
        self.mode_until = time.monotonic() + duration
        self._mode_dur = duration
        cap = _max_linear()
        steer = _max_steer()
        if mode == Mode.CRUISE:
            self._cruise_speed = random.uniform(cap * 0.78, cap)
            self._cruise_curve = random.choice([-1, 1]) * random.uniform(0.02, steer * 0.55)
        elif mode == Mode.WANDER:
            self.wander_ang = random.choice([-1, 1]) * random.uniform(0.03, steer * 0.75)
            self.wander_lin = random.uniform(cap * 0.70, cap * 0.92)
        elif mode == Mode.BURST:
            self._cruise_curve = random.choice([-1, 1]) * random.uniform(0.02, steer * 0.45)

    def _tick(self):
        now = time.monotonic()
        self._tick_count += 1
        self._update_front_clear_gate(now)

        if _front_ir_stop() and self.mode not in (Mode.ESCAPE, Mode.AVOID):
            if self._front_blocked():
                self._trigger_front_avoid(now)

        if self._bump_pending or self._stall_pending:
            self._trigger_escape()

        if self.mode not in (Mode.ESCAPE, Mode.AVOID) and now >= self.mode_until:
            self._next_mode()

        lin = 0.0
        ang = 0.0
        front_blocked = _front_ir_stop() and self._front_blocked()

        if self.mode == Mode.CRUISE:
            lin = self._cruise_speed
            ang = self._cruise_curve

        elif self.mode == Mode.BURST:
            peak = _burst_max()
            base = _max_linear()
            t_left = max(0.0, self.mode_until - now)
            frac = 1.0 - (t_left / max(0.01, self._mode_dur))
            lin = base + (peak - base) * min(1.0, frac / 0.35) if frac < 0.35 else peak
            ang = self._cruise_curve * 0.5

        elif self.mode == Mode.WANDER:
            lin = self.wander_lin
            ang = self.wander_ang

        elif self.mode == Mode.AVOID:
            blocked = _front_ir_stop() and self._front_blocked()
            if self.avoid_phase == AvoidPhase.REVERSE:
                lin = -_reverse_power()
                if now < self.avoid_start + FRONT_AVOID_STRAIGHT_SEC:
                    ang = 0.0
                else:
                    ang = self.avoid_steer_dir * FRONT_AVOID_STEER
                if now >= self.avoid_phase_until:
                    if blocked:
                        self.avoid_start = now
                        self.avoid_phase_until = now + FRONT_AVOID_REVERSE_SEC
                        self.avoid_steer_dir *= -1.0
                        self.get_logger().info(
                            'front still blocked — reverse+steer again'
                        )
                    else:
                        self.avoid_phase = AvoidPhase.DRIVE_AWAY
                        self.avoid_phase_until = now + FRONT_DRIVE_AWAY_SEC
                        self.get_logger().info(
                            f'front avoid — drive away '
                            f'({"right" if self.avoid_steer_dir > 0 else "left"})'
                        )
            else:
                if blocked:
                    self.avoid_phase = AvoidPhase.REVERSE
                    self.avoid_start = now
                    self.avoid_phase_until = now + FRONT_AVOID_REVERSE_SEC
                    lin = -_reverse_power()
                    ang = self.avoid_steer_dir * FRONT_AVOID_STEER
                else:
                    lin = FRONT_DRIVE_AWAY_POWER
                    ang = self.avoid_steer_dir * FRONT_DRIVE_AWAY_STEER
                    if now >= self.avoid_phase_until:
                        self._finish_front_avoid(now)

        elif self.mode == Mode.ESCAPE:
            if self.escape_phase == EscapePhase.REVERSE:
                if now < self.reverse_until:
                    lin = self.escape_backoff
                else:
                    self.escape_phase = EscapePhase.SCAN
            elif self.escape_phase == EscapePhase.SCAN:
                if not self._scan_done:
                    self._run_escape_scan()
            elif self.escape_phase == EscapePhase.ARC:
                if front_blocked:
                    self.escape_phase = EscapePhase.REVERSE
                    self.reverse_until = now + ESCAPE_REVERSE_SEC * 0.6
                    lin = self.escape_backoff
                    ang = 0.0
                    self.get_logger().info('escape arc blocked — reverse again')
                else:
                    lin = ESCAPE_ARC_LIN
                    ang = self.arc_steer
                    if now >= self.arc_until:
                        self._finish_escape_arc(now)

        if self._bg_radar and self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST):
            ang += self._bg_radar.steer_bias()

        lin, ang = self._apply_front_stop(lin, ang, front_blocked)

        if self._ir_wait_clear and front_blocked:
            lin = 0.0
            ang = 0.0

        lin, ang = _arc_not_spin(lin, ang)

        if self._wheel is not None and now >= self._wheel_log_next:
            self._wheel_log_next = now + 15.0
            s = self._wheel.snapshot()
            if s.ticks_ok:
                self.get_logger().info(
                    f'wheel odom: {s.distance_m:.2f} m '
                    f'({s.left_m:.2f}/{s.right_m:.2f} L/R), '
                    f'heading {math.degrees(s.heading_rad):.0f}°, '
                    f'rates {s.left_rate:.1f}/{s.right_rate:.1f} t/s'
                )

        if self.mode != Mode.ESCAPE:
            self._driving_linear = lin
        burst_mode = self.mode == Mode.BURST
        lin = _cap_linear(lin, burst=burst_mode)
        ang = _cap_angular(ang)
        msg = Twist()
        set_twist(msg, lin, ang)
        self.pub.publish(msg)
        self._sync_bg_radar()

    def _next_mode(self):
        now = time.monotonic()
        post_escape = now < self.straight_block_until
        roll = random.random()

        if post_escape:
            self._enter(Mode.CRUISE, random.uniform(5.0, 9.0))
            self.get_logger().info('cruise (post-escape)')
        elif roll < _burst_prob():
            dur = random.uniform(*BURST_DURATION)
            self._enter(Mode.BURST, dur)
            self.get_logger().info(f'burst ({dur:.1f}s)')
        elif roll < 0.68:
            self._enter(Mode.CRUISE, random.uniform(4.0, 8.0))
            self.get_logger().info('cruise')
        else:
            self._enter(Mode.WANDER, random.uniform(3.0, 5.0))
            self.get_logger().info('wander')


def main():
    def _on_stop(_signum, _frame):
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _on_stop)
    signal.signal(signal.SIGINT, _on_stop)

    rclpy.init()
    node = AutonomousExplore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if type(exc).__name__ != 'ExternalShutdownException':
            raise
    finally:
        BackgroundRadar.stop()
        try:
            if rclpy.ok():
                stop = Twist()
                for _ in range(8):
                    node.pub.publish(stop)
                    time.sleep(0.05)
        except Exception:
            pass
        ir_close()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
