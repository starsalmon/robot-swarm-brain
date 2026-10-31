"""Shared wander + IR beacon steering for fleet brains."""
from __future__ import annotations

import math
import os
import random
import time
from dataclasses import dataclass, field

from geometry_msgs.msg import Twist


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name, '1' if default else '0').strip().lower()
    return raw in ('1', 'yes', 'true', 'on')


@dataclass
class WanderConfig:
    """Footprint-scaled explore + escape tuning (10×8 cm mini-bot defaults)."""

    stop_m: float = 0.14
    slow_m: float = 0.26
    body_half_m: float = 0.05
    lookahead_s: float = 0.40
    close_any_angle_m: float = 0.22
    cruise: float = 0.22
    cruise_min: float = 0.16
    cruise_max: float = 0.26
    steer: float = 0.32
    spin_rate: float = 0.65
    burst_speed: float = 0.70
    burst_duration_s: float = 1.5
    burst_min_interval_s: float = 90.0
    burst_max_interval_s: float = 240.0
    burst_enabled: bool = False
    pan_center: float = 90.0
    pan_use_deg: float = 18.0
    close_hold_s: float = 0.50
    range_stale_s: float = 1.4
    turn_min_deg: float = 50.0
    turn_max_deg: float = 72.0
    avoid_backup_s: float = 0.55
    avoid_reverse_lin: float = -0.15
    avoid_cooldown_s: float = 1.8
    avoid_trap_cooldown_s: float = 2.4
    commit_cruise_s: float = 1.6
    turn_tolerance_deg: float = 10.0
    turn_timeout_base_s: float = 1.1
    turn_timeout_max_s: float = 3.2
    escape_max_lin: float = 0.40
    burst_min_range_m: float = 2.0
    burst_chance: float = 0.01
    carpet_mode: bool = True
    avoid_spin_turn: bool = True
    scan_on_trap: bool = True
    avoid_spin_close_m: float = 0.40
    avoid_spin_max: float = 0.65
    avoid_arc_lin: float = 0.12
    avoid_close_backup_extra_s: float = 0.30
    gyro_sign: float = -1.0
    curiosity_enable: bool = True
    imu_cal_s: float = 2.5
    imu_cal_min_samples: int = 50
    rear_stop_m: float = 0.12
    # Forward gate: nonzero forward lin needs a valid centred range newer than this.
    fwd_fresh_s: float = 0.5
    # Smoothing (never more permissive): taper fwd lin to 0 as reading ages
    # fwd_ramp_s → fwd_fresh_s, and cap forward acceleration (decel is instant).
    fwd_ramp_s: float = 0.25
    fwd_accel: float = 0.8

    @classmethod
    def from_env(cls, *, pan_center: float = 90.0) -> WanderConfig:
        carpet = _env_bool('WANDER_CARPET', True)
        burst_on = _env_bool('WANDER_BURST_ENABLE', False)
        cruise = float(os.environ.get('WANDER_CRUISE', '0.20' if carpet else '0.26'))
        return cls(
            stop_m=float(os.environ.get('WANDER_STOP_M', '0.14')),
            slow_m=float(os.environ.get('WANDER_SLOW_M', '0.26')),
            body_half_m=float(os.environ.get('WANDER_BODY_HALF_M', '0.05')),
            lookahead_s=float(os.environ.get('WANDER_LOOKAHEAD_S', '0.40')),
            close_any_angle_m=float(os.environ.get('WANDER_CLOSE_M', '0.22')),
            cruise=cruise,
            cruise_min=float(os.environ.get('WANDER_CRUISE_MIN', '0.14' if carpet else '0.18')),
            cruise_max=float(os.environ.get('WANDER_CRUISE_MAX', '0.24' if carpet else '0.30')),
            steer=float(os.environ.get('WANDER_STEER', '0.30' if carpet else '0.35')),
            spin_rate=float(os.environ.get('WANDER_SPIN_RATE', '0.65')),
            burst_speed=float(os.environ.get('WANDER_BURST_SPEED', '0.70')),
            burst_duration_s=float(os.environ.get('WANDER_BURST_SEC', '1.5')),
            burst_min_interval_s=float(os.environ.get('WANDER_BURST_MIN_INTERVAL', '90')),
            burst_max_interval_s=float(os.environ.get('WANDER_BURST_MAX_INTERVAL', '240')),
            burst_enabled=burst_on,
            pan_center=pan_center,
            pan_use_deg=float(os.environ.get('WANDER_PAN_USE_DEG', '18')),
            carpet_mode=carpet,
            avoid_spin_turn=_env_bool('WANDER_AVOID_SPIN', True),
            scan_on_trap=_env_bool('WANDER_SCAN_ON_TRAP', True),
            avoid_spin_close_m=float(os.environ.get('WANDER_AVOID_SPIN_CLOSE_M', '0.40')),
            avoid_spin_max=float(os.environ.get('WANDER_AVOID_SPIN_MAX', '0.65')),
            avoid_arc_lin=float(os.environ.get('WANDER_AVOID_ARC_LIN', '0.12')),
            avoid_close_backup_extra_s=float(
                os.environ.get('WANDER_AVOID_CLOSE_BACKUP_EXTRA_S', '0.30')
            ),
            avoid_backup_s=float(os.environ.get('WANDER_AVOID_BACKUP_S', '0.60' if carpet else '0.45')),
            avoid_reverse_lin=float(os.environ.get('WANDER_AVOID_REVERSE_LIN', '-0.15')),
            avoid_cooldown_s=float(os.environ.get('WANDER_AVOID_COOLDOWN_S', '1.8')),
            avoid_trap_cooldown_s=float(os.environ.get('WANDER_TRAP_COOLDOWN_S', '2.4')),
            commit_cruise_s=float(os.environ.get('WANDER_COMMIT_CRUISE_S', '1.6')),
            escape_max_lin=float(os.environ.get('WANDER_ESCAPE_MAX_LIN', '0.40')),
            burst_min_range_m=float(os.environ.get('WANDER_BURST_MIN_RANGE_M', '2.0')),
            burst_chance=float(os.environ.get('WANDER_BURST_CHANCE', '0.01')),
            gyro_sign=float(os.environ.get('WANDER_GYRO_SIGN', '-1.0')),
            curiosity_enable=_env_bool('WANDER_CURIOSITY', True),
            imu_cal_s=float(os.environ.get('IMU_CAL_SEC', '2.5')),
            imu_cal_min_samples=int(os.environ.get('IMU_CAL_MIN_SAMPLES', '50')),
            rear_stop_m=float(os.environ.get('WANDER_REAR_STOP_M', '0.12')),
            fwd_fresh_s=float(os.environ.get('WANDER_FWD_FRESH_S', '0.5')),
            fwd_ramp_s=float(os.environ.get('WANDER_FWD_RAMP_S', '0.25')),
            fwd_accel=float(os.environ.get('WANDER_FWD_ACCEL', '0.8')),
        )


def _wrap_deg(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


@dataclass
class HeadingHold:
    """Keep a straight course using gyro yaw — bias-learned while still at boot."""

    kp: float = 0.022
    max_w: float = 0.22
    gyro_sign: float = -1.0
    cal_s: float = 2.5
    cal_min_samples: int = 50
    cal_max_gz: float = 0.35
    _yaw_deg: float = 0.0
    _target_deg: float = 0.0
    _bias_gz: float = 0.0
    _bias_n: int = 0
    _trim_ang: float = 0.0
    _last_imu_s: float = 0.0
    _cal_start_s: float = 0.0
    _ready: bool = False
    _armed: bool = False

    @property
    def yaw_deg(self) -> float:
        return self._yaw_deg

    @property
    def ready(self) -> bool:
        return self._ready

    def cal_progress(self) -> float:
        if self._ready:
            return 1.0
        if self._cal_start_s <= 0.0:
            return 0.0
        elapsed = time.monotonic() - self._cal_start_s
        sample_frac = min(1.0, self._bias_n / max(self.cal_min_samples, 1))
        time_frac = min(1.0, elapsed / max(self.cal_s, 0.1))
        return min(sample_frac, time_frac)

    def update(self, gz_rad_s: float, *, stationary: bool) -> None:
        now = time.monotonic()
        if self._cal_start_s <= 0.0:
            self._cal_start_s = now

        if not self._ready:
            if abs(gz_rad_s) > self.cal_max_gz:
                self._cal_start_s = now
                self._bias_n = max(0, self._bias_n - 15)
            elif stationary and abs(gz_rad_s) < 0.8:
                self._bias_n += 1
                self._bias_gz += (gz_rad_s - self._bias_gz) / self._bias_n
            elapsed = now - self._cal_start_s
            if elapsed >= self.cal_s and self._bias_n >= self.cal_min_samples:
                self._ready = True
                self.reset_target()
        elif stationary and abs(gz_rad_s) < 0.8:
            self._bias_n += 1
            self._bias_gz += (gz_rad_s - self._bias_gz) / self._bias_n

        gz = (gz_rad_s - self._bias_gz) * self.gyro_sign
        if self._last_imu_s > 0.0:
            dt = now - self._last_imu_s
            if 0.0 < dt < 0.25:
                self._yaw_deg += math.degrees(gz) * dt
        self._last_imu_s = now

    def fresh(self) -> bool:
        return (time.monotonic() - self._last_imu_s) < 0.5

    def force_ready(self) -> None:
        """Tracked-bot etc. with no gyro — wander without waiting for /imu."""
        self._ready = True
        if self._cal_start_s <= 0.0:
            self._cal_start_s = time.monotonic()
        self.reset_target()

    def reset_target(self) -> None:
        self._target_deg = self._yaw_deg
        self._armed = True

    def release(self) -> None:
        self._armed = False

    @property
    def trim_ang(self) -> float:
        return self._trim_ang

    def learn_drift_trim(self, gz_rad_s: float, *, lin: float, explore_w: float) -> None:
        """IMU integral trim — learns wheel bias so heading hold can keep a straight line."""
        if not self._ready or not self._armed or lin < 0.22:
            return
        if abs(explore_w) > 0.10:
            return
        gz = (gz_rad_s - self._bias_gz) * self.gyro_sign
        self._trim_ang += 0.004 * (-gz)
        self._trim_ang = max(-0.30, min(0.30, self._trim_ang))

    def correction(self) -> float:
        if not self.fresh():
            return 0.0
        if not self._armed:
            return 0.0
        err = _wrap_deg(self._target_deg - self._yaw_deg)
        if abs(err) > 35.0:
            self.reset_target()
            return self._trim_ang
        w = self.kp * err + self._trim_ang
        return max(-self.max_w, min(self.max_w, w))


@dataclass
class WanderState:
    """Explore: sonar + pan wiggle, IMU straight hold, trap escape with commitment."""

    stop_m: float = 0.14
    slow_m: float = 0.26
    body_half_m: float = 0.05
    lookahead_s: float = 0.40
    cruise: float = 0.22
    cruise_min: float = 0.16
    cruise_max: float = 0.26
    steer: float = 0.30
    spin_rate: float = 0.65
    burst_speed: float = 0.70
    burst_duration_s: float = 1.5
    burst_min_interval_s: float = 90.0
    burst_max_interval_s: float = 240.0
    burst_enabled: bool = False
    pan_center: float = 90.0
    pan_use_deg: float = 18.0
    close_any_angle_m: float = 0.22
    close_hold_s: float = 0.50
    range_stale_s: float = 1.4
    turn_min_deg: float = 50.0
    turn_max_deg: float = 72.0
    avoid_backup_s: float = 0.55
    avoid_reverse_lin: float = -0.15
    avoid_cooldown_s: float = 1.8
    avoid_trap_cooldown_s: float = 2.4
    commit_cruise_s: float = 1.6
    carpet_mode: bool = True
    avoid_spin_turn: bool = True
    scan_on_trap: bool = True
    avoid_spin_close_m: float = 0.40
    avoid_spin_max: float = 0.65
    avoid_arc_lin: float = 0.12
    avoid_close_backup_extra_s: float = 0.30
    turn_tolerance_deg: float = 10.0
    turn_timeout_base_s: float = 1.1
    turn_timeout_max_s: float = 3.2
    escape_max_lin: float = 0.40
    burst_min_range_m: float = 2.0
    burst_chance: float = 0.01
    gyro_sign: float = -1.0
    curiosity_enable: bool = True
    imu_cal_s: float = 2.5
    imu_cal_min_samples: int = 50
    rear_stop_m: float = 0.12
    # Forward gate: nonzero forward lin needs a valid centred range newer than this.
    fwd_fresh_s: float = 0.5
    # Smoothing (never more permissive): taper fwd lin to 0 as reading ages
    # fwd_ramp_s → fwd_fresh_s, and cap forward acceleration (decel is instant).
    fwd_ramp_s: float = 0.25
    fwd_accel: float = 0.8
    _range_m: float = field(default_factory=lambda: float('nan'))
    _fwd_m: float = field(default_factory=lambda: float('nan'))
    _fwd_at: float = 0.0
    _gate_lin: float = 0.0
    _gate_at: float = 0.0
    _last_good_m: float = field(default_factory=lambda: float('nan'))
    _recent_close_m: float = field(default_factory=lambda: float('nan'))
    _recent_close_ms: float = 0.0
    _last_range_ms: float = 0.0
    _pan_deg: float = 90.0
    _pan_best_deg: float = 90.0
    _pan_best_m: float = field(default_factory=lambda: float('nan'))
    _pan_best_ms: float = 0.0
    _pan_best_ttl_s: float = 3.5
    _pan_bucket_range: dict[str, tuple[float, float]] = field(default_factory=dict)
    _heading: HeadingHold = field(default_factory=HeadingHold)
    _turn_sign: float = 1.0
    _avoid_active: bool = False
    _avoid_phase: str = ''
    _avoid_backup_until: float = 0.0
    _avoid_heading0: float = 0.0
    _avoid_turn_start: float = 0.0
    _avoid_sign: float = 1.0
    _turn_target_deg: float = 70.0
    _burst_until: float = 0.0
    _burst_next_at: float = 0.0
    _burst_just_started: bool = False
    _curiosity_until: float = 0.0
    _curiosity_w: float = 0.0
    _curiosity_spin: bool = False
    _next_curiosity_at: float = 0.0
    _last_driving: bool = False
    _cruise_since: float = 0.0
    _last_gz: float = 0.0
    _mood_speed: float = 0.30
    _mood_next_at: float = 0.0
    _avoid_recent: list[float] = field(default_factory=list)
    _trap_level: int = 0
    _committed_escape_sign: float = 1.0
    _avoid_cooldown_until: float = 0.0
    _commit_until: float = 0.0
    _close_persist_since: float = 0.0
    _avoid_start_range: float = field(default_factory=lambda: float('nan'))
    _rear_m: float = field(default_factory=lambda: float('nan'))
    _rear_ms: float = 0.0

    @classmethod
    def from_config(cls, cfg: WanderConfig) -> WanderState:
        return cls(
            stop_m=cfg.stop_m,
            slow_m=cfg.slow_m,
            body_half_m=cfg.body_half_m,
            lookahead_s=cfg.lookahead_s,
            cruise=cfg.cruise,
            cruise_min=cfg.cruise_min,
            cruise_max=cfg.cruise_max,
            steer=cfg.steer,
            spin_rate=cfg.spin_rate,
            burst_speed=cfg.burst_speed,
            burst_duration_s=cfg.burst_duration_s,
            burst_min_interval_s=cfg.burst_min_interval_s,
            burst_max_interval_s=cfg.burst_max_interval_s,
            burst_enabled=cfg.burst_enabled,
            pan_center=cfg.pan_center,
            pan_use_deg=cfg.pan_use_deg,
            close_any_angle_m=cfg.close_any_angle_m,
            close_hold_s=cfg.close_hold_s,
            range_stale_s=cfg.range_stale_s,
            turn_min_deg=cfg.turn_min_deg,
            turn_max_deg=cfg.turn_max_deg,
            avoid_backup_s=cfg.avoid_backup_s,
            avoid_reverse_lin=cfg.avoid_reverse_lin,
            avoid_cooldown_s=cfg.avoid_cooldown_s,
            avoid_trap_cooldown_s=cfg.avoid_trap_cooldown_s,
            commit_cruise_s=cfg.commit_cruise_s,
            carpet_mode=cfg.carpet_mode,
            avoid_spin_turn=cfg.avoid_spin_turn,
            scan_on_trap=cfg.scan_on_trap,
            avoid_spin_close_m=cfg.avoid_spin_close_m,
            avoid_spin_max=cfg.avoid_spin_max,
            avoid_arc_lin=cfg.avoid_arc_lin,
            avoid_close_backup_extra_s=cfg.avoid_close_backup_extra_s,
            turn_tolerance_deg=cfg.turn_tolerance_deg,
            turn_timeout_base_s=cfg.turn_timeout_base_s,
            turn_timeout_max_s=cfg.turn_timeout_max_s,
            escape_max_lin=cfg.escape_max_lin,
            burst_min_range_m=cfg.burst_min_range_m,
            burst_chance=cfg.burst_chance,
            gyro_sign=cfg.gyro_sign,
            curiosity_enable=cfg.curiosity_enable,
            imu_cal_s=cfg.imu_cal_s,
            imu_cal_min_samples=cfg.imu_cal_min_samples,
            rear_stop_m=cfg.rear_stop_m,
            fwd_fresh_s=cfg.fwd_fresh_s,
            fwd_ramp_s=cfg.fwd_ramp_s,
            fwd_accel=cfg.fwd_accel,
        )

    def __post_init__(self) -> None:
        self._heading.gyro_sign = self.gyro_sign
        self._heading.cal_s = self.imu_cal_s
        self._heading.cal_min_samples = self.imu_cal_min_samples
        now = time.monotonic()
        self._schedule_burst(now)
        self._next_curiosity_at = now + random.uniform(8.0, 16.0)
        self._pan_best_deg = self.pan_center
        self._mood_speed = self.cruise
        self._mood_next_at = now + random.uniform(4.0, 9.0)

    def debug_range_m(self) -> float:
        return self._effective_range()

    def trap_level(self) -> int:
        return self._trap_level

    def imu_ready(self) -> bool:
        return self._heading.ready

    def imu_cal_progress(self) -> float:
        return self._heading.cal_progress()

    def _schedule_burst(self, after: float) -> None:
        if not self.burst_enabled:
            self._burst_next_at = after + 1e9
            return
        self._burst_next_at = after + random.uniform(
            self.burst_min_interval_s,
            self.burst_max_interval_s,
        )

    def _stop_threshold(self, speed: float) -> float:
        dyn = self.body_half_m + max(speed, 0.04) * self.lookahead_s + 0.03
        return min(self.slow_m - 0.04, max(self.stop_m, dyn))

    def _slow_threshold(self, speed: float) -> float:
        return max(self.slow_m, self._stop_threshold(speed) + 0.10)

    def _cap_escape_lin(self, lin: float) -> float:
        cap = self.escape_max_lin
        if lin >= 0.0:
            return min(lin, cap)
        return max(lin, -cap)

    def _cap_escape_ang(self, ang: float) -> float:
        cap = self.avoid_spin_max
        if ang >= 0.0:
            return min(ang, cap)
        return max(ang, -cap)

    def _wants_spin_avoid(self) -> bool:
        """Mini-bot carpet spin — not for large rovers (body_half_m gates carpet spin)."""
        if not self.avoid_spin_turn:
            return False
        if self.carpet_mode and self.body_half_m < 0.08:
            return True
        return self.avoid_spin_turn and not self.carpet_mode

    def _avoid_too_close_for_spin(self) -> bool:
        if self._avoid_start_range < self.avoid_spin_close_m:
            return True
        rng = self._forward_range()
        return not math.isnan(rng) and rng < self.avoid_spin_close_m

    def _apply_speed_limits(self, msg: Twist, now: float) -> None:
        """Cap forward speed only — never limit angular (diff drive can spin on the spot)."""
        if self._avoid_active:
            msg.linear.x = self._cap_escape_lin(msg.linear.x)
            msg.angular.z = self._cap_escape_ang(msg.angular.z)
            return

        if msg.linear.x <= 0.0:
            return

        rng = self._effective_range()
        open_road = not math.isnan(rng) and rng >= self.burst_min_range_m

        if now < self._burst_until:
            msg.linear.x = min(self.burst_speed, self.escape_max_lin)
            return

        msg.linear.x = min(msg.linear.x, self.cruise_max)

        if (
            self.burst_enabled
            and open_road
            and msg.linear.x >= self.cruise * 0.90
            and now >= self._burst_next_at
            and random.random() < self.burst_chance
        ):
            self._burst_until = now + self.burst_duration_s
            self._schedule_burst(self._burst_until)
            self._burst_just_started = True
            msg.linear.x = min(self.burst_speed, self.escape_max_lin)

    def consume_burst_started(self) -> bool:
        if self._burst_just_started:
            self._burst_just_started = False
            return True
        return False

    def _refresh_mood_speed(self, now: float) -> None:
        if now < self._mood_next_at:
            return
        self._mood_speed = random.uniform(self.cruise_min, self.cruise_max)
        self._mood_next_at = now + random.uniform(5.0, 12.0)

    def _forward_speed(self, rng: float, now: float, *, lin_hint: float = 0.0) -> float:
        """Open-road cruise varies with clearance + slow mood changes."""
        if math.isnan(rng):
            return 0.0  # no forward range → no forward motion
        slow_lim = self._slow_threshold(lin_hint)
        stop_lim = self._stop_threshold(lin_hint)
        if rng < slow_lim:
            span = max(slow_lim - stop_lim, 0.01)
            t = max(0.0, min(1.0, (rng - stop_lim) / span))
            return self.cruise * (0.42 + 0.58 * t)
        self._refresh_mood_speed(now)
        extra = min(0.04, max(0.0, (rng - slow_lim) * 0.03))
        return min(self.cruise_max, self._mood_speed + extra)

    def update_imu(self, gz_rad_s: float) -> None:
        self._last_gz = gz_rad_s
        self._heading.update(gz_rad_s, stationary=not self._last_driving)

    def _pan_bucket(self, pan_deg: float) -> str:
        if pan_deg < self.pan_center - 5.0:
            return 'left'
        if pan_deg > self.pan_center + 5.0:
            return 'right'
        return 'center'

    def update_rear(self, range_m: float) -> None:
        if not math.isfinite(range_m) or range_m <= 0.0:
            self._rear_m = float('nan')
            return
        self._rear_m = range_m
        self._rear_ms = time.monotonic()

    def _rear_blocked(self) -> bool:
        if math.isnan(self._rear_m):
            return False
        if (time.monotonic() - self._rear_ms) > 0.45:
            return False
        return self._rear_m < self.rear_stop_m

    def _begin_avoid_turn(self, now: float) -> None:
        self._avoid_phase = 'turn'
        self._avoid_heading0 = self._heading.yaw_deg
        self._avoid_turn_start = now
        self._turn_target_deg = self._pick_avoid_turn_deg()

    def update_range(self, range_m: float, pan_deg: float | None = None) -> None:
        if pan_deg is not None:
            self._pan_deg = pan_deg
        now = time.monotonic()

        if not math.isfinite(range_m) or range_m <= 0.0:
            # NaN / inf / 0 / -1 = no echo or fault: forward is NOT clear.
            self._range_m = float('nan')
            self._fwd_m = float('nan')
            self._last_range_ms = now
            return

        bucket = self._pan_bucket(self._pan_deg)
        self._pan_bucket_range[bucket] = (range_m, now)
        if math.isnan(self._pan_best_m) or range_m >= self._pan_best_m:
            self._pan_best_m = range_m
            self._pan_best_deg = self._pan_deg
            self._pan_best_ms = now
        elif now - self._pan_best_ms > self._pan_best_ttl_s:
            self._pan_best_m = range_m
            self._pan_best_deg = self._pan_deg
            self._pan_best_ms = now

        centered = abs(self._pan_deg - self.pan_center) <= self.pan_use_deg
        close = range_m < self.close_any_angle_m

        if centered:
            self._fwd_m = range_m
            self._fwd_at = now
            self._range_m = range_m
            self._last_good_m = range_m
            self._last_range_ms = now
            if close:
                self._recent_close_m = range_m
                self._recent_close_ms = now
            return

        if close:
            self._recent_close_m = range_m
            self._recent_close_ms = now

        self._last_range_ms = now

    def _bucket_range(self, bucket: str, max_age_s: float = 1.2) -> float:
        sample = self._pan_bucket_range.get(bucket)
        if sample is None:
            return float('nan')
        value, ts = sample
        if time.monotonic() - ts > max_age_s:
            return float('nan')
        return value

    def _steer_toward_open(self, gain: float = 1.0) -> float:
        """Steer toward the pan angle that recently had the furthest sonar hit."""
        err = self._pan_best_deg - self.pan_center
        if abs(err) < 6.0:
            left = self._bucket_range('left')
            right = self._bucket_range('right')
            if not math.isnan(left) and not math.isnan(right):
                diff = right - left
                if abs(diff) > 0.08:
                    return (left - right) * 0.55 * gain
            return 0.0
        steer_dir = -1.0 if err > 0.0 else 1.0
        return steer_dir * self.steer * gain

    def _curiosity_steer(self, now: float, rng: float) -> tuple[float, bool]:
        """Return (angular, spin_in_place)."""
        if not self.curiosity_enable:
            return 0.0, False
        if self._avoid_active or self._trap_level > 0:
            return 0.0, False
        if now < self._commit_until or now < self._avoid_cooldown_until:
            return 0.0, False

        if now < self._curiosity_until:
            return self._curiosity_w, self._curiosity_spin

        # Blind (nan range, e.g. Tracker with no ToF): still do random turns.
        # Only suppress curiosity when we *have* range and it is already close.
        if (not math.isnan(rng) and rng < self._slow_threshold(self.cruise)) or now < self._next_curiosity_at:
            return 0.0, False

        if math.isnan(rng):
            self._next_curiosity_at = now + random.uniform(2.5, 5.5)
            spin_p, fire_p = 0.55, 0.70
            spin_w = (0.45, 0.75)
            curve_w = (0.18, 0.40)
            hold = (0.8, 1.8)
        else:
            self._next_curiosity_at = now + random.uniform(10.0, 20.0)
            spin_p, fire_p = 0.65, 0.45
            spin_w = (0.55, 0.90)
            curve_w = (0.15, 0.35)
            hold = (0.7, 1.6)

        if random.random() > fire_p:
            return 0.0, False

        self._curiosity_spin = random.random() < spin_p
        self._curiosity_w = random.choice([-1.0, 1.0]) * (
            random.uniform(*spin_w) if self._curiosity_spin else random.uniform(*curve_w)
        )
        self._curiosity_until = now + random.uniform(*hold)
        self._heading.release()
        self._heading.reset_target()
        return self._curiosity_w, self._curiosity_spin

    def _update_heading_arm(self, speed: float, now: float) -> None:
        cruising = speed >= self.cruise * 0.85 and not self._avoid_active
        if cruising:
            if self._cruise_since <= 0.0:
                self._cruise_since = now
            elif now - self._cruise_since >= 0.9 and not self._heading._armed:
                self._heading.reset_target()
        else:
            self._cruise_since = 0.0
            self._heading.release()

    def _blend_steer(self, msg: Twist, speed: float, now: float, *, explore_gain: float) -> None:
        self._update_heading_arm(speed, now)
        rng = self._effective_range()

        if now < self._commit_until:
            msg.linear.x = speed
            msg.angular.z = self._heading.correction()
            return

        curiosity_w, curiosity_spin = self._curiosity_steer(now, rng)
        if curiosity_spin and abs(curiosity_w) > 0.05:
            msg.linear.x = 0.0
            msg.angular.z = curiosity_w
            return

        w = self._heading.correction()
        open_w = 0.0
        if now >= self._avoid_cooldown_until:
            open_w = self._steer_toward_open(gain=explore_gain)
            if (
                self._heading._armed
                and not math.isnan(rng)
                and rng >= self._slow_threshold(speed)
            ):
                open_w *= 0.25
        w += open_w
        w += curiosity_w
        slow_lim = self._slow_threshold(speed)
        if not math.isnan(rng) and rng < slow_lim and now >= self._avoid_cooldown_until:
            w += self._steer_toward_open(gain=1.1)
        msg.linear.x = speed
        msg.angular.z = w
        self._heading.learn_drift_trim(self._last_gz, lin=speed, explore_w=open_w + curiosity_w)

    def _forward_range(self) -> float:
        """Forward clearance — centered pan only (ignore side-glance close hits)."""
        now = time.monotonic()
        if not math.isnan(self._range_m) and (now - self._last_range_ms) <= self.range_stale_s:
            return self._range_m
        # No fallback to an older 'last good' reading: an invalid echo means unknown.
        return float('nan')

    def _effective_range(self) -> float:
        """Alias for forward stop / cruise decisions."""
        return self._forward_range()

    def _pick_avoid_turn_deg(self) -> float:
        if self._trap_level >= 2:
            return random.uniform(85.0, 110.0)
        if self._trap_level >= 1:
            return random.uniform(65.0, 85.0)
        return random.uniform(self.turn_min_deg, self.turn_max_deg)

    def _bump_trap(self, now: float) -> None:
        self._avoid_recent = [t for t in self._avoid_recent if now - t < 10.0]
        self._avoid_recent.append(now)
        if len(self._avoid_recent) >= 3:
            self._trap_level = min(2, self._trap_level + 1)

    def _note_close_persist(self, rng: float, now: float) -> bool:
        """True when hugging a wall too long — force breakout avoid."""
        if math.isnan(rng) or rng > self.stop_m + 0.02:
            self._close_persist_since = 0.0
            return False
        if self._close_persist_since <= 0.0:
            self._close_persist_since = now
            return False
        return (now - self._close_persist_since) > 2.5

    def _note_open_road(self, rng: float) -> None:
        if not math.isnan(rng) and rng >= self.slow_m:
            self._trap_level = max(0, self._trap_level - 1)
            if self._trap_level == 0:
                self._avoid_recent.clear()
                self._close_persist_since = 0.0

    def _best_escape_sign(self) -> float:
        """Pick a turn direction from pan buckets — no ping-pong alternation in corners."""
        left = self._bucket_range('left', max_age_s=2.5)
        right = self._bucket_range('right', max_age_s=2.5)
        if not math.isnan(left) and not math.isnan(right):
            if left - right > 0.05:
                return 1.0
            if right - left > 0.05:
                return -1.0
        err = self._pan_best_deg - self.pan_center
        if abs(err) > 10.0:
            return -1.0 if err > 0.0 else 1.0
        # Trap: try the other way on the next escape so we don't grind 270° right.
        if self._trap_level > 0 and len(self._avoid_recent) >= 2:
            return -self._committed_escape_sign
        if self._trap_level > 0:
            return self._committed_escape_sign
        self._turn_sign *= -1.0
        return self._turn_sign

    def _turn_timeout_s(self, target_deg: float) -> float:
        return min(
            self.turn_timeout_max_s,
            self.turn_timeout_base_s * max(target_deg, self.turn_min_deg) / 90.0,
        )

    def _avoid_turn(
        self,
        msg: Twist,
        heading0: float,
        target_deg: float,
        sign: float,
        turn_start: float,
    ) -> bool:
        """Spin in place on carpet; short forward arc otherwise."""
        elapsed = time.monotonic() - turn_start
        yaw = self._heading.yaw_deg
        if elapsed >= self._turn_timeout_s(target_deg):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            return True

        delta = _wrap_deg(yaw - heading0)
        if elapsed >= 0.35 and self._heading.fresh():
            progress = delta * sign
            # Shortest arc only — unwrapped yaw used to keep going to 270° the long way.
            if progress >= target_deg - self.turn_tolerance_deg:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                return True
            if abs(delta) >= 120.0:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                return True
            # Stall/trap: commanding a turn but gyro shows ~no rotation -> end turn, escalate trap.
            stall_s = float(os.environ.get('TURN_STALL_SEC', '1.2'))
            if elapsed >= stall_s and abs(delta) < float(os.environ.get('TURN_STALL_DEG', '4.0')):
                self.turn_stalls = getattr(self, turn_stalls, 0) + 1
                self._bump_trap(time.monotonic())
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                return True

        if self._wants_spin_avoid() and not self._avoid_too_close_for_spin():
            msg.linear.x = 0.0
            msg.angular.z = min(self.spin_rate, self.avoid_spin_max) * sign
            return False

        open_w = self._steer_toward_open(gain=0.8)
        turn_w = min(self.steer * 0.65, self.avoid_spin_max) * sign
        if self._trap_level < 1 and abs(open_w) > 0.03:
            turn_w = 0.60 * turn_w + 0.40 * open_w
        msg.linear.x = self._cap_escape_lin(self.avoid_arc_lin)
        msg.angular.z = turn_w
        return False

    def _start_avoid(self, now: float, range_m: float) -> None:
        self._bump_trap(now)
        sign = self._best_escape_sign()
        self._committed_escape_sign = sign
        self._avoid_sign = sign
        self._avoid_active = True
        self._avoid_start_range = range_m
        self._heading.release()

        reverse_s = self.avoid_backup_s
        if range_m < self.stop_m + 0.08:
            reverse_s += self.avoid_close_backup_extra_s
        if self._trap_level >= 1:
            reverse_s = max(reverse_s, 0.65)

        if self.scan_on_trap and self._trap_level >= 2:
            self._avoid_phase = 'scan'
            self._avoid_backup_until = now + 0.80
        elif range_m < self.stop_m + 0.03:
            self._avoid_phase = 'reverse'
            self._avoid_backup_until = now + reverse_s
        else:
            self._avoid_phase = 'backup'
            self._avoid_backup_until = now + max(0.25, reverse_s * 0.45)

    def _finish_avoid(self, now: float) -> None:
        self._avoid_active = False
        self._avoid_phase = ''
        cooldown = self.avoid_trap_cooldown_s if self._trap_level else self.avoid_cooldown_s
        self._avoid_cooldown_until = now + cooldown
        self._commit_until = now + self.commit_cruise_s
        self._heading.reset_target()

        if self._trap_level > 0:
            self._trap_level -= 1
            if self._trap_level == 0:
                self._avoid_recent.clear()
                self._close_persist_since = 0.0
    def _fill_avoid(self, msg: Twist, now: float) -> None:
        if self._avoid_phase == 'scan':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if now >= self._avoid_backup_until:
                self._avoid_sign = self._best_escape_sign()
                self._committed_escape_sign = self._avoid_sign
                self._avoid_phase = 'reverse'
                self._avoid_backup_until = now + self.avoid_backup_s
            return

        if self._avoid_phase == 'reverse':
            if self._rear_blocked():
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self._begin_avoid_turn(now)
                return
            msg.linear.x = self._cap_escape_lin(self.avoid_reverse_lin)
            msg.angular.z = self._avoid_sign * 0.05
            if now >= self._avoid_backup_until:
                self._begin_avoid_turn(now)
            return

        if self._avoid_phase == 'backup':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if now >= self._avoid_backup_until:
                self._begin_avoid_turn(now)
            return

        if self._avoid_phase == 'turn':
            done = self._avoid_turn(
                msg,
                self._avoid_heading0,
                self._turn_target_deg,
                self._avoid_sign,
                self._avoid_turn_start,
            )
            if done:
                self._finish_avoid(now)

    def fill_twist(self, msg: Twist) -> None:
        if not self.imu_ready():
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self._last_driving = False
            return

        now = time.monotonic()
        rng = self._effective_range()
        stale = now - self._last_range_ms > self.range_stale_s

        if self._avoid_active:
            self._fill_avoid(msg, now)
            self._apply_speed_limits(msg, now)
            self._last_driving = msg.linear.x > 0.05
            return

        if math.isnan(rng):
            # No valid forward range: hold still (never cruise blind).
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self._apply_speed_limits(msg, now)
            self._last_driving = msg.linear.x > 0.05
            return

        if rng < self._stop_threshold(0.0):
            if self._note_close_persist(rng, now):
                self._trap_level = max(self._trap_level, 2)
                self._avoid_cooldown_until = 0.0
            if now < self._avoid_cooldown_until:
                msg.linear.x = 0.0
                msg.angular.z = self._heading.correction()
                self._apply_speed_limits(msg, now)
                self._last_driving = msg.linear.x > 0.05
                return
            self._start_avoid(now, rng)
            self._fill_avoid(msg, now)
            self._apply_speed_limits(msg, now)
            self._last_driving = msg.linear.x > 0.05
            return

        self._note_open_road(rng)
        slow_lim = self._slow_threshold(self.cruise)
        if rng < slow_lim:
            self._blend_steer(
                msg, self._forward_speed(rng, now, lin_hint=self.cruise), now, explore_gain=0.6
            )
            self._apply_speed_limits(msg, now)
            self._last_driving = msg.linear.x > 0.05
            return

        self._blend_steer(
            msg, self._forward_speed(rng, now, lin_hint=self.cruise), now, explore_gain=0.55
        )
        self._apply_speed_limits(msg, now)
        self._last_driving = msg.linear.x > 0.05

    def apply_sonar_stop(self, msg: Twist, *, stop_m: float | None = None) -> bool:
        limit = self.stop_m if stop_m is None else stop_m
        rng = self._effective_range()
        if math.isnan(rng):
            # Unknown forward range counts as blocked (forward only).
            if msg.linear.x > 0.0:
                msg.linear.x = 0.0
                return True
            return False
        if rng >= limit:
            return False
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        return True

    def forward_block_reason(self, now: float | None = None) -> str | None:
        """None if forward is clear on a valid, fresh, centred range; else why not."""
        now = time.monotonic() if now is None else now
        if math.isnan(self._fwd_m):
            return 'no valid forward range (nan/0/-1 or never seen)'
        age = now - self._fwd_at
        if age > self.fwd_fresh_s:
            return f'forward range stale {age:.2f}s > {self.fwd_fresh_s:.2f}s'
        if self._fwd_m < self.stop_m:
            return f'obstacle {self._fwd_m:.2f}m < stop {self.stop_m:.2f}m'
        return None

    def gate_forward(self, msg: Twist) -> str | None:
        """Final forward gate for every mode: zero positive lin unless forward is clear.

        Reverse and spin-in-place are untouched (rear ToF stop lives on-board).
        Returns the block reason when it zeroed a forward command, else None.
        """
        now = time.monotonic()
        dt = min(max(now - self._gate_at, 0.0), 0.2)
        self._gate_at = now
        if msg.linear.x <= 0.0:
            self._gate_lin = 0.0
            return None
        reason = self.forward_block_reason(now)
        if reason is not None:
            msg.linear.x = 0.0
            self._gate_lin = 0.0
            return reason
        # Taper as the reading ages (reaches 0 exactly at fwd_fresh_s).
        age = now - self._fwd_at
        if age > self.fwd_ramp_s and self.fwd_fresh_s > self.fwd_ramp_s:
            k = max(0.0, (self.fwd_fresh_s - age) / (self.fwd_fresh_s - self.fwd_ramp_s))
            msg.linear.x *= k
        # Accel cap up only; slowing is never delayed.
        if self.fwd_accel > 0.0:
            msg.linear.x = min(msg.linear.x, self._gate_lin + self.fwd_accel * dt)
        self._gate_lin = msg.linear.x
        return None


@dataclass
class IrSteerConfig:
    cruise: float = 0.24
    pan_center: float = 90.0
    pan_deadband: float = 8.0
    pan_align_deg: float = 28.0
    turn_gain: float = 0.022
    spin_rate: float = 0.45
    align_creep: float = 0.14


def steer_ir_beacon(msg: Twist, *, pan_deg: float, ir_now: bool, cfg: IrSteerConfig) -> None:
    """Chase IR beacon — creep+arc when misaligned; cruise when centered. No endless spin."""
    if not ir_now:
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        return

    err = pan_deg - cfg.pan_center
    w = err * cfg.turn_gain

    if abs(err) > cfg.pan_align_deg:
        # Wide miss — arc toward beacon, never pure spin-in-place.
        msg.linear.x = cfg.align_creep
        spin = min(cfg.spin_rate, abs(w) * 1.8)
        msg.angular.z = spin if w > 0.0 else -spin
    elif abs(err) > cfg.pan_deadband:
        msg.linear.x = cfg.cruise * 0.55
        msg.angular.z = w
    else:
        msg.linear.x = cfg.cruise
        msg.angular.z = w * 0.45
