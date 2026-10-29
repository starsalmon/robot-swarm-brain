#!/usr/bin/env python3
"""LEGO rover explore brain — dockerhost, WiFi micro-ROS.

Tap **Go** on the rover to start/stop. Explore vs wall-follow is selected on the
ESP mode menu (`/rover/drive_mode`). `ROVER_MODE` env is only the startup default
if no drive_mode message has arrived yet.
"""
from __future__ import annotations

import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32, Float32MultiArray, UInt8, UInt32

from explore_controller import ExploreController
from hallway_follow import HallwayConfig, HallwayFollow
from rover_qos import CMD_VEL_QOS
from cmd_vel_slew import CmdVelSlew
from rover_motion import shape_explore_cmd
from rover_wheel_odom import WheelOdometry
from fleet_meet_spin import MeetSpinController

BTN_ESTOP_LONG = 3
DRIVE_EXPLORE = 0
DRIVE_WALL = 1


class RoverBrain(Node):
    TICK = 0.05
    HEARTBEAT_PERIOD = 0.5

    def __init__(self) -> None:
        env_mode = os.environ.get('ROVER_MODE', 'explore').strip().lower()
        default_drive = DRIVE_WALL if env_mode == 'hallway' else DRIVE_EXPLORE
        super().__init__('rover_brain')

        # Arbiter mode (rover-behaviour owns /cmd_vel + /rover/session): explore output goes to
        # ROVER_CMD_TOPIC, the session is driven by /rover/behaviour/explore_active, allow_drive is
        # enforced by the arbiter (explore keeps thinking while driving is disabled = dry-run).
        self._arbiter = os.environ.get('ROVER_EXPLORE_ARBITER', '0').strip().lower() in ('1', 'true', 'yes')
        cmd_topic = os.environ.get('ROVER_CMD_TOPIC', '/cmd_vel')
        ses_topic = os.environ.get('ROVER_SESSION_TOPIC', '/rover/session')
        self.pub = self.create_publisher(Twist, cmd_topic, CMD_VEL_QOS)
        self.session_pub = self.create_publisher(Bool, ses_topic, 10)
        self.heartbeat_pub = self.create_publisher(UInt32, '/rover/heartbeat', 10)
        self.sonar_cal_pub = self.create_publisher(Bool, '/rover/sonar/cal_sweep', 10)

        self._explore = ExploreController(log=self.get_logger().info)
        self._hallway: HallwayFollow | None = None
        self._drive_mode = default_drive
        self._drive_mode_from_esp = False

        self._heartbeat_n = 0
        self._session_active = False
        self._last_session_pub: bool | None = None
        self._pan_deg = float(os.environ.get('PAN_CENTER', '90'))
        self._cal_last: bool | None = None
        self._sonar_avoid_active = False
        self._sonar_avoid_since: float | None = None
        self._brake_scan_until = 0.0
        self._brake_scan_cooldown_until = 0.0
        self._slew = CmdVelSlew.from_env()
        self._slew_t = time.monotonic()
        self._wheels = WheelOdometry()
        self._wheel_dist_m = 0.0
        self.get_logger().info(f'wheel odom: {self._wheels.config_summary()}')
        self._meet = MeetSpinController(fleet_id=0)
        self._meet_was_active = False
        self._last_imu_t = time.monotonic()
        self._front_c = float('nan')
        self._front_l = float('nan')
        self._front_r = float('nan')
        self._nav2 = os.environ.get('ROVER_NAV2', '0').strip().lower() in ('1', 'true', 'yes')
        self._nav_cmd = (0.0, 0.0)
        self._nav_cmd_t = 0.0
        self._nav_smooth = (0.0, 0.0)
        if self._nav2:
            self.create_subscription(Twist, '/cmd_vel_nav', self._on_nav_cmd, 10)
            self.get_logger().info('motion source: Nav2 /cmd_vel_nav (Go still gates the wheels)')

        self._allow_drive = True
        if self._arbiter:
            self.create_subscription(Bool, '/rover/behaviour/explore_active',
                                     lambda m: self._set_session(bool(m.data), 'arbiter'), 10)
            self.get_logger().info(f'ARBITER mode: explore -> {cmd_topic}, session via /rover/behaviour/explore_active')
        else:
            self.create_subscription(Bool, '/fleet/allow_drive', self._on_allow_drive, 10)

        self.create_subscription(Bool, '/rover/button', self._on_button, 10)
        self.create_subscription(UInt8, '/rover/button_event', self._on_button_event, 10)
        self.create_subscription(UInt8, '/rover/drive_mode', self._on_drive_mode, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.create_subscription(Bool, '/rover/bump', self._on_bump, 10)
        self.create_subscription(Bool, '/rover/stall', self._on_stall, 10)
        self.create_subscription(UInt32, '/rover/wheel/left_ticks', self._on_wheel_left, 10)
        self.create_subscription(UInt32, '/rover/wheel/right_ticks', self._on_wheel_right, 10)
        self.create_subscription(Range, '/rover/sonar/range', self._on_range, 10)
        self.create_subscription(Float32, '/rover/sonar/pan_deg', self._on_pan, 10)
        self.create_subscription(Bool, '/rover/sonar/avoid_active', self._on_sonar_avoid, 10)
        self.create_subscription(Range, '/rover/tof/left', self._on_tof_left, 10)
        self.create_subscription(Range, '/rover/tof/right', self._on_tof_right, 10)
        self.create_subscription(Range, '/rover/tof/front', self._on_tof_front, 10)
        self.create_subscription(Range, '/rover/tof/front_left', self._on_tof_front_left, 10)
        self.create_subscription(Range, '/rover/tof/front_right', self._on_tof_front_right, 10)
        self.create_subscription(Float32MultiArray, '/rover/tof/l8_cols', self._on_l8_cols, 10)
        self.create_subscription(UInt8, '/rover/ir/hits', self._on_ir_hits, 10)
        self.create_subscription(UInt8, '/rover/ir/peer', self._on_ir_peer, 10)

        self.create_timer(self.TICK, self._tick)
        self.create_timer(self.HEARTBEAT_PERIOD, self._send_heartbeat)
        self._log_drive_mode('startup default' if env_mode == 'hallway' else 'startup')

    def _wall_follow(self) -> bool:
        return self._drive_mode == DRIVE_WALL

    def _ensure_hallway(self) -> HallwayFollow:
        if self._hallway is None:
            self._hallway = HallwayFollow(HallwayConfig.from_env())
        return self._hallway

    def _log_drive_mode(self, reason: str) -> None:
        label = 'wall follow' if self._wall_follow() else 'explore'
        self.get_logger().info(f'rover_brain: {label} ({reason})')

    def _set_drive_mode(self, mode: int, *, reason: str) -> None:
        mode = DRIVE_WALL if mode == DRIVE_WALL else DRIVE_EXPLORE
        if mode == self._drive_mode:
            return
        if self._session_active:
            self._set_session(False, f'{reason} (mode change)')
        self._drive_mode = mode
        if self._wall_follow():
            self._hallway = HallwayFollow(HallwayConfig.from_env())
        self._log_drive_mode(reason)

    def _on_drive_mode(self, msg: UInt8) -> None:
        self._drive_mode_from_esp = True
        self._set_drive_mode(int(msg.data), reason='ESP menu')

    def _publish_stop(self, count: int = 8) -> None:
        stop = Twist()
        for _ in range(count):
            self.pub.publish(stop)

    def _set_session(self, active: bool, reason: str) -> None:
        if active == self._session_active:
            return
        self._session_active = active
        if active:
            if self._wall_follow():
                self._hallway = HallwayFollow(HallwayConfig.from_env())
            else:
                self._explore.reset()
            self._slew.reset()
            self._slew_t = time.monotonic()
            self._nav_smooth = (0.0, 0.0)
            self.get_logger().info(f'session START ({reason}) — {"wall" if self._wall_follow() else "explore"}')
        else:
            self.get_logger().info(f'session STOP ({reason})')
            try:
                self._publish_stop()
            except Exception:
                pass
        self._publish_session()

    def _publish_session(self, *, force: bool = False) -> None:
        if not force and self._session_active == self._last_session_pub:
            return
        msg = Bool()
        msg.data = self._session_active
        self.session_pub.publish(msg)
        self._last_session_pub = self._session_active

    def _on_allow_drive(self, msg: Bool) -> None:
        allow = bool(msg.data)
        if allow == self._allow_drive:
            return
        self._allow_drive = allow
        if not allow and self._session_active:
            self._set_session(False, 'HA stop all')
        self.get_logger().info(f'allow_drive={allow}')

    def _on_button(self, msg: Bool) -> None:
        if not msg.data or self._arbiter:
            return
        if not self._allow_drive:
            self.get_logger().info('Go ignored — wall panel halt (Allow drive is off)')
            return
        if self._session_active:
            self._set_session(False, 'Go tap')
        else:
            self._set_session(True, 'Go tap')

    def _on_button_event(self, msg: UInt8) -> None:
        if int(msg.data) == BTN_ESTOP_LONG:
            self._set_session(False, 'E-stop long')

    def _on_range(self, msg: Range) -> None:
        rng = float(msg.range)
        if self._wall_follow():
            return
        if self._session_active:
            self._explore.on_sonar_range(rng)

    def _on_pan(self, msg: Float32) -> None:
        self._pan_deg = float(msg.data)
        if not self._wall_follow() and self._session_active:
            self._explore.on_pan(self._pan_deg)

    def _on_sonar_avoid(self, msg: Bool) -> None:
        active = bool(msg.data)
        now = time.monotonic()
        if active and not self._sonar_avoid_active:
            self._sonar_avoid_since = now
        if not active:
            self._sonar_avoid_since = None
        self._sonar_avoid_active = active
        if not self._wall_follow() and self._session_active:
            self._explore.on_sonar_avoid(active)

    def _on_ir_peer(self, msg: UInt8) -> None:
        if self._session_active:
            self._meet.on_peer(int(msg.data))

    def _on_imu(self, msg: Imu) -> None:
        now = time.monotonic()
        dt = now - self._last_imu_t
        self._last_imu_t = now
        gz = float(msg.angular_velocity.z)
        self._meet.on_imu_gz(gz, dt)
        if not self._wall_follow() and self._session_active:
            self._explore.on_imu(gz)

    def _on_bump(self, msg: Bool) -> None:
        if not msg.data or self._wall_follow() or not self._session_active:
            return
        # A jolt while the wheels are still turning is not a reason to spin.
        # The last runs were bump → 60° arc → drive → bump, over and over.
        if self._wheels.wheels_turning() and not self._explore._proximity.is_nose_jammed():
            self.get_logger().info('bump ignored — wheels still turning, nose not jammed')
            return
        self._explore.on_bump()

    def _on_stall(self, msg: Bool) -> None:
        if not msg.data or self._wall_follow() or not self._session_active:
            return
        if self._wheels.wheels_turning():
            self.get_logger().info('stall ignored — wheel ticks are still moving')
            return
        self._explore.on_stall()

    def _on_wheel_left(self, msg: UInt32) -> None:
        self._wheels.on_left(int(msg.data))
        self._note_wheel_distance()

    def _on_wheel_right(self, msg: UInt32) -> None:
        self._wheels.on_right(int(msg.data))
        self._note_wheel_distance()

    def _note_wheel_distance(self) -> None:
        dist = self._wheels.snapshot().distance_m
        delta = dist - self._wheel_dist_m
        if delta > 0.0:
            self._wheel_dist_m = dist
            if self._session_active and not self._wall_follow():
                self._explore.add_wheel_travel(delta)

    def _on_tof_left(self, msg: Range) -> None:
        rng = float(msg.range)
        self._ensure_hallway().update_left(rng)
        if self._session_active:
            self._explore.on_tof_left(rng)

    def _on_tof_right(self, msg: Range) -> None:
        rng = float(msg.range)
        self._ensure_hallway().update_right(rng)
        if self._session_active:
            self._explore.on_tof_right(rng)

    def _push_front_tof(self) -> None:
        if math.isfinite(self._front_c) and self._front_c > 0.02:
            self._ensure_hallway().update_front(self._front_c)
        if self._session_active:
            self._explore.on_tof_front(self._front_c, self._front_l, self._front_r)

    def _on_tof_front(self, msg: Range) -> None:
        self._front_c = float(msg.range)
        self._push_front_tof()

    def _on_tof_front_left(self, msg: Range) -> None:
        self._front_l = float(msg.range)
        self._push_front_tof()

    def _on_tof_front_right(self, msg: Range) -> None:
        self._front_r = float(msg.range)
        self._push_front_tof()

    def _on_l8_cols(self, msg: Float32MultiArray) -> None:
        cols = [float(v) for v in msg.data]
        self._explore.on_l8_cols(cols)

    def _on_ir_hits(self, msg: UInt8) -> None:
        bits = int(msg.data)
        if self._wall_follow():
            self._ensure_hallway().update_ir_hits(bits)
        elif self._session_active:
            self._explore.on_ir_hits(bits)

    def _on_nav_cmd(self, msg: Twist) -> None:
        lin = max(-0.12, min(0.15, float(msg.linear.x)))
        ang = max(-0.35, min(0.35, float(msg.angular.z)))
        # Nav2 publishes ~40 Hz with hard zeros when stuck — low-pass so we
        # don't stamp explore's smooth output to a stop every other frame.
        alpha = 0.18
        slin, sang = self._nav_smooth
        self._nav_smooth = (
            slin + alpha * (lin - slin),
            sang + alpha * (ang - sang),
        )
        self._nav_cmd = self._nav_smooth
        self._nav_cmd_t = time.monotonic()

    def _nav_cmd_live(self) -> bool:
        if not self._nav2:
            return False
        if time.monotonic() - self._nav_cmd_t > 0.5:
            return False
        lin, ang = self._nav_cmd
        # Ignore Nav2 "stop" — explore keeps cruise; only blend real motion.
        return abs(lin) > 0.04 or abs(ang) > 0.04

    def _blend_nav(self, lin: float, ang: float) -> tuple[float, float]:
        if not self._nav_cmd_live() or not self._explore.allows_nav_blend():
            return lin, ang
        nlin, nang = self._nav_cmd
        return (
            0.72 * lin + 0.28 * nlin,
            0.65 * ang + 0.35 * nang,
        )

    def _tick(self) -> None:
        msg = Twist()
        meet_cmd = self._meet.tick()
        meet_ok = self._session_active or self._meet.active
        if meet_cmd is not None and meet_ok:
            if not self._meet_was_active:
                self.get_logger().info(
                    f'meet dance — saw {self._meet.peer_label()}, spinning 2×360°'
                )
            self._meet_was_active = True
            msg.linear.x, msg.angular.z = meet_cmd
        elif self._meet_was_active:
            self._meet_was_active = False
            self.get_logger().info('meet dance done')
        elif self._session_active:
            if self._wall_follow():
                self._ensure_hallway().fill_twist(msg)
            else:
                self._explore.set_wheels_moving(self._wheels.wheels_turning())
                lin, ang = self._explore.tick()
                steer_cap = float(os.environ.get('ROVER_AUTO_MAX_ANGULAR', '0.18'))
                lin_cap = float(os.environ.get('ROVER_AUTO_MAX_LINEAR', '0.14'))
                lin, ang = shape_explore_cmd(
                    self._explore._proximity,
                    lin,
                    ang,
                    hints=self._explore._motion_hints,
                    max_lin=lin_cap,
                    max_steer=steer_cap,
                )
                msg.linear.x, msg.angular.z = self._blend_nav(lin, ang)
        now = time.monotonic()
        dt = now - self._slew_t
        self._slew_t = now
        if dt <= 0.0 or dt > 0.2:
            dt = self.TICK
        lin, ang = self._slew.step(
            float(msg.linear.x),
            float(msg.angular.z),
            dt,
            hard_stop=not self._session_active,
        )
        msg.linear.x = lin
        msg.angular.z = ang
        # Wheel stall only on straight cruise — turns + escape arcs look "stalled"
        # to tick math and were spamming reverse→spin-right loops.
        if (
            self._session_active
            and self._explore.allows_wheel_stall_check()
            and abs(ang) < 0.04
            and lin > 0.12
            and not self._meet.active
            and self._wheels.check_stall(lin, ang, time.monotonic())
        ):
            if not self._wheels.wheels_turning():
                self.get_logger().info('wheel stall — straight cruise, ticks stopped')
                self._explore.on_stall()
        self.pub.publish(msg)
        self._publish_cal_sweep()

    def _publish_cal_sweep(self) -> None:
        now = time.monotonic()
        want = False
        if self._session_active:
            # Primary: explore controller requests scan windows.
            if not self._wall_follow():
                want = self._explore.wants_cal_sweep(now)

            # Rear sonar avoid must not trigger a sit-and-scan. The tail is
            # not a pathfinder; L8 peels the nose.
        if want == self._cal_last:
            return
        self._cal_last = want
        m = Bool()
        m.data = bool(want)
        self.sonar_cal_pub.publish(m)

    def _send_heartbeat(self) -> None:
        self._heartbeat_n = (self._heartbeat_n + 1) & 0x7FFFFFFF
        driving_bit = 0
        driving = False
        if self._session_active:
            if self._wall_follow():
                driving = self._ensure_hallway()._last_driving
            else:
                driving = self._explore._last_driving
        if driving:
            driving_bit = 0x80000000
        hb = UInt32()
        hb.data = self._heartbeat_n | driving_bit
        self.heartbeat_pub.publish(hb)
        self._publish_session(force=self._session_active)


def main() -> None:
    rclpy.init()
    node = RoverBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._set_session(False, 'shutdown')
        node._explore.shutdown()
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
