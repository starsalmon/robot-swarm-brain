#!/usr/bin/env python3
"""Extra ROS → Home Assistant MQTT sensors for the existing fleet-ha bridge.

Throttled so the robots and MQTT broker see almost no extra load:
  - Fast values (ToF, angles, speed, mode): at most 1 Hz, or 30 s if unchanged.
  - Slow values (Wi-Fi, extra battery/solar): on change, else every 30 s.

This module never publishes /fleet/allow_drive or any cmd_vel. Liveness fo
each robot comes from ESP-originated topics, not NUC brain publishers.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Illuminance, Imu, Range
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String, UInt32

BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
REL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

FAST_S = 1.0
SLOW_S = 30.0
HEARTBEAT_S = 30.0
BEEPER_M_PER_COUNT = math.pi * 0.030 / 980.0


def subscribe_both(node: Node, typ, topic: str, cb) -> None:
    """Subscribe BEST_EFFORT and RELIABLE so micro-ROS default and sensor QoS both work."""
    node.create_subscription(typ, topic, cb, BE_QOS)
    node.create_subscription(typ, topic, cb, REL_QOS)


def _finite(v: float) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(v)


def range_cm(msg: Range) -> float | None:
    r = float(msg.range)
    if not _finite(r) or r < 0.0:
        return None
    mx = float(msg.max_range) if _finite(float(msg.max_range)) and msg.max_range > 0 else 0.0
    if mx and r >= mx * 0.99:
        return None
    return round(r * 100.0, 1)


def l8_min_cm(msg: Float32MultiArray) -> float | None:
    vals = []
    for v in msg.data:
        x = float(v)
        if _finite(x) and x > 0.0:
            vals.append(x)
    if not vals:
        return None
    return round(min(vals) * 100.0, 1)


def imu_has_orientation(msg: Imu) -> bool:
    cov = list(msg.orientation_covariance)
    if cov and cov[0] < 0:
        return False
    q = msg.orientation
    # Identity (w=1) with zero xy z is the micro-ROS "no estimate" default.
    if abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z) < 1e-6 and abs(abs(q.w) - 1.0) < 1e-6:
        return cov and cov[0] > 0
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    return n > 0.5


def quat_to_rpy_deg(q) -> tuple[float, float, float]:
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(90.0, sinp)
    else:
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def accel_to_rp_deg(msg: Imu) -> tuple[float, float] | None:
    ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
    if not (_finite(ax) and _finite(ay) and _finite(az)):
        return None
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    if mag < 1.0:  # m/s²; gravity should be ~9.8
        return None
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return roll, pitch


def imu_rpy(msg: Imu) -> tuple[float | None, float | None, float | None]:
    if imu_has_orientation(msg):
        r, p, y = quat_to_rpy_deg(msg.orientation)
        return (round(r, 1), round(p, 1), round(y, 1))
    rp = accel_to_rp_deg(msg)
    if rp is None:
        return (None, None, None)
    return (round(rp[0], 1), round(rp[1], 1), None)


@dataclass
class PubSlot:
    period: float
    eps: float | None = None
    last_pub: float = 0.0
    last_val: Any = None

    def should(self, val: Any, now: float) -> bool:
        if val is None:
            return False
        changed = self.last_val is None
        if not changed:
            if self.eps is not None and isinstance(val, (int, float)) and isinstance(self.last_val, (int, float)):
                changed = abs(float(val) - float(self.last_val)) >= self.eps
            else:
                changed = val != self.last_val
        if not changed and (now - self.last_pub) < HEARTBEAT_S:
            return False
        if (now - self.last_pub) < self.period:
            return False
        return True

    def mark(self, val: Any, now: float) -> None:
        self.last_pub = now
        self.last_val = val


@dataclass
class BotExtras:
    slug: st
    # latest values
    mode: str | None = None
    tof: dict = field(default_factory=dict)  # name -> cm
    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None
    speed: float | None = None
    wifi_rssi: float | None = None
    extras: dict = field(default_factory=dict)  # name -> (value, unit meta handled in discovery)
    # tick bookkeeping
    tick_t: float | None = None
    tick_l: int | None = None
    tick_r: int | None = None
    m_per_count: float = BEEPER_M_PER_COUNT
    # publish slots created late
    slots: dict = field(default_factory=dict)


class FleetHaSensors:
    def __init__(self, node: Node, mqtt, robots: dict, mqtt_base: str, ha_prefix: str, pub_cfg: Callable):
        self._node = node
        self._mqtt = mqtt
        self._robots = robots
        self._base = mqtt_base
        self._ha = ha_prefix
        self._pub_cfg = pub_cfg
        self._extras: dict[str, BotExtras] = {}
        self._online_last: dict[str, bool] = {}

        for slug, bot in robots.items():
            self._extras[slug] = BotExtras(slug=slug)
            self._wire_robot(slug, bot)

    def _wire_robot(self, slug: str, bot) -> None:
        ex = self._extras[slug]
        if slug == 'rover':
            subscribe_both(self._node, Imu, '/imu/data', self._mk_imu(slug, counts_as_live=True))
            for name, topic in (
                ('front', '/rover/tof/front'),
                ('left', '/rover/tof/left'),
                ('right', '/rover/tof/right'),
                ('front_left', '/rover/tof/front_left'),
                ('front_right', '/rover/tof/front_right'),
            ):
                subscribe_both(self._node, Range, topic, self._mk_range(slug, name, live=True))
            subscribe_both(self._node, Float32MultiArray, '/rover/tof/l8_cols', self._mk_l8(slug))
            subscribe_both(self._node, Float32MultiArray, '/rover/tof/wifi', self._mk_wifi(slug))
            subscribe_both(self._node, String, '/rover/behaviour/status', self._mk_behaviour(slug))
            subscribe_both(self._node, Odometry, '/rover/odom/wheel', self._mk_odom_speed(slug))
        elif slug == 'beeper':
            subscribe_both(self._node, Imu, '/bot3/imu/data', self._mk_imu(slug, counts_as_live=True))
            subscribe_both(self._node, Range, '/bot3/tof/front', self._mk_range(slug, 'front', live=True))
            subscribe_both(self._node, Int32, '/bot3/wheel/left_ticks', self._mk_tick(slug, 'l'))
            subscribe_both(self._node, Int32, '/bot3/wheel/right_ticks', self._mk_tick(slug, 'r'))
            subscribe_both(self._node, Twist, '/bot3/cmd_vel', self._mk_cmd_mode(slug))
            subscribe_both(self._node, Float32, '/bot3/wifi/rssi', self._mk_rssi(slug))
        elif slug == 'tracked':
            subscribe_both(self._node, Imu, '/bot2/imu', self._mk_imu(slug, counts_as_live=True))
            subscribe_both(self._node, Range, '/bot2/sonar/range', self._mk_range(slug, 'front', live=True))
            subscribe_both(self._node, Float32, '/bot2/power/solar_v', self._mk_extra(slug, 'solar', live=True))
            subscribe_both(self._node, Float32, '/bot2/power/battery_v', self._mk_extra(slug, 'main_battery'))
            subscribe_both(self._node, Float32, '/bot2/battery/percent', self._mk_extra(slug, 'fuel_gauge'))
            subscribe_both(self._node, Float32, '/bot2/battery/motor_voltage', self._mk_extra(slug, 'motor_voltage'))
            subscribe_both(self._node, Illuminance, '/bot2/als/illuminance', self._mk_als(slug))
            subscribe_both(self._node, Twist, '/bot2/cmd_vel', self._mk_cmd_mode(slug))
        elif slug == 'roby':
            subscribe_both(self._node, Imu, '/bot1/imu', self._mk_imu(slug, counts_as_live=True))
            subscribe_both(self._node, Range, '/bot1/sonar/range', self._mk_range(slug, 'front', live=True))
            subscribe_both(self._node, Twist, '/bot1/cmd_vel', self._mk_cmd_mode(slug))

    def _touch(self, slug: str) -> None:
        bot = self._robots.get(slug)
        if bot is not None:
            bot.last_seen = time.monotonic()

    def _mk_imu(self, slug: str, counts_as_live: bool):
        def _cb(msg: Imu) -> None:
            if counts_as_live:
                self._touch(slug)
            r, p, y = imu_rpy(msg)
            ex = self._extras[slug]
            ex.roll, ex.pitch, ex.yaw = r, p, y
        return _cb

    def _mk_range(self, slug: str, name: str, live: bool):
        def _cb(msg: Range) -> None:
            if live:
                self._touch(slug)
            self._extras[slug].tof[name] = range_cm(msg)
        return _cb

    def _mk_l8(self, slug: str):
        def _cb(msg: Float32MultiArray) -> None:
            self._touch(slug)
            self._extras[slug].tof['l8_min'] = l8_min_cm(msg)
        return _cb

    def _mk_wifi(self, slug: str):
        def _cb(msg: Float32MultiArray) -> None:
            self._touch(slug)
            if not msg.data:
                return
            rssi = float(msg.data[0])
            if _finite(rssi) and rssi > -126.0:
                self._extras[slug].wifi_rssi = round(rssi, 0)
        return _cb

    def _mk_rssi(self, slug: str):
        def _cb(msg: Float32) -> None:
            self._touch(slug)
            rssi = float(msg.data)
            if _finite(rssi) and rssi > -126.0:
                self._extras[slug].wifi_rssi = round(rssi, 0)
        return _cb

    def _mk_behaviour(self, slug: str):
        def _cb(msg: String) -> None:
            raw = (msg.data or '').strip()
            label = raw
            try:
                data = json.loads(raw)
                state = str(data.get('state') or '').strip()
                go = data.get('go')
                if state:
                    label = state if go in (None, True) else f'{state} (stopped)'
            except Exception:
                if len(label) > 64:
                    label = label[:64]
            if label:
                self._extras[slug].mode = label
        return _cb

    def _mk_cmd_mode(self, slug: str):
        def _cb(msg: Twist) -> None:
            # Inferred only while the ESP is fresh; tick() will not publish mode if offline.
            moving = abs(msg.linear.x) > 0.02 or abs(msg.angular.z) > 0.05
            # Don't override a richer mode source (Rover behaviour).
            if slug == 'rover' and self._extras[slug].mode and not self._extras[slug].mode.startswith(('idle', 'driving')):
                return
            self._extras[slug].mode = 'driving' if moving else 'idle'
        return _cb

    def _mk_odom_speed(self, slug: str):
        def _cb(msg: Odometry) -> None:
            vx = float(msg.twist.twist.linear.x)
            if _finite(vx):
                self._extras[slug].speed = round(vx, 3)
        return _cb

    def _mk_tick(self, slug: str, side: str):
        def _cb(msg) -> None:
            self._touch(slug)
            ex = self._extras[slug]
            now = time.monotonic()
            val = int(msg.data)
            if side == 'l':
                ex.tick_l = val
            else:
                ex.tick_r = val
            if ex.tick_l is None or ex.tick_r is None:
                return
            if ex.tick_t is None:
                ex.tick_t = now
                ex._tick_l0 = ex.tick_l
                ex._tick_r0 = ex.tick_r
                return
            dt = now - ex.tick_t
            if dt < 0.2:
                return
            dl = ex.tick_l - ex._tick_l0
            dr = ex.tick_r - ex._tick_r0
            ex.tick_t = now
            ex._tick_l0 = ex.tick_l
            ex._tick_r0 = ex.tick_r
            # Int32 wrap
            if dl > 2**31:
                dl -= 2**32
            if dl < -2**31:
                dl += 2**32
            if dr > 2**31:
                dr -= 2**32
            if dr < -2**31:
                dr += 2**32
            ex.speed = round(((dl + dr) / 2.0) * ex.m_per_count / dt, 3)
        return _cb

    def _mk_extra(self, slug: str, name: str, live: bool = False):
        def _cb(msg: Float32) -> None:
            if live:
                self._touch(slug)
            v = float(msg.data)
            if not _finite(v):
                return
            # Firmware uses -1 as "not present".
            if name in ('solar', 'main_battery', 'motor_voltage', 'fuel_gauge') and v < 0:
                return
            self._extras[slug].extras[name] = round(v, 2)
        return _cb

    def _mk_als(self, slug: str):
        def _cb(msg: Illuminance) -> None:
            v = float(msg.illuminance)
            if _finite(v) and v >= 0:
                self._extras[slug].extras['illuminance'] = round(v, 1)
        return _cb

    def publish_discovery(self) -> None:
        for slug, bot in self._robots.items():
            dev = {
                'identifiers': [f'lab_robot_{slug}'],
                'name': bot.name,
                'manufacturer': 'lab',
                'model': 'ESP32 fleet',
                'via_device': 'lab_robots',
            }
            nid = f'lab_robot_{slug}'
            avail_robot = [
                {
                    'topic': f'{self._base}/bridge/availability',
                    'payload_available': 'online',
                    'payload_not_available': 'offline',
                },
                {
                    'topic': f'{self._base}/{slug}/availability',
                    'payload_available': 'online',
                    'payload_not_available': 'offline',
                },
            ]
            avail_bridge = [
                {
                    'topic': f'{self._base}/bridge/availability',
                    'payload_available': 'online',
                    'payload_not_available': 'offline',
                },
            ]

            def sensor(object_id, name, topic_key, unit=None, device_class=None, precision=None, icon=None, robot_avail=True, state_class='measurement'):
                payload = {
                    'name': name,
                    'unique_id': f'{nid}_{object_id}',
                    'object_id': f'{slug}_{object_id}',
                    'state_topic': f'{self._base}/{slug}/{topic_key}',
                    'device': dev,
                    'availability': avail_robot if robot_avail else avail_bridge,
                    'availability_mode': 'all',
                }
                if unit:
                    payload['unit_of_measurement'] = unit
                if device_class:
                    payload['device_class'] = device_class
                if state_class:
                    payload['state_class'] = state_class
                if precision is not None:
                    payload['suggested_display_precision'] = precision
                if icon:
                    payload['icon'] = icon
                self._pub_cfg('sensor', nid, object_id, payload)

            # Mode: Rover uses brain state (available whenever the bridge is up).
            # Others use inferred idle/driving, only while the ESP is online.
            sensor(
                'mode',
                'Mode',
                'mode',
                icon='mdi:state-machine',
                robot_avail=(slug != 'rover'),
                state_class=None,
            )

            tof_names = []
            if slug == 'rover':
                tof_names = [
                    ('tof_front', 'ToF front', 'tof_front'),
                    ('tof_left', 'ToF left', 'tof_left'),
                    ('tof_right', 'ToF right', 'tof_right'),
                    ('tof_front_left', 'ToF front left', 'tof_front_left'),
                    ('tof_front_right', 'ToF front right', 'tof_front_right'),
                    ('tof_l8_min', 'ToF L8 min', 'tof_l8_min'),
                ]
            elif slug == 'beeper':
                tof_names = [('tof_front', 'ToF front', 'tof_front')]
            elif slug in ('tracked', 'roby'):
                tof_names = [('tof_front', 'ToF', 'tof_front')]
            for oid, name, key in tof_names:
                sensor(oid, name, key, unit='cm', device_class='distance', precision=1, icon='mdi:arrow-expand-horizontal')

            sensor('pitch', 'Pitch', 'pitch', unit='°', precision=1, icon='mdi:angle-acute', device_class=None)
            sensor('roll', 'Roll', 'roll', unit='°', precision=1, icon='mdi:screen-rotation', device_class=None)
            if slug == 'rover':
                sensor('yaw', 'Yaw', 'yaw', unit='°', precision=1, icon='mdi:compass', device_class=None)
            if slug in ('rover', 'beeper'):
                sensor('speed', 'Speed', 'speed', unit='m/s', precision=3, icon='mdi:speedometer')
            if slug in ('rover', 'beeper'):
                sensor('wifi', 'Wi-Fi', 'wifi', unit='dBm', device_class='signal_strength', precision=0, icon='mdi:wifi')
            if slug == 'tracked':
                sensor('solar', 'Solar', 'solar', unit='V', device_class='voltage', precision=2, icon='mdi:solar-power')
                sensor('main_battery', 'Main battery', 'main_battery', unit='V', device_class='voltage', precision=2)
                sensor('fuel_gauge', 'Fuel gauge', 'fuel_gauge', unit='%', device_class='battery', precision=0)
                sensor('motor_voltage', 'Motor voltage', 'motor_voltage', unit='V', device_class='voltage', precision=2)
                sensor('illuminance', 'Light', 'illuminance', unit='lx', device_class='illuminance', precision=0, icon='mdi:brightness-5')

    def _pub(self, slug: str, key: str, value: Any, retain: bool = True) -> None:
        if value is None:
            return
        if isinstance(value, float):
            if value == int(value) and key in ('wifi', 'fuel_gauge'):
                payload = str(int(value))
            else:
                payload = f'{value:.3f}'.rstrip('0').rstrip('.') if abs(value) < 10 else f'{value:.2f}'
                if key == 'speed':
                    payload = f'{value:.3f}'
                elif key in ('pitch', 'roll', 'yaw') or key.startswith('tof'):
                    payload = f'{value:.1f}'
                elif key in ('solar', 'main_battery', 'motor_voltage'):
                    payload = f'{value:.2f}'
        else:
            payload = str(value)
        self._mqtt.publish(f'{self._base}/{slug}/{key}', payload, qos=1, retain=retain)

    def _slot(self, slug: str, key: str, period: float, eps: float | None) -> PubSlot:
        ex = self._extras[slug]
        slot = ex.slots.get(key)
        if slot is None:
            slot = PubSlot(period=period, eps=eps)
            ex.slots[key] = slot
        return slot

    def _maybe(self, slug: str, key: str, value: Any, period: float, eps: float | None = None) -> None:
        if value is None:
            return
        slot = self._slot(slug, key, period, eps)
        now = time.monotonic()
        if slot.should(value, now):
            self._pub(slug, key, value)
            slot.mark(value, now)

    def tick(self, now: float, online: dict[str, bool]) -> None:
        if not self._mqtt.is_connected():
            return
        for slug, bot in self._robots.items():
            is_on = bool(online.get(slug, False))
            prev = self._online_last.get(slug)
            if prev is None or prev != is_on:
                self._mqtt.publish(
                    f'{self._base}/{slug}/availability',
                    'online' if is_on else 'offline',
                    qos=1,
                    retain=True,
                )
                self._online_last[slug] = is_on
            ex = self._extras[slug]
            # Rover mode comes from the NUC brain and stays available.
            # Other robots only expose inferred idle/driving while the ESP is live.
            if slug == 'rover':
                self._maybe(slug, 'mode', ex.mode, FAST_S)
            if not is_on:
                continue
            if slug != 'rover':
                self._maybe(slug, 'mode', ex.mode, FAST_S)
            for name, val in ex.tof.items():
                key = 'tof_l8_min' if name == 'l8_min' else f'tof_{name}'
                self._maybe(slug, key, val, FAST_S, eps=0.3)
            self._maybe(slug, 'pitch', ex.pitch, FAST_S, eps=0.3)
            self._maybe(slug, 'roll', ex.roll, FAST_S, eps=0.3)
            self._maybe(slug, 'yaw', ex.yaw, FAST_S, eps=0.3)
            self._maybe(slug, 'speed', ex.speed, FAST_S, eps=0.01)
            self._maybe(slug, 'wifi', ex.wifi_rssi, SLOW_S, eps=1.0)
            # extras
            key_map = {
                'solar': ('solar', SLOW_S, 0.02),
                'main_battery': ('main_battery', SLOW_S, 0.02),
                'fuel_gauge': ('fuel_gauge', SLOW_S, 0.5),
                'motor_voltage': ('motor_voltage', SLOW_S, 0.05),
                'illuminance': ('illuminance', SLOW_S, 1.0),
            }
            for name, val in ex.extras.items():
                key, period, eps = key_map.get(name, (name, SLOW_S, 0.05))
                self._maybe(slug, key, val, period, eps)


def attach_sensors(node: Node, mqtt, robots: dict, mqtt_base: str, ha_prefix: str, pub_cfg: Callable) -> FleetHaSensors:
    return FleetHaSensors(node, mqtt, robots, mqtt_base, ha_prefix, pub_cfg)
