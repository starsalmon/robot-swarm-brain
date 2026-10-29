#!/usr/bin/env python3
"""ROS 2 → Home Assistant MQTT (same broker as yoRadio).

Always-on on dockerhost. Battery topics are retained so the wall panel keeps
the last reading after a bot powers off. Stop-all turns off /fleet/allow_drive
so brains actually halt, not just one zeroed cmd_vel.
"""
from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32, UInt8

from rover_qos import CMD_VEL_QOS

HA_PREFIX = os.environ.get('HA_DISCOVERY_PREFIX', 'homeassistant')
MQTT_BASE = os.environ.get('FLEET_MQTT_BASE', 'lab/robots')
STALE_S = float(os.environ.get('FLEET_HA_STALE_S', '8.0'))

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


def _pct(volts: float, lo: float, hi: float) -> float:
    if volts <= 0.2 or hi <= lo:
        return 0.0
    return round(max(0.0, min(100.0, (volts - lo) / (hi - lo) * 100.0)), 1)


@dataclass
class Robot:
    slug: str
    name: str
    cmd_vel: str
    live_topics: list  # ESP-originated — used for Online (not brain cmd_vel)
    battery_topic: str | None
    v_empty: float
    v_full: float
    session_topic: str | None = None
    drive_mode_topic: str | None = None
    last_seen: float = 0.0
    volts: float | None = None
    session: bool | None = None
    drive_mode: int | None = None


def _robots() -> list[Robot]:
    return [
        Robot(
            slug='rover',
            name='Rover',
            cmd_vel='/cmd_vel',
            live_topics=[('/imu/data', Imu), ('/rover/sonar/range', Range)],
            battery_topic='/rover/battery/voltage',
            v_empty=6.2,
            v_full=8.3,
            session_topic='/rover/session',
            drive_mode_topic='/rover/drive_mode',
        ),
        Robot(
            slug='roby',
            name='Roby',
            cmd_vel='/bot1/cmd_vel',
            live_topics=[('/bot1/imu', Imu), ('/bot1/sonar/range', Range)],
            battery_topic='/bot1/battery/voltage',
            v_empty=3.40,
            v_full=4.15,
        ),
        Robot(
            slug='tracked',
            name='Tracked',
            cmd_vel='/bot2/cmd_vel',
            live_topics=[('/bot2/imu', Imu), ('/bot2/sonar/range', Range)],
            battery_topic='/bot2/battery/voltage',
            v_empty=3.40,
            v_full=4.15,
        ),
    ]


def _device_robot(bot: Robot) -> dict:
    return {
        'identifiers': [f'lab_robot_{bot.slug}'],
        'name': bot.name,
        'manufacturer': 'lab',
        'model': 'ESP32 fleet',
        'via_device': 'lab_robots',
    }


def _device_fleet() -> dict:
    return {
        'identifiers': ['lab_robots'],
        'name': 'Lab robots',
        'manufacturer': 'lab',
        'model': 'dockerhost fleet',
    }


class FleetHaBridge(Node):
    def __init__(self) -> None:
        super().__init__('fleet_ha_bridge')
        self._robots = {b.slug: b for b in _robots()}
        self._allow_drive = False
        self._lock = threading.Lock()
        self._mqtt_ready = threading.Event()
        self._allow_restored = False
        self._allow_restore_deadline = 0.0

        self._allow_pub = self.create_publisher(Bool, '/fleet/allow_drive', 10)
        self._session_pub = self.create_publisher(Bool, '/rover/session', 10)
        self._cmd_pubs = {
            slug: self.create_publisher(Twist, bot.cmd_vel, CMD_VEL_QOS)
            for slug, bot in self._robots.items()
        }

        for bot in self._robots.values():
            if bot.battery_topic:
                # micro-ROS rclc_publisher_init_default is RELIABLE — best-effort
                # subs never see /bot1/battery/voltage (and IMU/sonar).
                self.create_subscription(
                    Float32,
                    bot.battery_topic,
                    self._mk_battery_cb(bot.slug),
                    REL_QOS,
                )
            for topic, msg_type in bot.live_topics:
                self.create_subscription(
                    msg_type,
                    topic,
                    self._mk_seen_cb(bot.slug),
                    REL_QOS,
                )
            if bot.session_topic:
                self.create_subscription(
                    Bool,
                    bot.session_topic,
                    self._mk_session_cb(bot.slug),
                    REL_QOS,
                )
            if bot.drive_mode_topic:
                self.create_subscription(
                    UInt8,
                    bot.drive_mode_topic,
                    self._mk_mode_cb(bot.slug),
                    REL_QOS,
                )

        host = os.environ.get('MQTT_HOST', '').strip()
        port = int(os.environ.get('MQTT_PORT', '1883'))
        user = os.environ.get('MQTT_USER', '').strip()
        password = os.environ.get('MQTT_PASS', '')
        if not host:
            raise SystemExit('MQTT_HOST is not set (mqtt_secrets.env on dockerhost)')

        cid = os.environ.get('MQTT_CLIENT_ID', 'fleet-ha-bridge')
        try:
            self._mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=cid)
        except AttributeError:
            self._mqtt = mqtt.Client(client_id=cid)
        if user:
            self._mqtt.username_pw_set(user, password)
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_message = self._on_mqtt_message
        self._mqtt.will_set(f'{MQTT_BASE}/bridge/availability', 'offline', qos=1, retain=True)
        self._mqtt.connect_async(host, port, keepalive=30)
        self._mqtt.loop_start()

        self.create_timer(0.5, self._tick_ros)
        self.create_timer(2.0, self._tick_ha)
        self.get_logger().info(f'fleet HA bridge → mqtt {host}:{port} as {MQTT_BASE}')

    def _mk_battery_cb(self, slug: str):
        def _cb(msg: Float32) -> None:
            bot = self._robots[slug]
            volts = float(msg.data)
            first = bot.volts is None
            bot.volts = volts
            bot.last_seen = time.monotonic()
            self._publish_battery(bot)
            if first:
                self.get_logger().info(f'{bot.name} battery {volts:.2f} V')

        return _cb

    def _mk_seen_cb(self, slug: str):
        def _cb(_msg) -> None:
            self._robots[slug].last_seen = time.monotonic()

        return _cb

    def _mk_session_cb(self, slug: str):
        def _cb(msg: Bool) -> None:
            bot = self._robots[slug]
            bot.session = bool(msg.data)
            bot.last_seen = time.monotonic()

        return _cb

    def _mk_mode_cb(self, slug: str):
        def _cb(msg: UInt8) -> None:
            bot = self._robots[slug]
            bot.drive_mode = int(msg.data)
            bot.last_seen = time.monotonic()

        return _cb

    def _on_mqtt_connect(self, client, _userdata, _flags, reason_code, _properties=None):
        rc = int(getattr(reason_code, 'value', reason_code))
        if rc != 0:
            self.get_logger().error(f'MQTT connect failed: {reason_code}')
            return
        client.subscribe(f'{MQTT_BASE}/cmd/stop_all', qos=1)
        client.subscribe(f'{MQTT_BASE}/allow_drive/set', qos=1)
        # One-shot retained state only — do not stay subscribed or HA and the
        # bridge echo ON/OFF at each other and the wall switch flickers.
        self._allow_restored = False
        self._allow_restore_deadline = time.monotonic() + 1.5
        client.subscribe(f'{MQTT_BASE}/allow_drive', qos=1)
        client.publish(f'{MQTT_BASE}/bridge/availability', 'online', qos=1, retain=True)
        self._publish_discovery()
        self._mqtt_ready.set()
        self.get_logger().info('MQTT connected — HA discovery published')

    def _on_mqtt_message(self, _client, _userdata, msg: mqtt.MQTTMessage) -> None:
        payload = (msg.payload or b'').decode('utf-8', errors='replace').strip()
        if msg.topic.endswith('/cmd/stop_all'):
            self._set_allow(False, reason='HA Stop all')
            return
        if msg.topic.endswith('/allow_drive/set'):
            want = payload.upper() in ('ON', '1', 'TRUE', 'YES')
            self._set_allow(want, reason='HA allow_drive')
            return
        if msg.topic.endswith('/allow_drive'):
            if self._allow_restored:
                return
            self._allow_restored = True
            want = payload.upper() in ('ON', '1', 'TRUE', 'YES')
            with self._lock:
                self._allow_drive = want
            self.get_logger().info(f'restored allow_drive={want}')
            try:
                self._mqtt.unsubscribe(f'{MQTT_BASE}/allow_drive')
            except Exception:
                pass
            if not want:
                self._halt_now()
            return

    def _set_allow(self, allow: bool, *, reason: str) -> None:
        with self._lock:
            changed = allow != self._allow_drive
            self._allow_drive = allow
        if not changed:
            return
        self.get_logger().info(f'{reason} → allow_drive={allow}')
        self._publish_allow_mqtt()
        if not allow:
            self._halt_now()

    def _halt_now(self) -> None:
        stop = Twist()
        sess = Bool()
        sess.data = False
        for _ in range(8):
            for pub in self._cmd_pubs.values():
                pub.publish(stop)
            self._session_pub.publish(sess)

    def _publish_allow_mqtt(self) -> None:
        if not self._mqtt_ready.is_set() and not self._mqtt.is_connected():
            return
        with self._lock:
            allow = self._allow_drive
        self._mqtt.publish(
            f'{MQTT_BASE}/allow_drive',
            'ON' if allow else 'OFF',
            qos=1,
            retain=True,
        )

    def _pub_cfg(self, component: str, node_id: str, object_id: str, payload: dict) -> None:
        topic = f'{HA_PREFIX}/{component}/{node_id}/{object_id}/config'
        self._mqtt.publish(topic, json.dumps(payload), qos=1, retain=True)

    def _publish_discovery(self) -> None:
        fleet = _device_fleet()
        avail = {
            'availability_topic': f'{MQTT_BASE}/bridge/availability',
            'payload_available': 'online',
            'payload_not_available': 'offline',
        }
        self._pub_cfg(
            'button',
            'lab_robots',
            'stop_all',
            {
                'name': 'Stop all robots',
                'unique_id': 'lab_robots_stop_all',
                'object_id': 'robots_stop_all',
                'command_topic': f'{MQTT_BASE}/cmd/stop_all',
                'payload_press': 'PRESS',
                'icon': 'mdi:stop-circle',
                'device': fleet,
                **avail,
            },
        )
        self._pub_cfg(
            'switch',
            'lab_robots',
            'allow_drive',
            {
                'name': 'Allow drive',
                'unique_id': 'lab_robots_allow_drive',
                'object_id': 'robots_allow_drive',
                'command_topic': f'{MQTT_BASE}/allow_drive/set',
                'state_topic': f'{MQTT_BASE}/allow_drive',
                'payload_on': 'ON',
                'payload_off': 'OFF',
                'optimistic': False,
                'icon': 'mdi:robot',
                'device': fleet,
                **avail,
            },
        )

        for bot in self._robots.values():
            dev = _device_robot(bot)
            nid = f'lab_robot_{bot.slug}'
            # Battery has no availability — last value stays on the panel when the bot is off.
            self._pub_cfg(
                'sensor',
                nid,
                'battery_voltage',
                {
                    'name': 'Battery voltage',
                    'unique_id': f'{nid}_battery_voltage',
                    'object_id': f'{bot.slug}_battery_voltage',
                    'state_topic': f'{MQTT_BASE}/{bot.slug}/battery_voltage',
                    'unit_of_measurement': 'V',
                    'device_class': 'voltage',
                    'state_class': 'measurement',
                    'suggested_display_precision': 2,
                    'icon': 'mdi:flash',
                    'device': dev,
                },
            )
            self._pub_cfg(
                'sensor',
                nid,
                'battery',
                {
                    'name': 'Battery',
                    'unique_id': f'{nid}_battery',
                    'object_id': f'{bot.slug}_battery',
                    'state_topic': f'{MQTT_BASE}/{bot.slug}/battery',
                    'unit_of_measurement': '%',
                    'device_class': 'battery',
                    'state_class': 'measurement',
                    'suggested_display_precision': 1,
                    'icon': 'mdi:battery',
                    'device': dev,
                },
            )
            self._pub_cfg(
                'binary_sensor',
                nid,
                'online',
                {
                    'name': 'Online',
                    'unique_id': f'{nid}_online',
                    'object_id': f'{bot.slug}_online',
                    'state_topic': f'{MQTT_BASE}/{bot.slug}/online',
                    'payload_on': 'ON',
                    'payload_off': 'OFF',
                    'device_class': 'connectivity',
                    'device': dev,
                    **avail,
                },
            )
            if bot.session_topic:
                self._pub_cfg(
                    'binary_sensor',
                    nid,
                    'session',
                    {
                        'name': 'Session',
                        'unique_id': f'{nid}_session',
                        'object_id': f'{bot.slug}_session',
                        'state_topic': f'{MQTT_BASE}/{bot.slug}/session',
                        'payload_on': 'ON',
                        'payload_off': 'OFF',
                        'icon': 'mdi:play-circle',
                        'device': dev,
                    },
                )
            if bot.drive_mode_topic:
                self._pub_cfg(
                    'sensor',
                    nid,
                    'drive_mode',
                    {
                        'name': 'Drive mode',
                        'unique_id': f'{nid}_drive_mode',
                        'object_id': f'{bot.slug}_drive_mode',
                        'state_topic': f'{MQTT_BASE}/{bot.slug}/drive_mode',
                        'icon': 'mdi:map-marker-path',
                        'device': dev,
                    },
                )

    def _publish_battery(self, bot: Robot) -> None:
        if bot.volts is None:
            return
        self._mqtt.publish(
            f'{MQTT_BASE}/{bot.slug}/battery_voltage',
            f'{bot.volts:.2f}',
            qos=1,
            retain=True,
        )
        self._mqtt.publish(
            f'{MQTT_BASE}/{bot.slug}/battery',
            f'{_pct(bot.volts, bot.v_empty, bot.v_full):.1f}',
            qos=1,
            retain=True,
        )

    def _tick_ros(self) -> None:
        if (
            self._mqtt_ready.is_set()
            and not self._allow_restored
            and self._allow_restore_deadline > 0.0
            and time.monotonic() >= self._allow_restore_deadline
        ):
            self._allow_restored = True
            try:
                self._mqtt.unsubscribe(f'{MQTT_BASE}/allow_drive')
            except Exception:
                pass
            self._publish_allow_mqtt()

        with self._lock:
            allow = self._allow_drive
        msg = Bool()
        msg.data = allow
        self._allow_pub.publish(msg)
        if not allow:
            stop = Twist()
            for pub in self._cmd_pubs.values():
                pub.publish(stop)

    def _tick_ha(self) -> None:
        if not self._mqtt.is_connected():
            return
        now = time.monotonic()
        for bot in self._robots.values():
            online = bot.last_seen > 0.0 and (now - bot.last_seen) < STALE_S
            self._mqtt.publish(
                f'{MQTT_BASE}/{bot.slug}/online',
                'ON' if online else 'OFF',
                qos=1,
                retain=True,
            )
            self._publish_battery(bot)
            if bot.session is not None:
                self._mqtt.publish(
                    f'{MQTT_BASE}/{bot.slug}/session',
                    'ON' if bot.session else 'OFF',
                    qos=1,
                    retain=True,
                )
            if bot.drive_mode is not None:
                label = 'wall follow' if bot.drive_mode == 1 else 'explore'
                self._mqtt.publish(
                    f'{MQTT_BASE}/{bot.slug}/drive_mode',
                    label,
                    qos=1,
                    retain=True,
                )

    def destroy_node(self) -> None:
        try:
            self._mqtt.publish(f'{MQTT_BASE}/bridge/availability', 'offline', qos=1, retain=True)
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = FleetHaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
