#!/usr/bin/env python3
"""Read-only: watch for the Pro S3 power cycle and a healthy L8/lidar/IMU recovery. Never drives."""
import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu, Range
from std_msgs.msg import Float32MultiArray, Bool
MAXT = 720; PER = 5.0
rclpy.init(); n = Node("grok_l8watch"); st = dict(nscan=0, nimu=0, ncols=0)
def scan(m):
    st["nscan"] += 1; st["scan_age"] = time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    st["scan_pts"] = sum(1 for r in m.ranges if math.isfinite(r) and r > 0.05); st["t_scan"] = time.time()
n.create_subscription(LaserScan, "/rover/scan_sync", scan, qos_profile_sensor_data)
n.create_subscription(Float32MultiArray, "/rover/tof/status", lambda m: st.update(status=list(m.data), t_status=time.time()), 10)
n.create_subscription(Float32MultiArray, "/rover/tof/wifi", lambda m: st.update(wifi=list(m.data), t_wifi=time.time()), 10)
def cols(m): st["ncols"] += 1; st["cols"] = tuple(round(x, 3) for x in m.data); st["t_cols"] = time.time(); st.setdefault("colset", set()).add(st["cols"])
n.create_subscription(Float32MultiArray, "/rover/tof/l8_cols", cols, 10)
n.create_subscription(Imu, "/imu/data", lambda m: st.update(nimu=st["nimu"] + 1, t_imu=time.time()), qos_profile_sensor_data)
n.create_subscription(Bool, "/rover/odom/imu_ok", lambda m: st.update(imu_ok=m.data), 10)
t0 = time.time(); tl = t0; prev_frames = None; good = 0; seen_drop = False
while time.time() - t0 < MAXT:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - tl < PER: continue
    now = time.time(); tl = now
    s = st.get("status"); frames = s[6] if s else None
    status_age = now - st.get("t_status", 0); scan_age = now - st.get("t_scan", 0)
    if status_age > 3 or scan_age > 3: seen_drop = True
    distinct = len(st.pop("colset", set()))
    df = None if (frames is None or prev_frames is None) else frames - prev_frames
    prev_frames = frames
    healthy = (status_age < 2 and scan_age < 1 and st.get("scan_pts", 0) > 300 and df is not None and df > 0
               and distinct >= 3 and now - st.get("t_imu", 0) < 1 and st.get("imu_ok") is True)
    good = good + 1 if healthy else 0
    print(f"t={now-t0:5.0f} mel={time.strftime('%H:%M:%S')} status_age={status_age:.1f} l8_frames={frames} d={df} cols_distinct={distinct} cols={st.get('cols')} "
          f"scan_age_rx={scan_age:.2f} stamp_age={st.get('scan_age', -1):.3f} pts={st.get('scan_pts')} nscan={st['nscan']} wifi={st.get('wifi')} "
          f"imu_n={st['nimu']} imu_ok={st.get('imu_ok')} drop_seen={seen_drop} healthy={healthy} good={good}", flush=True)
    st["nscan"] = 0; st["nimu"] = 0
    if good >= 3: print("HEALTHY", flush=True); break
else:
    print("TIMEOUT", flush=True)
