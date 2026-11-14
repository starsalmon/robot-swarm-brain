import time, math, json, sys, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String, Bool, Float32
import tf2_ros
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60
rclpy.init(); n = Node("grok_mon"); st = {}
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
n.create_subscription(Odometry, "/odometry/filtered", lambda m: st.update(ekf=yaw(m.pose.pose.orientation), ewz=m.twist.twist.angular.z), 10)
n.create_subscription(Imu, "/imu/data", lambda m: st.update(gz=m.angular_velocity.z, imu_t=time.time()), qos_profile_sensor_data)
n.create_subscription(String, "/rover/behaviour/status", lambda m: st.__setitem__("beh", json.loads(m.data)), 10)
n.create_subscription(Bool, "/rover/odom/imu_ok", lambda m: st.__setitem__("imu_ok", m.data), 10)
n.create_subscription(Float32, "/rover/battery/voltage", lambda m: st.__setitem__("bat", round(m.data, 2)), 10)
n.create_subscription(Range, "/rover/sonar/range", lambda m: st.__setitem__("rear", round(m.range, 2)), qos_profile_sensor_data)
t0 = time.time(); tl = 0; maxrate = 0; prev = None
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - tl > 2.0 and "beh" in st:
        tl = time.time()
        try:
            tr = buf.lookup_transform("map", "base_link", rclpy.time.Time()); my = round(yaw(tr.transform.rotation)); mx = (round(tr.transform.translation.x, 2), round(tr.transform.translation.y, 2))
        except Exception as e:
            my = None; mx = None
        b = st["beh"]
        if prev and my is not None and prev[0] is not None:
            r = abs(math.remainder(math.radians(my - prev[0]), 2*math.pi)) / (tl - prev[1]); maxrate = max(maxrate, math.degrees(r))
        prev = (my, tl)
        print(f"t={tl-t0:4.0f} map={mx} myaw={my} ekf_wz={math.degrees(st.get('ewz',0)):.0f}d/s gz={math.degrees(st.get('gz',0)):.0f}d/s imu_age={time.time()-st.get('imu_t',0):.1f} imu_ok={st.get('imu_ok')} bat={st.get('bat')} rear={st.get('rear')} go={b.get('go')} v={b['v']} w={b['w']} goal={b.get('map_goal')} lah={b.get('lidar_ahead')} why={b['why'][:70]}", flush=True)
print("max map yaw rate deg/s", round(maxrate))
