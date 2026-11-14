import rclpy, time, json, numpy as np, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import tf2_ros
rclpy.init(); n = Node("gb_livemap")
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got = {}
n.create_subscription(OccupancyGrid, "/map", lambda m: got.__setitem__("m", m), q)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
t0 = time.time(); pose = None
while time.time() - t0 < 12 and ("m" not in got or pose is None):
    rclpy.spin_once(n, timeout_sec=0.2)
    try:
        tr = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        r = tr.transform.rotation
        pose = [tr.transform.translation.x, tr.transform.translation.y, math.atan2(2*(r.w*r.z+r.x*r.y), 1-2*(r.y*r.y+r.z*r.z))]
    except Exception: pass
m = got["m"]; a = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
np.savez("/maps/lidar/house_run/gb_livemap.npz", grid=a, res=m.info.resolution, ox=m.info.origin.position.x, oy=m.info.origin.position.y, pose=np.array(pose if pose else [np.nan]*3), stamp=m.header.stamp.sec)
print(json.dumps({"w": m.info.width, "h": m.info.height, "occ": int((a > 50).sum()), "free": int((a == 0).sum()), "pose": pose, "map_stamp": m.header.stamp.sec, "now": time.time()}))
