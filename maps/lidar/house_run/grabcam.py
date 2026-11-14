import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
rclpy.init(); n = Node("grok_grabcam"); got = {}; ev = []
n.create_subscription(CompressedImage, "/rover/camera/image_raw/compressed", lambda m: got.setdefault("img", m), qos_profile_sensor_data)
n.create_subscription(Bool, "/rover/stall", lambda m: ev.append(("stall", m.data)), 10)
n.create_subscription(Bool, "/rover/bump", lambda m: ev.append(("bump", m.data)), 10)
t0 = time.time()
while time.time() - t0 < 4: rclpy.spin_once(n, timeout_sec=0.1)
if "img" in got:
    open("/maps/lidar/house_run/grok_cam.jpg", "wb").write(bytes(got["img"].data)); print("saved", got["img"].format)
print("stall/bump msgs:", ev[:10], len(ev))
