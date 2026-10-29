#!/bin/sh
# Sample IMU at rest — gyro bias and accel magnitude. Bot must be still on a flat surface.
# Usage: ./imu-check.sh [seconds]
set -e

SEC="${1:-10}"
NS="${BOT_NS:-bot1}"
TOPIC="/${NS}/imu"

. "$(dirname "$0")/_root.sh"

echo "=== IMU check: ${TOPIC} for ${SEC}s (bot STILL) ==="

if ! ./ros.sh topic list 2>/dev/null | grep -Fxq "$TOPIC"; then
  echo "ERROR: ${TOPIC} not visible" >&2
  exit 1
fi

docker run --rm --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e TOPIC="$TOPIC" \
  -e SEC="$SEC" \
  ros:jazzy-ros-base \
  bash -lc 'source /opt/ros/jazzy/setup.bash
python3 - <<PY
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class S(Node):
    def __init__(self):
        super().__init__("imu_check")
        self.n = 0
        self.gz = []
        self.ax = []
        self.ay = []
        self.az = []
        self.create_subscription(Imu, "'"$TOPIC"'", self.cb, 10)
    def cb(self, m):
        self.n += 1
        self.gz.append(float(m.angular_velocity.z))
        self.ax.append(float(m.linear_acceleration.x))
        self.ay.append(float(m.linear_acceleration.y))
        self.az.append(float(m.linear_acceleration.z))

def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0

def std(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))

rclpy.init()
node = S()
t0 = time.monotonic()
while time.monotonic() - t0 < '"$SEC"':
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_node()
rclpy.shutdown()

if node.n < 5:
    print("ERROR: only", node.n, "samples — IMU topic dead?")
    raise SystemExit(1)

mag = [math.sqrt(x*x+y*y+z*z) for x,y,z in zip(node.ax, node.ay, node.az)]
print(f"samples: {node.n}")
print(f"gyro z:  mean={mean(node.gz):+.4f} rad/s  std={std(node.gz):.4f}  ({math.degrees(mean(node.gz)):+.2f} deg/s bias)")
print(f"accel |g|: mean={mean(mag):.2f} m/s²  (expect ~9.8 flat; z-axis dominant if level)")
print("If |gz bias| > 0.05 rad/s (~3 deg/s), note sign for ROVER_IMU_GYRO_SIGN / firmware trim.")
PY'
