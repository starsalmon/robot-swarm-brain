#!/bin/sh
# Log IMU + cmd_vel to CSV while fleet brain runs (or any driver).
# Usage: ./record-explore-imu.sh [seconds] [output.csv]
# Start this, then let explore run — or drive manually with speed-pulse.sh in another shell.
set -e

SEC="${1:-120}"
OUT="${2:-/tmp/bot1-explore-imu.csv}"
NS="${BOT_NS:-bot1}"

cd "$(dirname "$0")"

echo "=== recording ${SEC}s → ${OUT} ==="
echo "Topics: /${NS}/imu  /${NS}/cmd_vel"
echo "Start explore or drive now."
echo ""

docker run --rm --network host \
  -v "$(pwd):/opt/fleet:ro" \
  -v "$(dirname "$OUT"):/out" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e NS="$NS" \
  -e SEC="$SEC" \
  -e OUT="/out/$(basename "$OUT")" \
  ros:jazzy-ros-base \
  bash -lc 'source /opt/ros/jazzy/setup.bash
python3 - <<PY
import csv, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class L(Node):
    def __init__(self):
        super().__init__("rec")
        ns = "'"$NS"'"
        self.rows = []
        self.create_subscription(Imu, f"/{ns}/imu", self.imu, 10)
        self.create_subscription(Twist, f"/{ns}/cmd_vel", self.cmd, 10)
    def imu(self, m):
        self.rows.append(("imu", time.time(), m))
    def cmd(self, m):
        self.rows.append(("cmd", time.time(), m))

rclpy.init()
node = L()
t0 = time.time()
while time.time() - t0 < '"$SEC"':
    rclpy.spin_once(node, timeout_sec=0.02)

out = "'"$OUT"'"
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["kind", "t", "ax", "ay", "az", "gx", "gy", "gz", "lin_x", "ang_z"])
    for kind, t, m in sorted(node.rows, key=lambda r: r[1]):
        if kind == "imu":
            w.writerow([kind, f"{t:.3f}", m.linear_acceleration.x, m.linear_acceleration.y,
                        m.linear_acceleration.z, m.angular_velocity.x, m.angular_velocity.y,
                        m.angular_velocity.z, "", ""])
        else:
            w.writerow([kind, f"{t:.3f}", "", "", "", "", "", "", m.linear.x, m.angular.z])

print(f"wrote {len(node.rows)} events to {out}")
node.destroy_node()
rclpy.shutdown()
PY'

echo "Saved: $OUT"
