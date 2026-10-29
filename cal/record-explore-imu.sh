#!/bin/sh
# Log IMU + cmd_vel to CSV while fleet brain runs.
# Usage: ./record-explore-imu.sh [seconds] [output.csv]
set -e

SEC="${1:-120}"
OUT="${2:-/tmp/bot1-explore-imu.csv}"
NS="${BOT_NS:-bot1}"

. "$(dirname "$0")/_root.sh"
LOG_DIR="$(cd "$(dirname "$OUT")" && pwd)"
mkdir -p "$LOG_DIR"
OUT_HOST="$LOG_DIR/$(basename "$OUT")"

echo "=== recording ${SEC}s → ${OUT_HOST} ==="
echo "Topics: /${NS}/imu  /${NS}/cmd_vel"
echo ""

docker compose exec -T fleet-shell bash -lc "source /opt/ros/jazzy/setup.bash
export NS='$NS' SEC='$SEC'
python3 <<'PY'
import csv, os, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

class L(Node):
    def __init__(self):
        super().__init__('rec')
        ns = os.environ['NS']
        self.rows = []
        self.create_subscription(Imu, f'/{ns}/imu', self.imu, QOS)
        self.create_subscription(Twist, f'/{ns}/cmd_vel', self.cmd, QOS)
    def imu(self, m):
        self.rows.append(('imu', time.time(), m))
    def cmd(self, m):
        self.rows.append(('cmd', time.time(), m))

rclpy.init()
node = L()
t0 = time.time()
sec = float(os.environ['SEC'])
while time.time() - t0 < sec:
    rclpy.spin_once(node, timeout_sec=0.02)

out = '/tmp/rec-drive.csv'
with open(out, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['kind', 't', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'lin_x', 'ang_z'])
    for kind, t, m in sorted(node.rows, key=lambda r: r[1]):
        if kind == 'imu':
            w.writerow([kind, f'{t:.3f}', m.linear_acceleration.x, m.linear_acceleration.y,
                        m.linear_acceleration.z, m.angular_velocity.x, m.angular_velocity.y,
                        m.angular_velocity.z, '', ''])
        else:
            w.writerow([kind, f'{t:.3f}', '', '', '', '', '', '', m.linear.x, m.angular.z])

imu_n = sum(1 for r in node.rows if r[0] == 'imu')
cmd_n = sum(1 for r in node.rows if r[0] == 'cmd')
print(f'wrote {len(node.rows)} events ({imu_n} imu, {cmd_n} cmd) to {out}')
node.destroy_node()
rclpy.shutdown()
PY"

docker cp fleet-shell:/tmp/rec-drive.csv "$OUT_HOST"
echo "Saved: $OUT_HOST"
