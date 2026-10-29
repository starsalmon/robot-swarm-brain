#!/usr/bin/env bash
# One supervised short forward drive with bag + guaranteed cleanup. Run on the NUC in /data/swarm/robot-fleet.
#   ./safe_drive.sh [ticks=60] [clearance_m=0.6] [vmax=0.10] [timeout_s=3] [turn_deg=0]   (BAGSECS env, default 25)
# Sets allow_drive ON (retained), pauses rover-brain (its idle zero cmd_vel would fight the test),
# records a bag in captures/, runs rover_safe_drive.py in fleet-shell, then ALWAYS: unpause rover-brain,
# stop_all, allow_drive OFF (retained), session false.
set -uo pipefail
cd "$(dirname "$0")"
TICKS=${1:-60}; CLEAR=${2:-0.6}; VMAX=${3:-0.10}; TIMEOUT=${4:-3}; TURN=${5:-0}; BAGSECS=${BAGSECS:-25}
set -a; . ./mqtt_secrets.env; set +a
H=${MQTT_HOST%%:*}; P=${MQTT_PORT:-1883}
mq() { docker run --rm --network host eclipse-mosquitto:2 mosquitto_pub -h "$H" -p "$P" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 "$@"; }
ros() { docker exec fleet-shell bash -c "source /opt/ros/jazzy/setup.bash; $*"; }
BAG=loop_$(date +%Y%m%d_%H%M%S)
cleanup() {
  docker unpause rover-brain >/dev/null 2>&1
  ros "ros2 topic pub -r 20 -t 10 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null; ros2 topic pub -r 10 -t 5 /rover/session std_msgs/msg/Bool '{data: false}' >/dev/null"
  mq -t lab/robots/cmd/stop_all -m PRESS; mq -r -t lab/robots/allow_drive/set -m OFF
  sleep 3; docker exec fleet-shell pkill -INT -f "bag record -o $BAG"; sleep 5; mkdir -p captures/${BAG}_logs; docker exec fleet-shell bash -c "cd /tmp && tar cf - $BAG.*.json" | tar xf - -C captures/${BAG}_logs; docker cp fleet-shell:/tmp/$BAG captures/$BAG >/dev/null 2>&1 && echo "bag: captures/$BAG"
  echo "cleanup done: allow_drive OFF (retained), session off, rover-brain unpaused"
}
trap cleanup EXIT
docker exec -d fleet-shell bash -c "source /opt/ros/jazzy/setup.bash; cd /tmp; timeout $BAGSECS ros2 bag record -o $BAG \
  /rover/camera/image_raw/compressed /rover/camera/image_rect/compressed /rover/camera/depth_small \
  /rover/odom/wheel /odometry/filtered /imu/data /rover/imu/stamped /rover/wheel/left_ticks /rover/wheel/right_ticks \
  /rover/tof/front /rover/tof/l8_cols /rover/tof/l8_grid /rover/tof/left /rover/tof/right /rover/camera/camera_info /cmd_vel /tf /tf_static >/tmp/$BAG.log 2>&1"
sleep 4
mq -r -t lab/robots/allow_drive/set -m ON; sleep 1.5
docker pause rover-brain >/dev/null
docker cp rover_safe_drive.py fleet-shell:/tmp/rover_safe_drive.py
for lap in 1 2 3; do for side in 1 2 3 4; do
  ros "timeout 30 python3 /tmp/rover_safe_drive.py --ticks $TICKS --clear $CLEAR --vmax $VMAX --timeout $TIMEOUT --log /tmp/$BAG.l${lap}s${side}.fwd.json" 2>&1 | tail -1
  sleep 1.5
  ros "timeout 30 python3 /tmp/rover_safe_drive.py --turn-deg $TURN --ticks 999 --clear 0 --wmax 0.12 --timeout 16 --log /tmp/$BAG.l${lap}s${side}.turn.json" 2>&1 | tail -1
  sleep 1.5
done; done
sleep 2
