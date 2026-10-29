#!/usr/bin/env bash
# One supervised person-follow run with bag + guaranteed cleanup (same pattern as safe_drive.sh). On the NUC in
# /data/swarm/robot-swarm-brain:   ./follow.sh [max_s=0] [vmax=0.10] [wmax=0.15] [target_m=1.0]   (max_s 0 = run until stopped)
#   ./follow.sh stop   (any shell: kills the follower, zeros /cmd_vel, session off, stop_all, allow_drive OFF). Ctrl-C does the same.
# Stops the dry-run rover-follow service, sets allow_drive ON (retained), pauses rover-brain, records a bag,
# runs person_follow.py with FOLLOW_DRY_RUN=0 (dead-man = max_s), then ALWAYS: unpause rover-brain, zero
# cmd_vel, session false, stop_all, allow_drive OFF (retained).
set -uo pipefail
cd "$(dirname "$0")"
MAXS=${1:-0}; VMAX=${2:-0.10}; WMAX=${3:-0.15}; TGT=${4:-1.0}
set -a; . ./mqtt_secrets.env; set +a
H=${MQTT_HOST%%:*}; P=${MQTT_PORT:-1883}
mq() { docker run --rm --network host eclipse-mosquitto:2 mosquitto_pub -h "$H" -p "$P" -u "$MQTT_USER" -P "$MQTT_PASS" -q 1 "$@"; }
ros() { docker exec fleet-shell bash -c "source /opt/ros/jazzy/setup.bash; $*"; }
if [ "${1:-}" = "stop" ]; then
  docker rm -f rover-follow-drive >/dev/null 2>&1
  ros "pkill -INT -f \"[r]os2 bag record -o follow_\" ; ros2 topic pub -r 20 -t 10 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null; ros2 topic pub -r 10 -t 5 /rover/session std_msgs/msg/Bool '{data: false}' >/dev/null"
  mq -t lab/robots/cmd/stop_all -m PRESS; mq -r -t lab/robots/allow_drive/set -m OFF
  docker unpause rover-brain >/dev/null 2>&1
  echo "stopped: follower killed, cmd_vel zero, session off, allow_drive OFF (retained)"; exit 0
fi
BAG=follow_$(date +%Y%m%d_%H%M%S)
BAGSECS=$(( MAXS > 0 ? MAXS + 10 : 1800 ))   # continuous: bag capped at 30 min
cleanup() {
  docker rm -f rover-follow-drive >/dev/null 2>&1
  ros "pkill -INT -f \"[r]os2 bag record -o $BAG\"" >/dev/null 2>&1
  docker unpause rover-brain >/dev/null 2>&1
  ros "ros2 topic pub -r 20 -t 10 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null; ros2 topic pub -r 10 -t 5 /rover/session std_msgs/msg/Bool '{data: false}' >/dev/null"
  mq -t lab/robots/cmd/stop_all -m PRESS; mq -r -t lab/robots/allow_drive/set -m OFF
  sleep 8; docker cp fleet-shell:/tmp/$BAG captures/$BAG >/dev/null 2>&1 && echo "bag: captures/$BAG"
  echo "cleanup done: allow_drive OFF (retained), session off, rover-brain unpaused"
}
trap cleanup EXIT
trap "exit 130" INT TERM
docker compose --profile follow stop rover-follow >/dev/null 2>&1
docker exec -d fleet-shell bash -c "source /opt/ros/jazzy/setup.bash; cd /tmp; timeout $BAGSECS ros2 bag record -o $BAG \
  /rover/camera/image_raw/compressed /rover/follow/cmd /rover/follow/status /rover/odom/wheel /imu/data \
  /rover/wheel/left_ticks /rover/wheel/right_ticks /rover/tof/front /rover/tof/l8_cols /rover/tof/l8_grid /cmd_vel >/tmp/$BAG.log 2>&1"
sleep 4
mq -r -t lab/robots/allow_drive/set -m ON; sleep 1.5
docker pause rover-brain >/dev/null
docker compose --profile follow run --rm --name rover-follow-drive -e FOLLOW_DRY_RUN=0 -e FOLLOW_MAX_S=$MAXS \
  -e FOLLOW_VMAX=$VMAX -e FOLLOW_WMAX=$WMAX -e FOLLOW_TARGET_M=$TGT rover-follow
sleep 1
