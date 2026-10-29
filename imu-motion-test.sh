#!/bin/sh
# Run imu_motion_test.py (straight / rotate). Pauses fleet brain.
# Usage:
#   ./imu-motion-test.sh straight [linear] [seconds]
#   ./imu-motion-test.sh rotate [angular_z] [degrees]
set -e

MODE="${1:?usage: imu-motion-test.sh straight|rotate [args...]}"
shift

cd "$(dirname "$0")"
NS="${BOT_NS:-bot1}"

docker compose --profile explore stop fleet-explore-chase 2>/dev/null || true
sleep 0.3

docker run --rm -it --network host \
  -v "$(pwd):/opt/fleet:ro" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e BOT_NS="$NS" \
  ros:jazzy-ros-base \
  bash -lc "source /opt/ros/jazzy/setup.bash && exec python3 /opt/fleet/imu_motion_test.py $MODE $*"

docker compose --profile explore start fleet-explore-chase
echo "fleet-explore-chase restarted"
