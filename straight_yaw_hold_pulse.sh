#!/usr/bin/env bash
# Safe wrapper: stop explore brain, run IMU yaw-hold pulse, restart brain.
# Default is DRY-RUN (no motors). Pass --drive as last args to move.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT="$DIR/straight_yaw_hold_pulse.py"
OUT="${STRAIGHT_YAW_OUT:-/tmp/pulse_out_yawhold}"
mkdir -p "$OUT"
TELEM_DIR="${TELEM_DIR:-/data/swarm/rover-telem}"

echo "Stopping rover-brain (explore must not fight)"
docker stop rover-brain >/dev/null
cleanup() {
  docker start rover-brain >/dev/null || true
}
trap cleanup EXIT

# kill stale pubs
/data/swarm/robot-swarm-brain/kill-stale-ros-pubs.sh 2>/dev/null || true

docker run --rm --network host \
  -v "$SCRIPT:/pulse.py:ro" \
  -v "$OUT:/tmp/pulse_out" \
  -v "$TELEM_DIR:/telem:ro" \
  -e ROS_DOMAIN_ID=0 \
  ros:jazzy-ros-base \
  bash -lc "source /opt/ros/jazzy/setup.bash && python3 /pulse.py --out /tmp/pulse_out --telem-dir /telem --yaw-source quat --ang-lim 0.08 --abort-err-deg 20 --max-age-ms 150 $*"

echo "Artifacts in $OUT"
