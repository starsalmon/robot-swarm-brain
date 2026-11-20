#!/usr/bin/env bash
# Record /cmd_vel + /imu/data while testing steering feel (rosbag2).
# Usage: record_steer_session.sh [seconds] [prep_seconds]
set -uo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
SECS="${1:-45}"
PREP="${2:-12}"

set +u
source /opt/ros/jazzy/setup.bash
source ~/uros_agent_ws/install/local_setup.bash 2>/dev/null || true
set -u

OUT="$HOME/rover_steer_$(date +%Y%m%d_%H%M%S)"

_analyze() {
  if [[ -f "$OUT/metadata.yaml" ]]; then
    cp "$DIR/teleop_mac.yaml" "$OUT/teleop_mac.yaml" 2>/dev/null || true
    python3 "$DIR/analyze_tune_bag.py" "$OUT"
  else
    echo "WARN: no bag written."
    return 1
  fi
}

echo ""
echo "=============================================="
echo "  STEER TUNE CAPTURE"
echo "  Mac: ./go_mac.sh running"
echo "  Record: ${SECS}s  |  Do NOT press Ctrl+C"
echo "  Saved: $OUT"
echo "=============================================="
echo ""
bash "$DIR/check_drive_ready.sh" || true
echo ""
echo "When recording starts:"
echo "  1) Drive forward + steer left/right"
echo "  2) Stop and spin slowly in place"
echo "  3) Drive a few gentle arcs"
echo ""

while (( PREP > 0 )); do
  echo "  *** GET READY — recording starts in ${PREP}s ***"
  sleep 1
  PREP=$((PREP - 1))
done

for i in 3 2 1; do
  echo ""
  echo "  >>> RECORD IN ${i} <<<"
  sleep 1
done

echo ""
echo "=============================================="
echo "  >>>>>>  DRIVE / STEER / SPIN NOW  <<<<<<"
echo "  >>>>>>       ${SECS} SECONDS        <<<<<<"
echo "=============================================="
echo ""

ros2 bag record \
  --qos-profile-overrides-path "$(dirname "$0")/rover-record-qos.yaml" \
  --topics /cmd_vel /imu/data \
  -s mcap \
  -d "$SECS" \
  --max-cache-size "${ROVER_BAG_CACHE:-1048576}" \
  -o "$OUT"

echo ""
echo "=============================================="
echo "  STOP — capture finished"
echo "  Saved: $OUT"
echo "=============================================="
_analyze
