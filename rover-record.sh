#!/usr/bin/env bash
# Always-on rover ROS bag recorder — hourly files under ~/rover-bags.
set -eo pipefail
source /opt/ros/jazzy/setup.bash

OUT="${ROVER_BAG_DIR:-/bags}"
mkdir -p "$OUT"
HOURS="${ROVER_BAG_HOURS:-1}"
RETENTION_DAYS="${ROVER_BAG_RETENTION_DAYS:-3}"

prune_old_bags() {
  if [ "${RETENTION_DAYS}" -le 0 ] 2>/dev/null; then
    return 0
  fi
  # session_YYYY-MM-DD_HHMMSS — drop folders older than N days.
  find "$OUT" -maxdepth 1 -mindepth 1 -type d -name 'session_*' -mtime +"${RETENTION_DAYS}" -print 2>/dev/null | while read -r old; do
    echo "  prune $old (>${RETENTION_DAYS}d)" >&2
    rm -rf "$old"
  done
}

echo "rover-record → $OUT (${HOURS}h chunks, keep ${RETENTION_DAYS}d)" >&2
prune_old_bags
while true; do
  prune_old_bags
  STAMP=$(date -u +%Y-%m-%d_%H%M%S)
  DIR="$OUT/session_${STAMP}"
  echo "  recording $DIR" >&2
  # Record everything (topics + services). We can always ignore data later.
  # Exclude rosbag's own internal event topic to avoid self-noise.
  timeout "${HOURS}h" ros2 bag record -a --include-hidden-topics \
    --exclude-topics /events/write_split \
    -o "$DIR" \
    --qos-profile-overrides-path /opt/fleet/rover-record-qos.yaml \
    || true
  sleep 2
done
