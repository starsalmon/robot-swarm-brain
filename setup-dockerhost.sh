#!/usr/bin/env bash
# Run ON dockerhost (Linux VM). Creates ~/robot-fleet and starts the full stack.
set -eo pipefail

DEST="${ROBOT_FLEET_DIR:-$HOME/robot-fleet}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

mkdir -p "$DEST/cal"
cp -f "$SCRIPT_DIR/docker-compose.yml" "$SCRIPT_DIR/ros.sh" \
  "$SCRIPT_DIR/topic-once.sh" "$SCRIPT_DIR/swarm-explore.sh" \
  "$SCRIPT_DIR/setup-dockerhost.sh" "$DEST/"
cp -f "$ROS2_DIR/mini_wander.py" "$ROS2_DIR/beacon_chase.py" \
  "$ROS2_DIR/explore_then_chase.py" "$ROS2_DIR/fleet_steering.py" \
  "$ROS2_DIR/imu_motion_test.py" "$DEST/"
for f in _root.sh battery-check.sh imu-check.sh imu-motion-test.sh record-explore-imu.sh \
  cruise-sweep.sh min-speed-sweep.sh speed-pulse.sh speed-step.sh drive.sh \
  kill-stale-ros-pubs.sh ros-wait-bot.sh; do
  if [ -f "$SCRIPT_DIR/cal/$f" ]; then
    cp -f "$SCRIPT_DIR/cal/$f" "$DEST/cal/"
  elif [ -f "$SCRIPT_DIR/$f" ]; then
    cp -f "$SCRIPT_DIR/$f" "$DEST/cal/"
  fi
done
chmod +x "$DEST/ros.sh" "$DEST/topic-once.sh" "$DEST/swarm-explore.sh"
chmod +x "$DEST/cal/"*.sh 2>/dev/null || true

cd "$DEST"
docker compose pull
docker compose up -d
docker compose ps

echo ""
echo "Fleet running: microros-agent + fleet-brain (explore/chase) + fleet-shell."
echo "Power on a mini-bot — it connects and drives autonomously; no scripts to start."
echo ""
echo "ROS CLI:  $DEST/ros.sh topic list"
echo "Logs:     docker logs -f fleet-brain"
echo ""
echo "Bench motor cal (USB c3_bench only): see $DEST/cal/"
