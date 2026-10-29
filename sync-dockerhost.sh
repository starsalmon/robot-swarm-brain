#!/usr/bin/env bash
# Run from repo root OR anywhere — syncs fleet stack to dockerhost.
set -eo pipefail

HOST="${DOCKERHOST:-dockerhost}"
DEST="${ROBOT_FLEET_DIR:-robot-fleet}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROVER_ROS2_DIR="$(cd "$SCRIPT_DIR/../../../lego-rover-ros2" && pwd)"

echo "→ $HOST:~/$DEST/"

ssh "$HOST" "mkdir -p ~/$DEST/cal"
scp \
  "$SCRIPT_DIR/docker-compose.yml" \
  "$SCRIPT_DIR/Dockerfile.ha" \
  "$SCRIPT_DIR/mqtt_secrets.env.example" \
  "$SCRIPT_DIR/fastdds_udp.xml" \
  "$SCRIPT_DIR/ros.sh" \
  "$SCRIPT_DIR/rover-record.sh" \
  "$SCRIPT_DIR/rover-record-qos.yaml" \
  "$SCRIPT_DIR/topic-once.sh" \
  "$SCRIPT_DIR/swarm-explore.sh" \
  "$SCRIPT_DIR/setup-dockerhost.sh" \
  "$SCRIPT_DIR/expand-root.sh" \
  "$ROS2_DIR/mini_wander.py" \
  "$ROS2_DIR/beacon_chase.py" \
  "$ROS2_DIR/explore_then_chase.py" \
  "$ROS2_DIR/fleet_steering.py" \
  "$ROS2_DIR/fleet_meet_spin.py" \
  "$ROS2_DIR/fleet_ha_bridge.py" \
  "$ROS2_DIR/analyze_drive_log.py" \
  "$ROS2_DIR/imu_motion_test.py" \
  "$ROVER_ROS2_DIR/rover_brain.py" \
  "$ROVER_ROS2_DIR/fleet_meet_spin.py" \
  "$ROVER_ROS2_DIR/rover_motion.py" \
  "$ROVER_ROS2_DIR/rover_wheel_odom.py" \
  "$ROVER_ROS2_DIR/rover_wheel_cal.py" \
  "$ROVER_ROS2_DIR/rover_nav_sensors.py" \
  "$ROVER_ROS2_DIR/rover_nav_goals.py" \
  "$ROVER_ROS2_DIR/prepare_nav2_params.py" \
  "$ROVER_ROS2_DIR/rover_nav.sh" \
  "$ROVER_ROS2_DIR/explore_controller.py" \
  "$ROVER_ROS2_DIR/cmd_vel_slew.py" \
  "$ROVER_ROS2_DIR/rover_proximity.py" \
  "$ROVER_ROS2_DIR/rover_local_memory.py" \
  "$ROVER_ROS2_DIR/hallway_follow.py" \
  "$ROVER_ROS2_DIR/rover_radar.py" \
  "$ROVER_ROS2_DIR/analyze_rover_telem.py" \
  "$ROVER_ROS2_DIR/rover_pan_validate.py" \
  "$ROVER_ROS2_DIR/rover_qos.py" \
  "$ROVER_ROS2_DIR/rover_twist.py" \
  "$HOST:~/$DEST/"
if [ -f "$SCRIPT_DIR/mqtt_secrets.env" ]; then
  scp "$SCRIPT_DIR/mqtt_secrets.env" "$HOST:~/$DEST/"
fi
scp "$SCRIPT_DIR/cal/"*.sh "$SCRIPT_DIR/cal/"*.py "$HOST:~/$DEST/cal/" 2>/dev/null || true

ssh "$HOST" "chmod +x ~/$DEST/ros.sh ~/$DEST/rover-record.sh ~/$DEST/topic-once.sh ~/$DEST/swarm-explore.sh \
  ~/$DEST/expand-root.sh ~/$DEST/cal/*.sh ~/$DEST/cal/*.py 2>/dev/null || true"
echo "Done. On dockerhost:"
echo "  cd ~/$DEST && docker compose up -d"
echo "  ./ros.sh topic list"
echo ""
echo "Bench/cal tools (not normal operation): ~/$DEST/cal/"
