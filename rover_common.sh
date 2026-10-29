#!/usr/bin/env bash
# Shared helpers for LEGO rover launch scripts (run on the Pi).
set -eo pipefail

ROVER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyAMA0}"
BAUD="${BAUD:-460800}"
AGENT_SETUP="${AGENT_SETUP:-$HOME/uros_agent_ws/install/local_setup.bash}"

# Pi GPIO / I2C hardware defaults (override via environment).
export ROVER_IR_AUX_LED_GPIO="${ROVER_IR_AUX_LED_GPIO:-26}"
export ROVER_IR_FRONT_LED_GPIO="${ROVER_IR_FRONT_LED_GPIO:-6}"
export ROVER_IR_FRONT_PULSE="${ROVER_IR_FRONT_PULSE:-1}"
export ROVER_IR_FRONT_OFF_MS="${ROVER_IR_FRONT_OFF_MS:-50}"
export ROVER_IR_FRONT_ON_MS="${ROVER_IR_FRONT_ON_MS:-10}"
export ROVER_IR_AUX_GPIO="${ROVER_IR_AUX_GPIO:-27}"
export ROVER_IR_FRONT_GPIO="${ROVER_IR_FRONT_GPIO:-22}"
export ROVER_PCA9685_OE_GPIO="${ROVER_PCA9685_OE_GPIO:-0}"
export ROVER_PCA9685_OE_ACTIVE_LOW="${ROVER_PCA9685_OE_ACTIVE_LOW:-1}"
export ROVER_SERVO_SOURCE="${ROVER_SERVO_SOURCE:-esp}"
export ROVER_IR_SOURCE="${ROVER_IR_SOURCE:-esp}"
export ROVER_WHEEL_STRIPS="${ROVER_WHEEL_STRIPS:-60}"
export ROVER_WHEEL_DIAM_MM="${ROVER_WHEEL_DIAM_MM:-95}"
export ROVER_WHEEL_TRACK_MM="${ROVER_WHEEL_TRACK_MM:-130}"
export ROVER_WHEEL_ODOM="${ROVER_WHEEL_ODOM:-1}"
export ROVER_WHEEL_STALL="${ROVER_WHEEL_STALL:-0}"

# Radar / servo sweep
export ROVER_RADAR_SMOOTH="${ROVER_RADAR_SMOOTH:-0}"
export ROVER_RADAR_STEP_DEG="${ROVER_RADAR_STEP_DEG:-4}"
export ROVER_RADAR_STEP_S="${ROVER_RADAR_STEP_S:-0.045}"
export ROVER_CRUISE_RADAR="${ROVER_CRUISE_RADAR:-1}"
export ROVER_CRUISE_RADAR_STEPS="${ROVER_CRUISE_RADAR_STEPS:-5}"
export ROVER_RADAR_SETTLE_S="${ROVER_RADAR_SETTLE_S:-0.18}"
export ROVER_CRUISE_RADAR_SETTLE="${ROVER_CRUISE_RADAR_SETTLE:-0.06}"
export ROVER_CRUISE_RADAR_BIAS="${ROVER_CRUISE_RADAR_BIAS:-0.07}"
export ROVER_CRUISE_RADAR_PING="${ROVER_CRUISE_RADAR_PING:-1}"
export ROVER_SPEAKER_GPIO="${ROVER_SPEAKER_GPIO:-17}"
export ROVER_RING_GPIO="${ROVER_RING_GPIO:-18}"
export ROVER_SPEAKER_VOL="${ROVER_SPEAKER_VOL:-0.38}"
export ROVER_WALL_SIDE="${ROVER_WALL_SIDE:-right}"
export ROVER_WALL_LINEAR="${ROVER_WALL_LINEAR:-0.28}"
export ROVER_WALL_STEER="${ROVER_WALL_STEER:-0.08}"

# cmd_vel signs (software — no rewire). Tune via env if a motor is reversed.
export ROVER_LINEAR_SIGN="${ROVER_LINEAR_SIGN:-1}"
export ROVER_ANGULAR_SIGN="${ROVER_ANGULAR_SIGN:-1}"
export ROVER_IMU_GYRO_SIGN="${ROVER_IMU_GYRO_SIGN:-1}"
export ROVER_AUTO_MAX_LINEAR="${ROVER_AUTO_MAX_LINEAR:-0.23}"
export ROVER_AUTO_REVERSE_FRAC="${ROVER_AUTO_REVERSE_FRAC:-0.72}"

rover_source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  if [[ -f "$AGENT_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$AGENT_SETUP"
  fi
  set -u
}

rover_preflight() {
  local errors=0

  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ERROR: ROS 2 Jazzy not installed. Run setup_ros2_jazzy.sh"
    errors=$((errors + 1))
  fi

  if [[ ! -f "$AGENT_SETUP" ]]; then
    echo "ERROR: micro-ROS agent not built."
    echo "  Run: bash ~/lego-rover-ros2/install_microros_agent.sh"
    echo "  Or resume: cd ~/uros_agent_ws && source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && ros2 run micro_ros_setup build_agent.sh"
    errors=$((errors + 1))
  fi

  if [[ ! -e "$SERIAL_DEV" ]]; then
    echo "ERROR: $SERIAL_DEV missing. Run enable_uart.sh then reboot."
    errors=$((errors + 1))
  fi

  if ! groups | grep -q '\bdialout\b'; then
    echo "WARN: user not in dialout group — serial may fail."
    echo "  Fix: sudo usermod -aG dialout $USER  (then log out/in)"
  fi

  if ! ros2 pkg prefix micro_ros_agent &>/dev/null; then
    echo "ERROR: micro_ros_agent package not in ROS path after sourcing agent workspace."
    errors=$((errors + 1))
  fi

  return "$errors"
}

rover_stop_motors() {
  set +u
  source /opt/ros/jazzy/setup.bash 2>/dev/null || true
  source ~/uros_agent_ws/install/local_setup.bash 2>/dev/null || true
  set -u
  local i
  for i in 1 2 3 4 5 6 7 8; do
    ros2 topic pub --once --no-wait-for-subscribers --qos-reliability reliable \
      /cmd_vel geometry_msgs/msg/Twist "{}" 2>/dev/null || true
    sleep 0.05
  done
}

rover_ir_close() {
  python3 -c "import sys; sys.path.insert(0,'$(dirname "${BASH_SOURCE[0]}")'); from rover_ir import close; close()" \
    2>/dev/null || true
}

rover_stop_all() {
  bash "$(dirname "${BASH_SOURCE[0]}")/stop_rover.sh" 2>/dev/null || {
    pkill -f micro_ros_agent 2>/dev/null || true
    pkill -f demo_showcase 2>/dev/null || true
    pkill -f demo_smooth_drive 2>/dev/null || true
    rover_stop_motors
  }
}

rover_start_agent() {
  echo "Starting micro-ROS agent on $SERIAL_DEV @ $BAUD"
  ros2 run micro_ros_agent micro_ros_agent serial --dev "$SERIAL_DEV" -b "$BAUD" &
  AGENT_PID=$!
  sleep 5
  if ! kill -0 "$AGENT_PID" 2>/dev/null; then
    echo "ERROR: micro-ROS agent exited immediately."
    return 1
  fi
  echo "Agent running (pid $AGENT_PID). ESP32 should report 'micro-ROS ready' on USB serial."
}

rover_esp_linked() {
  ros2 node list 2>/dev/null | grep -q 'lego_rover_esp32'
}

rover_wait_for_esp() {
  local timeout="${1:-45}"
  local i=0
  while (( i < timeout )); do
    if rover_esp_linked; then
      echo "ESP32 linked (lego_rover_esp32 visible on ROS graph)."
      return 0
    fi
    sleep 1
    i=$((i + 1))
  done
  echo "ERROR: ESP32 not on ROS graph after ${timeout}s."
  echo "  USB serial: hb [wait] = waiting for agent, hb [up] = connected"
  echo "  Check UART GPIO20/21; legacy firmware needs one ESP power-cycle"
  return 1
}

rover_ensure_agent() {
  local log="${ROVER_AGENT_LOG:-/tmp/rover_agent.log}"
  AGENT_PID=""

  bash "$ROVER_DIR/rover_kill.sh" --keep-agent

  if systemctl --user is-active --quiet rover-agent.service 2>/dev/null; then
    if rover_esp_linked; then
      echo "rover-agent.service running, ESP32 linked."
      return 0
    fi
    echo "rover-agent.service running — waiting for ESP32..."
    rover_wait_for_esp 45
    return $?
  fi

  if rover_esp_linked; then
    AGENT_PID=$(pgrep -f 'micro_ros_agent serial' | head -1 || true)
    if [[ -n "$AGENT_PID" ]]; then
      echo "micro-ROS agent + ESP32 already linked (pid $AGENT_PID)."
      return 0
    fi
  fi

  if pgrep -f 'micro_ros_agent serial' >/dev/null; then
    echo "Restarting agent — ESP32 should auto-reconnect (no power-cycle)..."
    pkill -f micro_ros_agent 2>/dev/null || true
    if command -v fuser &>/dev/null; then
      sudo fuser -k "$SERIAL_DEV" 2>/dev/null || true
    fi
    sleep 1
  fi

  echo "Starting micro-ROS agent on $SERIAL_DEV @ $BAUD..."
  ros2 run micro_ros_agent micro_ros_agent serial --dev "$SERIAL_DEV" -b "$BAUD" \
    >>"$log" 2>&1 &
  AGENT_PID=$!
  sleep 2

  if ! kill -0 "$AGENT_PID" 2>/dev/null; then
    echo "ERROR: agent died. tail $log"
    tail -5 "$log" 2>/dev/null || true
    return 1
  fi

  echo "Waiting for ESP32 (auto-reconnect, up to 45s)..."
  rover_wait_for_esp 45
}

rover_pulse_forward() {
  local speed="${1:-0.6}"
  local secs="${2:-3}"
  timeout "$secs" ros2 topic pub --rate 20 --qos-reliability reliable \
    /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $speed}}" || true
  timeout 1 ros2 topic pub --rate 20 --qos-reliability reliable \
    /cmd_vel geometry_msgs/msg/Twist "{}" || true
}
