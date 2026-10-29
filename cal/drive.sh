#!/bin/sh
# DEPRECATED — motor cal belongs on USB bench firmware (c3_bench + L/R/B serial).
# This script required /wheel_speeds over ROS; fleet bots only accept /cmd_vel from fleet-brain.
#
#   L 15     left wheel 15%
#   R 15     right wheel 15%
#   B 25     both 25%
#   S        stop
#   ?        help
#   q        quit
#
# Percent -100..100. Holds until you change it or S.
set -e

NS="${BOT_NS:-bot1}"
TOPIC="/${NS}/wheel_speeds"
RATE=10

. "$(dirname "$0")/_root.sh"
. "$(dirname "$0")/ros-wait-bot.sh"

PUB_PID=""
STOP=0

stop_stream() {
  STOP=1
  if [ -n "$PUB_PID" ]; then
    kill "$PUB_PID" 2>/dev/null || true
    wait "$PUB_PID" 2>/dev/null || true
    PUB_PID=""
  fi
}

cleanup() {
  stop_stream
  ./ros.sh topic pub "$TOPIC" geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}" \
    --once >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

pct_to_float() {
  pct="$1"
  awk "BEGIN { v=$pct; if (v>100) v=100; if (v<-100) v=-100; printf \"%.3f\", v/100.0 }"
}

stream_wheels() {
  L="$1"
  R="$2"
  stop_stream
  STOP=0
  echo "→ L=${L} R=${R} (hold until S or new command)"
  # One container, steady --rate; looping --once per docker spin caused ~3s stop/go gaps.
  (
    ./ros.sh topic pub "$TOPIC" geometry_msgs/msg/Vector3 \
      "{x: ${L}, y: ${R}, z: 0.0}" --rate "$RATE"
  ) >/dev/null 2>&1 &
  PUB_PID=$!
}

show_help() {
  echo ""
  echo "Drive test on battery (percent -100..100):"
  echo "  L <pct>   left wheel only"
  echo "  R <pct>   right wheel only"
  echo "  B <pct>   both wheels"
  echo "  S         stop"
  echo "  ?         help"
  echo "  q         quit"
  echo ""
}

docker compose stop fleet-brain 2>/dev/null || true
"$(dirname "$0")/kill-stale-ros-pubs.sh" 2>/dev/null || true

echo "=== drive.sh (battery / WiFi) ==="
echo "Unplug USB. Wait for bot green/blue LED + topics up."
wait_for_bot_topic "$TOPIC" 30 || wait_for_bot_topic "/${NS}/cmd_vel" 10
show_help

while read -r cmd arg _extra; do
  case "$cmd" in
    ''|'#') continue ;;
    q|Q) break ;;
    '?'|h|H) show_help ;;
    s|S)
      stop_stream
      ./ros.sh topic pub "$TOPIC" geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}" \
        --once >/dev/null 2>&1 || true
      echo "stop"
      ;;
    l|L)
      [ -z "$arg" ] && echo "usage: L <percent>" && continue
      stream_wheels "$(pct_to_float "$arg")" "0.0"
      ;;
    r|R)
      [ -z "$arg" ] && echo "usage: R <percent>" && continue
      stream_wheels "0.0" "$(pct_to_float "$arg")"
      ;;
    b|B)
      [ -z "$arg" ] && echo "usage: B <percent>" && continue
      F="$(pct_to_float "$arg")"
      stream_wheels "$F" "$F"
      ;;
    *)
      echo "unknown: $cmd  (? for help)"
      ;;
  esac
done
