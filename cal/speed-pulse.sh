#!/bin/sh
# Brief cmd_vel burst for bench / speed checks. Run from ~/robot-fleet on dockerhost.
# Usage: ./speed-pulse.sh [linear_x] [seconds] [rate_hz]
#   ./speed-pulse.sh 0.2 4     # gentle creep test
#   ./speed-pulse.sh 0.8 4     # strong forward
# Rate default 10 Hz — bot watchdog is 400 ms; anything >= 3 Hz keeps drive alive.
# Set RESTART_BRAIN=1 to bring fleet-brain back when done (default: leave stopped).
set -e

LINEAR="${1:-1.0}"
SECS="${2:-2}"
RATE="${3:-10}"
NS="${BOT_NS:-bot1}"
TOPIC="/${NS}/cmd_vel"
TIMES=$((SECS * RATE))

. "$(dirname "$0")/_root.sh"
. "$(dirname "$0")/ros-wait-bot.sh"

"$(dirname "$0")/kill-stale-ros-pubs.sh" 2>/dev/null || true

echo "=== speed pulse: linear.x=${LINEAR} for ${SECS}s @ ${RATE}Hz on ${TOPIC} ==="
echo "(fleet brain should be stopped)"

docker compose stop fleet-brain 2>/dev/null || true
sleep 0.3

wait_for_bot_topic "$TOPIC" 20

PUBS=$(./ros.sh topic info "$TOPIC" 2>/dev/null | awk '/Publisher count:/ {print $3}')
if [ -n "$PUBS" ] && [ "$PUBS" != "0" ]; then
  echo "WARN: ${PUBS} publisher(s) on ${TOPIC} — run cal/kill-stale-ros-pubs.sh" >&2
fi

./ros.sh topic pub "$TOPIC" geometry_msgs/msg/Twist "{linear: {x: ${LINEAR}}}" \
  --rate "$RATE" --times "$TIMES"
./ros.sh topic pub "$TOPIC" geometry_msgs/msg/Twist "{}" --once >/dev/null 2>&1
echo "stopped"

if [ "${RESTART_BRAIN:-0}" = "1" ]; then
  docker compose start fleet-brain
  echo "fleet-brain restarted"
fi
