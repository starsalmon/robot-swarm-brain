#!/bin/sh
# Find minimum cmd_vel linear.x that moves the bot. Pauses fleet brain during sweep.
# Usage: ./min-speed-sweep.sh [hold_sec_per_step]
# Watch the wheels — note the first speed that actually creeps forward.
set -e

HOLD="${1:-3}"
NS="${BOT_NS:-bot1}"
TOPIC="/${NS}/cmd_vel"
RATE=10
TIMES=$((HOLD * RATE))

. "$(dirname "$0")/_root.sh"
. "$(dirname "$0")/ros-wait-bot.sh"

"$(dirname "$0")/kill-stale-ros-pubs.sh" 2>/dev/null || true

echo "=== min speed sweep (linear.x, ${HOLD}s each) ==="
echo "Fleet brain paused. Put bot on clear floor, tail wheel down."
echo ""

docker compose stop fleet-brain 2>/dev/null || true
sleep 0.3

wait_for_bot_topic "$TOPIC" 20

PUBS=$(./ros.sh topic info "$TOPIC" 2>/dev/null | awk '/Publisher count:/ {print $3}')
if [ -n "$PUBS" ] && [ "$PUBS" != "0" ]; then
  echo "WARN: ${PUBS} publisher(s) on ${TOPIC} — run ./kill-stale-ros-pubs.sh" >&2
fi

for LIN in 0.08 0.10 0.12 0.14 0.16 0.18 0.20 0.22 0.25; do
  echo "--- linear.x=${LIN} for ${HOLD}s ---"
  ./ros.sh topic pub "$TOPIC" geometry_msgs/msg/Twist \
    "{linear: {x: ${LIN}}}" --rate "$RATE" --times "$TIMES"
  ./ros.sh topic pub "$TOPIC" geometry_msgs/msg/Twist "{}" --once >/dev/null 2>&1
  sleep 1
done

echo ""
echo "Done. Note the lowest speed that moved reliably."
echo "(fleet-brain still stopped — docker compose start fleet-brain when ready)"
