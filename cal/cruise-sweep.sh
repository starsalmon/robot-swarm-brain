#!/bin/sh
# Test cruise-speed candidates (noise, power sag, obstacle reaction time).
# Usage: ./cruise-sweep.sh [hold_sec_per_step]
# Mark the highest speed that still feels controllable — that's your 95% cruise.
set -e

HOLD="${1:-5}"
NS="${BOT_NS:-bot1}"
TOPIC="/${NS}/cmd_vel"
RATE=10
TIMES=$((HOLD * RATE))

. "$(dirname "$0")/_root.sh"
. "$(dirname "$0")/ros-wait-bot.sh"

"$(dirname "$0")/kill-stale-ros-pubs.sh" 2>/dev/null || true

echo "=== cruise speed sweep (linear.x, ${HOLD}s each) ==="
echo "Floor on clear space. Note: first creep, max comfortable cruise, too fast/noisy."
echo ""

docker compose stop fleet-brain 2>/dev/null || true
sleep 0.3
wait_for_bot_topic "$TOPIC" 20

for LIN in 0.18 0.20 0.22 0.25 0.28 0.30 0.35 0.40; do
  echo "--- linear.x=${LIN} for ${HOLD}s ---"
  ./ros.sh topic pub "$TOPIC" geometry_msgs/msg/Twist \
    "{linear: {x: ${LIN}}}" --rate "$RATE" --times "$TIMES"
  ./ros.sh topic pub "$TOPIC" geometry_msgs/msg/Twist "{}" --once >/dev/null 2>&1
  sleep 2
done

echo ""
echo "Done. Pick cruise = highest speed still quiet + controllable."
echo "Set WANDER_CRUISE on dockerhost (see fleet .env) before restarting explore brain."
