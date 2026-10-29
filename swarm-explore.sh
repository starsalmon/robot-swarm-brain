#!/bin/sh
# Start (or restart) the full fleet stack on dockerhost.
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export BOT_NS="${BOT_NS:-bot1}"

echo "=== Fleet: agent + explore/chase brain ==="
docker compose --profile wander down 2>/dev/null || true
docker compose --profile chase down 2>/dev/null || true
docker compose up -d

echo ""
echo "Waiting for /${BOT_NS} topics (mini on WiFi)..."
ok=0
i=0
while [ "$i" -lt 22 ]; do
  if ./ros.sh topic list 2>/dev/null | grep -q "/${BOT_NS}/sonar/range"; then
    ok=1
    break
  fi
  sleep 2
  i=$((i + 1))
done

if [ "$ok" -eq 1 ]; then
  echo "OK: ${BOT_NS} connected."
  ./ros.sh topic list 2>/dev/null | grep "/${BOT_NS}/" || true
else
  echo "WARN: /${BOT_NS} not seen — check mini WiFi, AGENT_IP, microros-agent logs."
fi

echo ""
docker compose ps
echo ""
echo "Brain logs: docker logs -f fleet-brain"
echo "ROS CLI:    ./ros.sh topic echo /${BOT_NS}/cmd_vel"
