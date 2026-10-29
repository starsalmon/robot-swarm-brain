#!/bin/sh
# Wheel tick stall cal — brain drives via Go button (same path as normal operation).
set -euo pipefail
cd "$(dirname "$0")/.."
. cal/_root.sh

echo "=== wheel tick cal (brain drives — wheels must spin freely) ==="
docker compose up -d rover-brain fleet-shell microros-agent

echo "Waiting for rover-brain..."
for _ in $(seq 1 20); do
  if docker compose logs rover-brain --tail 5 2>/dev/null | grep -q 'explore (startup)'; then
    break
  fi
  sleep 1
done
sleep 2

cal/kill-stale-ros-pubs.sh 2>/dev/null || true

if ! ./ros.sh topic list 2>/dev/null | grep -q '/rover/wheel/left_ticks'; then
  echo "ERROR: /rover/wheel/left_ticks not found — is rover-esp online?" >&2
  exit 1
fi

docker compose exec -T fleet-shell bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /opt/fleet/cal/wheel_tick_cal.py'

docker compose exec -T fleet-shell cat /tmp/wheel_stall.env > cal/wheel_stall.env
echo "Wrote cal/wheel_stall.env:"
cat cal/wheel_stall.env

docker compose restart rover-brain
echo "Done — brain restarted with new cal."
