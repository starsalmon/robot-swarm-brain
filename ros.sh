#!/bin/sh
# ROS 2 CLI via the persistent fleet-shell container (not a new container per command).
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"

TTY_FLAG=""
if [ -t 0 ]; then
  TTY_FLAG="-it"
fi

if ! docker compose ps fleet-shell 2>/dev/null | grep -qE 'Up|running'; then
  echo "fleet-shell not running — start the fleet: docker compose up -d" >&2
  exit 1
fi

exec docker compose exec $TTY_FLAG fleet-shell bash -lc \
  "source /opt/ros/jazzy/setup.bash && exec ros2 $*"
