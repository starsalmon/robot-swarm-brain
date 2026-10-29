#!/bin/sh
# Orphaned one-shot `docker run … ros2 topic pub` containers can fight fleet-brain.
# Does not touch fleet-shell, fleet-brain, or microros-agent.
set -e

. "$(dirname "$0")/_root.sh"

n=0
for id in $(docker ps --filter ancestor=ros:jazzy-ros-base -q 2>/dev/null); do
  name=$(docker inspect -f '{{.Name}}' "$id" 2>/dev/null | tr -d '/')
  case "$name" in
    fleet-shell|fleet-brain|rover-brain|fleet-wander|fleet-beacon-chase|microros-agent) continue ;;
  esac
  docker kill "$id" >/dev/null 2>&1 || true
  n=$((n + 1))
done

if [ "$n" -gt 0 ]; then
  echo "Killed ${n} stale ros:jazzy-ros-base container(s)."
  sleep 0.5
fi
