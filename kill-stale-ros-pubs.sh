#!/bin/sh
# Orphaned `ros.sh topic pub --rate` containers keep publishing cmd_vel after timeout.
# Run before drive tests / calibration pulses.
set -e

COUNT=$(docker ps --filter ancestor=ros:jazzy-ros-base -q 2>/dev/null | wc -l | tr -d ' ')
if [ "$COUNT" = "0" ]; then
  exit 0
fi

echo "Killing ${COUNT} stale ros:jazzy-ros-base container(s) (leftover topic pub)..."
docker ps --filter ancestor=ros:jazzy-ros-base -q | xargs -r docker kill >/dev/null 2>&1 || true
sleep 0.5
