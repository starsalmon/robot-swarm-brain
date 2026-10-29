#!/bin/sh
# Wait for the first message on a topic (battery is 1 Hz — plain --once races discovery).
set -e
TOPIC="${1:?usage: topic-once.sh /bot1/battery/voltage [timeout_sec]}"
WAIT="${2:-15}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
TTY_FLAG=""
if [ -t 0 ]; then
  TTY_FLAG="-it"
fi
docker run --rm $TTY_FLAG --network host \
  -e ROS_DOMAIN_ID \
  -e TOPIC="$TOPIC" \
  -e WAIT="$WAIT" \
  ros:jazzy-ros-base \
  bash -lc 'source /opt/ros/jazzy/setup.bash
deadline=$((SECONDS + WAIT))
while (( SECONDS < deadline )); do
  if ros2 topic list 2>/dev/null | grep -Fxq "$TOPIC"; then
    if out=$(timeout 3 ros2 topic echo "$TOPIC" --once 2>/dev/null); then
      if printf "%s\n" "$out" | grep -q "^data:"; then
        printf "%s\n" "$out" | grep "^data:"
        exit 0
      fi
    fi
  fi
  sleep 1
done
echo "timeout: no message on $TOPIC after ${WAIT}s" >&2
exit 1'
