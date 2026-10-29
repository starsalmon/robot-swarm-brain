#!/bin/sh
# Wait until the mini-bot is subscribed on a namespaced topic (DDS discovery).
# Usage: . ./ros-wait-bot.sh && wait_for_bot_topic /bot1/cmd_vel

wait_for_bot_topic() {
  topic="${1:?topic required}"
  max_wait="${2:-20}"
  n=0
  while [ "$n" -lt "$max_wait" ]; do
    if ./ros.sh topic info "$topic" 2>/dev/null | grep -qE 'Subscription count: [1-9]'; then
      return 0
    fi
    n=$((n + 1))
    echo "waiting for ${topic} (${n}/${max_wait})..."
    sleep 1
  done
  return 1
}
