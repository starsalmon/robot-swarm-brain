#!/bin/sh
# Read pack voltage over ROS and show calibration hint.
# For best accuracy: measure cell with DMM, compare to reported value, update BAT_ADC_SCALE in bot_pins.h.
# Usage: ./battery-check.sh [samples]
set -e

NS="${BOT_NS:-bot1}"
TOPIC="/${NS}/battery/voltage"
SAMPLES="${1:-5}"

. "$(dirname "$0")/_root.sh"

echo "=== battery check: ${TOPIC} (${SAMPLES} samples) ==="
echo "Measure the cell/pack with a DMM at the same time."
echo "New scale = DMM_volts / (GPIO_tap_volts) × current_scale"
echo "  (bench USB: press v on serial — shows GPIO mV and scale)"
echo ""

i=0
while [ "$i" -lt "$SAMPLES" ]; do
  if out=$(./topic-once.sh "$TOPIC" 12 2>/dev/null); then
    echo "$out"
  else
    echo "WARN: no sample (mini on WiFi? agent up?)" >&2
  fi
  i=$((i + 1))
  [ "$i" -lt "$SAMPLES" ] && sleep 1
done
