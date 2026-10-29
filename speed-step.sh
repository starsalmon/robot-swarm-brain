#!/bin/sh
# Single speed pulse — for one-off science tests.
# Usage: ./speed-step.sh 0.22 6
set -e
exec "$(dirname "$0")/speed-pulse.sh" "${1:-0.2}" "${2:-4}" "${3:-10}"
