#!/bin/bash
# Low-level house run launcher: watchdog + supervisor (supervisor starts the frontier planner and taps Go).
# Use go_house_explore.sh in the morning; it runs the preflight first and then calls this.
set -uo pipefail
HOST_MAP=/data/swarm/robot-swarm-brain/maps/lidar
RUN=$HOST_MAP/house_run
SRC_DIR=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$RUN"
copy_ok() { src=$1; dst=$2; [ "$src" -ef "$dst" ] && return 0; cp -f "$src" "$dst"; }
copy_ok "$SRC_DIR/house_status_node.py" "$HOST_MAP/house_status_node.py"
copy_ok "$SRC_DIR/render_map_png.py" "$HOST_MAP/render_map_png.py"
for f in house_supervisor.py lidar_watch.py lidar_confirm.py pub_cmd.py watchdog.sh frontier_core.py rover_frontier.py map_match_check.py; do
  [ -f "$SRC_DIR/$f" ] && copy_ok "$SRC_DIR/$f" "$RUN/$f"
done
chmod +x "$RUN/"*.py "$RUN/"*.sh 2>/dev/null || true
# clear leftovers BEFORE the watchdog starts so it can never judge a stale heartbeat/pid/done flag
rm -f "$RUN/supervisor.done" "$RUN/stop.flag" "$RUN/supervisor.heartbeat" "$RUN/supervisor.pid"
# stop a leftover watchdog so it cannot e-stop the new run
if [ -f "$RUN/watchdog.pid" ]; then
  oldwd=$(cat "$RUN/watchdog.pid" 2>/dev/null || true)
  if [ -n "${oldwd:-}" ]; then kill "$oldwd" 2>/dev/null || true; fi
fi
pkill -f "house_run/watchdog.sh" 2>/dev/null || true
sleep 1
export HOUSE_DURATION_S=${HOUSE_DURATION_S:-1200}
export HOUSE_SNAPSHOT_EVERY_S=${HOUSE_SNAPSHOT_EVERY_S:-180}
nohup bash "$RUN/watchdog.sh" >"$RUN/watchdog.nohup" 2>&1 &
echo $! > "$RUN/watchdog.pid"
python3 "$RUN/house_supervisor.py" &
sup=$!
echo $sup > "$RUN/supervisor.pid"
wait $sup
rc=$?
[ -f "$RUN/supervisor.done" ] || date +%s > "$RUN/supervisor.done"
exit $rc
