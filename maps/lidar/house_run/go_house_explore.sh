#!/bin/bash
# Morning entry point: preflight -> (continue or fresh SLAM map) -> frontier planner + supervisor + watchdog -> Go.
#
#   bash /data/swarm/robot-swarm-brain/maps/lidar/house_run/go_house_explore.sh            # full run
#   PREFLIGHT_ONLY=1 bash .../go_house_explore.sh                                          # checks only, never taps Go
#   MAP_MODE=continue|fresh|auto (default auto)   HOUSE_DURATION_S=1200   MIN_START_V=7.0
#
# Never writes /fleet/allow_drive. Never flashes anything. Rover only moves after the supervisor's Go tap,
# and only if allow_drive is already true.
set -uo pipefail
HOST_MAP=/data/swarm/robot-swarm-brain/maps/lidar
RUN=$HOST_MAP/house_run
SLAM=rover-lidar-slam
BEH=rover-behaviour
MAP_MODE=${MAP_MODE:-auto}
MIN_START_V=${MIN_START_V:-7.0}
LOG=$RUN/go_house_explore.log
mkdir -p "$RUN"
say() { echo "$(TZ=Australia/Melbourne date '+%H:%M:%S %Z') $*" | tee -a "$LOG"; }
die() { say "ABORT: $*"; exit 2; }
ros() { docker exec "$SLAM" bash -lc "source /opt/ros/jazzy/setup.bash && $1"; }
jget() { python3 -c "import json,sys; d=json.loads(sys.argv[1]); v=d.get(sys.argv[2]); print('' if v is None else v)" "$1" "$2"; }

say "=== house explore start (MAP_MODE=$MAP_MODE PREFLIGHT_ONLY=${PREFLIGHT_ONLY:-0}) ==="
if [ -f "$RUN/supervisor.pid" ] && kill -0 "$(cat "$RUN/supervisor.pid")" 2>/dev/null && [ ! -f "$RUN/supervisor.done" ]; then
  die "a house run supervisor is already running (pid $(cat "$RUN/supervisor.pid")); touch $RUN/stop.flag to stop it"
fi

# 1. containers
for c in "$SLAM" "$BEH" rover-brain rover-odom microros-agent; do
  [ "$(docker inspect -f '{{.State.Running}}' "$c" 2>/dev/null)" = "true" ] || die "container $c is not running"
done
# rover-behaviour must have loaded the map-explore code (restart leaves go=False: motors stay stopped)
beh_start=$(date -d "$(docker inspect -f '{{.State.StartedAt}}' "$BEH")" +%s)
code_mtime=$(stat -c %Y /data/swarm/robot-swarm-brain/rover_explore.py /data/swarm/robot-swarm-brain/rover_behaviour.py | sort -n | tail -1)
if [ "$beh_start" -lt "$code_mtime" ]; then
  say "rover-behaviour started before the map-explore code changed -> restarting it (go=False, no Go tap)"
  docker restart "$BEH" >/dev/null || die "rover-behaviour restart failed"
  sleep 25
fi

# 2. read-only preflight
pf=$(ros "python3 /maps/lidar/house_run/preflight_explore.py 8" 2>/dev/null | tail -1)
[ -n "$pf" ] || pf='{}'
say "preflight: $pf"
allow=$(jget "$pf" allow); batt=$(jget "$pf" batt); scan=$(jget "$pf" scan_sync_live); beh=$(jget "$pf" behaviour_live)
lah=$(jget "$pf" lidar_ahead); front=$(jget "$pf" tof_front); hasmap=$(jget "$pf" behaviour_has_map_explore)
[ "$allow" = "True" ] || die "/fleet/allow_drive is '${allow:-unknown}' (not touched). Turn Allow drive on in HA first."
[ "$beh" = "True" ] || die "rover-behaviour status not live"
[ "$hasmap" = "True" ] || die "rover-behaviour is not running the map-explore code (no lidar_ahead in status)"
[ "$scan" = "True" ] || die "/rover/scan_sync not live (lidar off? Pro S3 not on WiFi?)"
imu=$(jget "$pf" imu_mpu)
# No IMU = heading comes only from directionless wheel ticks -> smeared map (2026-11-13). Refuse unless overridden.
[ "$imu" = "1" ] || [ "${ALLOW_NO_IMU:-0}" = "1" ] || die "Rover IMU not answering (drive_diag mpu=${imu:-unknown}). Fix the MPU power/I2C on the drive board first (or ALLOW_NO_IMU=1)."
if [ -z "$batt" ]; then
  say "WARN: no /rover/battery/voltage reading; supervisor still stops at 6.4 V if it appears"
else
  python3 -c "import sys; sys.exit(0 if float('$batt') >= float('$MIN_START_V') else 1)" \
    || die "battery $batt V < MIN_START_V $MIN_START_V V (charge first)"
fi
# lidar self-return sanity (stationary): lidar says blocked but ToF front is clear -> chassis in the beam
rm -f "$RUN/lidar_guard.off"
if [ -n "$lah" ] && [ -n "$front" ]; then
  if python3 -c "import sys; sys.exit(0 if float('$lah') < 0.15 and float('$front') > 0.40 else 1)"; then
    touch "$RUN/lidar_guard.off"
    say "WARN: lidar sees something ${lah} m ahead while ToF front is ${front} m -> probably Rover's own chassis. Lidar guard OFF for this run (ToF guards unchanged). Raise LIDAR_SELF_M to fix."
  fi
fi

# 3. SLAM: continue last night's map or start fresh
mm=$(ros "python3 /maps/lidar/house_run/map_match_check.py 15 0.55" 2>/dev/null | tail -1)
say "map match: ${mm:-none}"
[ -n "$mm" ] || mm='{}'
ok=$(jget "$mm" ok 2>/dev/null)
want_fresh=0
case "$MAP_MODE" in
  fresh) want_fresh=1 ;;
  continue) [ "$ok" = "True" ] || say "WARN: MAP_MODE=continue but scan does not match the map; continuing anyway as asked" ;;
  *) [ "$ok" = "True" ] || want_fresh=1 ;;
esac
if [ "$want_fresh" = 1 ]; then
  stamp=$(TZ=Australia/Melbourne date +%Y%m%d_%H%M)
  say "starting a FRESH map (old one saved as house_prev_$stamp*)"
  ros "ros2 run nav2_map_server map_saver_cli -f /maps/lidar/house_prev_$stamp --ros-args -p save_map_timeout:=10.0" >>"$LOG" 2>&1 || true
  ros "timeout 30 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '/maps/lidar/house_prev_$stamp'}\"" >>"$LOG" 2>&1 || true
  [ -f "$RUN/visited.npz" ] && mv "$RUN/visited.npz" "$RUN/visited_prev_$stamp.npz"
  if [ "${PREFLIGHT_ONLY:-0}" = "1" ]; then
    say "PREFLIGHT_ONLY: would restart $SLAM for a fresh map here; not restarting"
  else
    docker restart "$SLAM" >/dev/null || die "SLAM restart failed"
    sleep 15
    ros "python3 /maps/lidar/house_run/lidar_confirm.py 5" | tee -a "$LOG" | grep -q ALIVE || die "lidar not alive after SLAM restart"
    sleep 5
  fi
else
  say "continuing the existing map"
fi

if [ "${PREFLIGHT_ONLY:-0}" = "1" ]; then
  # show what the planner would do, without Go
  docker exec "$SLAM" bash -lc "pkill -INT -f rover_frontier.py || true"; sleep 1
  docker exec -d "$SLAM" bash -lc "source /opt/ros/jazzy/setup.bash && exec python3 -u /maps/lidar/house_run/rover_frontier.py >> /maps/lidar/house_run/frontier.log 2>&1"
  sleep 12
  say "frontier (dry): $(cat "$RUN/frontier.json" 2>/dev/null)"
  docker exec "$SLAM" bash -lc "pkill -INT -f rover_frontier.py || true"
  say "PREFLIGHT_ONLY done: Rover not started. Picture: $RUN/frontier_latest.png"
  exit 0
fi

# 4. run (watchdog + supervisor; supervisor starts the frontier planner, checks it has a pose, then taps Go)
say "starting watchdog + supervisor (Go tap comes from the supervisor). Stop any time: touch $RUN/stop.flag"
nohup bash "$RUN/start_house_map.sh" >"$RUN/start.nohup" 2>&1 &
echo $! > "$RUN/start.pid"
say "started (pid $(cat "$RUN/start.pid")). Logs: $RUN/supervisor.log  $RUN/frontier.log  picture: $RUN/frontier_latest.png"
