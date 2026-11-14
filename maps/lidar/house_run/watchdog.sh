#!/bin/bash
# Hard safety net for the house run: if the supervisor dies, hangs, or overruns, e-stop Rover
# (button_event 3 -> rover_behaviour go=False -> it publishes zero /cmd_vel) and push a burst of zero /cmd_vel.
set -u
RUN=/data/swarm/robot-swarm-brain/maps/lidar/house_run
LOG=$RUN/watchdog.log
HB=$RUN/supervisor.heartbeat
PIDF=$RUN/supervisor.pid
SLAM=rover-lidar-slam
DUR=${HOUSE_DURATION_S:-1200}
MAX_S=${HOUSE_WATCHDOG_S:-$((DUR + 300))}
STALE_S=90          # snapshots (map_saver + render) can legitimately block the heartbeat ~80 s
mkdir -p "$RUN"
echo "$(date -Is) watchdog start max=${MAX_S}s" | tee -a "$LOG"
t0=$(date +%s)

estop() {
  reason=$1
  echo "$(date -Is) WATCHDOG ESTOP: $reason" | tee -a "$LOG"
  docker exec "$SLAM" bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /maps/lidar/house_run/pub_cmd.py estop' >>"$LOG" 2>&1 \
    || docker exec "$SLAM" bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /rover/button_event std_msgs/msg/UInt8 "{data: 3}"' >>"$LOG" 2>&1 || true
  docker exec "$SLAM" bash -lc 'source /opt/ros/jazzy/setup.bash && timeout 4 ros2 topic pub -r 10 -t 20 /cmd_vel geometry_msgs/msg/Twist "{}"' >>"$LOG" 2>&1 || true
  docker exec "$SLAM" bash -lc 'pkill -INT -f rover_frontier.py || true' >>"$LOG" 2>&1 || true
}

while true; do
  now=$(date +%s)
  elapsed=$((now - t0))
  if [ "$elapsed" -ge "$MAX_S" ]; then
    estop "hard deadline ${MAX_S}s"
    exit 0
  fi
  if [ -f "$RUN/supervisor.done" ]; then
    echo "$(date -Is) supervisor.done seen, watchdog exit" | tee -a "$LOG"
    exit 0
  fi
  # supervisor process gone without writing supervisor.done -> it crashed/was killed
  if [ -f "$PIDF" ]; then
    sp=$(cat "$PIDF" 2>/dev/null || echo "")
    if [ -n "$sp" ] && ! kill -0 "$sp" 2>/dev/null; then
      sleep 2
      if [ ! -f "$RUN/supervisor.done" ]; then
        estop "supervisor pid $sp died without supervisor.done"
        exit 1
      fi
    fi
  fi
  if [ -f "$HB" ]; then
    hb=$(stat -c %Y "$HB" 2>/dev/null || echo 0)
    # Ignore a leftover heartbeat from a previous supervisor (older than this watchdog).
    if [ "$hb" -lt "$t0" ]; then
      if [ "$elapsed" -gt 60 ]; then
        estop "no fresh supervisor heartbeat after 60s"
        exit 1
      fi
    else
      age=$((now - hb))
      if [ "$age" -lt 0 ] || [ "$age" -gt 3600 ]; then
        echo "$(date -Is) WARN: ignoring absurd heartbeat age ${age}s (using file mtime already)" | tee -a "$LOG"
      elif [ "$age" -gt "$STALE_S" ]; then
        estop "supervisor heartbeat stale ${age}s"
        exit 1
      fi
    fi
  else
    if [ "$elapsed" -gt 60 ]; then
      estop "no supervisor heartbeat after 60s"
      exit 1
    fi
  fi
  sleep 2
done
