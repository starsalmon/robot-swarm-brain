#!/bin/bash
# House mapping babysitter: restarts continue-mode runs after supervisor stops; halts on battery/allow/map-match/repeat failures.
L=/data/swarm/robot-swarm-brain/maps/lidar; R=$L/house_run; LOG=$R/babysit.log; SLAM=rover-lidar-slam
MAXREP=${MAXREP:-2}; VMIN=${VMIN:-6.6}
say(){ echo "$(TZ=Australia/Melbourne date '+%H:%M:%S') $*" | tee -a $LOG; }
estop(){ touch $R/stop.flag; docker exec $SLAM bash -lc "source /opt/ros/jazzy/setup.bash && python3 /maps/lidar/house_run/pub_cmd.py estop" >/dev/null 2>&1; }
jget(){ python3 -c "import json,sys;d=json.load(open('$R/live.json'));print(d.get('$1'))" 2>/dev/null; }
start(){
  mm=$(docker exec $SLAM bash -lc "source /opt/ros/jazzy/setup.bash && python3 /maps/lidar/house_run/map_match_check.py 10 0.55" 2>/dev/null | tail -1)
  say "pre-restart map match: $(echo "$mm" | cut -c1-160)"
  echo "$mm" | grep -q '"ok": true' || { say "HALT: map match failed before restart (not restarting, would smear)"; echo HALT_MAPMATCH > $R/babysit.state; exit 3; }
  rm -f $R/stop.flag
  say "START run #$n (MAP_MODE=continue)"
  (cd $R && MAP_MODE=continue MIN_START_V=${MIN_START_V:-6.8} HOUSE_DURATION_S=${HOUSE_DURATION_S:-5400} timeout 150 bash $R/go_house_explore.sh 2>&1 | tail -6 | tee -a $LOG)
  sleep 20
}
echo RUNNING > $R/babysit.state; echo $$ > $R/babysit.pid
n=${N0:-0}; last=""; rep=0; lowc=0
say "babysit up pid $$ (MAXREP=$MAXREP VMIN=$VMIN)"
if ! pgrep -f house_supervisor.py >/dev/null; then n=$((n+1)); start; fi
while true; do
  sleep 10
  [ -f $R/babysit.quit ] && { say "quit file: exiting (run left as is)"; rm -f $R/babysit.quit; echo QUIT > $R/babysit.state; exit 0; }
  if pgrep -f house_supervisor.py >/dev/null; then
    a=$(jget allow); b=$(jget batt)
    if [ "$a" = "False" ]; then say "HALT: allow_drive off"; estop; echo HALT_ALLOW > $R/babysit.state; exit 4; fi
    if [ -n "$b" ] && [ "$b" != "None" ] && python3 -c "import sys;sys.exit(0 if float('$b')<$VMIN else 1)"; then lowc=$((lowc+1)); else lowc=0; fi
    if [ $lowc -ge 4 ]; then say "HALT: battery $b V < $VMIN for 40 s"; estop; echo HALT_BATT > $R/babysit.state; exit 5; fi
    continue
  fi
  reason=$(grep "STOP requested" $R/supervisor.log | tail -1 | sed 's/.*STOP requested: //')
  say "supervisor exited: $reason"
  case "$reason" in
    *"house covered"*) say "DONE: pass complete"; echo DONE > $R/babysit.state; exit 0;;
    *battery*) say "HALT: battery"; echo HALT_BATT > $R/babysit.state; exit 5;;
    stop.flag) say "HALT: stop.flag set externally"; echo HALT_FLAG > $R/babysit.state; exit 6;;
  esac
  key=$(echo "$reason" | cut -c1-20)
  if [ "$key" = "$last" ]; then rep=$((rep+1)); else rep=1; last="$key"; fi
  if [ $rep -gt $MAXREP ]; then say "HALT: '$reason' $rep times in a row; needs a fix"; echo HALT_REPEAT > $R/babysit.state; exit 7; fi
  b=$(jget batt); python3 -c "import sys;sys.exit(0 if float('$b')<${RESTART_VMIN:-6.8} else 1)" 2>/dev/null && { say "HALT: battery $b < 6.8 V start minimum"; echo HALT_BATT > $R/babysit.state; exit 5; }
  sleep 5; n=$((n+1)); start
done
