#!/bin/bash
R=/data/swarm/robot-swarm-brain/maps/lidar/house_run
TZ=Australia/Melbourne date +%T; echo "babysit=$(cat $R/babysit.state) sup=$(pgrep -f house_supervisor.py | wc -l)"
tail -4 $R/babysit.log
grep -e "STOP requested" -e "snapshot house" -e WEDGE $R/supervisor.log | tail -3 | cut -c1-160
python3 - <<'P'
import json,time
R="/data/swarm/robot-swarm-brain/maps/lidar/house_run"
d=json.load(open(R+"/live.json")); f=json.load(open(R+"/frontier.json"))
t=d.get("tof",{}); p=d.get("pose",{}); m=d.get("map",{})
print(f"age {time.time()-d['wall_t']:.0f}s go={d['go']} ses={d['session']} allow={d['allow']} batt={d['batt']:.2f} v={d['beh_v']:.2f} w={d['beh_w']:.2f} pose=({p.get('x',0):.2f},{p.get('y',0):.2f},{p.get('yaw_deg',0):.0f}) map={m.get('size_m')} occ={m.get('occ')} free={m.get('free')}")
print(f"tof front={t.get('front')} fl={t.get('fl')} fr={t.get('fr')} L={t.get('left')} R={t.get('right')} | why={d['why'][:150]}")
print(f"frontier state={f['state']} clusters={f.get('n_clusters')} cells={f.get('n_frontier_cells')} goal={f.get('goal')} path={f.get('path_m')} goals={f.get('n_goals')} black={len(f.get('blacklist',[]))} visited={f.get('visited_m2')}")
P
