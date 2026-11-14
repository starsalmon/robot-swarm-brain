#!/bin/bash
R=/data/swarm/robot-swarm-brain/maps/lidar/house_run; O=$R/track_20261114_run2.csv
while [ ! -f $R/track_logger.stop ]; do
python3 - >> $O 2>/dev/null <<'P'
import json,time
d=json.load(open("/data/swarm/robot-swarm-brain/maps/lidar/house_run/live.json"))
p=d.get("pose") or {}; a=d.get("ages") or {}; m=d.get("map") or {}
print("%.1f,%.1f,%s,%s,%s,%s,%s,%s,%.3f,%.3f,%s,%s,%s,%s" % (time.time(), d["wall_t"], p.get("x"), p.get("y"), p.get("yaw_deg"), d.get("batt"), d.get("go"), d.get("state"), d.get("beh_v") or 0, d.get("beh_w") or 0, a.get("front"), a.get("scan"), m.get("occ"), m.get("free")))
P
sleep 3; done
