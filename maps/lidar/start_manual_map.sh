#!/bin/bash
# Start a FRESH SLAM-only session for manual (PS4) mapping. Never publishes cmd_vel, never starts explore,
# never writes /fleet/allow_drive. Backs up the current in-memory map first.
set -u
L=/data/swarm/robot-swarm-brain/maps/lidar
LOG=$L/manual_map.log
LIVE=${LIVE:-house_manual_20261120_live}
echo "=== $(date) start_manual_map" >> $LOG
bash $L/save_manual_map.sh house_prev_20261120_pre_manual >> $LOG 2>&1
docker restart rover-lidar-slam >> $LOG 2>&1
sleep 15
# live PNG every 20 s (dies with the container)
docker exec -d rover-lidar-slam bash -lc "source /opt/ros/jazzy/setup.bash; while true; do timeout 15 python3 /maps/lidar/render_map_png.py /maps/lidar/$LIVE >/dev/null 2>&1; sleep 20; done"
echo "=== $(date) slam restarted, live png $L/$LIVE.png" >> $LOG