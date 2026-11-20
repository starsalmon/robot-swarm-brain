#!/bin/bash
# Snapshot the live slam_toolbox map (manual PS4 mapping). Usage: save_manual_map.sh [NAME]
# Writes /data/swarm/robot-swarm-brain/maps/lidar/NAME.{pgm,yaml,posegraph,data,png,txt}. No drive commands.
NAME=${1:-house_manual_20261120_$(TZ=Australia/Melbourne date +%H%M)}
ros() { docker exec rover-lidar-slam bash -lc "source /opt/ros/jazzy/setup.bash && $1"; }
ros "ros2 run nav2_map_server map_saver_cli -f /maps/lidar/$NAME --ros-args -p save_map_timeout:=10.0" >/dev/null 2>&1 && echo "saved $NAME.pgm/.yaml" || echo "map_saver failed"
ros "timeout 30 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '/maps/lidar/$NAME'}\"" 2>&1 | grep -q "result=0\|result: 0" && echo "saved $NAME.posegraph/.data" || echo "serialize: check $NAME.posegraph"
ros "timeout 20 python3 /maps/lidar/render_map_png.py /maps/lidar/$NAME" >/dev/null 2>&1 && echo "saved $NAME.png"