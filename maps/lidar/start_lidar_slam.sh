#!/usr/bin/env bash
# Start STL-19P mapping on the NUC. Does not publish cmd_vel or touch /fleet/allow_drive.
# TF: base_link -> lidar_link
#   xyz: 0.00, 0.00, 0.12 m  (z is an unmeasured placeholder near camera-deck height)
#   rpy_deg: 0, 0, -90
#   Reason: lidar triangle (scan 0 rad / +X) is mounted pointing starboard (base_link -Y).
#   Confirmed 2026-11-12 from live /rover/scan vs ToF: angle 0 matches /rover/tof/right (~0.82 vs 0.86 m);
#   +90 deg CCW matches /rover/tof/front (sector min ~1.24 vs 1.33 m).
# Scans are restamped onto the brain clock as /rover/scan_sync because the Pro S3
# uses its own uptime clock, which does not line up with odom->base_link.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.12 \
  --roll 0 --pitch 0 --yaw -1.5707963267948966 \
  --frame-id base_link --child-frame-id lidar_link &
python3 /maps/lidar/scan_stamp.py &
exec ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/maps/lidar/mapper_params_online_async.yaml \
  use_sim_time:=false
