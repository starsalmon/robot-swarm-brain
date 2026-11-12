#!/bin/bash
# Start (or restart) the Jazz camera pitch stabilizer on the NUC.
# Preview: http://192.168.3.172:8181/stream  and  /jpg  and  /health
# Tracker hook: JAZZ_CAM_STAB_URL=http://127.0.0.1:8181/stream
set -euo pipefail
NAME=jazz-cam-stab
IMAGE=${JAZZ_STAB_IMAGE:-rover-detector:jazzy}
PORT=${JAZZ_STAB_PORT:-8181}
BRAIN=${JAZZ_STAB_BRAIN:-/data/swarm/robot-swarm-brain}
CALIB_DIR=${JAZZ_STAB_CALIB_DIR:-/home/cain/rover-handoff/tracker/calib}
CAM_URL=${JAZZ_CAM_URL:-http://192.168.3.146:81/stream}

docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" --network host --restart unless-stopped \
  -e ROS_DOMAIN_ID=0 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fleet/fastdds_udp.xml \
  -e JAZZ_CAM_URL="$CAM_URL" \
  -e JAZZ_CAM_JPG="${JAZZ_CAM_JPG:-http://192.168.3.146:81/jpg}" \
  -e JAZZ_STAB_PORT="$PORT" \
  -e JAZZ_CALIB=/calib/cam146.json \
  -e JAZZ_STAB_TRIM_DEG="${JAZZ_STAB_TRIM_DEG:-0}" \
  -e JAZZ_STAB_PITCH_GAIN="${JAZZ_STAB_PITCH_GAIN:-1}" \
  -e PYTHONUNBUFFERED=1 \
  -v "$BRAIN":/opt/fleet:ro \
  -v "$CALIB_DIR":/calib:ro \
  "$IMAGE" \
  bash -lc 'source /opt/ros/jazzy/setup.bash && exec python3 -u /opt/fleet/jazz_cam_stab/jazz_cam_stab_node.py'

echo "Started $NAME. Preview:"
echo "  http://192.168.3.172:${PORT}/stream"
echo "  http://192.168.3.172:${PORT}/jpg"
echo "  http://127.0.0.1:${PORT}/health"
echo "Stop: docker rm -f $NAME"
echo "Logs: docker logs -f $NAME"
