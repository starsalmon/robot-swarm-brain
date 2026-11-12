# Jazz camera pitch stabilizer (NUC, software gimbal)

Keeps the Pi Zero 2 W as a stream-only box (`CAM_ROT=0`,
`http://192.168.3.146:81/stream`). The NUC subscribes to Jazz IMU pitch
and warps each frame so the view stays level as the body nods.

## How pitch is used

- Primary: `/jazz/pitch` (`std_msgs/Float32`, **radians**, same value as
  the balance loop; positive = nose-up / lying back).
- Fallback: pitch extracted from `/jazz/imu/data` orientation if
  `/jazz/pitch` is stale (>0.5 s).
- If `/jazz/pitch` goes quiet, the preview **HOLD**s the last good pitch
  for `JAZZ_STAB_HOLD_S` (default 2 s) after the 0.5 s live window, then
  passes the raw frame through (**STALE**). Overlay and `/health` distinguish
  live, HOLD, and STALE passthrough.
- Warp: plane-at-infinity homography `H = K Rx(gain·pitch + trim) K⁻¹`
  using `cam146` intrinsics. Default `gain=+1` undoes a nose-up nod
  (OpenCV Rx). If the horizon moves the wrong way:
  `JAZZ_STAB_PITCH_GAIN=-1`. Trim: `JAZZ_STAB_TRIM_DEG`.

## Start / view

```bash
/data/swarm/robot-swarm-brain/jazz_cam_stab/go_jazz_cam_stab.sh
```

- Preview stream: http://192.168.3.172:8181/stream
- Still: http://192.168.3.172:8181/jpg  (raw unstabilized: `/raw.jpg`)
- Status: http://127.0.0.1:8181/health
- Logs: `docker logs -f jazz-cam-stab`
- Stop: `docker rm -f jazz-cam-stab`

## Hook for the AprilTag tracker

The stabilizer is a library hook (`stabilize_bgr` / `JazzPitchStabilizer.warp`)
and also an HTTP drop-in for cam146. In `rover-handoff/tracker`, set:

```bash
JAZZ_CAM_STAB_URL=http://127.0.0.1:8181/stream
```

`tracker.py` / `tracker_v2.py` use that URL for cam146 when the env is set;
otherwise they keep the raw Pi stream.

## Limitations

- Same 1024×768 output with **black borders**; |pitch| eats corners/FOV.
  No auto-zoom, so tag scale stays consistent.
- Horizon is IMU-stabilized, not vision-estimated; mount tilt needs trim.
- Added latency: about one camera frame plus a few ms of warp, plus HTTP.
- Plane-at-infinity warp: nearby objects still show parallax.
- If pitch is older than 0.5 s the preview HOLDs the last warp for 2 s
  (HUD: HOLD, still shows age); after that it passes the raw frame
  (HUD: STALE passthrough).
