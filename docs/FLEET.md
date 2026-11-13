# Fleet map (ROS 2 brain)

Source of truth for this file is **this repo’s code** (`docker-compose.yml`, brain nodes, `fleet_ha_bridge.py`). Firmware hostnames and pins live in each bot’s firmware repo.

## Brain host

| Item | Value (from code) |
|------|-------------------|
| Host | NUC Ubuntu `brain` |
| LAN IP | `192.168.3.172` (`rover_speech.py` `SPEECH_HOST_IP`, compose comments) |
| Checkout on NUC | `/data/swarm/robot-swarm-brain` |
| Stack | `docker compose up -d` in this directory (`docker-compose.yml`) |
| ROS 2 | Jazzy, `ROS_DOMAIN_ID=0` |
| micro-ROS agent | UDP **8888** (`microros-agent` service) |
| DDS | `fastdds_udp.xml` on all brain containers |

## Fleet IDs and namespaces

From `explore_then_chase.py` (`fleet_ids`) and `docker-compose.yml` (`FLEET_IR_ID` where set):

| Bot | Display name | ROS namespace | Fleet IR ID | Always-on brain container |
|-----|----------------|---------------|-------------|---------------------------|
| Rover | Rover | `/rover` (drive topics often unprefixed, e.g. `/imu/data`, `/cmd_vel`) | 0 | `rover-brain` + `rover-behaviour` |
| Roby | Roby (c3-mini) | `/bot1` | 1 | `fleet-brain` (`BOT_NS=bot1`) |
| Tracker | Tracker | `/bot2` | 2 | `tracked-brain` |
| Beeper | Beeper / Beeper Baby | `/bot3` | 3 (`FLEET_IR_ID=3` on `beeper-brain`) | `beeper-brain` (compose profile `beeper`) |
| Jazz | Jazz | *(not wired in this repo)* | *(not assigned)* | — |

`fleet_ha_bridge.py` HA slugs: `rover`, `roby`, `tracked` (no Beeper entry yet).

## Rover

**Firmware repo:** `lego-rover-esp32` (not vendored here).

### Boards

| Board | Role | Hostname / IP in brain code |
|-------|------|-----------------------------|
| LilyGO T-Display S3 | Drive: motors, IMU, wheel ticks, display, speaker, Go button | `rover-esp` / `rover-esp.local`; bench ping target **`192.168.3.71`** (`tools/rover-powercycle/verify_up.py`) |
| Unexpected Maker Pro S3 | ToF (VL53L8 + side VL53L0X), IR RX/TX, optional sensors | `rover-tof` (name in `README.md`; no fixed IP in this repo) |
| DFRobot ESP32-S3 AI Camera | MJPEG stream, whistle/speech, light sensor | `rover-cam.local` (`rover_camera_bridge.py` default); **`192.168.3.130`** (`ROVER_CAM_IP` in `docker-compose.yml`) |

### ROS topics the brain uses

**`rover-brain`** (`rover_brain.py`) — explore suggestion on `/rover/explore/cmd_vel`, session `/rover/explore/session` when `ROVER_EXPLORE_ARBITER=1`; subscribes including:

- `/cmd_vel`, `/cmd_vel_nav` (nav profile), `/fleet/allow_drive`, `/rover/button`, `/rover/button_event`, `/rover/drive_mode`
- `/imu/data`, `/rover/bump`, `/rover/stall`, `/rover/wheel/left_ticks`, `/rover/wheel/right_ticks`
- `/rover/sonar/range`, `/rover/sonar/pan_deg`, `/rover/sonar/avoid_active`
- `/rover/tof/front`, `/rover/tof/front_left`, `/rover/tof/front_right`, `/rover/tof/left`, `/rover/tof/right`, `/rover/tof/l8_cols`
- `/rover/ir/hits`, `/rover/ir/peer`
- Publishes `/rover/heartbeat`, `/rover/sonar/cal_sweep`

**`rover-behaviour`** (`rover_behaviour.py`) — sole publisher of **`/cmd_vel`** and **`/rover/session`** when active; subscribes `/fleet/allow_drive`, `/rover/camera/image_raw/compressed`, ToF topics, `/rover/whistle`, `/rover/button`, wheel odom; publishes `/rover/behaviour/{status,cmd,explore_active}`, `/rover/speaker/code`, `/rover/servo/angle`, `/rover/apriltags`.

**Optional ingest (profile `camera`):** `rover-camera` → `/rover/camera/image_raw/compressed`, `/rover/camera/camera_info`; `rover-light` → `/rover/light/lux`, `/rover/light/night`.

**SLAM profile:** `rover-odom` fuses `/rover/odom/wheel` + `/rover/imu/stamped` (`config/ekf_rover.yaml`); `rover-depth` uses camera + `/rover/tof/front`.

**`fleet-ha`:** online liveness uses `/imu/data` + `/rover/sonar/range`; battery `/rover/battery/voltage`; session `/rover/session`.

## Roby (c3-mini, bot 1)

| Item | Value |
|------|--------|
| Namespace | `/bot1` |
| Fleet IR ID | 1 |
| Brain | `fleet-brain` — `explore_then_chase.py` with `BOT_NS=bot1` |

**Topics used by `explore_then_chase.py`:** `/{ns}/cmd_vel` (pub), `/{ns}/sonar/range`, `/{ns}/tof/rear`, `/{ns}/imu` (default `IMU_TOPIC=imu`), `/{ns}/ir/detected`, `/{ns}/ir/peer`, `/{ns}/sonar/pan_deg`, `/{ns}/side/left`, `/{ns}/side/right`; subscribes `/fleet/allow_drive` (read-only gate).

**`fleet-ha`:** `/bot1/imu`, `/bot1/sonar/range`, `/bot1/battery/voltage`, `/bot1/cmd_vel`.

Hostname for the ESP is not defined in this repo (set in c3-mini firmware `BOT_NAMESPACE`).

## Tracker (bot 2)

| Item | Value |
|------|--------|
| Namespace | `/bot2` |
| Fleet IR ID | 2 |
| Brain | `tracked-brain` — `explore_then_chase.py`, `BOT_NS=bot2`, `REQUIRE_IMU=0` in compose |

Same topic pattern as Roby under `/bot2`. Extra topics referenced in bench tools: `/bot2/power/battery_v`, `/bot2/power/solar_v`, `/bot2/solar/servo_now`, `/bot2/solar/servo_deg` (`tools/tracker-aim/`).

**`fleet-ha`:** `/bot2/imu`, `/bot2/sonar/range`, `/bot2/battery/voltage`.

## Beeper (bot 3)

| Item | Value |
|------|--------|
| Namespace | `/bot3` |
| Fleet IR ID | 3 (`FLEET_IR_ID` in `beeper-brain` service) |
| Brain | `beeper-brain` (profile `beeper`) — `explore_then_chase.py` with `PERSON_FOLLOW=1`, `IMU_TOPIC=imu/data` |
| Camera (detector pull) | `BEEPER_CAM_IP` default **`beeper.local`** MJPEG `:81/stream` (`docker-compose.yml` `DET_CAMS`) |

Obstacle path: compose documents forward gate on `/bot3/sonar/range` plus firmware ToF gate (`docker-compose.yml` comment).

## Fleet-wide ROS topics

| Topic | Publisher | Consumers (examples) |
|-------|-----------|----------------------|
| `/fleet/allow_drive` | `fleet_ha_bridge.py` (from MQTT `lab/robots/allow_drive/set`) | `rover_behaviour.py`, `rover_brain.py`, `explore_then_chase.py` — **subscribe only** in wander/explore nodes |

Do not add new publishers of `/fleet/allow_drive`; only `fleet-ha` bridges HA/MQTT.

## Detector (optional profile `detector`)

HTTP **`127.0.0.1:8765`** (`DET_URL`). Pull cams: `DET_CAMS` in compose (Beeper stream by default). Rover AI cam can POST frames (`DET_PULL` / client behaviour in `det_client.py`).

## Related docs

| Doc | Purpose |
|-----|---------|
| [ROVER_BEHAVIOR.md](../ROVER_BEHAVIOR.md) | Rover explore / follow |
| [ROADMAP.md](../ROADMAP.md) | Jazz, dock, unset items |
| [WIRING.md](../WIRING.md) | Brain env vars |
| [README.md](../README.md) | Quick start |
