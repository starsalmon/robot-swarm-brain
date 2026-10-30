# Fleet roadmap

Written 30 Oct 2026 from Cain's brief. This file is the plan. It is not hardware, firmware, or a wiring doc.

Anything under **Open** was not chosen. Do not fill those gaps from a datasheet, a similar bot, or a guess. Fleet IDs, ROS namespaces, hostnames, pins, and brain containers for Beeper Baby and Jazz are unset.

## Already running

These are in the repos today. They are not part of the new-bot brief.

| Bot | Fleet ID | IR ID | IMU and IR in the wiring docs |
|-----|----------|-------|-------------------------------|
| Rover | 0 | 0 | MPU6050 on the drive board. IR TX on drive GPIO 21 (off unless enabled) and on the Pro S3 GPIO 21. IR RX on the Pro S3 GPIO 14. |
| Roby (c3-mini) | 1 | 1 | MPU6050. IR TX GPIO 20. IR RX GPIO 7 (TSOP). |
| Tracker | 2 | 2 | MPU6050. IR TX GPIO 34. IR RX GPIO 3. Separate side proximity sensors on GPIO 5 and 6. |

IR identity that already exists: `fleet_ir_proto.h` (copied in the rover, mini, and tracked firmware). 38 kHz frames. The receiver counts bursts and gets a peer id. Named constants are rover 0, mini 1, tracked 2. `FLEET_IR_MAX_ID` is 7. No other id is assigned.

**Beaper = Beeper Baby = Beeper** (Cain, 2026-10-30). Fleet ID 3 (`/bot3`), repo `starsalmon/beeper` with WIRING.md. Older notes said ID 4; awaiting Cain's confirmation.

Low battery on the bots that have it is a local cutoff or a published voltage. Nothing in the repos negotiates a charger.

## Rover — camera mapping (next, already prepared)

DFRobot ESP32-S3 AI Camera V1.1 (OV3660 wide-angle, IR LEDs) as a third Rover ESP node, then RTAB-Map, Nav2, explore_lite.

Prepared, not yet in use: firmware env `aicam_rover_cam` / `aicam_rover_cam_ota` (lego-rover-esp32, host `rover-cam`; MJPEG `:81/stream`, `/capture`, `/control`, `/status`, `/ir`), and the brain ingest `rover_camera_bridge.py` (compose service `rover-camera`, profile `camera`, off by default) publishing `/rover/camera/image_raw/compressed` + `/rover/camera/camera_info` (placeholder calibration in `config/rover_cam_info.yaml`).

Still to do: mount and power the camera, USB first flash, checkerboard calibration (`camera_calibration`) at the SLAM frame size, fixed camera TF (`base_link` → `camera_link`), wheel-tick + IMU odometry fused with `robot_localization` EKF, then RTAB-Map (mono RGB + odometry, with the L8/side ToF as the range source, since a single camera gives no depth), then Nav2 + explore_lite.

## Beeper Baby

Small robot. First bot meant to have real high-precision wheel sensors. Otherwise Cain does not see a standout feature beyond the chassis.

### Stated

| Item | What Cain said |
|------|----------------|
| Name | Beeper Baby |
| Chassis | Modified beeper-bot chassis. Dovetail features are why this chassis is interesting. No drawing or vendor part is in the repos. |
| Front sensors | A ToF and an IR camera, both up front. Models not named. The IR camera is a camera. It is separate from the fleet IR LED / receiver used to recognise other bots. |
| Drive | N20 gear motors with hall-effect wheel sensors. Motor count, gear ratio, counts per revolution, and wheel size were not given. |
| Board | FireBeetle 2. Which FireBeetle 2 board was not given. |
| Motor driver | One DRV8835. |
| Power | 1200 mAh LiPo. A boost circuit is likely ("probably"), and is not decided. Cell count was not given. |
| Dock | Candidate chassis for prototyping the shared charging dock. Not committed ("I might"). |

### Open

- Whether this is Beaper (fleet ID 4) or a different robot.
- Fleet ID, IR ID, ROS namespace, hostname, brain container.
- FireBeetle 2 variant.
- LiPo cell count, and whether the pack uses a boost.
- ToF model. IR-camera model.
- How many N20s, gear ratio, hall counts per revolution, wheel diameter, track width.
- IMU chip. IR LED and receiver parts and placement. Every bot is supposed to have both (see below). Pins were not given.
- Dock geometry, even for the prototype.

## Jazz (Smooth Jazz)

On legs. Cain called this the most ambitious bot so far.

### Stated

| Item | What Cain said |
|------|----------------|
| Name | Jazz, as in Smooth Jazz |
| Power | 3S LiPo. Capacity was not given. |
| Drive | 2208 3-phase BLDC, 12 V, 1800 RPM. Direct drive: the motors are in the wheels. How many motors, and how many driver boards, was not given. |
| Motor controller | SimpleFOC Mini brushless controller. |
| Legs | Two servo-driven extendable legs. Purpose: lean into corners, and get over obstacles. |
| Robot controller | Pro S3. It does motor control and the rest of the robot. |
| Camera computer | Raspberry Pi Zero 2 W and Camera Module 3 Wide. Its only job is streaming that camera to the brain (the NUC). High resolution is not available on an ESP32, which is why the Pi is there. The Pi is not a second brain and does not run behaviour. |
| Stream | To the brain. Protocol, resolution, and network path were not chosen. |

### Bench motor test (30 Oct 2026)

One motor and one SimpleFOC Mini, on the Pro S3 that enumerated as USB `68:EE:8F:D8:F5:88`. Pins and the 1.0 V cap are for that test, written in `jazz/WIRING.md`. Not a final robot pinout. Pole pairs are still unknown (sketch tries 7). Second motor, legs, IMU, and IR are not in this test.

### Open

- Can it jump? Cain asked. The brief does not answer it. Do not treat jumping as a requirement or as impossible.
- Fleet ID, IR ID, ROS namespace, hostname, brain container.
- How many 2208s and how many SimpleFOC Mini boards.
- Servo model, leg linkage, and how "extendable" is built.
- IMU chip. IR LED and receiver. Pins.
- How the Pi stream reaches the NUC.
- What "the rest" of the Pro S3's job is, beyond motor control and the fleet-wide IMU and IR.

## Every bot

Applies to Rover, Roby, Tracker, Beaper, Beeper Baby, and Jazz.

### Stated

- Each bot will have an IMU and IR transmit and receive.
- Rover, Roby, and Tracker already have both in their wiring docs (table above). Beaper has no wiring doc. The two new bots have no chip or pin chosen.
- The same charging dock, eventually, for all of them.
- When a bot is low on charge it negotiates (with the other bots). No rules for that negotiation were given: who goes first, what "low" means, or what happens if two arrive together.
- Bots recognise each other by IR. Rover, Roby, and Tracker already do this with the fleet IR ids above. New bots have no id yet. The current frame format only goes up to id 7.

Dock mechanics under consideration, not chosen: magnets and springs. Beeper Baby is the chassis Cain might use to try that. No magnet count, spring, contact, or alignment is specified.

### Open

- Order of any IMU or IR retrofit. Nothing in the brief says the current three are missing parts.
- Dock electrical contacts, voltage, and which packs it must accept (the stated packs are a 1200 mAh LiPo of unknown cell count, Rover's 2S, and Jazz's 3S).
- The negotiation itself.
- Which IR ids the new bots use, and whether Beaper takes one.

When this is built, negotiation and recognition belong on the bot and the always-on brain, triggered by battery state and IR. Same rule as the rest of the fleet: not a script Cain has to start.
