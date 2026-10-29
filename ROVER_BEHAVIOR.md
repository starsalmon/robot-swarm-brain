# Rover behaviour — explore (current)

Explore runs in the `rover-brain` container on the NUC brain (`/data/swarm/robot-swarm-brain`: `rover_brain.py`, `explore_controller.py`, `rover_proximity.py`, `rover_wheel_odom.py`). The drive board executes `/cmd_vel` and enforces local safety. All speeds below are **usable %** (cmd value × 100).

## Starting and stopping

| Input | Effect |
|-------|--------|
| `allow_drive` (MQTT `lab/robots/allow_drive/set` ON/OFF → `/fleet/allow_drive`) | Master gate. OFF stops an active session and makes Go taps ignored. |
| **Go tap** (`/rover/button` true) | Toggles the session (START / STOP). |
| **E-stop long** (`/rover/button_event` 3) | Stops the session. E-stop hold also cuts motors on the ESP. |
| Mode menu (`/rover/drive_mode`) | Explore or Wall; changing mode stops the session. |

Session state is published on `/rover/session`; the drive board only drives while it is true and `/cmd_vel` is fresh (400 ms timeout).

## Speed limits (docker-compose env)

| Env | Value | Meaning |
|-----|-------|---------|
| `ROVER_AUTO_MAX_LINEAR` | 0.25 | Forward cap, 25% usable |
| `ROVER_AUTO_BURST_MAX` | 0.25 | Burst cap (same as forward) |
| `ROVER_AUTO_REVERSE_POWER` | 0.10 | Reverse, 10% usable |
| `ROVER_AUTO_MAX_ANGULAR` | 0.18 | Turn cap |

Slew: linear +0.32/s (brake 0.18/s), angular 0.22/s (brake 0.14/s). In cruise modes angular is also clamped to `0.08 + |lin| × 1.35` so slow turns stay arcs, not spins.

## Drive modes

While no obstacle needs handling the controller rolls between **cruise** (≈70%, 4–8 s), **wander** (≈30%, 3–5 s) and a rare **burst** (1% chance, 0.8–1.2 s). After an escape it holds straight cruise for 5–9 s.

## Picture (`ROVER_PICTURE=1`)

Inputs: the 8 L8 columns plus front/front_left/front_right (`/rover/tof/*`, Pro S3) and the left/right side ToF.

- **Floor baseline:** L8 returns near the floor distance are ignored as floor. The baseline is seeded at **0.18 m** and locks within 0.15–0.24 m. A return 0.03 m inside the baseline counts as an obstacle.
- **Stale:** no fresh front data for 3.2 s or side data for 2.5 s → stop (`picture stale`).
- **Side veto:** never turn toward a side whose ToF reads under 0.20 m.

### Steer-around (`ROVER_PICTURE_STEER_AROUND`, default on)

- **Far** (nose clear past `ROVER_PICTURE_STEER_FAR_M` = **1.10 m**): cruise, with a held heading commit (2.5 s / 0.70 m) toward the open side.
- **Near / close** (closest front return under 1.10 m): keep rolling forward. Speed drops linearly toward `cruise × (1 − ROVER_PICTURE_STEER_SLOW)` (0.80 → down to 20% of cruise, never below 6% usable). Turn rate grows from 0.04 to 0.18 as the distance shrinks toward the box distance.
- **Gap side:** compares left vs right openness (L8 half + that hip's side ToF × 1.5). Switches only when the other side is better by `ROVER_PICTURE_GAP_HYST_M` = **0.15 m** and the current side has been held ≥ 0.8 s.
- **Hip close, nose clear:** keep going at 70% of cruise and peel off the wall.
- **Boxed:** both L8 halves under `ROVER_PICTURE_BOX_M` = **0.20 m** → reverse at 8% usable while turning toward the open side, until either half clears `ROVER_PICTURE_BOX_EXIT_M` = **0.30 m**.

## Bump / stall backup

- **Bump:** IMU jerk on the drive board → `/rover/bump`.
- **Stall:** drive board (IMU + wheel ticks, motors cut ~2.5 s, `/rover/stall`) and brain wheel odometry (straight cruise, ticks stopped; grace 900 ms, cooldown 5 s).
- Either triggers an **escape**: reverse 1.0 s at 10% usable, then an IMU-limited arc of 20–35° toward the gap, then straight cruise. Max 3 escapes per minute, then a 10 s pause.
- **Recover:** if the picture says the nose is blocked while the wheels aren't moving (< 0.06 m travel), reverse, scan and drive out toward the best opening.

## Drive-board safety (wins over the brain)

1. E-stop / battery critical (6.0 V)
2. Stall lockout (~2.5 s)
3. Heading hold trim (MPU6050)
4. Brain `/cmd_vel`
5. Cmd timeout 400 ms → stop

## Logs

`docker logs rover-brain` shows `picture far|near|close|boxed|stale`, `escape (...)`, `wheel stall`, `session START/STOP`, `allow_drive=`.
