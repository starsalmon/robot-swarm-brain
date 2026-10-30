# Rover behaviour (current)

`rover-behaviour` (`rover_behaviour.py`, NUC brain, image `rover-follow:jazzy`, GPU) is the only publisher of `/cmd_vel` and `/rover/session`. `rover-brain` still runs, but its explore output goes to `/rover/explore/cmd_vel` and nothing drives from it. All speeds are **usable power** (cmd value x 100 = % usable, cap 25%).

## Starting and stopping

| Input | Effect |
|-------|--------|
| `allow_drive` (MQTT `lab/robots/allow_drive/set` -> `/fleet/allow_drive`) | Master gate. OFF stops the session; Go taps are ignored. |
| **Go tap** (`/rover/button` true) | Toggles the session. |
| **E-stop long** (`/rover/button_event` 3) | Stops the session (ESP also cuts motors). |

The drive board only drives while `/rover/session` is true and `/cmd_vel` is fresh (400 ms).

## Priority (highest first)

1. **ORANGE** - a real orange fruit is confirmed by a camera:
   - Rover camera: YOLO11s COCO class 49 (the fruit, conf >= `ORANGE_CONF` 0.40), run in the same pass as person detection.
   - LifeCam (`room-eye`): the same detector at 1280 px. This path is used only while room-eye has a **confirmed** Rover track (`/room/overhead` field 8 = 1). A blob becomes Rover only after it is seen moving (MOG2 background subtraction) while `/cmd_vel` commands motion. Static tubs, boxes and a parked Rover are never confirmed. A track that stays still for 2.5 s while `/cmd_vel` says drive is dropped (`rover_track.py`).
   - Either one needs the orange in **4 of the last 6 frames** at a consistent spot and roundish shape (`orange_vote.py`). Orange-coloured tubs, boxes and toys are not the class and are ignored.
   - It drives at 16% usable, dropping to 11% then 7% as it gets close, and steers toward the orange. At 5 cm or less it stops and plays the tune.
2. **FOLLOW** - a person (COCO class 0, ByteTrack) is held for `BEHAVIOUR_ENTER_S` 0.5 s:
   - Uses the proven `person_follow.Follower` at up to `FOLLOW_VMAX` 22% usable (explore cruise is 18%), with turns up to 10%.
   - Keeps `FOLLOW_TARGET_M` 1.0 m: it slows from 0.50 m and stops at 0.30 m.
   - Returns to explore after the person has been lost for 5 s.
   - While a person is only a candidate, explore continues at 60% speed rather than stopping.
3. **EXPLORE** - `rover_explore.py`, continuous steering:
   - **Inputs:** the 8 L8 columns (column 0 = left) plus the front, front_left, front_right, left and right ToF. If none is fresh it stops.
   - **Speed:** `EXPLORE_CRUISE` 18% in the open. Speed scales down with the clearance ahead from 0.9 m, to at least 8%.
   - **Steering:** it is pulled toward the most open direction and pushed off side walls closer than 0.30 m, at up to `EXPLORE_WMAX` 12%. When heading at a wall it commits early to one turn direction. A slow random wander makes it curve in open space instead of driving dead straight.
   - **Blocked** (under 0.25 m ahead): it spins on the spot toward the more open side at `EXPLORE_PIVOT_W` 14%, with hysteresis so it does not flip-flop.
   - **Bump / stall backup** (`/rover/bump`, `/rover/stall`): it reverses at 14% for 0.6 s, then spins toward the open side for 0.9-3 s, then carries on exploring. It never just stops.

A final safety check stops forward motion into anything under 0.20 m, unless that thing is the centred orange being approached.

Status JSON is published on `/rover/behaviour/status`. It includes the state, the reason, v/w, nfruit, orange_cam and orange_room.

## Offline tests

- `tools/test_orange_offline.py img...` runs the old HSV finder against YOLO + vote on still images.
- `tools/test_rover_track.py frame.jpg` checks that the overhead Rover track ignores static blobs and confirms a moving one.
- `tools/replay_explore.py <bag>` feeds recorded ToF into the Explorer and summarises its speed and turning.
- `python3 person_follow.py --bag <mcap bag>` runs the person/orange detection replay.

## Drive-board safety (wins over the brain)

1. E-stop / battery critical (6.0 V)
2. Stall lockout (~2.5 s)
3. Heading hold trim (MPU6050)
4. Brain `/cmd_vel`
5. Cmd timeout 400 ms → stop

## Logs

`docker logs rover-brain` shows `picture far|near|close|boxed|stale`, `escape (...)`, `wheel stall`, `session START/STOP`, `allow_drive=`.
