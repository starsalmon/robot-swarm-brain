# Rover house mapping — morning run (map-based explore)

## TL;DR
1. Charge Rover, power it on, put it on the floor. Turn **Allow drive** on in HA (the script never touches it).
2. On the NUC (`ssh nuc`):
   ```
   bash /data/swarm/robot-swarm-brain/maps/lidar/house_run/go_house_explore.sh
   ```
   Optional dry check first (never taps Go): `PREFLIGHT_ONLY=1 bash .../go_house_explore.sh`
3. Stop any time: `touch /data/swarm/robot-swarm-brain/maps/lidar/house_run/stop.flag`
   (or long-press E-stop on Rover / turn Allow drive off).
4. Watch: `house_run/frontier_latest.png` (map + green trail + blue frontiers + pink planned path),
   `house_run/supervisor.log`, `house_run/frontier.log`. Snapshots: `maps/lidar/house_t*.png` and `*_frontier.png`.

## What changed (and why Rover used to loop)
- Old EXPLORE (`rover_explore.Explorer` in rover-behaviour) was purely reactive: steer toward open ToF rays
  plus a random wander. It had no idea where in the house it had been. The "memory"
  (`rover_local_memory.py`) only remembered IMU-yaw headings that hit something, decayed in 45 s, and lives in
  rover_brain whose explore output is not even used (rover-behaviour is the only `/cmd_vel` publisher).
  rover_behaviour also built a heading histogram it never used. AprilTags only made sounds.
- New: `house_run/rover_frontier.py` (runs in rover-lidar-slam) reads the slam_toolbox `/map` + TF
  map→base_link, finds **frontiers** (known free floor touching unexplored space), plans a shortest path over
  free space (walls inflated by Rover's half-width), prefers big openings, penalises places Rover already
  drove (persisted in `visited.npz`), blacklists goals it can't make progress on, and publishes a steering hint
  `/rover/explore/goal_hint` (bearing to a point ~0.8 m along the path). **It never publishes /cmd_vel.**
- `rover_explore.py`: when a fresh hint exists, EXPLORE steers along the map path (no random wander), turns on
  the spot toward goals > 70° off the nose, picks the goal-side when it must pivot around an obstacle. Speed
  still comes from clearance ahead: 0.30 cruise (EXPLORE_CRUISE) down to 0.08 near things. No hint for 1.5 s →
  falls back to the old reactive explore.
- `rover_behaviour.py`: subscribes `/rover/scan_sync`; lidar forward clearance (corridor ±0.13 m, returns
  < 0.12 m ignored as chassis) slows the explorer and adds a **lidar hard stop** at 0.15 m from the nose. All
  ToF/L8 guards, bump/stall recovery, allow_drive and Go gating are unchanged. Status JSON now has
  `map_goal`, `lidar_ahead`, `lidar_guard_off`. `touch house_run/lidar_guard.off` disables only the lidar guard.
- Supervisor: starts the planner before Go and aborts if it has no map pose (`HOUSE_ALLOW_REACTIVE=1` overrides);
  stops when the planner reports no reachable frontiers for 45 s ("house covered"), when the planner is silent
  10 s, lidar gap > 15 s, ToF stale, battery < 6.4 V, allow_drive off, real pose stuck 90 s, or 20 min.
  Missing pose / broken TF is NOT treated as stuck; house_status_node scan_age is NOT a stop reason.
- Watchdog: clears old heartbeat/pid/done first; e-stops (button_event 3 → go=False → zero /cmd_vel) plus a 2 s
  burst of zero /cmd_vel if the supervisor process dies, its heartbeat is stale 90 s, or at duration + 5 min.
- `config/rover_frames.yaml`: lidar_link (0, 0, 0.12; yaw −90°) recorded.

## Map: continue or fresh?
`go_house_explore.sh` runs `map_match_check.py` (stationary): lines up the live scan with last night's map at
Rover's current map pose. ≥ 55 % agreement → continue the existing map. Otherwise (Rover was carried
somewhere else / odom reset on reboot) it saves + serializes the old map as `maps/lidar/house_prev_<time>` and
restarts rover-lidar-slam for a fresh map (motors stay stopped). Force with `MAP_MODE=continue` or `MAP_MODE=fresh`.
Tip: putting Rover back roughly where it finished last night (or where the map shows it) lets it continue.

## Preflight (script does this; all read-only)
containers up · rover-behaviour running the new code (restarts it if older; go stays off) · allow_drive is
true (read only) · battery ≥ 7.0 V (`MIN_START_V`) · /rover/scan_sync live · lidar self-return sanity · map match.

## Knobs (env)
HOUSE_DURATION_S=1200 · MIN_START_V=7.0 · MAP_MODE=auto|continue|fresh · HOUSE_ALLOW_REACTIVE=1 ·
Planner defaults (FRONTIER_INFLATE_M=0.16, LOOKAHEAD 0.8 m, PROGRESS 30 s) live at the top of house_run/frontier_core.py and rover_frontier.py.
Explorer (rover-behaviour env, needs container recreate): EXPLORE_GOAL_K=0.70 EXPLORE_GOAL_PIVOT_DEG=70
LIDAR_SELF_M=0.12 LIDAR_GUARD_M=0.15.

## Known risks
- 5 V sag → lidar dropout under motion (seen: ~16 s). Supervisor stops at 15 s gap (not loosened). During a
  short gap the map pose coasts on odom and the explorer keeps ToF guards; fix the 5 V rail for long runs.
- Battery: 6.4 V cutoff stops the run; start ≥ 7.0 V.
- TF z of lidar (0.12) is a placeholder; yaw −90° is confirmed. Low obstacles under the lidar plane are only
  seen by the ToF/L8 (unchanged).
- Dark/glass surfaces leave permanent frontiers; the planner blacklists them after 30 s without progress or
  8 s parked at the goal.
