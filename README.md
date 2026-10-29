# Robot swarm brain (ROS 2 Jazzy)

Shared brain for every robot in the swarm (Rover, Tracker, and the bots still to come), repo `starsalmon/robot-swarm-brain`.

Rover's brain runs on the NUC `brain` (192.168.3.172) as the `rover-brain` container under `/data/swarm/robot-swarm-brain`, with one micro-ROS agent on UDP 8888 for all ESP clients. Rover is dual-board: T-Display drive (`rover-esp`) + Pro S3 ToF (`rover-tof`).

| Doc / file | Purpose |
|------------|---------|
| [ROVER_BEHAVIOR.md](ROVER_BEHAVIOR.md) | Explore: picture, steer-around, escapes, limits |
| [ROVER_WANDER.md](ROVER_WANDER.md) | Mode roll (cruise / wander / burst) |
| [BUTTON.md](BUTTON.md) | Go / E-stop, topics, ring and speaker cues |
| [WIRING.md](WIRING.md) | Brain env; hardware pins live in `lego-rover-esp32/WIRING.md` |
| [ROADMAP.md](ROADMAP.md) | Unbuilt fleet: Beeper Baby, Jazz, shared dock, IR identity |
| `rover_brain.py`, `explore_controller.py`, `rover_proximity.py`, `rover_wheel_odom.py` | Rover brain node |
| `docker-compose.yml` | Brain stack (synced from the brain) |

**Firmware:** `../lego-rover-esp32`.
