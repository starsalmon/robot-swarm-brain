# LEGO Rover — wiring

All Rover hardware is on the two ESP32-S3 boards; see [`lego-rover-esp32/WIRING.md`](../lego-rover-esp32/WIRING.md). Nothing is wired to the brain.

## Brain environment (rover-brain, `/data/swarm/robot-swarm-brain/docker-compose.yml`)

| Env | Value |
|-----|-------|
| `ROVER_MODE` | `explore` |
| `ROVER_PICTURE` | `1` |
| `ROVER_AUTO_MAX_LINEAR` / `ROVER_AUTO_BURST_MAX` | `0.25` (25% usable) |
| `ROVER_AUTO_REVERSE_POWER` | `0.10` |
| `ROVER_AUTO_MAX_ANGULAR` | `0.18` |
| `ROVER_PICTURE_STEER_FAR_M` / `ROVER_PICTURE_STEER_SLOW` | `1.10` / `0.80` |
| `ROVER_WHEEL_DIAM_MM` / `ROVER_WHEEL_STRIPS` / `ROVER_WHEEL_TRACK_MM` | `95` / `60` / `130` |
| `ROVER_WHEEL_STALL_GRACE_MS` / `ROVER_WHEEL_STALL_COOLDOWN_MS` | `900` / `5000` |

## Related docs

- [ROVER_BEHAVIOR.md](ROVER_BEHAVIOR.md) — explore
- [BUTTON.md](BUTTON.md) — buttons and feedback
