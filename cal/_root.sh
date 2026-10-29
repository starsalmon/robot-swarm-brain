# Source from cal/*.sh — sets FLEET_DIR and cd to ~/robot-fleet root.
CAL_DIR="$(cd "$(dirname "$0")" && pwd)"
FLEET_DIR="$(cd "$CAL_DIR/.." && pwd)"
cd "$FLEET_DIR"
