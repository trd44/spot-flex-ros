#!/usr/bin/env bash
# Launch the MoveIt arm simulation with RViz visible via noVNC.
# Open: http://localhost:6080/vnc.html?autoconnect=1&resize=scale
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_SCRIPTS_DIR="${SPOT_SIM_SCRIPTS:-/repo/workspace/src/spot_flex_sim/scripts}"
WORKSPACE_ROOT="${SPOT_WORKSPACE:-/repo/workspace}"

# Start the virtual display (Xvfb + x11vnc + websockify) if not already running.
"${SIM_SCRIPTS_DIR}/start_virtual_display.sh"

# Set DISPLAY, software rendering flags, etc.
# shellcheck source=/dev/null
source "${SIM_SCRIPTS_DIR}/virtual_display_env.sh"

if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "Workspace not built. Run: colcon build --packages-select spot_flex_moveit" >&2
  exit 1
fi

set +u
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

exec ros2 launch spot_flex_moveit spot_arm_moveit.launch.py "$@"
