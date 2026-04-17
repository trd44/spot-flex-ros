#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SPOT_WORKSPACE:-/repo/workspace}"

"${SCRIPT_DIR}/start_virtual_display.sh"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/virtual_display_env.sh"

if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "Could not find ${WORKSPACE_ROOT}/install/setup.bash" >&2
  exit 1
fi

cd "${WORKSPACE_ROOT}"
set +u
# shellcheck source=/dev/null
source install/setup.bash
set -u

launch_args=("$@")
have_headless_arg=0
have_rviz_arg=0

for arg in "${launch_args[@]}"; do
  [[ "${arg}" == headless:=* ]] && have_headless_arg=1
  [[ "${arg}" == rviz:=* ]] && have_rviz_arg=1
done

if [[ "${have_headless_arg}" -eq 0 ]]; then
  launch_args=("headless:=false" "${launch_args[@]}")
fi

if [[ "${have_rviz_arg}" -eq 0 ]]; then
  launch_args=("rviz:=${SPOT_FLEX_VNC_RVIZ:-true}" "${launch_args[@]}")
fi

exec ros2 launch spot_flex_sim simulation.launch.py "${launch_args[@]}"
