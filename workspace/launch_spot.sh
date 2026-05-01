#!/usr/bin/env bash
# One-command Spot driver launch that loads credentials from /repo/.env
# Usage: bash /repo/workspace/launch_spot.sh

# Load .env (SPOT_IP, SPOT_USERNAME, SPOT_PASSWORD)
if [[ -f /repo/.env ]]; then
    set -a
    source /repo/.env
    set +a
fi

# Map to the env vars the BD driver actually reads
export BOSDYN_CLIENT_USERNAME="${BOSDYN_CLIENT_USERNAME:-$SPOT_USERNAME}"
export BOSDYN_CLIENT_PASSWORD="${BOSDYN_CLIENT_PASSWORD:-$SPOT_PASSWORD}"
# SPOT_IP is read directly by the driver — no rename needed.

source /opt/ros/humble/setup.bash
source /repo/workspace/install/setup.bash 2>/dev/null

echo "Connecting to Spot at ${SPOT_IP} as ${BOSDYN_CLIENT_USERNAME}"

driver_pid=""
graphnav_pid=""

cleanup() {
    if [[ -n "$graphnav_pid" ]] && kill -0 "$graphnav_pid" 2>/dev/null; then
        kill "$graphnav_pid" 2>/dev/null || true
    fi
    if [[ -n "$driver_pid" ]] && kill -0 "$driver_pid" 2>/dev/null; then
        kill "$driver_pid" 2>/dev/null || true
    fi
}
trap cleanup INT TERM

ros2 launch spot_driver spot_driver.launch.py \
    spot_name:=spot \
    launch_rviz:=false \
    config_file:=/repo/workspace/src/spot_ros2/spot_driver/config/hrilab.yaml &
driver_pid=$!

if [[ "${SPOT_GRAPHNAV_INIT:-true}" == "true" ]]; then
    (
        sleep "${SPOT_GRAPHNAV_INIT_DELAY_SEC:-2}"
        echo "Initializing GraphNav map ${SPOT_GRAPHNAV_MAP_PATH:-/repo/workspace/maps/demo.walk}"
        graphnav_args=(
            --ros-args
            -p "map_path:=${SPOT_GRAPHNAV_MAP_PATH:-/repo/workspace/maps/demo.walk}"
            -p "localization_method:=${SPOT_GRAPHNAV_LOCALIZATION_METHOD:-fiducial}"
            -p "service_wait_timeout_sec:=${SPOT_GRAPHNAV_SERVICE_WAIT_TIMEOUT_SEC:-120.0}"
            -p "localization_retry_count:=${SPOT_GRAPHNAV_LOCALIZATION_RETRY_COUNT:-3}"
            -p "localization_retry_delay_sec:=${SPOT_GRAPHNAV_LOCALIZATION_RETRY_DELAY_SEC:-2.0}"
        )
        if [[ -n "${SPOT_GRAPHNAV_LOCALIZATION_WAYPOINT:-}" ]]; then
            graphnav_args+=(-p "localization_waypoint:=${SPOT_GRAPHNAV_LOCALIZATION_WAYPOINT}")
        fi
        ros2 run spot_flex_control graphnav_initializer_node "${graphnav_args[@]}"
    ) &
    graphnav_pid=$!
fi

wait "$driver_pid"
status=$?
cleanup
exit "$status"
