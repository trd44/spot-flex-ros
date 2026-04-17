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
exec ros2 launch spot_driver spot_driver.launch.py \
    spot_name:=spot \
    launch_rviz:=false \
    config_file:=/repo/workspace/src/spot_ros2/spot_driver/config/hrilab.yaml
