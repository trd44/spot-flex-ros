#!/usr/bin/env bash
# container_startup.sh — runs automatically on every VS Code Dev Container
# connect (via postStartCommand in devcontainer.json).
#
# Does three things:
#   1. Ensures ~/.bashrc has ROS sourced so every terminal is ready.
#   2. Creates /tmp dirs that ROS and X11 expect.
#   3. Starts the virtual display (Xvfb + x11vnc + noVNC) in the background.

# ── 1. .bashrc setup ──────────────────────────────────────────────────────
BASHRC="$HOME/.bashrc"

add_if_missing() {
    local line="$1"
    grep -qxF "$line" "$BASHRC" 2>/dev/null || echo "$line" >> "$BASHRC"
}

add_if_missing "source /opt/ros/humble/setup.bash"
add_if_missing "source /repo/workspace/install/setup.bash 2>/dev/null || true"
add_if_missing "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
add_if_missing "export DISPLAY=:1"

# Spot credentials from .env (if present) — available in every terminal.
add_if_missing 'if [[ -f /repo/.env ]]; then set -a; source /repo/.env; set +a; fi'
add_if_missing 'export BOSDYN_CLIENT_USERNAME="${BOSDYN_CLIENT_USERNAME:-${SPOT_USERNAME:-}}"'
add_if_missing 'export BOSDYN_CLIENT_PASSWORD="${BOSDYN_CLIENT_PASSWORD:-${SPOT_PASSWORD:-}}"'

# Handy aliases.
add_if_missing "alias spot-driver='bash /repo/workspace/launch_spot.sh'"
add_if_missing "alias spot-map='ros2 launch spot_flex_nav mapping_multi.launch.py use_sim_time:=false'"
add_if_missing "alias spot-rviz='bash /repo/workspace/rviz_novnc.sh -d /repo/workspace/src/spot_flex_nav/rviz/mapping.rviz'"
add_if_missing "alias spot-teleop='ros2 run spot_flex_nav teleop_arrows --ros-args -p cmd_vel_topic:=/spot/cmd_vel'"
add_if_missing "alias spot-savemap='ros2 run nav2_map_server map_saver_cli -f /repo/workspace/my_map'"
add_if_missing "alias spot-estop='ros2 service call /spot/estop/gentle std_srvs/srv/Trigger {}'"
add_if_missing "alias spot-sit='ros2 service call /spot/sit std_srvs/srv/Trigger {}'"
add_if_missing "alias spot-dock='ros2 service call /spot/dock spot_msgs/srv/Dock \"{dock_id: 521}\"'"

# ── 2. Runtime dirs ────────────────────────────────────────────────────────
mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
mkdir -p /tmp/display_logs

# ── 3. Virtual display ─────────────────────────────────────────────────────
bash /repo/workspace/start_display.sh

echo ""
echo "✓ Container ready. Open http://localhost:6080/vnc.html in your browser."
echo "  In any terminal, use the spot-* aliases to launch each component:"
echo "    spot-driver    T1  Spot driver (claims, powers on, stands)"
echo "    spot-map       T2  SLAM mapping + Nav2"
echo "    spot-rviz      T3  RViz in browser"
echo "    spot-teleop    T4  Keyboard teleop (arrow keys)"
