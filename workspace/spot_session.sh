#!/usr/bin/env bash
# spot_session.sh — print the exact commands for each terminal so you
# never have to remember them.  The virtual display is started inline
# so you can open the browser immediately after running this script.

set -euo pipefail

source /opt/ros/humble/setup.bash
source /repo/workspace/install/setup.bash 2>/dev/null || true

# Start the virtual display immediately (idempotent).
bash /repo/workspace/start_display.sh

cat <<'EOF'

══════════════════════════════════════════════════════════════════
  SPOT ROS2 SESSION — open a NEW terminal for each numbered step
══════════════════════════════════════════════════════════════════

  BROWSER  →  http://localhost:6080/vnc.html   (open now)

  T1 — Spot driver (claims robot, powers on, stands):
       bash /repo/workspace/launch_spot.sh

  T2 — SLAM mapping + Nav2  (wait ~10 s for T1 to finish):
       ros2 launch spot_flex_nav mapping_multi.launch.py use_sim_time:=false

  T3 — RViz  (view the map):
       bash /repo/workspace/rviz_novnc.sh \
         -d /repo/workspace/src/spot_flex_nav/rviz/mapping.rviz

  T4 — Teleop  (arrow keys to drive):
       ros2 run spot_flex_nav teleop_arrows \
         --ros-args -p cmd_vel_topic:=/spot/cmd_vel

══════════════════════════════════════════════════════════════════
  SAVE MAP when happy:
    ros2 run nav2_map_server map_saver_cli -f /repo/workspace/my_map

  GENTLE E-STOP:
    ros2 service call /spot/estop/gentle std_srvs/srv/Trigger {}

  SIT / DOCK:
    ros2 service call /spot/sit std_srvs/srv/Trigger {}
    ros2 service call /spot/dock spot_msgs/srv/Dock "{dock_id: 521}"
══════════════════════════════════════════════════════════════════
EOF
