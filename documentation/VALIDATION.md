# Validation

Smoke tests for a rebuilt or pulled Spot Flex ROS container.

Run these checks inside the dev container unless a command explicitly starts with `docker`.

## Dependency Check

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash

rosdep check --from-paths src --ignore-src \
  --skip-keys "ament_python bosdyn bosdyn_msgs spot_wrapper bosdyn_cmake_module"
```

Expected result:

```text
All system dependencies have been satisfied
```

`ament_python` is skipped because it is the ROS package build type, not a rosdep-installable system package. The `bosdyn*` and `spot_wrapper` keys are provided by the Boston Dynamics install path used in this workspace.

## Full Workspace Build

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Dockerfile Image Check

This check is useful after changing `.devcontainer/Dockerfile`. Run it from the repository root on the host:

```bash
docker build -f .devcontainer/Dockerfile -t spot-flex-ros:dockerfile-check .
docker run --rm spot-flex-ros:dockerfile-check bash -lc \
  'source /opt/ros/humble/setup.bash && python3 -c "import torch, torchvision, transformers, segment_anything, cv2, open3d, yasmin, yasmin_ros; print(\"imports ok\")"'
```

Expected result:

```text
imports ok
```

## Mock Task Demo

This validates the high-level task stack without a robot:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch spot_flex_plan demo.launch.py
```

In another terminal:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 action send_goal /fetch_item spot_flex_msgs/action/FetchItem \
  "{item_name: 'can', location: 'cabinet'}" --feedback
```

Expected result:

```text
Goal finished with status: SUCCEEDED
success: true
message: DONE
```

## Nav2 Simulation Smoke Test

Use the headless form for an automated check:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 35s ros2 launch spot_flex_sim nav2_demo.launch.py headless:=true rviz:=false
```

Expected behavior: the launch stays alive until `timeout` stops it, Gazebo bridge topics start, and Nav2 lifecycle nodes become active. A timeout exit is expected for this check.

To test the interactive version, use the README command with `headless:=false rviz:=true`. On macOS, use the noVNC workflow from the main README.

## MoveIt Simulation Smoke Test

Launch MoveIt without RViz:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch spot_flex_moveit moveit_demo.launch.py launch_rviz:=false
```

Expected launch output includes:

```text
You can start planning now!
MoveIt service bridge ready.
```

In another terminal, test the documented service and pose examples:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /moveit_spot/arm_unstow std_srvs/srv/Trigger "{}"
ros2 service call /moveit_spot/open_gripper std_srvs/srv/Trigger "{}"
ros2 service call /moveit_spot/set_gripper_angle spot_msgs/srv/SetGripperAngle \
  "{gripper_angle: 90.0}"
ros2 topic pub -1 /moveit_spot/pose_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'body'}, pose: {position: {x: 0.7, y: 0.0, z: 0.35}, orientation: {w: 1.0}}}"
```

Expected service results include `success=True`.

## Hardware Launch Smoke Tests Without A Robot

These checks validate launch files, parameters, package installation, and model loading. They do not prove robot motion because no Spot driver, hardware TF, or live controllers are available.

Spot-native fallback path:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 25s ros2 launch spot_flex_plan hardware_demo.launch.py \
  nav_backend:=graphnav \
  launch_nav2:=false \
  launch_moveit:=false \
  arm_service_prefix:=/spot
```

Expected behavior: conductor, nav, policy, and perception nodes start. A timeout exit is expected.

ROS-native Nav2 and MoveIt path:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 45s ros2 launch spot_flex_plan hardware_demo.launch.py \
  nav_backend:=nav2 \
  launch_nav2:=true \
  nav2_map:=/repo/workspace/src/spot_flex_nav/maps/my_room.yaml \
  launch_moveit:=true \
  moveit_use_mock_control:=false
```

Expected behavior: map server loads `my_room.yaml`, MoveIt reaches planning-ready, and the MoveIt bridge starts. Repeated missing TF messages such as `spot/body` to `spot/odom` are expected without the robot or Spot driver.

