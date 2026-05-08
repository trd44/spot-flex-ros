# spot-flex-ros

ROS 2 workspace for a Spot-based mobile manipulation demo using Nav2 for navigation, MoveIt for arm planning, perception actions for object localization, and a task-level conductor for fetch workflows.

The primary ROS architecture is built around Nav2 and MoveIt. For hardware demonstrations, the same task stack can fall back to Spot-native GraphNav and Spot SDK arm/gripper services when those are more reliable in the available environment.

## Assignments
- Initial Implementation: c8455e4
- Nav2 and MoveIt implemented: d71513b
- Perception implemented: 711dcad
- Supervisory Control implemented: ee73f85
- Custom node (spot_flex_control policy server): 50a4397

## Current State of the project

- Nav2 works in simulation on a spot with a LIDAR. Can be used for SLAM and navigating. Nav2 makes heavily distorted maps on my Spot Hardware that does not have a LIDAR.
- MoveIt controls arm in simulation and on hardware.
- Perception (OWL ViT, Segment Anything, and Color Blob) works great.
- Supervisory control uses a finite state machine to execute the predetermined plan. Is working great.
- Spots native navigation and arm control work much better than Nav2 and MoveIt, so there are options to use them instead.
- Spot is able to execute the demo in one shot (Push box out of the way, open cabinet, retrieve item, place item on dropoff table). Box pushing can get weird if the grasp fails but the spot is usually able to push it out of the way enough before losing grasp, or sometimes it pushes with its body insted. There is the option to start at different parts in the plan to test them individually. 


## Architecture

```text
FetchItem action
  -> spot_flex_plan conductor
  -> perception actions
  -> Nav2 navigation backend
  -> MoveIt arm/gripper backend
  -> optional Spot-native fallback services
```

See [documentation/ARCHITECTURE.md](documentation/ARCHITECTURE.md) for the full system design.

## Custom Packages

| Package | Purpose | Documentation |
| --- | --- | --- |
| `spot_flex_msgs` | Custom actions and services. | [README](workspace/src/spot_flex_msgs/README.md) |
| `spot_flex_nav` | Nav2, mapping, depth-to-scan, named locations, GraphNav helpers. | [README](workspace/src/spot_flex_nav/README.md) |
| `spot_flex_moveit` | MoveIt configuration, mock arm demo, arm/gripper services. | [README](workspace/src/spot_flex_moveit/README.md) |
| `spot_flex_control` | Navigation backend adapter, MoveIt service bridge, policy execution. | [README](workspace/src/spot_flex_control/README.md) |
| `spot_flex_perception` | Object, box, and cabinet-handle perception actions. | [README](workspace/src/spot_flex_perception/README.md) |
| `spot_flex_plan` | High-level task orchestration and hardware demo launch. | [README](workspace/src/spot_flex_plan/README.md) |
| `spot_flex_sim` | Gazebo, RViz, noVNC, and simulation Nav2 launch files. | [README](workspace/src/spot_flex_sim/README.md) |
| `spot_flex_mocks` | Mock servers for development without hardware. | [README](workspace/src/spot_flex_mocks/README.md) |
| `spot_flex_ui` | Operator-facing command node. | [README](workspace/src/spot_flex_ui/README.md) |

External packages in the workspace include `spot_ros2` for the Boston Dynamics ROS 2 driver and `spot_gazebo_ros2` for Gazebo assets.

## Dependencies

Primary frameworks and tools:

- ROS 2 Humble
- Nav2
- MoveIt 2
- Gazebo Fortress / Ignition
- Boston Dynamics `spot_ros2`
- OWL-ViT / perception model tooling
- Docker / VS Code Dev Containers

## Development Setup

Requires Docker and VS Code with the Dev Containers extension.

```bash
git clone https://github.com/trd44/spot-flex-ros.git
cd spot-flex-ros
cp .env_example .env
```

Edit `.env` with the Spot network and login configuration when hardware access is required.

Open the repository in VS Code and choose **Reopen in Container**.

Inside the container:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Perception model setup is documented in [workspace/model_cache/README.md](workspace/model_cache/README.md). That cache includes the OWLv2 Hugging Face model and the Segment Anything checkpoint used by `spot_flex_perception`.

## Hardware Demo With Nav2 And MoveIt

The ROS-native hardware path enables Nav2 and MoveIt:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch spot_flex_plan hardware_demo.launch.py \
  nav_backend:=nav2 \
  launch_nav2:=true \
  nav2_map:=/repo/workspace/src/spot_flex_nav/maps/my_room.yaml \
  launch_moveit:=true \
  moveit_use_mock_control:=false
```

With `launch_moveit:=true`, arm services, gripper services, and perception pixel grasps default to the MoveIt bridge under `/moveit_spot`. The pixel grasp bridge uses registered hand-camera depth and camera info to convert the detected 2D image pixel into a MoveIt pose goal.

Send the full task goal:

```bash
ros2 action send_goal /fetch_item spot_flex_msgs/action/FetchItem \
  "{item_name: 'soda can'}" --feedback
```

Task launch options and alternate entry points are documented in [spot_flex_plan](workspace/src/spot_flex_plan/README.md).

## Spot-Native Hardware Fallback

Spot-native GraphNav and SDK arm services can be used when hardware Nav2 or MoveIt is not stable enough for a live run.

Typical terminal layout:

```bash
# Terminal 1
bash /repo/workspace/launch_spot.sh

# Terminal 2
ros2 launch spot_flex_plan hardware_demo.launch.py \
  nav_backend:=graphnav \
  launch_nav2:=false \
  launch_moveit:=false \
  arm_service_prefix:=/spot

# Terminal 3
ros2 action send_goal /fetch_item spot_flex_msgs/action/FetchItem \
  "{item_name: 'soda can'}" --feedback
```

GraphNav map download, localization, and named waypoint commands are documented in [spot_flex_nav](workspace/src/spot_flex_nav/README.md).

## Nav2 Simulation Demo

The simulation demo starts Gazebo, bridge nodes, Nav2, and RViz:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch spot_flex_sim nav2_demo.launch.py headless:=false rviz:=true
```

Send a Nav2 goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Additional simulation and noVNC commands are documented in [spot_flex_sim](workspace/src/spot_flex_sim/README.md).

## MoveIt Simulation Demo

The MoveIt demo starts a mock Spot arm, mock controllers, `move_group`, and optional RViz:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch spot_flex_moveit moveit_demo.launch.py launch_rviz:=true
```

Send a MoveIt arm goal:

```bash
ros2 service call /moveit_spot/arm_unstow std_srvs/srv/Trigger "{}"
```

Send a MoveIt pose goal in the arm base frame:

```bash
ros2 topic pub -1 /moveit_spot/pose_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'body'}, pose: {position: {x: 0.7, y: 0.0, z: 0.35}, orientation: {w: 1.0}}}"
```

Low-level MoveIt arm, gripper, and pose commands are documented in [spot_flex_moveit](workspace/src/spot_flex_moveit/README.md).

## Perception

Start the perception server:

```bash
ros2 run spot_flex_perception perception_server_node
```

Perception action examples and offline detector checks are documented in [spot_flex_perception](workspace/src/spot_flex_perception/README.md).

## Network Notes

Spot commonly uses `192.168.80.3` on its own network. Hardware credentials and connection settings should be configured in `.env` and the Spot driver configuration used by `launch_spot.sh`.
