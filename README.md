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

## Project Requirement Map

| Requirement | Primary files | Notes |
| --- | --- | --- |
| MoveIt | [spot_flex_moveit launch](workspace/src/spot_flex_moveit/launch/moveit_demo.launch.py), [MoveIt config](workspace/src/spot_flex_moveit/config), [MoveIt service bridge](workspace/src/spot_flex_control/spot_flex_control/arm_node.py) | Provides the simulated MoveIt arm demo, MoveIt-backed arm/gripper services, pose goals, and the pixel-to-grasp bridge. |
| Nav2 | [Nav2 simulation launch](workspace/src/spot_flex_sim/launch/nav2_demo.launch.py), [Nav2 params](workspace/src/spot_flex_nav/config/nav2_sim_odom_params.yaml), [navigation adapter](workspace/src/spot_flex_control/spot_flex_control/nav_node.py), [Nav2 package README](workspace/src/spot_flex_nav/README.md) | Nav2 is demonstrated in Gazebo simulation and can be enabled for hardware with `launch_nav2:=true`. |
| Perception | [perception server](workspace/src/spot_flex_perception/spot_flex_perception/perception_server_node.py), [OWL detector](workspace/src/spot_flex_perception/spot_flex_perception/owl_detector.py), [box grasp detector](workspace/src/spot_flex_perception/spot_flex_perception/box_grasp_detector.py), [cabinet detector](workspace/src/spot_flex_perception/spot_flex_perception/cabinet_handle_detector.py) | Provides object, box grasp, and cabinet-handle perception actions. Model setup is documented in [workspace/model_cache/README.md](workspace/model_cache/README.md). |
| Custom components | [custom actions/services](workspace/src/spot_flex_msgs), [task conductor](workspace/src/spot_flex_plan/spot_flex_plan/conductor_node.py), [fetch FSM](workspace/src/spot_flex_plan/spot_flex_plan/demo_fsm.py), [hardware launch](workspace/src/spot_flex_plan/launch/hardware_demo.launch.py), [policy server](workspace/src/spot_flex_control/spot_flex_control/policy_server_node.py), [Nav2/GraphNav adapter](workspace/src/spot_flex_control/spot_flex_control/nav_node.py), [MoveIt arm bridge](workspace/src/spot_flex_control/spot_flex_control/arm_node.py), [mock servers](workspace/src/spot_flex_mocks/spot_flex_mocks), [operator UI node](workspace/src/spot_flex_ui/spot_flex_ui/ui_node.py) | The custom stack coordinates task planning, navigation backend selection, perception, policy execution, MoveIt/Spot arm service routing, gripper control, mock testing, and fallback hardware backends. |
| Hand-written node | [conductor_node.py](workspace/src/spot_flex_plan/spot_flex_plan/conductor_node.py) | Hand-written ROS 2 node for coordinating the sequence of the fetch demo. |

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

### Clone The Repository

```bash
git clone https://github.com/trd44/spot-flex-ros.git
cd spot-flex-ros
cp .env_example .env
```

Edit `.env` with the Spot network and login configuration when hardware access is required.

### Option 1: Build The Dev Container Locally

This is the default path. Open the repository in VS Code and choose **Reopen in Container**. VS Code uses `.devcontainer/Dockerfile` to build the container.

If this method is causing issues try Option 2.

### Option 2: Use The Preserved Docker Hub Image

To use a preserved container image instead of rebuilding the dev container from the Dockerfile:

```bash
docker pull tduggan93/spot-flex-ros:submission-freeze
```

Then edit `.devcontainer/devcontainer.json` and replace the `build` block:

```json
"build": {
    "dockerfile": "Dockerfile",
    "context": ".."
}
```

with:

```json
"image": "tduggan93/spot-flex-ros:submission-freeze"
```

Keep the existing `workspaceMount`, `workspaceFolder`, `containerEnv`, `runArgs`, and `postStartCommand` entries. After saving the file, open the repository in VS Code and choose **Reopen in Container**.

The Docker image preserves installed packages and container filesystem changes, but the repository is still mounted from the local checkout at `/repo`. Source code changes should be preserved with Git or a separate source archive.

### Build The Workspace

Inside the container:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "ament_python bosdyn bosdyn_msgs spot_wrapper bosdyn_cmake_module"
colcon build --symlink-install
source install/setup.bash
```

Container and launch smoke tests are documented in [documentation/VALIDATION.md](documentation/VALIDATION.md). Run those checks after rebuilding the dev container or switching to the preserved Docker Hub image.

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

If running on macOS, see the [noVNC display workaround](#novnc-display-workaround-on-macos) before launching Gazebo or RViz from the container.

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

If running on macOS, see the [noVNC display workaround](#novnc-display-workaround-on-macos) before launching RViz from the container.

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


## noVNC Display Workaround On macOS

When working on macOS, displaying RViz, Gazebo, or MoveIt directly from inside the Docker container can be unreliable because of X11/OpenGL forwarding issues. The recommended workaround is to run the GUI tools inside a virtual display in the container and view that display through noVNC in a browser.

Start the virtual display inside the container:

```bash
cd /repo/workspace
./src/spot_flex_sim/scripts/start_virtual_display.sh
source ./src/spot_flex_sim/scripts/virtual_display_env.sh
```

Open the browser client on the host:

```text
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
```

If VS Code does not open the port automatically, forward container port `6080` from the **Ports** panel.

Then launch GUI demos normally from the same terminal. For example:

```bash
ros2 launch spot_flex_sim nav2_demo.launch.py headless:=false rviz:=true
```

The simulation package also provides a wrapper that starts noVNC and launches the Gazebo/Nav2 demo:

```bash
./src/spot_flex_sim/scripts/launch_gazebo_vnc.sh
```

For the MoveIt RViz demo through noVNC:

```bash
./src/spot_flex_moveit/scripts/launch_moveit_vnc.sh
```

Stop the virtual display when finished:

```bash
./src/spot_flex_sim/scripts/stop_virtual_display.sh
```

## Preserving The Current Container

The current working container can be committed and pushed to Docker Hub as a preservation snapshot. This is useful before a submission or demo, but it should not replace keeping the source repository in Git.

Commit the running container to a local image:

```bash
docker commit eee8c5e0d94a tduggan93/spot-flex-ros:submission-freeze
```

Log in and push the image:

```bash
docker login
docker push tduggan93/spot-flex-ros:submission-freeze
```

Optional dated tag:

```bash
TAG=submission-freeze-$(date +%Y%m%d)
docker tag tduggan93/spot-flex-ros:submission-freeze tduggan93/spot-flex-ros:$TAG
docker push tduggan93/spot-flex-ros:$TAG
```

Save the container run metadata:

```bash
docker inspect eee8c5e0d94a > spot-flex-container-inspect.json
```

Because the dev container bind-mounts the repository into `/repo`, `docker commit` does not preserve the checked-out source tree. Preserve the repository separately with Git or an archive before relying on the Docker image snapshot.
