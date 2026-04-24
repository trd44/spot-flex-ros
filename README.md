# spot-flex-ros

ROS 2 system for extending Boston Dynamics Spot with navigation, perception, planning, and arm control.

Initial Implementation: c8455e4
Nav2 and MoveIt implemented: d71513b

## Project Structure

```
workspace/src/
├── spot_ros2/              # Boston Dynamics ROS 2 driver
├── spot_flex_msgs/         # Custom action and service definitions
├── spot_flex_plan/         # Task planning and execution
├── spot_flex_perception/   # Object detection and localization
├── spot_flex_control/      # Navigation and arm control
├── spot_flex_moveit/       # MoveIt configuration for the arm
├── spot_flex_sim/          # Simulation and browser GUI helpers
└── spot_flex_ui/           # Command interface
```

See [documentation/ARCHITECTURE.md](documentation/ARCHITECTURE.md) for the full system design.

## External Tools and Libraries

- spot_ros2 - ROS 2 driver for Boston Dynamics Spot
- Nav2 - navigation stack
- MoveIt! - Arm planner and control
- OWL-ViT - open-vocabulary object detection
- Segment Anything - object segmentation and edge detection
- YOLO - higher frequency object detection
- Docker / VS Code Dev Containers

## Development Setup

Requires Docker and VS Code with the Dev Containers extension.

```bash
# 1. Clone the repo
git clone https://github.com/yourusername/spot-flex-ros.git
cd spot-flex-ros

# 2. Copy and edit credentials
cp .env_example .env # edit .env with the actual Spot IP and password

# 3. Open in VS Code, then Reopen in Container

# 4. Inside the container, build the workspace:
cd /repo/workspace
colcon build --symlink-install
source install/setup.bash

# Run nodes
```

## Perception

Models are cached in the container at `/opt/spot_flex_model_cache`. The current container may also use `/repo/workspace/model_cache`.

Offline tests:

```bash
cd /repo/workspace
PYTHONPATH=/repo/workspace/src/spot_flex_perception:$PYTHONPATH python3 -m spot_flex_perception.test_owl
PYTHONPATH=/repo/workspace/src/spot_flex_perception:$PYTHONPATH python3 -m spot_flex_perception.test_box_grasp
PYTHONPATH=/repo/workspace/src/spot_flex_perception:$PYTHONPATH python3 -m spot_flex_perception.test_cabinet_handle
```

Action server:

```bash
cd /repo/workspace
source install/setup.bash
ros2 run spot_flex_perception perception_server_node
```

Action calls:

```bash
ros2 action send_goal /find_box_grasp_point spot_flex_msgs/action/FindBoxGraspPoint "{side: left}" --feedback
ros2 action send_goal /find_cabinet_handle spot_flex_msgs/action/FindCabinetHandle "{}" --feedback
ros2 action send_goal /find_object spot_flex_msgs/action/FindObject "{object_name: box}" --feedback
```

### noVNC Browser GUI

For macOS and container setups, the browser path is more reliable than direct X11 for Gazebo and RViz.

Start the virtual display:

```bash
cd /repo/workspace
./src/spot_flex_sim/scripts/start_virtual_display.sh
source ./src/spot_flex_sim/scripts/virtual_display_env.sh
```

Launch simulation tools on that display:

```bash
ros2 launch spot_flex_sim simulation.launch.py headless:=false rviz:=true
```

Then forward container port `6080` and open:

```text
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
```

There is also a wrapper that starts the display and launches Gazebo in one step:

```bash
cd /repo/workspace
./src/spot_flex_sim/scripts/launch_gazebo_vnc.sh
```

The wrapper opens Gazebo and RViz on the same desktop by default. To disable RViz:

```bash
./src/spot_flex_sim/scripts/launch_gazebo_vnc.sh rviz:=false
```

Stop the virtual display:

```bash
./src/spot_flex_sim/scripts/stop_virtual_display.sh
```

### MoveIt Spot Arm

This launch starts a mock Spot arm with `ros2_control`, `move_group`, and RViz.

If using noVNC, start the virtual display first:

```bash
cd /repo/workspace
./src/spot_flex_sim/scripts/start_virtual_display.sh
source ./src/spot_flex_sim/scripts/virtual_display_env.sh
```

Build and launch:

```bash
cd /repo/workspace
rosdep install --from-paths src --ignore-src -r -y --skip-keys "bosdyn bosdyn_msgs spot_wrapper bosdyn_cmake_module"
colcon build --symlink-install --packages-select spot_flex_moveit spot_flex_control
source /opt/ros/humble/setup.bash
source /repo/workspace/install/setup.bash
ros2 launch spot_flex_moveit spot_arm_moveit.launch.py
```

Optional second shell:

```bash
source /opt/ros/humble/setup.bash
source /repo/workspace/install/setup.bash
ros2 run spot_flex_control arm_node
```

### Camera-Based Nav2

The `spot_flex_nav` package provides the no-lidar navigation path. It converts a depth camera stream into a synthetic
2D scan, then runs Nav2 with SLAM Toolbox:

```bash
cd /repo/workspace
source install/setup.bash
ros2 launch spot_flex_nav mapping.launch.py
```

Drive during mapping with arrow keys:

```bash
ros2 run spot_flex_nav teleop_arrows
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f /repo/workspace/src/spot_flex_nav/maps/test_room
```

Run on a saved map:

```bash
ros2 launch spot_flex_nav navigation.launch.py map:=/repo/workspace/src/spot_flex_nav/maps/test_room.yaml
```

Tag and revisit named locations:

```bash
ros2 run spot_flex_nav tag_location cabinet --file /repo/workspace/src/spot_flex_nav/locations/test_room.yaml
ros2 run spot_flex_nav go_to_location cabinet --file /repo/workspace/src/spot_flex_nav/locations/test_room.yaml
```

For the real robot, keep the same Nav2 stack but switch to wall-clock time and point the depth converter at the real
Spot RGB-D topics. The exact topic names depend on the driver configuration, but this is the expected shape:

```bash
ros2 launch spot_flex_nav mapping.launch.py \
  use_sim_time:=false \
  depth_topic:=/spot/depth/frontleft/image \
  camera_info_topic:=/spot/depth/frontleft/camera_info
```

If the real driver already publishes the odometry TF from `odom`/`vision` to `base_link`, launch with
`start_odom_tf:=false` and set `odom_frame` in the Nav2 config to match the driver frame.

## Network

Spot's default IP is `192.168.80.3` on its own network.
