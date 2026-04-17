# spot-flex-ros

ROS2 system for expanding Boston Dynamics Spot's manipulation capabilities. The robot executes multi-step plans potenitally involving navigation, obstacle pushing, cabinet opening, and item retrieval using trained manipulation policies.

Initial Implementation: c8455e4

## Project Structure

```
workspace/src/
├── spot_ros2/              # Boston Dynamic's official ROS2 driver
├── spot_flex_msgs/         # Custom action/service definitions
├── spot_flex_plan/         # Conductor node + task planner
├── spot_flex_perception/   # Object detection and localization
├── spot_flex_control/      # Navigation, arm control, policy server
└── spot_flex_ui/           # Command interface
```

See [documentation/ARCHITECTURE.md](documentation/ARCHITECTURE.md) for the full system design.

## External Tools and Libraries

- spot_ros2 — ROS2 driver for Boston Dynamics Spot
- Nav2 — navigation message interfaces
- MoveIt! - Arm planner and control
- OWL-ViT - open-vocabulary object detection
- Segment Anything - Object segementation and edge detection
- YOLO - higher frequency object detection
- Docker / VS Code Dev COntainers

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
cd /ros_ws
colcon build --symlink-install
source install/setup.bash

# Run nodes
```

### GUI Tools In The Dev Container

The dev container now sources `/repo/workspace/src/spot_ros2/scripts/ros_gui_env.sh` for new shells so ROS GUI tools
default to Mesa software rendering, which is much more reliable in containers.

On macOS with XQuartz:

```bash
# on the Mac host, then restart XQuartz
defaults write org.xquartz.X11 enable_iglx -bool true
```

Then inside the container:

```bash
export SPOT_X11_DISPLAY=host.docker.internal:0
source /repo/workspace/src/spot_ros2/scripts/ros_gui_env.sh
ros2 launch spot_description description.launch.py arm:=True
```

If RViz is still unhappy, verify in XQuartz Preferences that network clients are allowed, then restart XQuartz and the
dev container.

### Optional Gazebo GUI In The Browser

Gazebo Fortress GUI is not reliable over macOS Docker + XQuartz GLX. The repo now includes an
optional browser-based path that keeps the current headless/native flows intact:

```bash
cd /repo/workspace
./src/spot_flex_sim/scripts/start_virtual_display.sh
source ./src/spot_flex_sim/scripts/virtual_display_env.sh
ros2 launch spot_flex_sim simulation.launch.py headless:=false rviz:=true
```

Then forward container port `6080` and open:

```text
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
```

There is also a convenience wrapper that starts the virtual display and launches Gazebo in one step:

```bash
cd /repo/workspace
./src/spot_flex_sim/scripts/launch_gazebo_vnc.sh
```

The wrapper opens Gazebo and RViz on the same noVNC desktop by default. To disable RViz:

```bash
./src/spot_flex_sim/scripts/launch_gazebo_vnc.sh rviz:=false
```

To stop the virtual display services:

```bash
./src/spot_flex_sim/scripts/stop_virtual_display.sh
```

This is optional. You can still run:

- headless Gazebo for navigation and testing on the laptop
- native Gazebo / RViz on an Ubuntu desktop if you want direct local rendering

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



Gazebo
cd /repo/workspace
./src/spot_flex_sim/scripts/launch_gazebo_vnc.sh

Slam cd /repo/workspace
source install/setup.bash
ros2 launch spot_flex_nav mapping.launch.py

Driving
cd /repo/workspace
source install/setup.bash
ros2 run spot_flex_nav teleop_arrows

Saving map
ros2 run nav2_map_server map_saver_cli -f /repo/workspace/src/spot_flex_nav/maps/test_room
ros2 launch spot_flex_nav navigation.launch.py map:=/repo/workspace/src/spot_flex_nav/maps/test_room.yaml
