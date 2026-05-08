# spot_flex_sim

Gazebo and RViz simulation support for Spot Flex, including a combined Nav2 demo.

This package provides the simulation environment used to demonstrate Nav2 without relying on hardware mapping quality.

## Build

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select spot_flex_nav spot_flex_sim
source install/setup.bash
```

## Basic Simulation

```bash
ros2 launch spot_flex_sim simulation.launch.py headless:=false rviz:=true
```

Headless mode:

```bash
ros2 launch spot_flex_sim simulation.launch.py headless:=true rviz:=false
```

## Nav2 Demo

```bash
ros2 launch spot_flex_sim nav2_demo.launch.py headless:=false rviz:=true
```

Send a goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

The default demo uses odom-frame Nav2 for a stable, repeatable navigation demonstration. SLAM can be enabled explicitly:

```bash
ros2 launch spot_flex_sim nav2_demo.launch.py use_slam:=true headless:=false rviz:=true
```

## noVNC Workflow

For Docker on macOS, noVNC is usually more reliable than direct X11 forwarding.

Start the virtual display:

```bash
cd /repo/workspace
./src/spot_flex_sim/scripts/start_virtual_display.sh
source ./src/spot_flex_sim/scripts/virtual_display_env.sh
```

Launch Gazebo/RViz on that display:

```bash
ros2 launch spot_flex_sim nav2_demo.launch.py headless:=false rviz:=true
```

Open the browser client:

```text
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
```

Stop the virtual display:

```bash
./src/spot_flex_sim/scripts/stop_virtual_display.sh
```

Wrapper launch:

```bash
./src/spot_flex_sim/scripts/launch_gazebo_vnc.sh
```

Disable RViz in the wrapper:

```bash
./src/spot_flex_sim/scripts/launch_gazebo_vnc.sh rviz:=false
```
