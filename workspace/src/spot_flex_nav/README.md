# spot_flex_nav

Nav2 integration, mapping utilities, camera-to-scan conversion, and GraphNav map helpers.

The primary navigation interface for the project is Nav2. This package supports both simulation and hardware-oriented Nav2 workflows. Spot GraphNav utilities are included as a hardware fallback and for map transfer.

## Build

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select spot_flex_msgs spot_flex_nav
source install/setup.bash
```

## Simulation Nav2

The most reliable Nav2 demo is launched through `spot_flex_sim`:

```bash
ros2 launch spot_flex_sim nav2_demo.launch.py headless:=false rviz:=true
```

By default the simulation demo uses odom-frame Nav2 for predictable planning and control. Send a goal in RViz or from the CLI:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

To demonstrate SLAM explicitly:

```bash
ros2 launch spot_flex_sim nav2_demo.launch.py use_slam:=true headless:=false rviz:=true
```

## Camera-Based Mapping

`mapping.launch.py` converts a depth stream to a synthetic 2D scan, starts SLAM Toolbox, and launches Nav2 components for mapping.

```bash
ros2 launch spot_flex_nav mapping.launch.py
```

Drive while mapping:

```bash
ros2 run spot_flex_nav teleop_arrows
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f /repo/workspace/src/spot_flex_nav/maps/test_room
```

## Saved-Map Navigation

```bash
ros2 launch spot_flex_nav navigation.launch.py \
  map:=/repo/workspace/src/spot_flex_nav/maps/test_room.yaml
```

Named locations can be tagged and revisited:

```bash
ros2 run spot_flex_nav tag_location cabinet \
  --file /repo/workspace/src/spot_flex_nav/locations/test_room.yaml

ros2 run spot_flex_nav go_to_location cabinet \
  --file /repo/workspace/src/spot_flex_nav/locations/test_room.yaml
```

## Hardware Nav2

For hardware, point the depth converter at the real Spot depth topics and use wall-clock time:

```bash
ros2 launch spot_flex_nav mapping.launch.py \
  use_sim_time:=false \
  depth_topic:=/spot/depth/frontleft/image_raw \
  camera_info_topic:=/spot/depth/frontleft/camera_info
```

If the Spot driver already publishes odometry TF, launch with `start_odom_tf:=false` and set Nav2 frames to match the driver.

The full hardware demo can enable Nav2:

```bash
ros2 launch spot_flex_plan hardware_demo.launch.py \
  nav_backend:=nav2 \
  launch_nav2:=true \
  nav2_map:=/repo/workspace/src/spot_flex_nav/maps/my_room.yaml
```

## GraphNav Fallback Utilities

GraphNav is useful when hardware Nav2 localization or mapping is not reliable enough for a live demo.

Download the active GraphNav map:

```bash
ros2 run spot_flex_nav graphnav_download_map \
  --output /repo/workspace/maps/demo.walk \
  --force
```

Tag GraphNav waypoints for task planning:

```bash
ros2 run spot_flex_nav tag_graphnav_location box box
ros2 run spot_flex_nav tag_graphnav_location cabinet cabinet
```
