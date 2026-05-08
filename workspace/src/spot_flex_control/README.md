# spot_flex_control

Control nodes for navigation backend selection, MoveIt-backed arm services, GraphNav initialization, and learned policy execution.

This package connects high-level task plans to lower-level navigation, arm, and policy actions. Nav2 and MoveIt are the primary interfaces. Spot-native GraphNav and SDK services are supported as hardware fallbacks.

## Build

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select spot_flex_msgs spot_flex_control
source install/setup.bash
```

## Nodes

| Executable | Purpose |
| --- | --- |
| `nav_node` | Converts planner navigation requests to Nav2, trajectory, or GraphNav backends. |
| `arm_node` | Exposes MoveIt-backed arm, gripper, pose, and pixel-grasp services under a namespace such as `/moveit_spot`. |
| `policy_server_node` | Executes manipulation policies for pushing, placing, and cabinet opening. |
| `graphnav_initializer_node` | Uploads and initializes Spot GraphNav maps for fallback navigation. |

## Navigation Backend

`nav_node` supports three backend modes:

| Backend | Use case |
| --- | --- |
| `nav2` | Primary ROS navigation backend using `NavigateToPose`. |
| `trajectory` | Direct body-relative trajectory action commands. |
| `graphnav` | Spot-native fallback using GraphNav waypoints. |

The backend is usually selected through `spot_flex_plan/launch/hardware_demo.launch.py`.

## MoveIt Arm Service Bridge

The MoveIt service bridge is normally started by `spot_flex_moveit/launch/moveit_demo.launch.py` or by the hardware demo when `launch_moveit:=true`.

Standalone usage:

```bash
ros2 run spot_flex_control arm_node --ros-args -r __ns:=/moveit_spot
```

Available services:

```text
/moveit_spot/arm_unstow
/moveit_spot/arm_stow
/moveit_spot/arm_carry
/moveit_spot/open_gripper
/moveit_spot/close_gripper
/moveit_spot/set_gripper_angle
/moveit_spot/grasp_pixel
```

Examples:

```bash
ros2 service call /moveit_spot/arm_unstow std_srvs/srv/Trigger "{}"
ros2 service call /moveit_spot/open_gripper std_srvs/srv/Trigger "{}"
```

The bridge also accepts `geometry_msgs/msg/PoseStamped` goals:

```bash
ros2 topic pub -1 /moveit_spot/pose_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'body'}, pose: {position: {x: 0.7, y: 0.0, z: 0.35}, orientation: {w: 1.0}}}"
```

The `grasp_pixel` service accepts the same `spot_flex_msgs/srv/GraspPixel` request used by the Spot SDK path. It projects the requested image pixel through a registered depth image and camera info, sends the resulting pose to MoveIt, then closes the gripper. The default topics are:

```text
/spot/depth_registered/hand/image
/spot/depth_registered/hand/camera_info
```

These can be changed with the `grasp_depth_topic` and `grasp_camera_info_topic` parameters.

## Policy Server

`policy_server_node` provides the `/execute_policy` action. Configuration is loaded from `config/policy_server.yaml`.

```bash
ros2 run spot_flex_control policy_server_node --ros-args \
  --params-file /repo/workspace/src/spot_flex_control/config/policy_server.yaml
```

Example:

```bash
ros2 action send_goal /execute_policy spot_flex_msgs/action/ExecutePolicy \
  "{policy_name: 'place'}" --feedback
```

## GraphNav Initialization

GraphNav initialization is available for hardware fallback workflows:

```bash
ros2 run spot_flex_control graphnav_initializer_node --ros-args \
  -p map_path:=/repo/workspace/maps/demo.walk \
  -p localization_method:=fiducial
```
