# spot_flex_moveit

MoveIt configuration and low-level demo controls for the Spot arm.

This package provides:

- A standalone MoveIt demo with a mock Spot arm and mock `ros2_control` controllers.
- A MoveIt launch path that can be enabled from the hardware demo when live arm controllers are available.
- A MoveIt-backed service bridge for named arm states, gripper commands, and pose goals.

## Build

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select spot_flex_control spot_flex_moveit
source install/setup.bash
```

## Standalone MoveIt Demo

Launch the mock arm, mock controllers, `move_group`, and optionally RViz:

```bash
ros2 launch spot_flex_moveit moveit_demo.launch.py launch_rviz:=true
```

If RViz has OpenGL issues in Docker:

```bash
ros2 launch spot_flex_moveit moveit_demo.launch.py launch_rviz:=false
```

The standalone demo launches mock controllers by default:

```bash
use_mock_control:=true
```

For a hardware-style MoveIt launch where real controllers are expected to already exist:

```bash
ros2 launch spot_flex_moveit moveit_demo.launch.py \
  use_mock_control:=false \
  launch_rviz:=false
```

## Service Namespace

The MoveIt service bridge is launched in `/moveit_spot` by default.

```text
/moveit_spot/arm_unstow
/moveit_spot/arm_stow
/moveit_spot/arm_carry
/moveit_spot/open_gripper
/moveit_spot/close_gripper
/moveit_spot/set_gripper_angle
```

## Named Arm States

Move to the ready pose:

```bash
ros2 service call /moveit_spot/arm_unstow std_srvs/srv/Trigger "{}"
```

Move to the stowed pose:

```bash
ros2 service call /moveit_spot/arm_stow std_srvs/srv/Trigger "{}"
```

Move to the carry pose:

```bash
ros2 service call /moveit_spot/arm_carry std_srvs/srv/Trigger "{}"
```

`arm_carry` currently maps to the same target as `arm_unstow`.

## Gripper Services

Open and close the gripper:

```bash
ros2 service call /moveit_spot/open_gripper std_srvs/srv/Trigger "{}"
ros2 service call /moveit_spot/close_gripper std_srvs/srv/Trigger "{}"
```

The `SetGripperAngle` compatibility service follows the Spot convention where `0` is closed and `90` is open:

```bash
ros2 service call /moveit_spot/set_gripper_angle spot_msgs/srv/SetGripperAngle "{gripper_angle: 90.0}"
ros2 service call /moveit_spot/set_gripper_angle spot_msgs/srv/SetGripperAngle "{gripper_angle: 0.0}"
```

## Pose Goals In A Frame

The MoveIt bridge subscribes to:

```text
/moveit_spot/pose_goal
```

Publish a `geometry_msgs/msg/PoseStamped`. The command frame is `header.frame_id`.

For the standalone MoveIt model, use `body` as the base-relative frame:

```bash
ros2 topic pub -1 /moveit_spot/pose_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'body'}, pose: {position: {x: 0.7, y: 0.0, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Any frame known to MoveIt TF can be used if `move_group` can transform it:

```bash
ros2 topic pub -1 /moveit_spot/pose_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'body'}, pose: {position: {x: 0.55, y: 0.15, z: 0.45}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## Real Spot Pose Commands

The Spot driver also accepts pose commands directly:

```text
/spot/arm_pose_commands
```

Example:

```bash
ros2 topic pub -1 /spot/arm_pose_commands geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'body'}, pose: {position: {x: 0.7, y: 0.0, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

On hardware, `spot_driver` passes `header.frame_id` through as the Spot SDK reference frame. Common useful frames are `body`, `vision`, and `odom`.

## Hardware Demo Integration

Enable MoveIt in the hardware demo:

```bash
ros2 launch spot_flex_plan hardware_demo.launch.py \
  launch_moveit:=true \
  moveit_use_mock_control:=false \
  arm_service_prefix:=/moveit_spot \
  perception_open_gripper_service:=/moveit_spot/open_gripper
```

This path expects hardware-compatible controllers and joint states to be available. Spot SDK arm services remain available through `arm_service_prefix:=/spot`.
