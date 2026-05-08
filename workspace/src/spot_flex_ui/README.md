# spot_flex_ui

Prototype command-line ROS 2 user interface for high-level Spot Flex task goals.

This package is intentionally small. The current node is a placeholder for an operator-facing command interface; direct action calls remain the reliable way to send task goals.

## Build

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select spot_flex_msgs spot_flex_ui
source install/setup.bash
```

## Run

```bash
ros2 run spot_flex_ui ui_node
```

The UI node expects the planning stack to expose the high-level fetch actions from `spot_flex_plan`. The interactive command handling is still minimal.

## Direct Action Equivalent

The same task can be sent directly with:

```bash
ros2 action send_goal /fetch_item spot_flex_msgs/action/FetchItem \
  "{item_name: 'soda can'}" --feedback
```

See `spot_flex_plan/README.md` for the full demo launch sequence.
