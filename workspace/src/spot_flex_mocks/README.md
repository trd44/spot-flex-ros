# spot_flex_mocks

Mock action and service servers for developing the Spot Flex task stack without hardware.

The mock package is useful for validating planner flow, action wiring, and service names before running the full hardware stack.

## Build

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select spot_flex_msgs spot_flex_mocks
source install/setup.bash
```

## Mock Servers

| Executable | Purpose |
| --- | --- |
| `mock_nav_server` | Provides navigation-like action responses. |
| `mock_perception_server` | Provides perception action responses. |
| `mock_policy_server` | Provides policy execution action responses. |
| `mock_spot_services` | Provides Spot-style arm, gripper, dock, and service responses. |

## Running Mocks

Each mock can be run independently:

```bash
ros2 run spot_flex_mocks mock_nav_server
ros2 run spot_flex_mocks mock_perception_server
ros2 run spot_flex_mocks mock_policy_server
ros2 run spot_flex_mocks mock_spot_services
```

For most development workflows, prefer the mock launch in `spot_flex_plan`:

```bash
ros2 launch spot_flex_plan demo.launch.py
```

Then send a task goal:

```bash
ros2 action send_goal /fetch_item spot_flex_msgs/action/FetchItem \
  "{item_name: 'can', location: 'cabinet'}" --feedback
```
