# spot_flex_plan

Task orchestration package for the Spot Flex fetch demo.

The planner presents high-level fetch actions and coordinates navigation, perception, manipulation policies, and gripper commands. Nav2 and MoveIt are the primary ROS interfaces. Spot-native navigation and arm services can be selected as fallback backends for hardware demonstrations.

## Build

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select spot_flex_msgs spot_flex_plan
source install/setup.bash
```

## Launch Files

| Launch file | Purpose |
| --- | --- |
| `demo.launch.py` | Mock/demo launch for development without hardware. |
| `hardware_demo.launch.py` | Hardware-oriented launch with selectable Nav2, MoveIt, GraphNav, perception, and policy options. |

## Mock Demo

```bash
ros2 launch spot_flex_plan demo.launch.py
```

In another terminal:

```bash
ros2 action send_goal /fetch_item spot_flex_msgs/action/FetchItem \
  "{item_name: 'can', location: 'cabinet'}" --feedback
```

## Hardware Demo With Nav2 And MoveIt

The ROS-native hardware path enables Nav2 for navigation and MoveIt for arm/gripper services:

```bash
ros2 launch spot_flex_plan hardware_demo.launch.py \
  nav_backend:=nav2 \
  launch_nav2:=true \
  nav2_map:=/repo/workspace/src/spot_flex_nav/maps/my_room.yaml \
  launch_moveit:=true \
  moveit_use_mock_control:=false \
  arm_service_prefix:=/moveit_spot \
  perception_open_gripper_service:=/moveit_spot/open_gripper
```

Send the full fetch goal:

```bash
ros2 action send_goal /fetch_item spot_flex_msgs/action/FetchItem \
  "{item_name: 'soda can'}" --feedback
```

## Spot-Native Hardware Fallback

When hardware Nav2 or MoveIt is not reliable enough for a live run, the same task planner can route navigation and arm services to Spot-native packages:

```bash
ros2 launch spot_flex_plan hardware_demo.launch.py \
  nav_backend:=graphnav \
  launch_nav2:=false \
  launch_moveit:=false \
  arm_service_prefix:=/spot
```

This mode expects `launch_spot.sh` or `spot_driver` to be running separately.

Typical terminal layout:

```bash
# Terminal 1
bash /repo/workspace/launch_spot.sh

# Terminal 2
ros2 launch spot_flex_plan hardware_demo.launch.py \
  nav_backend:=graphnav \
  arm_service_prefix:=/spot

# Terminal 3
ros2 action send_goal /fetch_item spot_flex_msgs/action/FetchItem \
  "{item_name: 'soda can'}" --feedback
```

## Alternate Entry Points

Start with the box already moved:

```bash
ros2 action send_goal /fetch_from_cabinet spot_flex_msgs/action/FetchItem \
  "{item_name: 'soda can'}" --feedback
```

Start with the cabinet already open:

```bash
ros2 action send_goal /fetch_from_open_cabinet spot_flex_msgs/action/FetchItem \
  "{item_name: 'soda can'}" --feedback
```

## Waypoint Configuration

Task locations are configured in:

```text
config/demo_waypoints.yaml
```

The file can store Nav2 poses, GraphNav waypoint ids, or both. The selected navigation backend determines which fields are used.
