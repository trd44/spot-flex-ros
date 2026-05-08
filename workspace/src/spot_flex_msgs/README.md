# spot_flex_msgs

Custom ROS 2 action and service interfaces for the Spot Flex demo stack.

This package contains only interface definitions. It should be built before packages that import the generated Python or C++ message bindings.

## Build

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select spot_flex_msgs
source install/setup.bash
```

## Actions

| Interface | Purpose |
| --- | --- |
| `FetchItem.action` | High-level task request for fetching an item from a location. |
| `FindBoxGraspPoint.action` | Detect a box grasp pixel for a requested side. |
| `FindCabinetHandle.action` | Detect a cabinet handle pixel. |
| `FindObject.action` | Detect a named object pixel. |
| `ExecutePolicy.action` | Execute a named low-level policy such as push, place, or cabinet opening. |

Example:

```bash
ros2 action send_goal /fetch_item spot_flex_msgs/action/FetchItem \
  "{item_name: 'soda can', location: 'cabinet'}" --feedback
```

## Services

| Interface | Purpose |
| --- | --- |
| `GetPlan.srv` | Return an action sequence for a task. |
| `GetHandPose.srv` | Return the current hand pose used by policy execution. |
| `GraspPixel.srv` | Request a pixel grasp from an image source through MoveIt or the Spot SDK. |
| `EstimateReactiveForce.srv` | Estimate contact force for reactive manipulation. |
| `ImpedanceSettle.srv` | Hold arm impedance for a fixed duration. |
| `PushObjectStep.srv` | Execute one object-pushing step. |

## Notes

Generated interfaces are available after sourcing the workspace:

```bash
source /repo/workspace/install/setup.bash
ros2 interface show spot_flex_msgs/action/FetchItem
ros2 interface show spot_flex_msgs/srv/GetPlan
```
