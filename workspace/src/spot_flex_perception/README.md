# spot_flex_perception

Perception action servers for object, box edge, and cabinet-handle detection.

The package uses camera images and model-backed detectors to return image pixels for downstream Spot grasping and task planning.

## Build

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select spot_flex_msgs spot_flex_perception
source install/setup.bash
```

## Model Cache

Models are expected in the container cache paths used by the workspace:

```text
/opt/spot_flex_model_cache
/repo/workspace/model_cache
```

The exact cache path depends on the container setup.

## Perception Server

```bash
ros2 run spot_flex_perception perception_server_node
```

The server exposes these actions:

```text
/find_box_grasp_point
/find_cabinet_handle
/find_object
```

## Action Examples

Find a box grasp point:

```bash
ros2 action send_goal /find_box_grasp_point spot_flex_msgs/action/FindBoxGraspPoint \
  "{side: left}" --feedback
```

Find a cabinet handle:

```bash
ros2 action send_goal /find_cabinet_handle spot_flex_msgs/action/FindCabinetHandle \
  "{}" --feedback
```

Find an object by name:

```bash
ros2 action send_goal /find_object spot_flex_msgs/action/FindObject \
  "{object_name: 'soda can'}" --feedback
```

## Offline Checks

The detector modules include offline test entry points that can be run from the source tree:

```bash
cd /repo/workspace
PYTHONPATH=/repo/workspace/src/spot_flex_perception:$PYTHONPATH python3 -m spot_flex_perception.test_owl
PYTHONPATH=/repo/workspace/src/spot_flex_perception:$PYTHONPATH python3 -m spot_flex_perception.test_box_grasp
PYTHONPATH=/repo/workspace/src/spot_flex_perception:$PYTHONPATH python3 -m spot_flex_perception.test_cabinet_handle
```

## Hardware Notes

The default hardware camera topic is configured by the launch file in `spot_flex_plan`. Common parameters include:

```text
perception_rgb_topic
perception_open_gripper_service
perception_grasp_pixel_service
perception_grasp_image_source
```

The hardware demo can route the same perception pixel to either pixel-grasp backend:

```text
/moveit_spot/grasp_pixel
/spot/grasp_pixel
```

When `launch_moveit:=true`, the hardware demo defaults to the MoveIt service. The MoveIt path projects the pixel through registered depth and camera info before planning the arm motion. The Spot SDK path remains available as a fallback by setting `perception_grasp_pixel_service:=/spot/grasp_pixel`.
