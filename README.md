# spot-flex-ros

ROS2 system for expanding Boston Dynamics Spot's manipulation capabilities. The robot executes multi-step plans potenitally involving navigation, obstacle pushing, cabinet opening, and item retrieval using trained manipulation policies.

Initial Implementation: 

## Project Structure

```
workspace/src/
├── spot_ros2/              # BD's official ROS2 driver
├── spot_flex_msgs/         # Custom action/service definitions
├── spot_flex_plan/         # Conductor node + task planner
├── spot_flex_perception/   # Object detection and localization
├── spot_flex_control/      # Navigation, arm control, policy server
└── spot_flex_ui/           # Terminal command interface
```

See [ARCHITECTURE.md](ARCHITECTURE.md) for the full system design.

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

## Network

Spot's default IP is `192.168.80.3` on its own network.
