# Project System Architecture

Git repo: https://github.com/trd44/spot-flex-ros

## Overview

Expand the Boston Dynamics Spot robot's capabilities to execute multi-step plans: navigating to locations, pushing obstacles, opening cabinets, retrieving items, and delivering them. The system uses trained manipulation policies for cabinet opening and box pushing.

**Bonus:** Demonstrate manipulation transferability with a Kinova robot.

## Package Organization

| Package | Type | Description |
|---|---|---|
| `spot_ros2` | External | Boston Dynamic's official ROS2 driver |
| `spot_flex_msgs` | Custom | Custom action/service/message definitions |
| `spot_flex_plan` | Custom | Task orchestration — conductor node + planner |
| `spot_flex_perception` | Custom | Object/cabinet detection and localization |
| `spot_flex_control` | Custom | Navigation wrapper, arm control, policy server |
| `spot_flex_ui` | Custom | Terminal UI for sending commands |

## System Architecture
![img](images/arch.png)

## Messages, Services, and Actions

### Custom (spot_flex_msgs)

| Interface | Type | Description |
|---|---|---|
| `/fetch_object` | Action | Goal: item_name, location. Feedback: current_step, progress. Result: success, message |
| `/get_plan` | Service | Request: task, item, location. Response: action_sequence[] |
| `/find_object` | Action | Goal: object_name. Result: found, object_pose (PoseStamped) |
| `/execute_policy` | Action | Goal: policy_name, target_pose. Feedback: progress. Result: success |

### From spot_ros2
#### Actions
| Name | Use |
|---|---|
| `/robot_command` | can specify almost any command but complex to fill out |
| `/mainpulation` | can specify almost any arm command but complex to fill out |
| `/navigate_to` | go to a specific waypoint |
| `/trajectory` | navigate through multiple waypoints |

#### Services
| Name | Use |
|---|---|
| `/arm_carry` | position arm in carry pose |
| `/arm_stow`, `/arm_unstow` | toggle arm stow state |
| `/open_gripper`, `/close_gripper` | toggle gripper open/closed state |
| `/dock` | dock the spot |
| `/estop/` | virtual estop commands (options: gentle, hard, release)  |
| `/graph_nav` | mapping and localization commands |

#### Topics
| Name | Use |
|---|---|
| `/arm_joint_commands` | publish joint angles to this topic to command them |
| `/body_pose` | move pody without walking |
| `/status/` | various useful statuses |

### From Nav2
| Name | Use |
|---|---|
| `/go_to` | go to a specified location given as a coordinate in vision frame |
| `/adjust` | dx,dy,dyaw command for fine position adjustments |

### From MoveIt!
| Name | Use |
|---|---|
| `/go_to_eef_pose` | can specify almost any command but complex to fill out |
| `/go_to_joint_pose` | can specify almost any arm command but complex to fill out |
| `/close_gripper` | go to a specific waypoint |
| `/open_gripper` | navigate through multiple waypoints |

## Node Descriptions

### UINode (spot_flex_ui)
- **Description:** Interface for the user to interact with the robot. Will basically call `/fetch_item` with some arguments that will be sent to the conductor node to be executed. Some lower level commands will be implemented too for debugging purposes. This will ideally be a GUI.

### ConductorNode (spot_flex_plan)
- **Description:** Handles planning and execution of the `/fetch_item` action.
- **Action servers:** `/fetch_item`
- **Member variables:** `TaskPlanner` instance, `current_plan` (list of steps), `current_step_index`, `robot_state` ex. dict (arm_stowed, holding_item, current_pose)
- **Functions:** `_execute_step()` dispatches each ActionStep to the right subsystem, `_do_navigate()`, `_do_perceive()`, `_do_policy()`, `_do_grasp()`, `_do_place()`

#### Task Planning
The conductor node uses a `TaskPlanner` class to generate action sequences for high-level goals. Right now this will be a simple state machine but can be upgraded to a behavior tree (py_trees) or pddl based system (PlanSys2).

##### Example: Fetch Item from Cabinet

1. **Navigate** to cabinet location → calls `/go_to` on nav node → calls spot_ros2 `/navigate_to`
2. **Perceive cabinet** handle → calls `/find_object` with argument cabinet on perception node → runs OWL-ViT on camera image
3. **Adjust position** to align with handle → calls `/adjust` on nav node
4. **Open cabinet** → calls `/execute_policy` with "open_cabinet" → policy server streams joint commands
5. **Perceive item** inside → calls `/find_object` on perception node
6. **Grasp item** → arm node plans and executes grasp via spot_ros2
7. **Arm carry** → calls spot_ros2 `/arm_carry`
8. **Navigate** to dropoff → `/go_to`
9. **Perceive dropoff** → `/find_object` with the argument table
10. **Place item** → arm node moves to place pose, opens gripper
11. **Arm stow** → calls spot_ros2 `/arm_stow`


### Perception Node (spot_flex_perception)
- **Description:** Handles perception with models like OWL, SAM and/or YOLO
- **Action servers:** `/find_object`
- **Member variables:** `ObjectDetector` instance, latest RGB/depth images (numpy arrays), camera intrinsics
- **Functions:** `detect(image, label)` returns bounding boxes and confidence values


### Nav Node (nav2 or spot_flex_control as a fallback)
- **Description:** Handles navigation 
- **Action servers:** `/go_to`, `/adjust/(dx,dy,dyaw)`
- **Note:** Will figure this out in the next homework.

### Arm Node (MoveIt! or spot_flex_control as a fallback)
- **Description:** Handles arm control (except for policy execution)
- **Member variables:** gripper state (open/closed), arm state (stowed/active), joint command publisher
- **Actions and Services:** `/open_gripper`, `/close_gripper`, `/arm_stow`, `/arm_unstow` services, `send_joint_command(positions)`

### Policy Server Node (spot_flex_control)
- **Description:** Loads and executes the pre-trained policies I have learned.
- **Action Server:** `/execute_policy`, runs policy loop publishing eef and body commands
- **Methods:**`start(policy_name)`, `step(joint_state, image) -> PolicyOutput`, `stop()`

## External Tools and Libraries

- **spot_ros2** — ROS2 driver for Boston Dynamics Spot
- Nav2 — navigation message interfaces
- MoveIt! - Arm planner and control
- OWL-ViT - open-vocabulary object detection
- Segment Anything - Object segementation and edge detection
- YOLO - higher frequency object detection
- Docker / VS Code Dev COntainers
