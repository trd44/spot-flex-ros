import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():
    pkg = get_package_share_directory('spot_flex_moveit')

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([FindPackageShare('spot_flex_moveit'), 'urdf',
                                  'spot_arm_mock.ros2_control.xacro']),
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    srdf_path = os.path.join(pkg, 'config', 'spot_arm.srdf')
    with open(srdf_path) as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml      = os.path.join(pkg, 'config', 'kinematics.yaml')
    ompl_yaml            = os.path.join(pkg, 'config', 'ompl_planning.yaml')
    moveit_ctrl_yaml     = os.path.join(pkg, 'config', 'moveit_controllers.yaml')
    joint_limits_yaml    = os.path.join(pkg, 'config', 'joint_limits.yaml')
    ros2_ctrl_yaml       = os.path.join(pkg, 'config', 'ros2_controllers.yaml')

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(kinematics_yaml)
    }
    joint_limits = {
        'robot_description_planning': load_yaml(joint_limits_yaml)
    }
    planning_pipeline = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': load_yaml(ompl_yaml),
    }
    moveit_controllers = load_yaml(moveit_ctrl_yaml)
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor_parameters = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_ctrl_yaml],
        output='screen',
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    spawn_arm_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    spawn_gripper_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    spawn_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_arm_ctrl])
    )
    spawn_gripper_after_arm = RegisterEventHandler(
        OnProcessExit(target_action=spawn_arm_ctrl, on_exit=[spawn_gripper_ctrl])
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': False},
        ],
        output='screen',
    )

    rviz_config = os.path.join(pkg, 'config', 'moveit.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline,
            moveit_controllers,
        ],
        output='log',
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        spawn_jsb,
        spawn_arm_after_jsb,
        spawn_gripper_after_arm,
        TimerAction(period=3.0, actions=[move_group]),
        TimerAction(period=4.0, actions=[rviz]),
    ])
