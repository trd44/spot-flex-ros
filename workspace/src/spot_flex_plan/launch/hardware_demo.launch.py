"""Hardware-oriented demo launch.

This keeps nav2 and MoveIt out of the critical path. The planner talks to the
same action/service names in every mode; this launch chooses GraphNav for base
navigation and a conservative policy shim for manipulation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_waypoints = os.path.join(
        get_package_share_directory('spot_flex_plan'),
        'config',
        'demo_waypoints.yaml',
    )

    waypoints_file = LaunchConfiguration('waypoints_file')
    nav_backend = LaunchConfiguration('nav_backend')
    policy_dry_run = LaunchConfiguration('policy_dry_run')
    launch_spot_driver = LaunchConfiguration('launch_spot_driver')
    spot_config_file = LaunchConfiguration('spot_config_file')
    spot_controllable = LaunchConfiguration('spot_controllable')
    use_mock_perception = LaunchConfiguration('use_mock_perception')
    use_mock_spot_services = LaunchConfiguration('use_mock_spot_services')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_waypoints,
            description='Demo locations YAML. GraphNav mode reads graphnav_waypoint fields.',
        ),
        DeclareLaunchArgument(
            'nav_backend',
            default_value='graphnav',
            description='spot_flex_control nav backend: graphnav, trajectory, or nav2.',
        ),
        DeclareLaunchArgument(
            'policy_dry_run',
            default_value='true',
            description='Keep ExecutePolicy as a no-motion shim until hardware motions are validated.',
        ),
        DeclareLaunchArgument(
            'launch_spot_driver',
            default_value='false',
            description='Include spot_driver/spot_driver.launch.py.',
        ),
        DeclareLaunchArgument(
            'spot_config_file',
            default_value='',
            description='spot_driver config file.',
        ),
        DeclareLaunchArgument(
            'spot_controllable',
            default_value='true',
            description='Pass controllable to spot_driver when launch_spot_driver is true.',
        ),
        DeclareLaunchArgument(
            'use_mock_perception',
            default_value='true',
            description='Use mock perception while validating hardware nav/control plumbing.',
        ),
        DeclareLaunchArgument(
            'use_mock_spot_services',
            default_value='false',
            description='Use mock dock/undock/arm services if spot_driver is not running.',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spot_driver'),
                    'launch',
                    'spot_driver.launch.py',
                ])
            ),
            launch_arguments={
                'config_file': spot_config_file,
                'controllable': spot_controllable,
                'launch_rviz': 'false',
            }.items(),
            condition=IfCondition(launch_spot_driver),
        ),

        Node(
            package='spot_flex_plan',
            executable='conductor_node',
            name='conductor_node',
            output='screen',
            parameters=[{'waypoints_file': waypoints_file}],
        ),
        Node(
            package='spot_flex_control',
            executable='nav_node',
            name='nav_node',
            output='screen',
            parameters=[{
                'backend': nav_backend,
                'waypoints_file': waypoints_file,
            }],
        ),
        Node(
            package='spot_flex_control',
            executable='policy_server_node',
            name='policy_server_node',
            output='screen',
            parameters=[{'dry_run': policy_dry_run}],
        ),
        Node(
            package='spot_flex_mocks',
            executable='mock_perception_server',
            name='mock_perception_server',
            output='screen',
            condition=IfCondition(use_mock_perception),
        ),
        Node(
            package='spot_flex_mocks',
            executable='mock_spot_services',
            name='mock_spot_services',
            output='screen',
            condition=IfCondition(use_mock_spot_services),
        ),
    ])
