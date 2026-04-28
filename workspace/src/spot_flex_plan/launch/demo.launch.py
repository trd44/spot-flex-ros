"""Bring up the conductor + (mock or real) subsystem nodes for the demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_mocks = LaunchConfiguration('use_mocks')
    fail_policy = LaunchConfiguration('fail_policy')

    return LaunchDescription([
        DeclareLaunchArgument('use_mocks', default_value='true',
                              description='Launch spot_flex_mocks instead of real nodes'),
        DeclareLaunchArgument('fail_policy', default_value='',
                              description='If set, mock_policy_server aborts on this policy_name'),

        Node(package='spot_flex_plan', executable='conductor_node',
             name='conductor_node', output='screen'),

        Node(package='spot_flex_mocks', executable='mock_nav_server',
             name='mock_nav_server', output='screen',
             condition=IfCondition(use_mocks)),
        Node(package='spot_flex_mocks', executable='mock_perception_server',
             name='mock_perception_server', output='screen',
             condition=IfCondition(use_mocks)),
        Node(package='spot_flex_mocks', executable='mock_policy_server',
             name='mock_policy_server', output='screen',
             parameters=[{'fail_policy': fail_policy}],
             condition=IfCondition(use_mocks)),
        Node(package='spot_flex_mocks', executable='mock_spot_services',
             name='mock_spot_services', output='screen',
             condition=IfCondition(use_mocks)),
    ])
