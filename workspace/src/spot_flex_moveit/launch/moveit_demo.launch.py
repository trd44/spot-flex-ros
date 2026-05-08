import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('spot_flex_moveit')

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_mock_control = LaunchConfiguration('use_mock_control')
    launch_arm_services = LaunchConfiguration('launch_arm_services')
    arm_services_namespace = LaunchConfiguration('arm_services_namespace')

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'spot_arm_moveit.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': launch_rviz,
            'use_mock_control': use_mock_control,
        }.items(),
    )

    arm_services = Node(
        package='spot_flex_control',
        executable='arm_node',
        name='moveit_arm_node',
        namespace=arm_services_namespace,
        output='screen',
        condition=IfCondition(launch_arm_services),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_mock_control',
            default_value='true',
            description='Launch mock ros2_control controllers for the standalone MoveIt demo.',
        ),
        DeclareLaunchArgument('launch_arm_services', default_value='true'),
        DeclareLaunchArgument(
            'arm_services_namespace',
            default_value='moveit_spot',
            description='Namespace for MoveIt-backed arm Trigger services.',
        ),
        moveit,
        TimerAction(period=5.0, actions=[arm_services]),
    ])
