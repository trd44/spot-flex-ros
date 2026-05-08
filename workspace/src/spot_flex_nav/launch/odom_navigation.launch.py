import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    spot_flex_nav = get_package_share_directory('spot_flex_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('nav2_params_file')
    start_odom_tf = LaunchConfiguration('start_odom_tf')

    odom_to_tf = Node(
        package='spot_flex_nav',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        condition=IfCondition(start_odom_tf),
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'prefer_msg_frames': False,
        }],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(spot_flex_nav, 'config', 'nav2_sim_odom_params.yaml'),
        ),
        DeclareLaunchArgument('start_odom_tf', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/spot/odometry'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        odom_to_tf,
        navigation,
    ])
