"""Mapping pipeline for the real Spot using all 5 body depth cameras.

Uses the driver's native TF frames (spot/odom, spot/body, spot/frontleft, etc.)
directly — NO odom_to_tf bridge, NO static_transform_publisher needed. Nav2 +
slam_toolbox configs have been updated to reference spot/odom and spot/body.
"""

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
    slam_toolbox = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    start_scan = LaunchConfiguration('start_scan')

    multi_scan = Node(
        package='spot_flex_nav',
        executable='multi_depth_to_scan',
        name='multi_depth_to_scan',
        output='screen',
        condition=IfCondition(start_scan),
        parameters=[{
            'cameras': LaunchConfiguration('cameras'),
            'depth_topic_template': LaunchConfiguration('depth_topic_template'),
            'info_topic_template': LaunchConfiguration('info_topic_template'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            # Publish scan in the driver's body frame — no bridging needed.
            'scan_frame_id': 'spot/body',
            'scan_height_pixels': 40,
            'row_center_fraction': 0.5,
            'range_min': 0.3,
            'range_max': 2.0,
            'angular_resolution_deg': 1.0,
            'publish_rate_hz': 5.0,
            'column_stride': 4,
            'z_min': -0.5,
            'z_max':  1.0,
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(spot_flex_nav, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(spot_flex_nav, 'config', 'slam_toolbox.yaml'),
        ),
        DeclareLaunchArgument('start_scan', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cameras', default_value='[frontleft, frontright, left, right, back]'),
        DeclareLaunchArgument('depth_topic_template', default_value='/spot/depth/{camera}/image'),
        DeclareLaunchArgument('info_topic_template', default_value='/spot/depth/{camera}/camera_info'),
        multi_scan,
        nav2,
        slam,
    ])
