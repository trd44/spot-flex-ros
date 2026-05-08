import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    spot_flex_sim = get_package_share_directory('spot_flex_sim')
    spot_flex_nav = get_package_share_directory('spot_flex_nav')

    world_file = LaunchConfiguration('world_file')
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    autostart = LaunchConfiguration('autostart')
    use_slam = LaunchConfiguration('use_slam')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(spot_flex_sim, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'world_file': world_file,
            'rviz': rviz,
            'headless': headless,
        }.items(),
    )

    nav2_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(spot_flex_nav, 'launch', 'odom_navigation.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': autostart,
            'nav2_params_file': nav2_params_file,
            'start_odom_tf': 'true',
            'odom_topic': '/spot/odometry',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }.items(),
        condition=UnlessCondition(use_slam),
    )

    nav2_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(spot_flex_nav, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': autostart,
            'nav2_params_file': nav2_params_file,
            'slam_params_file': slam_params_file,
            'start_depth_scan': 'false',
            'start_odom_tf': 'true',
            'odom_topic': '/spot/odometry',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'scan_topic': '/spot/lidar/scan',
        }.items(),
        condition=IfCondition(use_slam),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value='test_room.sdf'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Run SLAM Toolbox and use map-frame Nav2. Default false uses odom-frame Nav2 for a reliable demo.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(spot_flex_nav, 'config', 'nav2_sim_odom_params.yaml'),
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(spot_flex_nav, 'config', 'slam_toolbox_sim.yaml'),
        ),
        simulation,
        TimerAction(period=5.0, actions=[nav2_odom, nav2_mapping]),
    ])
