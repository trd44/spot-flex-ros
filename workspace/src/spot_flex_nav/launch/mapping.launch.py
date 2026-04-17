import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    spot_flex_nav = get_package_share_directory('spot_flex_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    start_depth_scan = LaunchConfiguration('start_depth_scan')
    start_odom_tf = LaunchConfiguration('start_odom_tf')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    scan_frame_id = LaunchConfiguration('scan_frame_id')

    depth_to_scan = Node(
        package='spot_flex_nav',
        executable='depth_to_scan',
        name='depth_to_scan',
        output='screen',
        condition=IfCondition(start_depth_scan),
        parameters=[
            os.path.join(spot_flex_nav, 'config', 'depth_to_scan.yaml'),
            {
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'scan_topic': scan_topic,
                'scan_frame_id': scan_frame_id,
                'range_min': ParameterValue(LaunchConfiguration('range_min'), value_type=float),
                'range_max': ParameterValue(LaunchConfiguration('range_max'), value_type=float),
                'horizontal_fov': ParameterValue(LaunchConfiguration('horizontal_fov'), value_type=float),
                'scan_height_pixels': ParameterValue(LaunchConfiguration('scan_height_pixels'), value_type=int),
                'row_center_fraction': ParameterValue(LaunchConfiguration('row_center_fraction'), value_type=float),
                'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
            },
        ],
    )

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
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(spot_flex_nav, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(spot_flex_nav, 'config', 'slam_toolbox.yaml'),
        ),
        DeclareLaunchArgument('start_depth_scan', default_value='true'),
        DeclareLaunchArgument('start_odom_tf', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/spot/odometry'),
        DeclareLaunchArgument('odom_frame', default_value='odom_spot'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('depth_topic', default_value='/spot/depth/frontleft/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/spot/depth/frontleft/camera_info'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_frame_id', default_value='base_link'),
        DeclareLaunchArgument('range_min', default_value='0.3'),
        DeclareLaunchArgument('range_max', default_value='2.5'),
        DeclareLaunchArgument('horizontal_fov', default_value='1.35'),
        DeclareLaunchArgument('scan_height_pixels', default_value='40'),
        DeclareLaunchArgument('row_center_fraction', default_value='0.5'),
        DeclareLaunchArgument('publish_rate_hz', default_value='15.0'),
        depth_to_scan,
        odom_to_tf,
        nav2,
        slam,
    ])
