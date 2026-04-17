import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('spot_flex_nav')

    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    scan_frame_id = LaunchConfiguration('scan_frame_id')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')
    horizontal_fov = LaunchConfiguration('horizontal_fov')
    scan_height_pixels = LaunchConfiguration('scan_height_pixels')
    row_center_fraction = LaunchConfiguration('row_center_fraction')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/spot/depth/frontleft/image_raw',
            description='Depth image topic. Use /spot/depth/frontleft/image on the real driver.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/spot/depth/frontleft/camera_info',
            description='CameraInfo topic matching the depth image.',
        ),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'scan_frame_id',
            default_value='base_link',
            description='Frame ID for the synthetic LaserScan. Empty string preserves the image frame.',
        ),
        DeclareLaunchArgument('range_min', default_value='0.2'),
        DeclareLaunchArgument('range_max', default_value='4.0'),
        DeclareLaunchArgument('horizontal_fov', default_value='1.35'),
        DeclareLaunchArgument('scan_height_pixels', default_value='18'),
        DeclareLaunchArgument('row_center_fraction', default_value='0.5'),
        DeclareLaunchArgument('publish_rate_hz', default_value='15.0'),
        Node(
            package='spot_flex_nav',
            executable='depth_to_scan',
            name='depth_to_scan',
            output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'depth_to_scan.yaml'),
                {
                    'depth_topic': depth_topic,
                    'camera_info_topic': camera_info_topic,
                    'scan_topic': scan_topic,
                    'scan_frame_id': scan_frame_id,
                    'range_min': ParameterValue(range_min, value_type=float),
                    'range_max': ParameterValue(range_max, value_type=float),
                    'horizontal_fov': ParameterValue(horizontal_fov, value_type=float),
                    'scan_height_pixels': ParameterValue(scan_height_pixels, value_type=int),
                    'row_center_fraction': ParameterValue(row_center_fraction, value_type=float),
                    'publish_rate_hz': ParameterValue(publish_rate_hz, value_type=float),
                },
            ],
        ),
    ])
