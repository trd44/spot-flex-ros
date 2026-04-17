import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def _resolve_spot_model_dir(pkg_spot_flex_sim: str, pkg_spot_description: str) -> Path:
    """Locate the Gazebo Spot model directory.

    The workspace currently contains two different packages named ``spot_description``:
    the Boston Dynamics ROS2 description package and the Gazebo model package from
    ``spot_gazebo_ros2``. The Gazebo package is the one we want here, but stale install
    artifacts can leave the old package in ``install/spot_description``. Fall back to the
    source checkout when the installed package does not expose the Gazebo model files.
    """

    installed_model_dir = Path(pkg_spot_description) / 'models' / 'spot'
    if installed_model_dir.is_dir():
        return installed_model_dir

    workspace_root = Path(pkg_spot_flex_sim).resolve().parents[3]
    source_model_dir = workspace_root / 'src' / 'spot_gazebo_ros2' / 'spot_description' / 'models' / 'spot'
    if source_model_dir.is_dir():
        return source_model_dir

    raise FileNotFoundError(
        'Could not find the Gazebo Spot model directory. Expected either '
        f'{installed_model_dir} or {source_model_dir}.'
    )


def generate_launch_description():
    pkg_spot_flex_sim = get_package_share_directory('spot_flex_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spot_gazebo = get_package_share_directory('spot_gazebo')
    pkg_spot_description = get_package_share_directory('spot_description')
    config_path = get_package_share_directory('champ_config')
    spot_model_dir = _resolve_spot_model_dir(pkg_spot_flex_sim, pkg_spot_description)
    spot_models_root = str(spot_model_dir.parent)
    spot_package_resource_root = str(spot_model_dir.parents[2])

    # Launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='test_room.sdf',
        description='World file name. Use files from spot_flex_sim/worlds/ or spot_gazebo/worlds/'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI). Use on Mac Docker if rendering is too slow.'
    )

    world_file = LaunchConfiguration('world_file')
    headless = LaunchConfiguration('headless')

    # Try our worlds/ directory first; spot_gazebo/worlds/ is also available via gz_args
    # The world path: check spot_flex_sim first, fall back to spot_gazebo
    # Since we can't do conditional path resolution easily in launch, we construct
    # the path from our package. Users can also pass a full path.
    world_path = PathJoinSubstitution([pkg_spot_flex_sim, 'worlds', world_file])

    # Gazebo Fortress simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                PythonExpression([
                    "'-r -s --headless-rendering ' if '",
                    headless,
                    "'.lower() == 'true' else '-r '",
                ]),
                world_path,
            ],
        }.items(),
    )

    # ROS-Gazebo bridge with our extended config (all cameras + depth enabled)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': os.path.join(pkg_spot_flex_sim, 'config', 'spot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }]
    )

    # Robot state publisher needs URDF, not SDF. Use the Gazebo model URDF.
    urdf_file = str(spot_model_dir / 'model.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
            {'publish_frequency': 200.0},
        ],
        remappings=[
            ('/joint_states', '/spot/joint_states')
        ]
    )

    # CHAMP quadruped controller (converts cmd_vel -> joint trajectories)
    links_config = PathJoinSubstitution([config_path, 'config', 'links', 'links.yaml'])
    links_param = ParameterFile(param_file=links_config, allow_substs=True)
    joints_config = PathJoinSubstitution([config_path, 'config', 'joints', 'joints.yaml'])
    joints_param = ParameterFile(param_file=joints_config, allow_substs=True)
    gait_config = PathJoinSubstitution([config_path, 'config', 'gait', 'gait.yaml'])
    gait_param = ParameterFile(param_file=gait_config, allow_substs=True)

    quadruped_controller_node = Node(
        package='champ_base',
        executable='quadruped_controller_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'gazebo': True},
            {'publish_joint_states': False},
            {'publish_foot_contacts': False},
            {'publish_joint_control': True},
            {'joint_controller_topic': '/spot/joint_trajectory'},
            {'urdf': urdf_file},
            links_param,
            joints_param,
            gait_param,
        ],
        remappings=[
            ('/cmd_vel/smooth', '/cmd_vel'),
        ],
    )

    # Point cloud transform (lidar_link -> base_link)
    pointcloud_transform = Node(
        package='spot_bringup',
        executable='pointcloud_transform',
        name='pointcloud_transform',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_link',
            'input_topic': '/spot/lidar/points',
            'output_topic': '/velodyne_points',
            'scan_rate': 10.0,
            'num_scan_lines': 16,
            'vertical_fov_min': -15.0,
            'vertical_fov_max': 15.0,
        }]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_spot_flex_sim, 'rviz', 'simulation.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_file_arg,
        rviz_arg,
        headless_arg,
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[spot_package_resource_root, os.pathsep, spot_models_root],
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[spot_package_resource_root, os.pathsep, spot_models_root],
        ),
        gz_sim,
        bridge,
        robot_state_publisher,
        quadruped_controller_node,
        pointcloud_transform,
        rviz,
    ])
