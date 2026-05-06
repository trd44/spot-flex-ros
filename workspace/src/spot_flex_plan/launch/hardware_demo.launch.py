"""Hardware-oriented demo launch.

This keeps nav2 and MoveIt out of the critical path. The planner talks to the
same action/service names in every mode; this launch chooses GraphNav for base
navigation and a conservative policy shim for manipulation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
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
    box_grasp_side = LaunchConfiguration('box_grasp_side')
    nav_backend = LaunchConfiguration('nav_backend')
    policy_dry_run = LaunchConfiguration('policy_dry_run')
    policy_use_learned = LaunchConfiguration('policy_use_learned')
    push_model_dir = LaunchConfiguration('push_model_dir')
    push_model_name = LaunchConfiguration('push_model_name')
    push_policy_max_steps = LaunchConfiguration('push_policy_max_steps')
    push_policy_step_duration_sec = LaunchConfiguration('push_policy_step_duration_sec')
    push_action_scale = LaunchConfiguration('push_action_scale')
    push_max_step_m = LaunchConfiguration('push_max_step_m')
    push_probe_force = LaunchConfiguration('push_probe_force')
    push_impedance_two_phase = LaunchConfiguration('push_impedance_two_phase')
    push_success_progress = LaunchConfiguration('push_success_progress')
    push_deviation_tolerance = LaunchConfiguration('push_deviation_tolerance')
    push_success_distance = LaunchConfiguration('push_success_distance')
    push_yaw_scale = LaunchConfiguration('push_yaw_scale')
    push_settle_between_steps = LaunchConfiguration('push_settle_between_steps')
    push_settle_duration_sec = LaunchConfiguration('push_settle_duration_sec')
    push_path_type = LaunchConfiguration('push_path_type')
    push_arc_radius = LaunchConfiguration('push_arc_radius')
    push_arc_angle_deg = LaunchConfiguration('push_arc_angle_deg')
    push_length = LaunchConfiguration('push_length')
    push_amplitude = LaunchConfiguration('push_amplitude')
    push_use_box_center = LaunchConfiguration('push_use_box_center')
    push_box_width = LaunchConfiguration('push_box_width')
    push_box_depth = LaunchConfiguration('push_box_depth')
    push_box_height = LaunchConfiguration('push_box_height')
    push_surface_type = LaunchConfiguration('push_surface_type')
    push_max_force = LaunchConfiguration('push_max_force')
    push_use_impedance = LaunchConfiguration('push_use_impedance')
    push_from_edge = LaunchConfiguration('push_from_edge')
    push_state_dim = LaunchConfiguration('push_state_dim')
    push_action_dim = LaunchConfiguration('push_action_dim')
    revolute_model_dir = LaunchConfiguration('revolute_model_dir')
    revolute_model_name = LaunchConfiguration('revolute_model_name')
    flex_revolute_flow = LaunchConfiguration('flex_revolute_flow')
    revolute_probe_step_m = LaunchConfiguration('revolute_probe_step_m')
    revolute_probe_max_steps = LaunchConfiguration('revolute_probe_max_steps')
    revolute_probe_min_steps = LaunchConfiguration('revolute_probe_min_steps')
    revolute_probe_phi_max_rad = LaunchConfiguration('revolute_probe_phi_max_rad')
    revolute_probe_phi_min_rad = LaunchConfiguration('revolute_probe_phi_min_rad')
    revolute_probe_confidence_thresh = LaunchConfiguration('revolute_probe_confidence_thresh')
    revolute_probe_initial_dir_x = LaunchConfiguration('revolute_probe_initial_dir_x')
    revolute_probe_initial_dir_y = LaunchConfiguration('revolute_probe_initial_dir_y')
    revolute_probe_initial_dir_frame = LaunchConfiguration('revolute_probe_initial_dir_frame')
    revolute_probe_settle_sec = LaunchConfiguration('revolute_probe_settle_sec')
    revolute_force_revolute = LaunchConfiguration('revolute_force_revolute')
    revolute_success_angle_deg = LaunchConfiguration('revolute_success_angle_deg')
    revolute_accept_angle_deg = LaunchConfiguration('revolute_accept_angle_deg')
    revolute_arc_points = LaunchConfiguration('revolute_arc_points')
    revolute_arc_direction = LaunchConfiguration('revolute_arc_direction')
    revolute_invert_action_x = LaunchConfiguration('revolute_invert_action_x')
    flex_place_flow = LaunchConfiguration('flex_place_flow')
    place_pre_trigger = LaunchConfiguration('place_pre_trigger')
    place_height_m = LaunchConfiguration('place_height_m')
    place_forward_m = LaunchConfiguration('place_forward_m')
    place_hand_forward_m = LaunchConfiguration('place_hand_forward_m')
    place_forward_duration_sec = LaunchConfiguration('place_forward_duration_sec')
    place_body_approach_mode = LaunchConfiguration('place_body_approach_mode')
    place_trajectory_action = LaunchConfiguration('place_trajectory_action')
    place_disable_obstacle_avoidance = LaunchConfiguration('place_disable_obstacle_avoidance')
    place_lower_m = LaunchConfiguration('place_lower_m')
    place_impedance_settle_sec = LaunchConfiguration('place_impedance_settle_sec')
    place_release_settle_sec = LaunchConfiguration('place_release_settle_sec')
    prismatic_model_dir = LaunchConfiguration('prismatic_model_dir')
    prismatic_model_name = LaunchConfiguration('prismatic_model_name')
    launch_spot_driver = LaunchConfiguration('launch_spot_driver')
    spot_name = LaunchConfiguration('spot_name')
    spot_config_file = LaunchConfiguration('spot_config_file')
    spot_controllable = LaunchConfiguration('spot_controllable')
    graphnav_init = LaunchConfiguration('graphnav_init')
    graphnav_map_path = LaunchConfiguration('graphnav_map_path')
    graphnav_localization_method = LaunchConfiguration('graphnav_localization_method')
    graphnav_localization_waypoint = LaunchConfiguration('graphnav_localization_waypoint')
    graphnav_upload_graph_service = LaunchConfiguration('graphnav_upload_graph_service')
    graphnav_list_graph_service = LaunchConfiguration('graphnav_list_graph_service')
    graphnav_set_localization_service = LaunchConfiguration('graphnav_set_localization_service')
    use_mock_perception = LaunchConfiguration('use_mock_perception')
    perception_use_test_images = LaunchConfiguration('perception_use_test_images')
    perception_rgb_topic = LaunchConfiguration('perception_rgb_topic')
    perception_open_gripper_service = LaunchConfiguration('perception_open_gripper_service')
    perception_open_gripper_before_box_image = LaunchConfiguration(
        'perception_open_gripper_before_box_image'
    )
    perception_require_box_gripper_open = LaunchConfiguration(
        'perception_require_box_gripper_open'
    )
    perception_open_gripper_before_handle_image = LaunchConfiguration(
        'perception_open_gripper_before_handle_image'
    )
    perception_require_handle_gripper_open = LaunchConfiguration(
        'perception_require_handle_gripper_open'
    )
    perception_open_gripper_before_object_image = LaunchConfiguration(
        'perception_open_gripper_before_object_image'
    )
    perception_require_object_gripper_open = LaunchConfiguration(
        'perception_require_object_gripper_open'
    )
    perception_grasp_object_after_detection = LaunchConfiguration(
        'perception_grasp_object_after_detection'
    )
    perception_gripper_settle_sec = LaunchConfiguration('perception_gripper_settle_sec')
    perception_box_detection_retry_count = LaunchConfiguration(
        'perception_box_detection_retry_count'
    )
    perception_grasp_box_after_detection = LaunchConfiguration(
        'perception_grasp_box_after_detection'
    )
    perception_grasp_handle_after_detection = LaunchConfiguration(
        'perception_grasp_handle_after_detection'
    )
    perception_grasp_pixel_service = LaunchConfiguration('perception_grasp_pixel_service')
    perception_grasp_image_source = LaunchConfiguration('perception_grasp_image_source')
    perception_save_debug_images = LaunchConfiguration('perception_save_debug_images')
    perception_debug_image_dir = LaunchConfiguration('perception_debug_image_dir')
    use_mock_spot_services = LaunchConfiguration('use_mock_spot_services')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_waypoints,
            description='Demo locations YAML. GraphNav mode reads graphnav_waypoint fields.',
        ),
        DeclareLaunchArgument(
            'box_grasp_side',
            default_value='right',
            description='Box edge to grasp for pushing: left or right.',
        ),
        DeclareLaunchArgument(
            'nav_backend',
            default_value='graphnav',
            description='spot_flex_control nav backend: graphnav, trajectory, or nav2.',
        ),
        DeclareLaunchArgument(
            'policy_dry_run',
            default_value='false',
            description='Keep ExecutePolicy as a no-motion shim until hardware motions are validated.',
        ),
        DeclareLaunchArgument(
            'policy_use_learned',
            default_value='true',
            description='Load learned FLEX actor weights from model_cache.',
        ),
        DeclareLaunchArgument(
            'push_model_dir',
            default_value='/repo/workspace/model_cache/flex/push',
            description='Directory containing push_actor.pth.',
        ),
        DeclareLaunchArgument(
            'push_model_name',
            default_value='push',
            description='Actor name prefix for box pushing.',
        ),
        DeclareLaunchArgument(
            'push_policy_max_steps',
            default_value='30',
            description='Maximum iterative FLEX push steps.',
        ),
        DeclareLaunchArgument(
            'push_policy_step_duration_sec',
            default_value='2.0',
            description='Duration of each iterative FLEX push step.',
        ),
        DeclareLaunchArgument(
            'push_action_scale',
            default_value='0.75',
            description='Scales learned force action before impedance displacement conversion.',
        ),
        DeclareLaunchArgument(
            'push_max_step_m',
            default_value='1.0',
            description='Clamp each FLEX push displacement component in metres.',
        ),
        DeclareLaunchArgument(
            'push_probe_force',
            default_value='true',
            description='Probe reactive force before running the push policy.',
        ),
        DeclareLaunchArgument(
            'push_impedance_two_phase',
            default_value='true',
            description='Use arm impedance push followed by body catch-up for each push step.',
        ),
        DeclareLaunchArgument(
            'push_success_progress',
            default_value='0.95',
            description='Path progress required for box push success.',
        ),
        DeclareLaunchArgument(
            'push_deviation_tolerance',
            default_value='1.0',
            description='Abort box push if path deviation exceeds this many metres.',
        ),
        DeclareLaunchArgument(
            'push_success_distance',
            default_value='0.8',
            description='Maximum path deviation allowed when declaring box push success.',
        ),
        DeclareLaunchArgument(
            'push_yaw_scale',
            default_value='0.05',
            description='Scale for yaw correction during box push.',
        ),
        DeclareLaunchArgument(
            'push_settle_between_steps',
            default_value='true',
            description='Hold impedance contact between push policy steps.',
        ),
        DeclareLaunchArgument(
            'push_settle_duration_sec',
            default_value='3.0',
            description='Impedance settle duration between push policy steps.',
        ),
        DeclareLaunchArgument(
            'push_path_type',
            default_value='arc',
            description='Push path primitive: arc, straight, s_curve, meander, or triple_s.',
        ),
        DeclareLaunchArgument(
            'push_arc_radius',
            default_value='1.5',
            description='Arc radius for the default box push path.',
        ),
        DeclareLaunchArgument(
            'push_arc_angle_deg',
            default_value='60.0',
            description='Arc sweep in degrees for the default box push path.',
        ),
        DeclareLaunchArgument(
            'push_length',
            default_value='1.5',
            description='Path length for non-arc box push paths.',
        ),
        DeclareLaunchArgument(
            'push_amplitude',
            default_value='0.5',
            description='Lateral amplitude for curved non-arc box push paths.',
        ),
        DeclareLaunchArgument(
            'push_use_box_center',
            default_value='false',
            description='Use estimated box center instead of EEF pose for push state/progress.',
        ),
        DeclareLaunchArgument(
            'push_box_width',
            default_value='0.465',
            description='Box width used when push_use_box_center is true.',
        ),
        DeclareLaunchArgument(
            'push_box_depth',
            default_value='0.61',
            description='Box depth used when push_use_box_center is true.',
        ),
        DeclareLaunchArgument(
            'push_box_height',
            default_value='0.63',
            description='Box height used when push_use_box_center is true.',
        ),
        DeclareLaunchArgument(
            'push_surface_type',
            default_value='floor',
            description='Surface type for reactive force calibration.',
        ),
        DeclareLaunchArgument(
            'push_max_force',
            default_value='200.0',
            description='Maximum force scale used by the push policy.',
        ),
        DeclareLaunchArgument(
            'push_use_impedance',
            default_value='true',
            description='Use impedance-based arm push execution.',
        ),
        DeclareLaunchArgument(
            'push_from_edge',
            default_value='false',
            description='Policy metadata from flex_spot; actor dimensions are inferred from weights.',
        ),
        DeclareLaunchArgument(
            'push_state_dim',
            default_value='6',
            description='Policy metadata from flex_spot for the push actor state size.',
        ),
        DeclareLaunchArgument(
            'push_action_dim',
            default_value='2',
            description='Policy metadata from flex_spot for the push actor action size.',
        ),
        DeclareLaunchArgument(
            'revolute_model_dir',
            default_value='/repo/workspace/model_cache/flex/revolute',
            description='Directory containing revolute_actor.pth.',
        ),
        DeclareLaunchArgument(
            'revolute_model_name',
            default_value='revolute',
            description='Actor name prefix for revolute opening.',
        ),
        DeclareLaunchArgument(
            'flex_revolute_flow',
            default_value='true',
            description='Use the wiggle-probe + arc path-following flow for open_cabinet.',
        ),
        DeclareLaunchArgument(
            'revolute_probe_step_m',
            default_value='0.04',
            description='Probe step length per iteration (meters).',
        ),
        DeclareLaunchArgument(
            'revolute_probe_max_steps',
            default_value='30',
            description='Maximum iterative probe/opening steps before giving up.',
        ),
        DeclareLaunchArgument(
            'revolute_probe_min_steps',
            default_value='5',
            description='Minimum samples before fit_model_2d is consulted.',
        ),
        DeclareLaunchArgument(
            'revolute_probe_phi_max_rad',
            default_value='0.7853981633974483',
            description='Max heading rotation per low-confidence probe step (radians, default pi/4).',
        ),
        DeclareLaunchArgument(
            'revolute_probe_phi_min_rad',
            default_value='0.0',
            description='Min heading rotation threshold (radians).',
        ),
        DeclareLaunchArgument(
            'revolute_probe_confidence_thresh',
            default_value='0.8',
            description='2D model confidence required to follow line/circle.',
        ),
        DeclareLaunchArgument(
            'revolute_probe_initial_dir_x',
            default_value='-1.0',
            description='Initial probe direction x-component in revolute_probe_initial_dir_frame.',
        ),
        DeclareLaunchArgument(
            'revolute_probe_initial_dir_y',
            default_value='0.0',
            description='Initial probe direction y-component in revolute_probe_initial_dir_frame.',
        ),
        DeclareLaunchArgument(
            'revolute_probe_initial_dir_frame',
            default_value='body',
            description="Frame for initial probe direction: 'body' or 'vision'. Default body (-X pulls toward Spot).",
        ),
        DeclareLaunchArgument(
            'revolute_probe_settle_sec',
            default_value='2.0',
            description='Settle time after each probe arm_pose command before reading achieved pose.',
        ),
        DeclareLaunchArgument(
            'revolute_force_revolute',
            default_value='false',
            description='Skip joint-type discrimination and force a revolute fit.',
        ),
        DeclareLaunchArgument(
            'revolute_success_angle_deg',
            default_value='75.0',
            description='Preferred arc sweep target angle for opening the revolute joint.',
        ),
        DeclareLaunchArgument(
            'revolute_accept_angle_deg',
            default_value='60.0',
            description='Minimum arc sweep angle accepted as successful if the preferred target is not reached.',
        ),
        DeclareLaunchArgument(
            'revolute_arc_points',
            default_value='60',
            description='Number of waypoints along the revolute arc path.',
        ),
        DeclareLaunchArgument(
            'revolute_arc_direction',
            default_value='negative',
            description="Arc sweep direction: 'positive' (CCW) or 'negative' (CW).",
        ),
        DeclareLaunchArgument(
            'revolute_invert_action_x',
            default_value='true',
            description='Invert action[0] sign (matches flex_spot door_open convention).',
        ),
        DeclareLaunchArgument(
            'flex_place_flow',
            default_value='true',
            description='Use the table drop-off sequence for place instead of a bare open_gripper.',
        ),
        DeclareLaunchArgument(
            'place_pre_trigger',
            default_value='',
            description='Optional trigger service to call before table drop-off arm motion. Empty preserves the current grasp pose.',
        ),
        DeclareLaunchArgument(
            'place_height_m',
            default_value='0.762',
            description='Hand z target before drop-off, in meters (30 inches).',
        ),
        DeclareLaunchArgument(
            'place_forward_m',
            default_value='0.5',
            description='Forward body motion at the table before release, in meters.',
        ),
        DeclareLaunchArgument(
            'place_hand_forward_m',
            default_value='0.50',
            description='Forward hand motion in the body frame after the body approach, in meters.',
        ),
        DeclareLaunchArgument(
            'place_forward_duration_sec',
            default_value='4.0',
            description='Duration for the forward body motion during table drop-off.',
        ),
        DeclareLaunchArgument(
            'place_body_approach_mode',
            default_value='trajectory',
            description="Body approach command mode for place: 'trajectory' or 'cmd_vel'.",
        ),
        DeclareLaunchArgument(
            'place_trajectory_action',
            default_value='/spot/trajectory',
            description='Spot trajectory action used for the body-relative table approach.',
        ),
        DeclareLaunchArgument(
            'place_disable_obstacle_avoidance',
            default_value='true',
            description='Disable obstacle avoidance for the short table approach trajectory.',
        ),
        DeclareLaunchArgument(
            'place_lower_m',
            default_value='0.23',
            description='Downward hand motion before release, in meters.',
        ),
        DeclareLaunchArgument(
            'place_impedance_settle_sec',
            default_value='2.0',
            description='Gentle impedance settle duration before opening the gripper.',
        ),
        DeclareLaunchArgument(
            'place_release_settle_sec',
            default_value='0.5',
            description='Pause after opening the gripper before stowing the arm.',
        ),
        DeclareLaunchArgument(
            'prismatic_model_dir',
            default_value='/repo/workspace/model_cache/flex/prismatic',
            description='Directory containing prismatic_actor.pth.',
        ),
        DeclareLaunchArgument(
            'prismatic_model_name',
            default_value='prismatic',
            description='Actor name prefix for prismatic operation.',
        ),
        DeclareLaunchArgument(
            'launch_spot_driver',
            default_value='false',
            description='Include spot_driver/spot_driver.launch.py.',
        ),
        DeclareLaunchArgument(
            'spot_name',
            default_value='spot',
            description='Spot driver namespace. launch_spot.sh uses "spot".',
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
            'graphnav_init',
            default_value='false',
            description=(
                'Upload GraphNav map and set initial localization during demo launch. '
                'Default false because launch_spot.sh owns one-shot GraphNav setup.'
            ),
        ),
        DeclareLaunchArgument(
            'graphnav_map_path',
            default_value='/repo/workspace/maps/demo.walk',
            description='GraphNav .walk directory to upload.',
        ),
        DeclareLaunchArgument(
            'graphnav_localization_method',
            default_value='fiducial',
            description='GraphNav localization method: fiducial or waypoint.',
        ),
        DeclareLaunchArgument(
            'graphnav_localization_waypoint',
            default_value='',
            description='Waypoint id when graphnav_localization_method:=waypoint.',
        ),
        DeclareLaunchArgument(
            'graphnav_upload_graph_service',
            default_value='/spot/graph_nav_upload_graph',
            description='GraphNav upload service name.',
        ),
        DeclareLaunchArgument(
            'graphnav_list_graph_service',
            default_value='/spot/list_graph',
            description='GraphNav list_graph service used to refresh waypoint name lookup.',
        ),
        DeclareLaunchArgument(
            'graphnav_set_localization_service',
            default_value='/spot/graph_nav_set_localization',
            description='GraphNav set localization service name.',
        ),
        DeclareLaunchArgument(
            'use_mock_perception',
            default_value='false',
            description='Use mock perception while validating hardware nav/control plumbing.',
        ),
        DeclareLaunchArgument(
            'perception_use_test_images',
            default_value='false',
            description='Use static test images in the real perception server.',
        ),
        DeclareLaunchArgument(
            'perception_rgb_topic',
            default_value='/spot/camera/hand/image',
            description='RGB image topic consumed by the real perception server.',
        ),
        DeclareLaunchArgument(
            'perception_open_gripper_service',
            default_value='/spot/open_gripper',
            description='Trigger service used before box hand-camera perception.',
        ),
        DeclareLaunchArgument(
            'perception_open_gripper_before_box_image',
            default_value='true',
            description='Open the gripper before capturing the box image.',
        ),
        DeclareLaunchArgument(
            'perception_require_box_gripper_open',
            default_value='true',
            description='Abort find_box_grasp_point if the gripper cannot be opened.',
        ),
        DeclareLaunchArgument(
            'perception_open_gripper_before_handle_image',
            default_value='true',
            description='Open the gripper before capturing the cabinet handle image.',
        ),
        DeclareLaunchArgument(
            'perception_require_handle_gripper_open',
            default_value='false',
            description='Abort find_cabinet_handle if the gripper cannot be opened.',
        ),
        DeclareLaunchArgument(
            'perception_open_gripper_before_object_image',
            default_value='true',
            description='Open the gripper before capturing the in-cabinet object image.',
        ),
        DeclareLaunchArgument(
            'perception_require_object_gripper_open',
            default_value='false',
            description='Abort find_object if the gripper cannot be opened.',
        ),
        DeclareLaunchArgument(
            'perception_grasp_object_after_detection',
            default_value='true',
            description='Run Spot grasp_pixel after detecting the object via OWL.',
        ),
        DeclareLaunchArgument(
            'perception_gripper_settle_sec',
            default_value='1.5',
            description='Delay after opening gripper before using a hand-camera image.',
        ),
        DeclareLaunchArgument(
            'perception_box_detection_retry_count',
            default_value='2',
            description='Number of box image/detection attempts before aborting.',
        ),
        DeclareLaunchArgument(
            'perception_grasp_box_after_detection',
            default_value='true',
            description='Run Spot manipulation grasp after finding the box edge pixel.',
        ),
        DeclareLaunchArgument(
            'perception_grasp_handle_after_detection',
            default_value='true',
            description='Run Spot manipulation grasp after finding the cabinet handle pixel.',
        ),
        DeclareLaunchArgument(
            'perception_grasp_pixel_service',
            default_value='/spot/grasp_pixel',
            description='Spot driver service that runs SDK PickObjectInImage for a hand-camera pixel.',
        ),
        DeclareLaunchArgument(
            'perception_grasp_image_source',
            default_value='hand_color_image',
            description='Spot SDK image source used for the pixel grasp request.',
        ),
        DeclareLaunchArgument(
            'perception_save_debug_images',
            default_value='true',
            description='Save raw and overlay box perception images.',
        ),
        DeclareLaunchArgument(
            'perception_debug_image_dir',
            default_value='/repo/workspace/perception_debug',
            description='Directory for saved perception debug images.',
        ),
        DeclareLaunchArgument(
            'use_mock_spot_services',
            default_value='false',
            description='Use mock dock/undock/arm services if spot_driver is not running.',
        ),
        SetEnvironmentVariable(
            'HF_HOME',
            '/repo/workspace/model_cache/huggingface',
        ),
        SetEnvironmentVariable(
            'HF_HUB_CACHE',
            '/repo/workspace/model_cache/huggingface/hub',
        ),
        SetEnvironmentVariable(
            'TRANSFORMERS_CACHE',
            '/repo/workspace/model_cache/huggingface/hub',
        ),
        SetEnvironmentVariable(
            'HF_HUB_OFFLINE',
            '1',
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
                'spot_name': spot_name,
            }.items(),
            condition=IfCondition(launch_spot_driver),
        ),

        Node(
            package='spot_flex_plan',
            executable='conductor_node',
            name='conductor_node',
            output='screen',
            parameters=[{
                'waypoints_file': waypoints_file,
                'box_grasp_side': box_grasp_side,
            }],
            remappings=[
                ('undock', '/spot/undock'),
                ('dock', '/spot/dock'),
                ('arm_carry', '/spot/arm_carry'),
                ('arm_stow', '/spot/arm_stow'),
                ('open_gripper', '/spot/open_gripper'),
                ('close_gripper', '/spot/close_gripper'),
                ('set_gripper_angle', '/spot/set_gripper_angle'),
            ],
        ),
        Node(
            package='spot_flex_control',
            executable='graphnav_initializer_node',
            name='graphnav_initializer_node',
            output='screen',
            parameters=[{
                'upload_graph': True,
                'refresh_waypoint_names': True,
                'map_path': graphnav_map_path,
                'set_localization': True,
                'localization_method': graphnav_localization_method,
                'localization_waypoint': graphnav_localization_waypoint,
                'upload_graph_service': graphnav_upload_graph_service,
                'list_graph_service': graphnav_list_graph_service,
                'set_localization_service': graphnav_set_localization_service,
            }],
            condition=IfCondition(graphnav_init),
        ),
        Node(
            package='spot_flex_control',
            executable='nav_node',
            name='nav_node',
            output='screen',
            parameters=[{
                'backend': nav_backend,
                'waypoints_file': waypoints_file,
                'trajectory_action': '/spot/trajectory',
                'graphnav_action': '/spot/navigate_to',
            }],
        ),
        Node(
            package='spot_flex_control',
            executable='policy_server_node',
            name='policy_server_node',
            output='screen',
            parameters=[{
                'dry_run': policy_dry_run,
                'use_learned_policies': policy_use_learned,
                'push_model_dir': push_model_dir,
                'push_model_name': push_model_name,
                'policy_max_steps': push_policy_max_steps,
                'policy_step_duration_sec': push_policy_step_duration_sec,
                'action_scale': push_action_scale,
                'push_max_step_m': push_max_step_m,
                'push_probe_force': push_probe_force,
                'push_impedance_two_phase': push_impedance_two_phase,
                'push_robot_side': box_grasp_side,
                'push_success_progress': push_success_progress,
                'push_deviation_tolerance': push_deviation_tolerance,
                'push_success_distance': push_success_distance,
                'push_yaw_scale': push_yaw_scale,
                'push_settle_between_steps': push_settle_between_steps,
                'push_settle_duration_sec': push_settle_duration_sec,
                'push_path_type': push_path_type,
                'push_arc_radius': push_arc_radius,
                'push_arc_angle_deg': push_arc_angle_deg,
                'push_length': push_length,
                'push_amplitude': push_amplitude,
                'push_use_box_center': push_use_box_center,
                'push_box_width': push_box_width,
                'push_box_depth': push_box_depth,
                'push_box_height': push_box_height,
                'push_surface_type': push_surface_type,
                'push_max_force': push_max_force,
                'push_use_impedance': push_use_impedance,
                'push_from_edge': push_from_edge,
                'push_state_dim': push_state_dim,
                'push_action_dim': push_action_dim,
                'revolute_model_dir': revolute_model_dir,
                'revolute_model_name': revolute_model_name,
                'flex_revolute_flow': flex_revolute_flow,
                'revolute_probe_step_m': revolute_probe_step_m,
                'revolute_probe_max_steps': revolute_probe_max_steps,
                'revolute_probe_min_steps': revolute_probe_min_steps,
                'revolute_probe_phi_max_rad': revolute_probe_phi_max_rad,
                'revolute_probe_phi_min_rad': revolute_probe_phi_min_rad,
                'revolute_probe_confidence_thresh': revolute_probe_confidence_thresh,
                'revolute_probe_initial_dir_x': revolute_probe_initial_dir_x,
                'revolute_probe_initial_dir_y': revolute_probe_initial_dir_y,
                'revolute_probe_initial_dir_frame': revolute_probe_initial_dir_frame,
                'revolute_probe_settle_sec': revolute_probe_settle_sec,
                'revolute_force_revolute': revolute_force_revolute,
                'revolute_success_angle_deg': revolute_success_angle_deg,
                'revolute_accept_angle_deg': revolute_accept_angle_deg,
                'revolute_arc_points': revolute_arc_points,
                'revolute_arc_direction': revolute_arc_direction,
                'revolute_invert_action_x': revolute_invert_action_x,
                'flex_place_flow': flex_place_flow,
                'place_pre_trigger': place_pre_trigger,
                'place_height_m': place_height_m,
                'place_forward_m': place_forward_m,
                'place_hand_forward_m': place_hand_forward_m,
                'place_forward_duration_sec': place_forward_duration_sec,
                'place_body_approach_mode': place_body_approach_mode,
                'place_trajectory_action': place_trajectory_action,
                'place_disable_obstacle_avoidance': place_disable_obstacle_avoidance,
                'place_lower_m': place_lower_m,
                'place_impedance_settle_sec': place_impedance_settle_sec,
                'place_release_settle_sec': place_release_settle_sec,
                'prismatic_model_dir': prismatic_model_dir,
                'prismatic_model_name': prismatic_model_name,
                'cmd_vel_topic': '/spot/cmd_vel',
                'trigger_service_prefix': '/spot',
            }],
        ),
        Node(
            package='spot_flex_mocks',
            executable='mock_perception_server',
            name='mock_perception_server',
            output='screen',
            condition=IfCondition(use_mock_perception),
        ),
        Node(
            package='spot_flex_perception',
            executable='perception_server_node',
            name='perception_server_node',
            output='screen',
            parameters=[{
                'use_test_images': perception_use_test_images,
                'rgb_topic': perception_rgb_topic,
                'open_gripper_service': perception_open_gripper_service,
                'open_gripper_before_box_image': perception_open_gripper_before_box_image,
                'require_box_gripper_open': perception_require_box_gripper_open,
                'open_gripper_before_handle_image': perception_open_gripper_before_handle_image,
                'require_handle_gripper_open': perception_require_handle_gripper_open,
                'open_gripper_before_object_image': perception_open_gripper_before_object_image,
                'require_object_gripper_open': perception_require_object_gripper_open,
                'grasp_object_after_detection': perception_grasp_object_after_detection,
                'gripper_settle_sec': perception_gripper_settle_sec,
                'box_detection_retry_count': perception_box_detection_retry_count,
                'grasp_box_after_detection': perception_grasp_box_after_detection,
                'grasp_handle_after_detection': perception_grasp_handle_after_detection,
                'grasp_pixel_service': perception_grasp_pixel_service,
                'grasp_image_source': perception_grasp_image_source,
                'save_debug_images': perception_save_debug_images,
                'debug_image_dir': perception_debug_image_dir,
            }],
            condition=UnlessCondition(use_mock_perception),
        ),
        Node(
            package='spot_flex_mocks',
            executable='mock_spot_services',
            name='mock_spot_services',
            output='screen',
            condition=IfCondition(use_mock_spot_services),
        ),
    ])
