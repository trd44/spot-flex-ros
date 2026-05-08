"""
Arm control node backed by MoveIt.
"""

import threading

import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import BoundingVolume, OrientationConstraint, PositionConstraint
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from spot_msgs.srv import SetGripperAngle
from spot_flex_msgs.srv import GraspPixel
from shape_msgs.msg import SolidPrimitive

ARM_GROUP = 'spot_arm'
GRIPPER_GROUP = 'spot_gripper'

ARM_NAMED_STATES = {
    'stowed': {
        'arm_sh0': 0.0,
        'arm_sh1': -3.13,
        'arm_el0': 3.13,
        'arm_el1': 0.0,
        'arm_wr0': 1.57,
        'arm_wr1': 0.0,
    },
    'ready': {
        'arm_sh0': 0.0,
        'arm_sh1': -0.9,
        'arm_el0': 1.8,
        'arm_el1': 0.0,
        'arm_wr0': -0.9,
        'arm_wr1': 0.0,
    },
}

GRIPPER_NAMED_STATES = {
    'open': {'arm_f1x': -1.57},
    'closed': {'arm_f1x': 0.0},
}


class ArmNode(Node):

    def __init__(self):
        super().__init__('arm_node')
        self.cb_group = ReentrantCallbackGroup()
        self._camera_info: CameraInfo | None = None
        self._last_depth: Image | None = None

        self.declare_parameter('enable_grasp_pixel_service', True)
        self.declare_parameter('grasp_depth_topic', '/spot/depth_registered/hand/image')
        self.declare_parameter('grasp_camera_info_topic', '/spot/depth_registered/hand/camera_info')
        self.declare_parameter('grasp_target_frame', '')
        self.declare_parameter('grasp_depth_window_px', 7)
        self.declare_parameter('grasp_depth_scale_16uc1', 0.001)
        self.declare_parameter('grasp_min_depth_m', 0.05)
        self.declare_parameter('grasp_max_depth_m', 2.0)
        self.declare_parameter('grasp_approach_offset_m', 0.08)
        self.declare_parameter('grasp_open_before_move', False)
        self.declare_parameter('grasp_close_after_move', True)
        self.declare_parameter('grasp_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])

        self.get_logger().info('Connecting to MoveIt move_group...')
        self._move_group = ActionClient(self, MoveGroup, '/move_action', callback_group=self.cb_group)

        self.create_service(Trigger, 'open_gripper',  self._open_gripper_cb,  callback_group=self.cb_group)
        self.create_service(Trigger, 'close_gripper', self._close_gripper_cb, callback_group=self.cb_group)
        self.create_service(SetGripperAngle, 'set_gripper_angle', self._set_gripper_angle_cb,
                            callback_group=self.cb_group)
        self.create_service(Trigger, 'arm_stow',      self._stow_cb,          callback_group=self.cb_group)
        self.create_service(Trigger, 'arm_unstow',    self._unstow_cb,        callback_group=self.cb_group)
        self.create_service(Trigger, 'arm_carry',     self._carry_cb,         callback_group=self.cb_group)
        self.create_subscription(PoseStamped, 'pose_goal', self._pose_goal_cb, 10,
                                 callback_group=self.cb_group)
        if self.get_parameter('enable_grasp_pixel_service').value:
            depth_topic = str(self.get_parameter('grasp_depth_topic').value)
            info_topic = str(self.get_parameter('grasp_camera_info_topic').value)
            self.create_subscription(
                Image, depth_topic, self._depth_cb, qos_profile_sensor_data,
                callback_group=self.cb_group,
            )
            self.create_subscription(
                CameraInfo, info_topic, self._camera_info_cb, qos_profile_sensor_data,
                callback_group=self.cb_group,
            )
            self.create_service(
                GraspPixel, 'grasp_pixel', self._grasp_pixel_cb,
                callback_group=self.cb_group,
            )
            self.get_logger().info(
                f'MoveIt pixel grasp bridge ready: {depth_topic} + {info_topic} -> grasp_pixel'
            )
        self.get_logger().info('MoveIt service bridge ready.')

    def _send_move_group_goal(self, goal: MoveGroup.Goal, goal_name: str) -> tuple[bool, str]:
        if not self._move_group.wait_for_server(timeout_sec=10.0):
            return False, 'move_group action server unavailable'

        send_future = self._move_group.send_goal_async(goal)
        send_done = threading.Event()
        send_future.add_done_callback(lambda _future: send_done.set())
        if not send_done.wait(timeout=10.0):
            return False, f'Timed out sending MoveIt goal {goal_name}'

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, f'MoveIt rejected goal {goal_name}'

        result_future = goal_handle.get_result_async()
        result_done = threading.Event()
        result_future.add_done_callback(lambda _future: result_done.set())
        if not result_done.wait(timeout=30.0):
            return False, f'Timed out executing MoveIt goal {goal_name}'

        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, f'MoveIt failed {goal_name}: error_code={result.error_code.val}'

        self.get_logger().info(f'Executed MoveIt goal: {goal_name}')
        return True, f'Moved to {goal_name}'

    def _base_move_group_goal(self, group_name: str) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def _move_group_named_state(self, group_name: str, goal_name: str, joints: dict[str, float]) -> tuple[bool, str]:
        """Plan and execute a named joint state through move_group."""
        constraints = Constraints(name=goal_name)
        for joint_name, position in joints.items():
            constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=joint_name,
                    position=float(position),
                    tolerance_above=0.02,
                    tolerance_below=0.02,
                    weight=1.0,
                )
            )

        goal = self._base_move_group_goal(group_name)
        goal.request.goal_constraints.append(constraints)
        return self._send_move_group_goal(goal, goal_name)

    def move_to_pose(self, pose: PoseStamped) -> tuple[bool, str]:
        """Plan and execute to an end-effector pose in the pose header frame."""
        frame_id = pose.header.frame_id or 'body'

        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.03])
        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(pose.pose)

        constraints = Constraints(name='pose_goal')
        constraints.position_constraints.append(
            PositionConstraint(
                header=pose.header,
                link_name='arm_link_wr1',
                constraint_region=region,
                weight=1.0,
            )
        )
        constraints.orientation_constraints.append(
            OrientationConstraint(
                header=pose.header,
                link_name='arm_link_wr1',
                orientation=pose.pose.orientation,
                absolute_x_axis_tolerance=0.15,
                absolute_y_axis_tolerance=0.15,
                absolute_z_axis_tolerance=0.15,
                weight=1.0,
            )
        )

        goal = self._base_move_group_goal(ARM_GROUP)
        goal.request.goal_constraints.append(constraints)
        success, message = self._send_move_group_goal(goal, f'pose_goal in {frame_id}')
        return success, message

    def _pose_goal_cb(self, pose: PoseStamped):
        success, message = self.move_to_pose(pose)
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _depth_cb(self, msg: Image) -> None:
        self._last_depth = msg

    def _image_to_depth_array(self, msg: Image) -> np.ndarray | None:
        encoding = msg.encoding.lower()
        if encoding in ('16uc1', 'mono16'):
            dtype = np.uint16
            scale = float(self.get_parameter('grasp_depth_scale_16uc1').value)
        elif encoding == '32fc1':
            dtype = np.float32
            scale = 1.0
        else:
            self.get_logger().warning(
                f'Unsupported pixel-grasp depth encoding: {msg.encoding}',
                throttle_duration_sec=5.0,
            )
            return None

        itemsize = np.dtype(dtype).itemsize
        row_width = msg.step // itemsize
        try:
            raw = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, row_width))
        except ValueError as exc:
            self.get_logger().warning(
                f'Could not reshape pixel-grasp depth image: {exc}',
                throttle_duration_sec=5.0,
            )
            return None

        depth = raw[:, :msg.width].astype(np.float32) * scale
        depth[depth <= 0.0] = np.nan
        return depth

    def _depth_at_pixel(self, pixel_x: int, pixel_y: int) -> tuple[float | None, str]:
        depth_msg = self._last_depth
        if depth_msg is None:
            return None, 'No depth image received for MoveIt pixel grasp'

        depth = self._image_to_depth_array(depth_msg)
        if depth is None:
            return None, 'Could not decode depth image for MoveIt pixel grasp'

        if pixel_x < 0 or pixel_y < 0 or pixel_x >= depth_msg.width or pixel_y >= depth_msg.height:
            return None, f'Pixel ({pixel_x}, {pixel_y}) is outside depth image {depth_msg.width}x{depth_msg.height}'

        window = max(1, int(self.get_parameter('grasp_depth_window_px').value))
        radius = window // 2
        x0 = max(0, pixel_x - radius)
        x1 = min(depth_msg.width, pixel_x + radius + 1)
        y0 = max(0, pixel_y - radius)
        y1 = min(depth_msg.height, pixel_y + radius + 1)

        patch = depth[y0:y1, x0:x1]
        min_depth = float(self.get_parameter('grasp_min_depth_m').value)
        max_depth = float(self.get_parameter('grasp_max_depth_m').value)
        valid = patch[np.isfinite(patch) & (patch >= min_depth) & (patch <= max_depth)]
        if valid.size == 0:
            return None, f'No valid depth near pixel ({pixel_x}, {pixel_y})'
        return float(np.median(valid)), 'ok'

    def _pose_from_pixel(self, pixel_x: int, pixel_y: int) -> tuple[PoseStamped | None, str]:
        info = self._camera_info
        depth_msg = self._last_depth
        if info is None:
            return None, 'No camera_info received for MoveIt pixel grasp'
        if depth_msg is None:
            return None, 'No depth image received for MoveIt pixel grasp'

        depth_m, message = self._depth_at_pixel(pixel_x, pixel_y)
        if depth_m is None:
            return None, message

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None, 'camera_info has invalid focal length'

        approach_offset = max(0.0, float(self.get_parameter('grasp_approach_offset_m').value))
        target_depth = max(
            float(self.get_parameter('grasp_min_depth_m').value),
            depth_m - approach_offset,
        )

        pose = PoseStamped()
        target_frame = str(self.get_parameter('grasp_target_frame').value).strip()
        pose.header.frame_id = target_frame or depth_msg.header.frame_id or info.header.frame_id
        pose.pose.position.x = (float(pixel_x) - cx) * target_depth / fx
        pose.pose.position.y = (float(pixel_y) - cy) * target_depth / fy
        pose.pose.position.z = target_depth

        orientation = list(self.get_parameter('grasp_orientation_xyzw').value)
        if len(orientation) != 4:
            return None, 'grasp_orientation_xyzw must contain four values'
        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])
        return pose, f'projected pixel depth={depth_m:.3f}m target_depth={target_depth:.3f}m'

    def _grasp_pixel_cb(self, request, response):
        pose, message = self._pose_from_pixel(int(request.pixel_x), int(request.pixel_y))
        if pose is None:
            response.success = False
            response.message = message
            return response

        if self.get_parameter('grasp_open_before_move').value:
            success, msg = self._move_group_named_state(GRIPPER_GROUP, 'open', GRIPPER_NAMED_STATES['open'])
            if not success:
                response.success = False
                response.message = f'open_gripper failed before pixel grasp: {msg}'
                return response

        success, msg = self.move_to_pose(pose)
        if not success:
            response.success = False
            response.message = f'MoveIt pixel grasp pose failed: {msg}; {message}'
            return response

        if self.get_parameter('grasp_close_after_move').value:
            success, msg = self._move_group_named_state(GRIPPER_GROUP, 'closed', GRIPPER_NAMED_STATES['closed'])
            if not success:
                response.success = False
                response.message = f'close_gripper failed after pixel grasp: {msg}'
                return response

        response.success = True
        response.message = f'MoveIt pixel grasp complete: {message}'
        return response

    def _stow_cb(self, request, response):
        success, msg = self._move_group_named_state(ARM_GROUP, 'stowed', ARM_NAMED_STATES['stowed'])
        response.success = success
        response.message = msg
        return response

    def _unstow_cb(self, request, response):
        success, msg = self._move_group_named_state(ARM_GROUP, 'ready', ARM_NAMED_STATES['ready'])
        response.success = success
        response.message = msg
        return response

    def _carry_cb(self, request, response):
        success, msg = self._move_group_named_state(ARM_GROUP, 'ready', ARM_NAMED_STATES['ready'])
        response.success = success
        response.message = msg
        return response

    def _open_gripper_cb(self, request, response):
        success, msg = self._move_group_named_state(GRIPPER_GROUP, 'open', GRIPPER_NAMED_STATES['open'])
        response.success = success
        response.message = msg
        return response

    def _close_gripper_cb(self, request, response):
        success, msg = self._move_group_named_state(GRIPPER_GROUP, 'closed', GRIPPER_NAMED_STATES['closed'])
        response.success = success
        response.message = msg
        return response

    def _set_gripper_angle_cb(self, request, response):
        goal = 'open' if float(request.gripper_angle) >= 45.0 else 'closed'
        success, msg = self._move_group_named_state(GRIPPER_GROUP, goal, GRIPPER_NAMED_STATES[goal])
        response.success = success
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmNode()
    except Exception as exc:
        bootstrap = rclpy.create_node('arm_node_bootstrap')
        bootstrap.get_logger().error(str(exc))
        bootstrap.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1) from exc
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
