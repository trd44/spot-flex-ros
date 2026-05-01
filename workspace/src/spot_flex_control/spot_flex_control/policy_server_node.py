"""Policy action server for Spot FLEX manipulation.

The server owns ROS I/O. Policy generation lives in ``policy_executor.py`` so
the policy logic is testable and independent of the old ``flex_spot`` scripts.
"""

import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from spot_flex_msgs.action import ExecutePolicy
from spot_flex_msgs.srv import (
    EstimateReactiveForce,
    GetHandPose,
    ImpedanceSettle,
    PushObjectStep,
)

from spot_flex_control.policy_executor import (
    BodyCommand,
    PolicyExecutor,
    PolicyExecutorConfig,
    TriggerCommand,
)


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _as_tuple(value, default=()) -> tuple[str, ...]:
    if value is None:
        return tuple(default)
    if isinstance(value, (list, tuple)):
        return tuple(str(item).strip() for item in value if str(item).strip())
    return tuple(item.strip() for item in str(value).split(',') if item.strip())


def _as_xy(value, default=(1.0, 0.0)) -> tuple[float, float]:
    if isinstance(value, (list, tuple)):
        parts = value
    else:
        parts = str(value).split(',')
    try:
        parsed = [float(part) for part in parts]
    except (TypeError, ValueError):
        return default
    if len(parsed) < 2:
        return default
    return parsed[0], parsed[1]


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _estimate_box_center_from_grasp(
    gripper_pos,
    box_dimensions: dict,
    current_yaw: float,
    robot_side: str,
):
    import numpy as np

    gripper_pos = np.asarray(gripper_pos, dtype=float)
    width = float(box_dimensions.get('width', 0.4))
    depth = float(box_dimensions.get('depth', 0.4))
    offset_width = width / 2.0
    offset_depth = depth / 2.0

    if robot_side == 'left':
        point_local = np.array([offset_depth, offset_width])
    else:
        point_local = np.array([offset_depth, -offset_width])

    c, s = math.cos(current_yaw), math.sin(current_yaw)
    rotation = np.array([[c, -s], [s, c]])
    offset_xy = rotation @ point_local
    return gripper_pos + np.array([offset_xy[0], offset_xy[1], 0.0])


class _PushStateEstimator:
    """Local ROS copy of flex_spot's 6D push state estimator."""

    def __init__(
        self,
        init_hand_pos,
        init_yaw: float,
        init_time: float,
        norm_reactive_force: float,
        box_dimensions,
        robot_side: str,
    ):
        import numpy as np

        self.norm_reactive_force = float(np.clip(norm_reactive_force, 0.0, 1.0))
        self.box_dimensions = box_dimensions
        self.robot_side = robot_side
        self._prev_pos = self.reference_from_hand(init_hand_pos, init_yaw)
        self._prev_yaw = float(init_yaw)
        self._prev_time = float(init_time)
        self.closest_idx = 0
        self.reference_pos_2d = self._prev_pos.copy()

    def reference_from_hand(self, hand_pos, hand_yaw: float):
        import numpy as np

        if self.box_dimensions is None:
            return np.asarray(hand_pos, dtype=float)[:2]
        center = _estimate_box_center_from_grasp(
            hand_pos,
            self.box_dimensions,
            hand_yaw,
            self.robot_side,
        )
        return center[:2]

    def compute(self, current_hand_pos, current_yaw: float, current_time: float, path_points):
        import numpy as np

        pts_2d = np.asarray(path_points, dtype=float)[:, :2]
        pos_2d = self.reference_from_hand(current_hand_pos, current_yaw)
        self.reference_pos_2d = pos_2d.copy()

        dists = np.linalg.norm(pts_2d - pos_2d, axis=1)
        self.closest_idx = int(np.argmin(dists))
        tangent, normal = self._path_tangent_normal(pts_2d, self.closest_idx)

        lateral_error = float(np.dot(pos_2d - pts_2d[self.closest_idx], normal))
        desired_yaw = math.atan2(float(tangent[1]), float(tangent[0]))
        orientation_error = math.atan2(
            math.sin(current_yaw - desired_yaw),
            math.cos(current_yaw - desired_yaw),
        )

        dt = float(current_time) - self._prev_time
        if dt > 1e-6:
            vel_2d = (pos_2d - self._prev_pos) / dt
            speed_fwd = float(np.dot(vel_2d, tangent))
            speed_lat = float(np.dot(vel_2d, normal))
            angular_vel = _wrap_to_pi(current_yaw - self._prev_yaw) / dt
        else:
            speed_fwd = 0.0
            speed_lat = 0.0
            angular_vel = 0.0

        self._prev_pos = pos_2d.copy()
        self._prev_yaw = float(current_yaw)
        self._prev_time = float(current_time)

        return np.array(
            [
                lateral_error,
                orientation_error,
                speed_fwd,
                speed_lat,
                angular_vel,
                self.norm_reactive_force,
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _path_tangent_normal(pts_2d, idx: int):
        import numpy as np

        count = len(pts_2d)
        if idx == count - 1 and count > 1:
            raw = pts_2d[idx] - pts_2d[idx - 1]
        else:
            next_idx = min(idx + 1, count - 1)
            raw = pts_2d[next_idx] - pts_2d[idx]

        norm = float(np.linalg.norm(raw))
        tangent = raw / norm if norm > 1e-8 else np.array([1.0, 0.0])
        normal = np.array([-tangent[1], tangent[0]])
        return tangent, normal


class PolicyServerNode(Node):
    """Executes named manipulation policies through currently available ROS controls."""

    def __init__(self):
        super().__init__('policy_server_node')
        self.cb_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self._executor = PolicyExecutor(self._make_executor_config())

        self._server = ActionServer(
            self,
            ExecutePolicy,
            'execute_policy',
            self._execute_policy,
            callback_group=self.cb_group,
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            10,
        )
        self._arm_pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('arm_pose_topic').value),
            10,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._sleep_timers = set()
        self._trigger_clients = {}
        self._service_clients = {}
        self.get_logger().info(
            f'policy_server_node ready (dry_run={self._dry_run()})')

    def _declare_parameters(self) -> None:
        self.declare_parameter('dry_run', True)
        self.declare_parameter('policy_delay_sec', 1.0)
        self.declare_parameter('body_command_period_sec', 0.1)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('trigger_service_prefix', '')
        self.declare_parameter('arm_pose_topic', '/spot/arm_pose_commands')
        self.declare_parameter('vision_frame', 'spot/vision')
        self.declare_parameter('hand_frame', 'spot/hand')
        self.declare_parameter('push_relevel_z', True)
        self.declare_parameter('push_pre_probe_lower_m', 0.2)
        self.declare_parameter('push_arm_pose_settle_sec', 1.5)

        self.declare_parameter('push_speed_mps', 0.12)
        self.declare_parameter('push_lateral_speed_mps', 0.0)
        self.declare_parameter('push_yaw_rate_radps', 0.0)
        self.declare_parameter('push_duration_sec', 2.0)
        self.declare_parameter('flex_push_flow', True)
        self.declare_parameter('push_step_service', '/spot/push_object_step')
        self.declare_parameter('impedance_settle_service', '/spot/impedance_settle')
        self.declare_parameter('estimate_reactive_force_service', '/spot/estimate_reactive_force')
        self.declare_parameter('get_hand_pose_service', '/spot/get_hand_pose')
        self.declare_parameter('push_probe_force', True)
        self.declare_parameter('push_max_force', 200.0)
        self.declare_parameter('push_probe_step_m', 0.005)
        self.declare_parameter('push_probe_max_steps', 30)
        self.declare_parameter('push_probe_settle_time_sec', 1.0)
        self.declare_parameter('push_force_drop_threshold', 4.0)
        self.declare_parameter('push_impedance_stiffness', 300.0)
        self.declare_parameter('push_impedance_damping', 45.0)
        self.declare_parameter('push_impedance_two_phase', True)
        self.declare_parameter('push_settle_duration_sec', 3.0)
        self.declare_parameter('push_settle_between_steps', True)
        self.declare_parameter('push_robot_side', 'right')
        self.declare_parameter('push_surface_type', 'floor')
        self.declare_parameter('push_max_step_m', 1.0)
        self.declare_parameter('push_max_step_yaw_rad', 0.12)
        self.declare_parameter('push_yaw_scale', 0.05)
        self.declare_parameter('push_success_progress', 0.95)
        self.declare_parameter('push_deviation_tolerance', 1.0)
        self.declare_parameter('push_success_distance', 0.8)
        self.declare_parameter('push_log_every', 1)
        self.declare_parameter('push_use_impedance', True)
        self.declare_parameter('push_use_box_center', False)
        self.declare_parameter('push_box_width', 0.465)
        self.declare_parameter('push_box_depth', 0.61)
        self.declare_parameter('push_box_height', 0.63)
        self.declare_parameter('push_path_type', 'arc')
        self.declare_parameter('push_arc_radius', 1.5)
        self.declare_parameter('push_arc_angle_deg', 60.0)
        self.declare_parameter('push_length', 1.5)
        self.declare_parameter('push_amplitude', 0.5)
        self.declare_parameter('push_from_edge', False)
        self.declare_parameter('push_state_dim', 6)
        self.declare_parameter('push_action_dim', 2)

        self.declare_parameter('revolute_linear_speed_mps', 0.04)
        self.declare_parameter('revolute_lateral_speed_mps', 0.08)
        self.declare_parameter('revolute_yaw_rate_radps', 0.18)
        self.declare_parameter('revolute_duration_sec', 4.0)

        self.declare_parameter('prismatic_speed_mps', 0.08)
        self.declare_parameter('prismatic_duration_sec', 3.0)
        self.declare_parameter('prismatic_axis_xy', '1.0,0.0')

        self.declare_parameter('policy_step_duration_sec', 2.0)
        self.declare_parameter('policy_max_steps', 30)
        self.declare_parameter('action_scale', 0.75)
        self.declare_parameter('max_linear_speed_mps', 0.18)
        self.declare_parameter('max_angular_speed_radps', 0.35)
        self.declare_parameter('invert_action_x', True)

        self.declare_parameter('use_learned_policies', False)
        self.declare_parameter('revolute_model_dir', '')
        self.declare_parameter('revolute_model_name', 'final')
        self.declare_parameter('prismatic_model_dir', '')
        self.declare_parameter('prismatic_model_name', 'final')
        self.declare_parameter('push_model_dir', '')
        self.declare_parameter('push_model_name', 'final_model')

        self.declare_parameter('position_for_grasp_triggers', 'arm_unstow')
        self.declare_parameter('grasp_triggers', 'close_gripper,arm_carry')
        self.declare_parameter('place_triggers', 'open_gripper')
        self.declare_parameter('revolute_pre_triggers', 'arm_unstow')
        self.declare_parameter('revolute_post_triggers', '')
        self.declare_parameter('prismatic_pre_triggers', 'arm_unstow')
        self.declare_parameter('prismatic_post_triggers', '')

    def _make_executor_config(self) -> PolicyExecutorConfig:
        value = lambda name: self.get_parameter(name).value
        return PolicyExecutorConfig(
            push_speed_mps=float(value('push_speed_mps')),
            push_lateral_speed_mps=float(value('push_lateral_speed_mps')),
            push_yaw_rate_radps=float(value('push_yaw_rate_radps')),
            push_duration_sec=float(value('push_duration_sec')),
            revolute_linear_speed_mps=float(value('revolute_linear_speed_mps')),
            revolute_lateral_speed_mps=float(value('revolute_lateral_speed_mps')),
            revolute_yaw_rate_radps=float(value('revolute_yaw_rate_radps')),
            revolute_duration_sec=float(value('revolute_duration_sec')),
            prismatic_speed_mps=float(value('prismatic_speed_mps')),
            prismatic_duration_sec=float(value('prismatic_duration_sec')),
            prismatic_axis_xy=_as_xy(value('prismatic_axis_xy')),
            policy_step_duration_sec=float(value('policy_step_duration_sec')),
            policy_max_steps=int(value('policy_max_steps')),
            action_scale=float(value('action_scale')),
            max_linear_speed_mps=float(value('max_linear_speed_mps')),
            max_angular_speed_radps=float(value('max_angular_speed_radps')),
            invert_action_x=_as_bool(value('invert_action_x')),
            use_learned_policies=_as_bool(value('use_learned_policies')),
            revolute_model_dir=str(value('revolute_model_dir')),
            revolute_model_name=str(value('revolute_model_name')),
            prismatic_model_dir=str(value('prismatic_model_dir')),
            prismatic_model_name=str(value('prismatic_model_name')),
            push_model_dir=str(value('push_model_dir')),
            push_model_name=str(value('push_model_name')),
            position_for_grasp_triggers=_as_tuple(value('position_for_grasp_triggers')),
            grasp_triggers=_as_tuple(value('grasp_triggers')),
            place_triggers=_as_tuple(value('place_triggers')),
            revolute_pre_triggers=_as_tuple(value('revolute_pre_triggers')),
            revolute_post_triggers=_as_tuple(value('revolute_post_triggers')),
            prismatic_pre_triggers=_as_tuple(value('prismatic_pre_triggers')),
            prismatic_post_triggers=_as_tuple(value('prismatic_post_triggers')),
        )

    async def _execute_policy(self, goal_handle):
        """Run the named policy."""
        policy = goal_handle.request.policy_name
        self._executor = PolicyExecutor(self._make_executor_config())
        self.get_logger().info(f'execute_policy: {policy}')
        self._publish_feedback(goal_handle, 0.0, f'planning {policy}')

        try:
            if self._is_push_policy(policy) and _as_bool(self.get_parameter('flex_push_flow').value):
                success, message = await self._execute_flex_push(goal_handle)
                if success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return ExecutePolicy.Result(success=success, message=message)

            plan = self._executor.build_plan(policy)
            self.get_logger().info(
                f'policy {policy}: {len(plan.commands)} commands, '
                f'{plan.duration_sec:.2f}s body motion')

            success, message = await self._execute_plan(plan, goal_handle)
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return ExecutePolicy.Result(success=success, message=message)
        except Exception as exc:
            self.get_logger().error(f'policy {policy} failed: {exc}')
            goal_handle.abort()
            return ExecutePolicy.Result(success=False, message=str(exc))

    async def _execute_flex_push(self, goal_handle) -> tuple[bool, str]:
        """Run the FLEX push loop instead of a one-shot body velocity rollout."""
        self._publish_feedback(goal_handle, 0.02, 'opening gripper after box grasp')
        if self._dry_run():
            return True, 'push_box dry-run complete'

        success, message = await self._call_trigger('open_gripper')
        if not success:
            return False, f'open_gripper failed: {message}'

        self._publish_feedback(goal_handle, 0.08, 'reestablishing box contact with impedance')
        success, message = await self._impedance_settle(
            duration_sec=float(self.get_parameter('push_settle_duration_sec').value),
        )
        if not success:
            return False, f'impedance settle failed: {message}'

        lower_m = float(self.get_parameter('push_pre_probe_lower_m').value)
        if lower_m > 0.0:
            self._publish_feedback(goal_handle, 0.12, f'lowering grip {lower_m:.2f}m before probe')
            try:
                tr = await self._lookup_hand_tf()
                p = tr.transform.translation
                q = tr.transform.rotation
                self._publish_arm_pose(p.x, p.y, p.z - lower_m, q.x, q.y, q.z, q.w)
                await self._sleep(float(self.get_parameter('push_arm_pose_settle_sec').value))
            except Exception as exc:
                self.get_logger().warning(f'pre-probe lower failed: {exc}')

        self._publish_feedback(goal_handle, 0.15, 'estimating reactive force')
        success, norm_reactive_force, message = await self._estimate_reactive_force()
        if not success:
            return False, f'reactive force estimate failed: {message}'
        self.get_logger().info(f'push_box: norm_reactive_force={norm_reactive_force:.3f}')

        actor = None
        config = self._make_executor_config()
        if config.use_learned_policies and config.push_model_dir:
            actor = self._executor._load_actor(  # pylint: disable=protected-access
                kind='path',
                model_dir=config.push_model_dir,
                model_name=config.push_model_name,
            )
        if actor is None:
            self.get_logger().warning(
                'push_box: learned actor unavailable, using conservative forward steps'
            )

        hand_pos, hand_yaw = await self._get_hand_pose()
        relevel_z = float(hand_pos[2])
        path_points = self._generate_push_path(hand_pos, hand_yaw)
        estimator = _PushStateEstimator(
            init_hand_pos=hand_pos,
            init_yaw=hand_yaw,
            init_time=time.time(),
            norm_reactive_force=norm_reactive_force,
            box_dimensions=self._push_box_dimensions(),
            robot_side=str(self.get_parameter('push_robot_side').value),
        )

        success_progress = float(self.get_parameter('push_success_progress').value)
        success_distance = float(self.get_parameter('push_success_distance').value)
        deviation_tolerance = float(self.get_parameter('push_deviation_tolerance').value)
        log_every = max(1, int(self.get_parameter('push_log_every').value))
        max_steps = max(1, int(self.get_parameter('policy_max_steps').value))

        path_len = self._path_length(path_points)
        self.get_logger().info(
            'push_box path: '
            f'type={self.get_parameter("push_path_type").value}, '
            f'points={len(path_points)}, length={path_len:.2f}m, '
            f'success(progress>={success_progress:.2f} and deviation<{success_distance:.2f}m), '
            f'abort(deviation>{deviation_tolerance:.2f}m)'
        )

        for step in range(max_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False, 'canceled'

            hand_pos, hand_yaw = await self._get_hand_pose()
            state = estimator.compute(hand_pos, hand_yaw, time.time(), path_points)
            ref_pos = estimator.reference_pos_2d
            progress, deviation, closest_idx = self._compute_progress(ref_pos, path_points)

            if deviation > deviation_tolerance:
                message = (
                    f'push_box aborted: deviation {deviation:.3f}m exceeds '
                    f'{deviation_tolerance:.3f}m at step {step + 1}'
                )
                self.get_logger().warning(message)
                return False, message

            if progress >= success_progress and deviation < success_distance:
                message = (
                    f'push_box succeeded: progress={progress:.3f}, '
                    f'deviation={deviation:.3f}m'
                )
                self._publish_feedback(goal_handle, 1.0, message)
                self.get_logger().info(message)
                return True, message

            action = self._select_push_action(actor, state)
            dx, dy, d_yaw = self._push_action_to_step(action, hand_yaw)
            d_yaw = 0.0
            if step % log_every == 0 or step < 3:
                self.get_logger().info(
                    f'push_box step {step + 1}/{max_steps}: '
                    f'idx={closest_idx}/{len(path_points) - 1}, '
                    f'progress={progress:.3f}, deviation={deviation:.3f}m, '
                    f'state={state.tolist()}, action={action}, '
                    f'dx={dx:.3f}, dy={dy:.3f}, d_yaw={d_yaw:.3f}'
                )

            feedback_progress = min(0.95, 0.2 + 0.7 * progress)
            self._publish_feedback(
                goal_handle,
                feedback_progress,
                f'push step {step + 1}/{max_steps} progress={progress:.2f}',
            )
            success, message = await self._push_object_step(dx, dy, d_yaw)
            if not success:
                return False, f'push step failed: {message}'

            if _as_bool(self.get_parameter('push_relevel_z').value):
                self._publish_feedback(
                    goal_handle,
                    feedback_progress,
                    f'releveling z {step + 1}/{max_steps}',
                )
                await self._move_hand_to_z(relevel_z, f'relevel step {step + 1}')

            if _as_bool(self.get_parameter('push_settle_between_steps').value):
                self._publish_feedback(
                    goal_handle,
                    feedback_progress,
                    f'reestablishing contact {step + 1}/{max_steps}',
                )
                success, message = await self._impedance_settle(
                    duration_sec=float(self.get_parameter('push_settle_duration_sec').value),
                )
                if not success:
                    return False, f'post-push settle failed: {message}'

        hand_pos, hand_yaw = await self._get_hand_pose()
        progress, deviation, _ = self._compute_progress(
            estimator.reference_from_hand(hand_pos, hand_yaw), path_points
        )
        if progress >= success_progress and deviation < success_distance:
            message = f'push_box succeeded: progress={progress:.3f}, deviation={deviation:.3f}m'
            self._publish_feedback(goal_handle, 1.0, message)
            return True, message

        message = (
            f'push_box failed: max_steps={max_steps} reached with '
            f'progress={progress:.3f}, deviation={deviation:.3f}m'
        )
        self.get_logger().warning(message)
        return False, message

    def _select_push_action(self, actor, state):
        if actor is None:
            return [0.35, 0.0]

        import numpy as np

        actor_state = np.zeros(actor.state_dim, dtype=np.float32)
        copy_count = min(actor.state_dim, len(state))
        actor_state[:copy_count] = np.asarray(state[:copy_count], dtype=np.float32)
        action = actor.select_action(actor_state).reshape(-1)
        action = np.clip(action, -1.0, 1.0)
        if action.size >= 2:
            norm = float(np.linalg.norm(action[:2]))
            if norm > 1e-6:
                action[:2] = action[:2] / norm
        return action.tolist()

    def _push_action_to_step(self, action, hand_yaw: float) -> tuple[float, float, float]:
        action_x = float(action[0]) if len(action) > 0 else 0.0
        action_y = float(action[1]) if len(action) > 1 else 0.0

        max_force = float(self.get_parameter('push_max_force').value)
        action_scale = float(self.get_parameter('action_scale').value)
        stiffness = float(self.get_parameter('push_impedance_stiffness').value)
        use_impedance = _as_bool(self.get_parameter('push_use_impedance').value)
        yaw_scale = float(self.get_parameter('push_yaw_scale').value)
        max_step = abs(float(self.get_parameter('push_max_step_m').value))
        max_yaw = abs(float(self.get_parameter('push_max_step_yaw_rad').value))

        k_eff = stiffness if use_impedance else max_force / max(action_scale, 1e-6)
        dx = action_x * max_force * action_scale / max(k_eff, 1e-6)
        dy = action_y * max_force * action_scale / max(k_eff, 1e-6)
        desired_yaw = math.atan2(dy, dx) if abs(dx) > 1e-6 or abs(dy) > 1e-6 else hand_yaw
        d_yaw = _wrap_to_pi(desired_yaw - hand_yaw) * yaw_scale
        if max_step > 0.0:
            dx = max(-max_step, min(max_step, dx))
            dy = max(-max_step, min(max_step, dy))
        if max_yaw > 0.0:
            d_yaw = max(-max_yaw, min(max_yaw, d_yaw))
        return dx, dy, d_yaw

    def _push_box_dimensions(self):
        if not _as_bool(self.get_parameter('push_use_box_center').value):
            return None
        return {
            'width': float(self.get_parameter('push_box_width').value),
            'depth': float(self.get_parameter('push_box_depth').value),
            'height': float(self.get_parameter('push_box_height').value),
        }

    async def _get_hand_pose(self) -> tuple[tuple[float, float, float], float]:
        response = await self._call_typed_service(
            GetHandPose,
            str(self.get_parameter('get_hand_pose_service').value),
            GetHandPose.Request(),
        )
        if not bool(response.success):
            raise RuntimeError(f'get_hand_pose failed: {response.message}')
        return (float(response.x), float(response.y), float(response.z)), float(response.yaw)

    async def _lookup_hand_tf(self, timeout_sec: float = 3.0):
        vision = str(self.get_parameter('vision_frame').value)
        hand = str(self.get_parameter('hand_frame').value)
        deadline = time.time() + timeout_sec
        last_exc = None
        while time.time() < deadline:
            try:
                return self._tf_buffer.lookup_transform(vision, hand, rclpy.time.Time())
            except Exception as exc:
                last_exc = exc
                await self._sleep(0.1)
        raise RuntimeError(f'TF unavailable {vision} -> {hand}: {last_exc}')

    def _publish_arm_pose(self, x, y, z, qx, qy, qz, qw, frame=None):
        if frame is None:
            frame = str(self.get_parameter('vision_frame').value)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        self._arm_pose_pub.publish(msg)

    async def _move_hand_to_z(self, target_z: float, label: str) -> bool:
        try:
            tr = await self._lookup_hand_tf()
        except Exception as exc:
            self.get_logger().warning(f'{label}: TF lookup failed: {exc}')
            return False
        p = tr.transform.translation
        q = tr.transform.rotation
        self._publish_arm_pose(p.x, p.y, target_z, q.x, q.y, q.z, q.w)
        await self._sleep(float(self.get_parameter('push_arm_pose_settle_sec').value))
        return True

    def _generate_push_path(
        self,
        hand_pos: tuple[float, float, float],
        hand_yaw: float,
    ):
        import numpy as np

        box_dimensions = self._push_box_dimensions()
        if box_dimensions is None:
            origin_xy = np.array(hand_pos[:2], dtype=float)
            z_height = float(hand_pos[2])
        else:
            center = _estimate_box_center_from_grasp(
                hand_pos,
                box_dimensions,
                hand_yaw,
                str(self.get_parameter('push_robot_side').value),
            )
            origin_xy = center[:2]
            z_height = float(center[2])

        path_type = str(self.get_parameter('push_path_type').value).strip().lower()
        length = float(self.get_parameter('push_length').value)
        amplitude = float(self.get_parameter('push_amplitude').value)

        if path_type == 'arc':
            radius = float(self.get_parameter('push_arc_radius').value)
            angle = math.radians(float(self.get_parameter('push_arc_angle_deg').value))
            theta = np.linspace(3.0 * math.pi / 2.0, 3.0 * math.pi / 2.0 + angle,
                                max(10, int(radius * 20)))
            pts_local = np.column_stack([radius * np.cos(theta), radius * np.sin(theta)])
        elif path_type == 'straight':
            pts_local = np.column_stack([
                np.linspace(0.0, length, max(10, int(length * 20))),
                np.zeros(max(10, int(length * 20))),
            ])
        elif path_type == 's_curve':
            count = 100
            t = np.linspace(0.0, 2.0 * math.pi, count)
            pts_local = np.column_stack([
                (t / (2.0 * math.pi)) * length,
                np.sin(t) * amplitude + np.sin(1.5 * t) * amplitude * 0.2,
            ])
        elif path_type == 'meander':
            count = 150
            t = np.linspace(0.0, 4.0 * math.pi, count)
            pts_local = np.column_stack([
                (t / (4.0 * math.pi)) * length,
                np.sin(t) * amplitude + np.sin(2.3 * t) * amplitude * 0.4,
            ])
        elif path_type == 'triple_s':
            count = 150
            t = np.linspace(0.0, 6.0 * math.pi, count)
            pts_local = np.column_stack([
                (t / (6.0 * math.pi)) * length,
                np.sin(t) * amplitude,
            ])
        else:
            raise ValueError(f'unknown push_path_type: {path_type}')

        pts = pts_local - pts_local[0]
        z = np.full((len(pts), 1), z_height)
        return np.column_stack([pts + origin_xy, z]).astype(np.float32)

    @staticmethod
    def _compute_progress(ref_pos_2d, path_points) -> tuple[float, float, int]:
        import numpy as np

        pts_2d = np.asarray(path_points, dtype=float)[:, :2]
        pos_2d = np.asarray(ref_pos_2d, dtype=float)[:2]
        dists = np.linalg.norm(pts_2d - pos_2d, axis=1)
        idx = int(np.argmin(dists))
        progress = idx / (len(path_points) - 1) if len(path_points) > 1 else 0.0
        return float(progress), float(dists[idx]), idx

    @staticmethod
    def _path_length(path_points) -> float:
        import numpy as np

        pts_2d = np.asarray(path_points, dtype=float)[:, :2]
        if len(pts_2d) < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(pts_2d, axis=0), axis=1)))

    async def _estimate_reactive_force(self) -> tuple[bool, float, str]:
        request = EstimateReactiveForce.Request()
        request.probe_force = _as_bool(self.get_parameter('push_probe_force').value)
        request.max_force = float(self.get_parameter('push_max_force').value)
        request.probe_step_m = float(self.get_parameter('push_probe_step_m').value)
        request.max_probe_steps = int(self.get_parameter('push_probe_max_steps').value)
        request.settle_time_sec = float(self.get_parameter('push_probe_settle_time_sec').value)
        request.force_drop_threshold = float(self.get_parameter('push_force_drop_threshold').value)
        request.stiffness = float(self.get_parameter('push_impedance_stiffness').value)
        request.damping = float(self.get_parameter('push_impedance_damping').value)
        request.grasp_strategy = 'edge_grasp'
        request.surface_type = str(self.get_parameter('push_surface_type').value)
        request.robot_side = str(self.get_parameter('push_robot_side').value)
        response = await self._call_typed_service(
            EstimateReactiveForce,
            str(self.get_parameter('estimate_reactive_force_service').value),
            request,
        )
        return bool(response.success), float(response.norm_reactive_force), response.message

    async def _impedance_settle(self, duration_sec: float) -> tuple[bool, str]:
        request = ImpedanceSettle.Request()
        request.duration_sec = float(duration_sec)
        request.stiffness = float(self.get_parameter('push_impedance_stiffness').value)
        request.damping = float(self.get_parameter('push_impedance_damping').value)
        response = await self._call_typed_service(
            ImpedanceSettle,
            str(self.get_parameter('impedance_settle_service').value),
            request,
        )
        return bool(response.success), response.message

    async def _push_object_step(self, dx: float, dy: float, d_yaw: float) -> tuple[bool, str]:
        request = PushObjectStep.Request()
        request.dx = float(dx)
        request.dy = float(dy)
        request.d_yaw = float(d_yaw)
        duration = float(self.get_parameter('policy_step_duration_sec').value)
        request.duration_sec = duration
        request.vx = max(abs(dx) / max(duration, 1e-6), 0.01)
        request.vy = max(abs(dy) / max(duration, 1e-6), 0.01)
        request.v_yaw = max(abs(d_yaw) / max(duration, 1e-6), 0.01)
        request.use_impedance = _as_bool(self.get_parameter('push_use_impedance').value)
        request.stiffness = float(self.get_parameter('push_impedance_stiffness').value)
        request.damping = float(self.get_parameter('push_impedance_damping').value)
        request.two_phase = _as_bool(self.get_parameter('push_impedance_two_phase').value)
        response = await self._call_typed_service(
            PushObjectStep,
            str(self.get_parameter('push_step_service').value),
            request,
        )
        return bool(response.success), response.message

    async def _execute_plan(self, plan, goal_handle) -> tuple[bool, str]:
        command_count = max(len(plan.commands), 1)
        for index, command in enumerate(plan.commands):
            if goal_handle.is_cancel_requested:
                self._stop_body()
                goal_handle.canceled()
                return False, 'canceled'

            progress = index / command_count
            status = command.status or getattr(command, 'service_name', 'policy step')
            self._publish_feedback(goal_handle, progress, status)

            if self._dry_run():
                await self._sleep(min(float(self.get_parameter('policy_delay_sec').value), 0.25))
                continue

            if isinstance(command, TriggerCommand):
                success, message = await self._call_trigger(command.service_name)
                if not success:
                    return False, f'{command.service_name} failed: {message}'
            elif isinstance(command, BodyCommand):
                success, message = await self._run_body_command(command, goal_handle)
                if not success:
                    return False, message
            else:
                return False, f'unsupported command: {command}'

        self._stop_body()
        self._publish_feedback(goal_handle, 1.0, 'complete')
        suffix = 'dry-run complete' if self._dry_run() else 'complete'
        return True, f'{plan.policy_name} {suffix}'

    async def _run_body_command(self, command: BodyCommand, goal_handle) -> tuple[bool, str]:
        twist = Twist()
        twist.linear.x = float(command.linear_x)
        twist.linear.y = float(command.linear_y)
        twist.angular.z = float(command.angular_z)

        period = max(0.02, float(self.get_parameter('body_command_period_sec').value))
        remaining = max(0.0, float(command.duration_sec))
        while remaining > 0.0:
            if goal_handle.is_cancel_requested:
                self._stop_body()
                goal_handle.canceled()
                return False, 'canceled'
            self._cmd_vel_pub.publish(twist)
            await self._sleep(min(period, remaining))
            remaining -= period

        self._stop_body()
        return True, command.status

    async def _call_trigger(self, name: str) -> tuple[bool, str]:
        name = self._service_name(name)
        client = self._trigger_clients.get(name)
        if client is None:
            client = self.create_client(Trigger, name, callback_group=self.cb_group)
            self._trigger_clients[name] = client

        if not client.wait_for_service(timeout_sec=3.0):
            return False, f'service {name} unavailable'

        response = await client.call_async(Trigger.Request())
        return bool(response.success), response.message

    async def _call_typed_service(self, srv_type, name: str, request):
        client_key = (srv_type, name)
        client = self._service_clients.get(client_key)
        if client is None:
            client = self.create_client(srv_type, name, callback_group=self.cb_group)
            self._service_clients[client_key] = client

        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'service {name} unavailable')
        return await client.call_async(request)

    @staticmethod
    def _is_push_policy(policy_name: str) -> bool:
        policy = policy_name.strip().lower().replace('-', '_').replace(' ', '_')
        return policy in {'push_box', 'box_push', 'push'}

    def _service_name(self, name: str) -> str:
        if name.startswith('/'):
            return name
        prefix = str(self.get_parameter('trigger_service_prefix').value).strip()
        if not prefix:
            return name
        return f'{prefix.rstrip("/")}/{name}'

    async def _sleep(self, seconds: float) -> None:
        seconds = float(seconds)
        if seconds <= 0.0:
            return

        future = Future()

        def _wake():
            if not future.done():
                future.set_result(None)
            self.destroy_timer(timer)
            self._sleep_timers.discard(timer)

        timer = self.create_timer(seconds, _wake, callback_group=self.cb_group)
        self._sleep_timers.add(timer)
        await future

    def _dry_run(self) -> bool:
        return _as_bool(self.get_parameter('dry_run').value)

    def _stop_body(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _publish_feedback(self, goal_handle, progress: float, status: str) -> None:
        feedback = ExecutePolicy.Feedback()
        feedback.progress = float(progress)
        feedback.status = status
        goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
