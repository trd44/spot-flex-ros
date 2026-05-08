"""
Arm control node backed by MoveIt.
"""

import threading

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import BoundingVolume, OrientationConstraint, PositionConstraint
from std_srvs.srv import Trigger
from spot_msgs.srv import SetGripperAngle
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
