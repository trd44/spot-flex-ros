"""
Arm control node backed by MoveIt.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

ARM_GROUP = 'spot_arm'
GRIPPER_GROUP = 'spot_gripper'


class ArmNode(Node):

    def __init__(self):
        super().__init__('arm_node')
        self.cb_group = ReentrantCallbackGroup()

        self.get_logger().info('Connecting to MoveIt move_group...')
        from moveit.planning import MoveItPy

        self._moveit = MoveItPy(node_name='arm_node_moveit_py')
        self._arm = self._moveit.get_planning_component(ARM_GROUP)
        self._gripper = self._moveit.get_planning_component(GRIPPER_GROUP)
        self.get_logger().info('MoveIt ready.')

        self.create_service(Trigger, 'open_gripper',  self._open_gripper_cb,  callback_group=self.cb_group)
        self.create_service(Trigger, 'close_gripper', self._close_gripper_cb, callback_group=self.cb_group)
        self.create_service(Trigger, 'arm_stow',      self._stow_cb,          callback_group=self.cb_group)
        self.create_service(Trigger, 'arm_unstow',    self._unstow_cb,        callback_group=self.cb_group)

    def _plan_and_execute(self, component, goal_name: str) -> tuple[bool, str]:
        """Plan to a named state and execute it."""
        component.set_start_state_to_current_state()
        component.set_goal_state(configuration_name=goal_name)
        plan_result = component.plan()
        if not plan_result:
            msg = f'Planning to "{goal_name}" failed'
            self.get_logger().error(msg)
            return False, msg
        component.execute()
        self.get_logger().info(f'Executed: {goal_name}')
        return True, f'Moved to {goal_name}'

    def move_to_pose(self, pose: PoseStamped) -> tuple[bool, str]:
        """Plan and execute to an end-effector pose."""
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=pose, pose_link='arm_link_wr1')
        plan_result = self._arm.plan()
        if not plan_result:
            msg = 'Cartesian planning failed'
            self.get_logger().error(msg)
            return False, msg
        self._arm.execute()
        return True, 'Moved to pose'

    def _stow_cb(self, request, response):
        success, msg = self._plan_and_execute(self._arm, 'stowed')
        response.success = success
        response.message = msg
        return response

    def _unstow_cb(self, request, response):
        success, msg = self._plan_and_execute(self._arm, 'ready')
        response.success = success
        response.message = msg
        return response

    def _open_gripper_cb(self, request, response):
        success, msg = self._plan_and_execute(self._gripper, 'open')
        response.success = success
        response.message = msg
        return response

    def _close_gripper_cb(self, request, response):
        success, msg = self._plan_and_execute(self._gripper, 'closed')
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
