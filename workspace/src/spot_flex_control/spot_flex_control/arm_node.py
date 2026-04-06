"""
Arm control node.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger


class ArmNode(Node):
    """
    Arm control interface for Spot's manipulator.

    Services:
        /open_gripper, /close_gripper — simple gripper commands
        /arm_stow, /arm_unstow — stow/unstow the arm

    Publishers:
        /arm_joint_commands — direct joint angle commands

    Forwards commands to the corresponding spot_ros2 topics/services.
    """

    def __init__(self):
        super().__init__('arm_node')
        self.cb_group = ReentrantCallbackGroup()

        # Joint command publisher (spot_ros2 subscribes to this)
        self._joint_pub = self.create_publisher(
            JointTrajectory, 'arm_joint_commands', 10
        )

        # Gripper services
        self._open_gripper_srv = self.create_service(
            Trigger, 'open_gripper', self._open_gripper_cb
        )
        self._close_gripper_srv = self.create_service(
            Trigger, 'close_gripper', self._close_gripper_cb
        )

        # Stow services
        self._stow_srv = self.create_service(
            Trigger, 'arm_stow', self._stow_cb
        )
        self._unstow_srv = self.create_service(
            Trigger, 'arm_unstow', self._unstow_cb
        )

        # Clients to spot_ros2 services
        # TODO: create clients once spot_ros2 services are available
        # self._spot_open_gripper = self.create_client(Trigger, '/spot/open_gripper')
        # self._spot_close_gripper = self.create_client(Trigger, '/spot/close_gripper')
        # self._spot_arm_stow = self.create_client(Trigger, '/spot/arm_stow')

        # State
        self.gripper_open = False
        self.arm_stowed = True

        self.get_logger().info('Arm control node started')

    def send_joint_command(self, joint_positions: list):
        """
        Publish a joint trajectory command to the arm.

        Args:
            joint_positions: list of 6 floats [sh0, sh1, el0, el1, wr0, wr1]
        """
        pass

    def _open_gripper_cb(self, request, response):
        """Open the gripper."""
        pass

    def _close_gripper_cb(self, request, response):
        """Close the gripper."""
        pass

    def _stow_cb(self, request, response):
        """Stow the arm."""
        pass

    def _unstow_cb(self, request, response):
        """Unstow the arm."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
