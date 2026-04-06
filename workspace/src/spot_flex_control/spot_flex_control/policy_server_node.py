"""
Policy server node

Runs trained manipulation policies (cabinet opening, box pushing)
and publishes arm commands at control rate. Wraps the PolicyExecutor
in a ROS action server interface.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from spot_flex_msgs.action import ExecutePolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from spot_flex_control.policy_executor import PolicyExecutor


class PolicyServerNode(Node):
    """
    Executes trained policies and streams joint commands.
    """

    def __init__(self):
        pass

    async def _execute_policy(self, goal_handle):
        """Run the named policy"""
        pass

    def _publish_joints(self, positions: list):
        """Publish a joint trajectory command"""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PolicyServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
