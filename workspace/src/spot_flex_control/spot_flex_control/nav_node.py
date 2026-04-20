"""
Navigation node?
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose

class NavNode(Node):
    """
    Wraps Spot's navigation in a nav2-compatible interface.

    Action server:
        /go_to (NavigateToPose) accepts goal poses in vision frame
    """

    def __init__(self):
        super().__init__('nav_node')
        self.cb_group = ReentrantCallbackGroup()

        # Accept nav2-style goals
        self._go_to_server = ActionServer(
            self,
            NavigateToPose,
            'go_to',
            self._execute_go_to,
            callback_group=self.cb_group,
        )

        # Connect to spot_ros2

        # Cmd_vel publisher for fine adjustments
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('Navigation node started')

    async def _execute_go_to(self, goal_handle):
        """Translate a NavigateToPose goal to Spot navigation commands."""
        pass

    def adjust_position(self, dx: float, dy: float, dyaw: float):
        """
        Small position adjustment
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
