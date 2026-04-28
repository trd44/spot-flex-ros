"""Mock /go_to NavigateToPose action server."""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class MockNavServer(Node):
    def __init__(self):
        super().__init__('mock_nav_server')
        self.declare_parameter('mock_delay_sec', 1.5)
        self._server = ActionServer(self, NavigateToPose, 'go_to', self._execute)
        self.get_logger().info('mock_nav_server ready')

    def _execute(self, goal_handle):
        p = goal_handle.request.pose.pose.position
        self.get_logger().info(f'go_to: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
        time.sleep(self.get_parameter('mock_delay_sec').value)
        goal_handle.succeed()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = MockNavServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
