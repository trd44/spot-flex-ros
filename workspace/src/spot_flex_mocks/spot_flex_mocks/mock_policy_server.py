"""Mock /execute_policy action server."""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from spot_flex_msgs.action import ExecutePolicy


class MockPolicyServer(Node):
    def __init__(self):
        super().__init__('mock_policy_server')
        self.declare_parameter('mock_delay_sec', 2.0)
        # If non-empty, abort whenever this policy_name is requested (failure-path testing).
        self.declare_parameter('fail_policy', '')
        self._server = ActionServer(self, ExecutePolicy, 'execute_policy', self._execute)
        self.get_logger().info('mock_policy_server ready')

    def _execute(self, goal_handle):
        name = goal_handle.request.policy_name
        self.get_logger().info(f'execute_policy: {name}')
        time.sleep(self.get_parameter('mock_delay_sec').value)

        if name == self.get_parameter('fail_policy').value:
            goal_handle.abort()
            return ExecutePolicy.Result(success=False, message=f'mock-forced failure: {name}')

        goal_handle.succeed()
        return ExecutePolicy.Result(success=True, message=f'{name} ok')


def main():
    rclpy.init()
    node = MockPolicyServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
