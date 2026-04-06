"""
User interface node - still working out best way to implement
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from spot_flex_msgs.action import FetchItem


class UINode(Node):
    """
    UI for sending commands to the conductor.
    """

    def __init__(self):
        super().__init__('ui_node')

        self._fetch_client = ActionClient(self, FetchItem, 'fetch_item')

        self._current_goal_handle = None
        self._executing = False

        self.get_logger().info('UI node started')
        self.get_logger().info('Type "help" for available commands')

    def run_interactive(self):
        """Run the interactive terminal prompt in a separate thread."""
        input_thread = threading.Thread(target=self._input_loop, daemon=True)
        input_thread.start()

    def _input_loop(self):
        """Read and dispatch user commands from stdin."""
        while rclpy.ok():
            try:
                cmd = input('\nspot_flex> ').strip()
            except EOFError:
                break
           
            if cmd:
                self._handle_fetch()
            elif cmd in ('quit', 'exit'):
                self.get_logger().info('Shutting down')
                rclpy.shutdown()
                break

    def _handle_fetch(self, cmd: str):
        """
        """
        pass

    def _feedback_cb(self, feedback_msg):
        """Print feedback from the conductor as steps execute."""
        pass

    def _goal_response_cb(self, future):
        """Handle the goal acceptance response."""
        goal_handle = future.result()
        pass

    def _result_cb(self, future):
        """Handle the final result."""
        result = future.result().result
        pass


def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    node.run_interactive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
