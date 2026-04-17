import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """Arrow-key teleop for /cmd_vel

  Up    : forward
  Down  : backward
  Left  : rotate left
  Right : rotate right
  Space : stop
  q     : quit
"""


class ArrowTeleop(Node):
    def __init__(self) -> None:
        super().__init__('teleop_arrows')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('reverse_speed', 0.1)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)
        # VS Code terminal has ~500 ms key-repeat delay. A short timeout
        # zeros the Twist during that gap, causing stutter. Keep the last
        # command alive longer — Space or q to stop explicitly.
        self.declare_parameter('key_timeout', 0.5)

        topic = str(self.get_parameter('cmd_vel_topic').value)
        self._publisher = self.create_publisher(Twist, topic, 10)
        self._last_twist = Twist()
        self._last_key_time = 0.0

    def spin_once(self) -> bool:
        key = self._read_key(timeout=0.0)
        if key:
            if not self._handle_key(key):
                return False

        timeout = float(self.get_parameter('key_timeout').value)
        if time.monotonic() - self._last_key_time > timeout:
            self._last_twist = Twist()

        self._publisher.publish(self._last_twist)
        rclpy.spin_once(self, timeout_sec=0.0)
        return True

    def stop(self) -> None:
        self._publisher.publish(Twist())

    def _handle_key(self, key: str) -> bool:
        linear_speed = float(self.get_parameter('linear_speed').value)
        reverse_speed = float(self.get_parameter('reverse_speed').value)
        angular_speed = float(self.get_parameter('angular_speed').value)

        twist = Twist()
        if key in ('\x1b[A', 'w'):
            twist.linear.x = linear_speed
        elif key in ('\x1b[B', 's'):
            twist.linear.x = -reverse_speed
        elif key in ('\x1b[D', 'a'):
            twist.angular.z = angular_speed
        elif key in ('\x1b[C', 'd'):
            twist.angular.z = -angular_speed
        elif key == ' ':
            twist = Twist()
        elif key in ('q', '\x03'):
            self._last_twist = Twist()
            self.stop()
            return False
        else:
            return True

        self._last_key_time = time.monotonic()
        self._last_twist = twist
        return True

    def _read_key(self, timeout: float) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return ''
        first = sys.stdin.read(1)
        if first == '\x1b':
            ready, _, _ = select.select([sys.stdin], [], [], 0.01)
            if ready:
                return first + sys.stdin.read(2)
        return first


def main(args=None) -> None:
    if not sys.stdin.isatty():
        raise RuntimeError('teleop_arrows must be run in an interactive terminal')

    rclpy.init(args=args)
    node = ArrowTeleop()
    old_settings = termios.tcgetattr(sys.stdin)
    rate_hz = float(node.get_parameter('publish_rate_hz').value)
    period = 1.0 / max(rate_hz, 1.0)

    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and node.spin_once():
            time.sleep(period)
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
