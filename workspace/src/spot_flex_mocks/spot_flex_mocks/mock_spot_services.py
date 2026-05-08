"""Mock Spot driver services: dock/undock, arm, and gripper commands.

Matches the names and types the real spot_driver exposes so the conductor talks
to a single interface regardless of whether the real driver or this mock is up.
"""

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from spot_msgs.srv import Dock, SetGripperAngle


TRIGGER_NAMES = (
    'undock',
    'arm_stow',
    'arm_unstow',
    'arm_carry',
    'open_gripper',
    'close_gripper',
)


class MockSpotServices(Node):
    def __init__(self):
        super().__init__('mock_spot_services')
        for name in TRIGGER_NAMES:
            self.create_service(Trigger, name, self._make_trigger_cb(name))
        self.create_service(Dock, 'dock', self._dock_cb)
        self.create_service(SetGripperAngle, 'set_gripper_angle', self._set_gripper_angle_cb)
        self.get_logger().info(
            f'mock_spot_services ready: dock + set_gripper_angle + {list(TRIGGER_NAMES)}'
        )

    def _make_trigger_cb(self, name):
        def cb(_request, response):
            self.get_logger().info(f'{name}')
            response.success = True
            response.message = f'{name} ok'
            return response
        return cb

    def _dock_cb(self, request, response):
        self.get_logger().info(f'dock: id={request.dock_id}')
        response.success = True
        response.message = 'dock ok'
        return response

    def _set_gripper_angle_cb(self, request, response):
        self.get_logger().info(f'set_gripper_angle: angle={request.gripper_angle:.1f}')
        response.success = True
        response.message = 'set_gripper_angle ok'
        return response


def main():
    rclpy.init()
    node = MockSpotServices()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
