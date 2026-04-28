"""Mock perception action servers: /find_object, /find_box_grasp_point, /find_cabinet_handle."""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from spot_flex_msgs.action import FindObject, FindBoxGraspPoint, FindCabinetHandle


class MockPerceptionServer(Node):
    def __init__(self):
        super().__init__('mock_perception_server')
        self.declare_parameter('mock_delay_sec', 1.0)
        ActionServer(self, FindObject, 'find_object', self._find_object)
        ActionServer(self, FindBoxGraspPoint, 'find_box_grasp_point', self._find_box)
        ActionServer(self, FindCabinetHandle, 'find_cabinet_handle', self._find_handle)
        self.get_logger().info('mock_perception_server ready')

    def _delay(self):
        time.sleep(self.get_parameter('mock_delay_sec').value)

    def _find_object(self, goal_handle):
        self.get_logger().info(f'find_object: {goal_handle.request.object_name}')
        self._delay()
        goal_handle.succeed()
        return FindObject.Result(found=True, pixel_x=320, pixel_y=240)

    def _find_box(self, goal_handle):
        self.get_logger().info(f'find_box_grasp_point: side={goal_handle.request.side}')
        self._delay()
        goal_handle.succeed()
        return FindBoxGraspPoint.Result(found=True, pixel_x=200, pixel_y=300)

    def _find_handle(self, goal_handle):
        self.get_logger().info('find_cabinet_handle')
        self._delay()
        goal_handle.succeed()
        return FindCabinetHandle.Result(found=True, pixel_x=400, pixel_y=250)


def main():
    rclpy.init()
    node = MockPerceptionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
