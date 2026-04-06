"""
Perception action server node.

Subscribes to Spot's camera topics, runs object detection on request,
and returns 3D poses by projecting detections through the depth image.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from spot_flex_msgs.action import FindObject

from spot_flex_perception.object_detector import ObjectDetector

import numpy as np


class PerceptionServerNode(Node):
    """
    Handles object detection requests.

    Subscribes to camera image and depth topics from spot_ros2.
    """

    def __init__(self):
        super().__init__('perception_server_node')
        self.cb_group = ReentrantCallbackGroup()

        # Detector
        self.detector = ObjectDetector(model_name="owl-vit")

        # Latest images from Spot cameras
        self._latest_rgb = None
        self._latest_depth = None
        self._camera_info = None

        # Camera subscribers
        # Using frontleft camera as default — need to decide what camera to use or choose one based on situation
        self._rgb_sub = self.create_subscription(
            Image,
            '/camera/frontleft/image',
            self._rgb_callback,
            10,
        )
        self._depth_sub = self.create_subscription(
            Image,
            '/camera/frontleft/depth',
            self._depth_callback,
            10,
        )
        self._info_sub = self.create_subscription(
            CameraInfo,
            '/camera/frontleft/camera_info',
            self._info_callback,
            10,
        )

        # Action servers
        self._find_object_server = ActionServer(
            self,
            FindObject,
            'find_object',
            self._execute_find_object,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Perception server started')

    def _rgb_callback(self, msg):
        self._latest_rgb = msg

    def _depth_callback(self, msg):
        self._latest_depth = msg

    def _info_callback(self, msg):
        self._camera_info = msg

    async def _execute_find_object(self, goal_handle):
        """Handle a FindObject request."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
