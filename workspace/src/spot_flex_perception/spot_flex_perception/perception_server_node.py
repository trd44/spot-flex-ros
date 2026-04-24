"""
Perception action server node.

Runs simple perception actions and returns image pixel coordinates.
"""

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from spot_flex_msgs.action import FindBoxGraspPoint, FindCabinetHandle, FindObject

from spot_flex_perception.box_grasp_detector import BoxGraspDetector
from spot_flex_perception.cabinet_handle_detector import CabinetHandleDetector
from spot_flex_perception.owl_detector import OwlDetector, load_rgb_image


class PerceptionServerNode(Node):
    """
    Handles object detection requests.

    Subscribes to camera image and depth topics from spot_ros2.
    """

    def __init__(self):
        super().__init__('perception_server_node')
        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        self.declare_parameter('use_test_images', True)
        self.declare_parameter(
            'box_test_image_path',
            '/repo/workspace/src/spot_flex_perception/test_images/cardboard_box.jpg',
        )
        self.declare_parameter(
            'cabinet_test_image_path',
            '/repo/workspace/src/spot_flex_perception/test_images/test_cabinet.jpg',
        )
        self.declare_parameter(
            'object_test_image_path',
            '/repo/workspace/src/spot_flex_perception/test_images/cardboard_box.jpg',
        )
        self.declare_parameter('rgb_topic', '/spot/camera/hand/image')

        self.owl_detector = OwlDetector()
        self.box_grasp_detector = BoxGraspDetector(owl_detector=self.owl_detector)
        self.cabinet_handle_detector = CabinetHandleDetector()

        self._latest_rgb = None

        self._rgb_sub = self.create_subscription(
            Image,
            self.get_parameter('rgb_topic').value,
            self._rgb_callback,
            10,
        )

        self._find_box_grasp_point_server = ActionServer(
            self,
            FindBoxGraspPoint,
            'find_box_grasp_point',
            self._execute_find_box_grasp_point,
            callback_group=self.cb_group,
        )
        self._find_cabinet_handle_server = ActionServer(
            self,
            FindCabinetHandle,
            'find_cabinet_handle',
            self._execute_find_cabinet_handle,
            callback_group=self.cb_group,
        )
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

    def _get_image_rgb(self, test_image_param):
        if self.get_parameter('use_test_images').value:
            image_path = Path(self.get_parameter(test_image_param).value)
            return load_rgb_image(str(image_path))

        if self._latest_rgb is None:
            return None

        image_bgr = self.bridge.imgmsg_to_cv2(self._latest_rgb, desired_encoding='bgr8')
        return image_bgr[:, :, ::-1]

    @staticmethod
    def _set_pixel_result(result, found, pixel=None):
        result.found = found
        if pixel is None:
            result.pixel_x = -1
            result.pixel_y = -1
        else:
            result.pixel_x = int(pixel[0])
            result.pixel_y = int(pixel[1])
        return result

    async def _execute_find_box_grasp_point(self, goal_handle):
        feedback = FindBoxGraspPoint.Feedback()
        feedback.status = 'finding box grasp point'
        goal_handle.publish_feedback(feedback)

        result = FindBoxGraspPoint.Result()
        image_rgb = self._get_image_rgb('box_test_image_path')
        if image_rgb is None:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        side = goal_handle.request.side.strip().lower() or 'left'
        grasp_result = self.box_grasp_detector.find_grasp_point(image_rgb, side=side)
        if grasp_result is None:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        goal_handle.succeed()
        return self._set_pixel_result(result, True, grasp_result.grasp_pixel)

    async def _execute_find_cabinet_handle(self, goal_handle):
        feedback = FindCabinetHandle.Feedback()
        feedback.status = 'finding cabinet handle'
        goal_handle.publish_feedback(feedback)

        result = FindCabinetHandle.Result()
        image_rgb = self._get_image_rgb('cabinet_test_image_path')
        if image_rgb is None:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        handle_result = self.cabinet_handle_detector.detect(image_rgb)
        if handle_result is None:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        goal_handle.succeed()
        return self._set_pixel_result(result, True, handle_result.center_pixel)

    async def _execute_find_object(self, goal_handle):
        feedback = FindObject.Feedback()
        feedback.status = 'finding object'
        goal_handle.publish_feedback(feedback)

        result = FindObject.Result()
        object_name = goal_handle.request.object_name.strip()
        if not object_name:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        image_rgb = self._get_image_rgb('object_test_image_path')
        if image_rgb is None:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        detection = self.owl_detector.best_detection(image_rgb, [object_name])
        if detection is None:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        goal_handle.succeed()
        return self._set_pixel_result(result, True, detection.center_pixel)


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
