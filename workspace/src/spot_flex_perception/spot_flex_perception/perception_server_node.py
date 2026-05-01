"""
Perception action server node.

Runs simple perception actions and returns image pixel coordinates.
"""

from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from spot_flex_msgs.action import FindBoxGraspPoint, FindCabinetHandle, FindObject
from spot_flex_msgs.srv import GraspPixel

from spot_flex_perception.box_grasp_detector import BoxGraspDetector, draw_box_grasp_result
from spot_flex_perception.cabinet_handle_detector import (
    CabinetHandleDetector,
    draw_cabinet_handle_result,
)
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
        self.declare_parameter('open_gripper_before_box_image', False)
        self.declare_parameter('require_box_gripper_open', True)
        self.declare_parameter('open_gripper_service', '/spot/open_gripper')
        self.declare_parameter('gripper_settle_sec', 1.5)
        self.declare_parameter('box_detection_retry_count', 2)
        self.declare_parameter('box_detection_retry_delay_sec', 0.5)
        self.declare_parameter('image_wait_timeout_sec', 2.0)
        self.declare_parameter('grasp_box_after_detection', True)
        self.declare_parameter('grasp_handle_after_detection', True)
        self.declare_parameter('grasp_pixel_service', '/spot/grasp_pixel')
        self.declare_parameter('grasp_image_source', 'hand_color_image')
        self.declare_parameter('save_debug_images', True)
        self.declare_parameter('debug_image_dir', '/repo/workspace/perception_debug')

        self.owl_detector = OwlDetector()
        self.box_grasp_detector = BoxGraspDetector(owl_detector=self.owl_detector)
        self.cabinet_handle_detector = CabinetHandleDetector()
        self.get_logger().info(f'OWLv2 model source: {self.owl_detector.model_ref}')

        self._latest_rgb = None
        self._latest_rgb_received_time = None
        self._sleep_timers = set()
        self._open_gripper_client = self.create_client(
            Trigger,
            self.get_parameter('open_gripper_service').value,
            callback_group=self.cb_group,
        )
        self._grasp_pixel_client = self.create_client(
            GraspPixel,
            self.get_parameter('grasp_pixel_service').value,
            callback_group=self.cb_group,
        )

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
        self._latest_rgb_received_time = self.get_clock().now()

    def _get_image_rgb(self, test_image_param, min_received_time=None):
        if self.get_parameter('use_test_images').value:
            image_path = Path(self.get_parameter(test_image_param).value)
            return load_rgb_image(str(image_path)), None

        if self._latest_rgb is None:
            return None, None
        if (
            min_received_time is not None
            and self._latest_rgb_received_time is not None
            and self._latest_rgb_received_time.nanoseconds <= min_received_time.nanoseconds
        ):
            return None, None

        image_bgr = self.bridge.imgmsg_to_cv2(self._latest_rgb, desired_encoding='bgr8')
        return image_bgr[:, :, ::-1].copy(), self._latest_rgb_received_time

    def _box_debug_stem(self, attempt, side, image_received_time=None, suffix='captured'):
        stamp = self.get_clock().now().nanoseconds
        if image_received_time is not None:
            stamp = image_received_time.nanoseconds
        return f'box_grasp_{stamp}_attempt{attempt}_{side}_{suffix}'

    def _save_box_capture_image(self, image_rgb, attempt, side, image_received_time=None):
        if not self.get_parameter('save_debug_images').value:
            return None

        try:
            output_dir = Path(self.get_parameter('debug_image_dir').value)
            output_dir.mkdir(parents=True, exist_ok=True)
            stem = self._box_debug_stem(attempt, side, image_received_time)
            raw_path = output_dir / f'{stem}_raw.jpg'
            cv2.imwrite(str(raw_path), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
            self.get_logger().info(f'Saved captured box image: {raw_path}')
            return stem
        except Exception as exc:
            self.get_logger().warning(f'Failed to save captured box image: {exc}')
            return None

    def _save_box_overlay_image(self, image_rgb, grasp_result, attempt, side, capture_stem=None):
        if not self.get_parameter('save_debug_images').value:
            return

        try:
            output_dir = Path(self.get_parameter('debug_image_dir').value)
            output_dir.mkdir(parents=True, exist_ok=True)
            if grasp_result is None:
                stem = capture_stem or self._box_debug_stem(attempt, side, suffix='no_detection')
            else:
                x, y = grasp_result.grasp_pixel
                stem = capture_stem or self._box_debug_stem(attempt, side, suffix=f'{x}_{y}')

            overlay_path = output_dir / f'{stem}_overlay.jpg'
            if grasp_result is None:
                cv2.imwrite(str(overlay_path), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
                self.get_logger().info(f'Saved no-detection box overlay image: {overlay_path}')
                return
            draw_box_grasp_result(image_rgb, grasp_result, str(overlay_path))
            self.get_logger().info(f'Saved box detection overlay: {overlay_path}')
        except Exception as exc:
            self.get_logger().warning(f'Failed to save box detection overlay: {exc}')

    async def _sleep(self, delay_sec):
        if delay_sec <= 0.0:
            return

        future = Future()

        def _wake():
            if not future.done():
                future.set_result(None)
            self.destroy_timer(timer)
            self._sleep_timers.discard(timer)

        timer = self.create_timer(delay_sec, _wake, callback_group=self.cb_group)
        self._sleep_timers.add(timer)
        await future

    async def _wait_for_image_rgb(self, test_image_param, min_received_time=None):
        timeout_sec = float(self.get_parameter('image_wait_timeout_sec').value)
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)

        while rclpy.ok():
            image_rgb, received_time = self._get_image_rgb(test_image_param, min_received_time)
            if image_rgb is not None:
                return image_rgb, received_time
            if self.get_clock().now().nanoseconds >= deadline:
                return None, None
            await self._sleep(0.05)

        return None, None

    async def _open_gripper_for_box_image(self, goal_handle):
        if not self.get_parameter('open_gripper_before_box_image').value:
            return True, self.get_clock().now()

        feedback = FindBoxGraspPoint.Feedback()
        feedback.status = 'opening gripper before box image'
        goal_handle.publish_feedback(feedback)

        service_name = self.get_parameter('open_gripper_service').value
        if not self._open_gripper_client.service_is_ready():
            self.get_logger().info(f"Waiting for service '{service_name}'")
            if not self._open_gripper_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service '{service_name}' is not available")
                return not self.get_parameter('require_box_gripper_open').value, self.get_clock().now()

        try:
            response = await self._open_gripper_client.call_async(Trigger.Request())
        except Exception as exc:  # pragma: no cover - defensive ROS service boundary
            self.get_logger().error(f"Failed to call '{service_name}': {exc}")
            return not self.get_parameter('require_box_gripper_open').value, self.get_clock().now()

        if not response.success:
            self.get_logger().error(
                f"Service '{service_name}' reported failure: {response.message}"
            )
            return not self.get_parameter('require_box_gripper_open').value, self.get_clock().now()

        settle_sec = float(self.get_parameter('gripper_settle_sec').value)
        if settle_sec > 0.0:
            await self._sleep(settle_sec)
        image_cutoff = self.get_clock().now()
        self.get_logger().info(
            'Gripper open command complete; waiting for box image after '
            f'{image_cutoff.nanoseconds}'
        )
        return True, image_cutoff

    async def _grasp_at_pixel(self, pixel, enabled_param: str, label: str) -> tuple[bool, str]:
        if not self.get_parameter(enabled_param).value:
            return True, 'grasp disabled'

        service_name = str(self.get_parameter('grasp_pixel_service').value)
        if not self._grasp_pixel_client.service_is_ready():
            self.get_logger().info(f"Waiting for service '{service_name}'")
            if not self._grasp_pixel_client.wait_for_service(timeout_sec=5.0):
                return False, f'service {service_name} unavailable'

        request = GraspPixel.Request()
        request.pixel_x = int(pixel[0])
        request.pixel_y = int(pixel[1])
        request.image_source = str(self.get_parameter('grasp_image_source').value)
        self.get_logger().info(
            f'Requesting Spot {label} grasp at pixel '
            f'({request.pixel_x}, {request.pixel_y}) from {request.image_source}'
        )

        try:
            response = await self._grasp_pixel_client.call_async(request)
        except Exception as exc:
            return False, f'service {service_name} call failed: {exc}'

        return bool(response.success), response.message

    async def _grasp_box_at_pixel(self, pixel) -> tuple[bool, str]:
        return await self._grasp_at_pixel(pixel, 'grasp_box_after_detection', 'box')

    async def _grasp_handle_at_pixel(self, pixel) -> tuple[bool, str]:
        return await self._grasp_at_pixel(pixel, 'grasp_handle_after_detection', 'handle')

    def _save_cabinet_handle_debug_image(self, image_rgb, handle_result):
        if not self.get_parameter('save_debug_images').value:
            return

        try:
            output_dir = Path(self.get_parameter('debug_image_dir').value)
            output_dir.mkdir(parents=True, exist_ok=True)
            stamp = self.get_clock().now().nanoseconds
            if handle_result is None:
                path = output_dir / f'cabinet_handle_{stamp}_no_detection.jpg'
                cv2.imwrite(str(path), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
                self.get_logger().info(f'Saved cabinet handle image: {path}')
                return

            x, y = handle_result.center_pixel
            path = output_dir / f'cabinet_handle_{stamp}_{x}_{y}_overlay.jpg'
            draw_cabinet_handle_result(image_rgb, handle_result, str(path))
            self.get_logger().info(f'Saved cabinet handle overlay: {path}')
        except Exception as exc:
            self.get_logger().warning(f'Failed to save cabinet handle debug image: {exc}')

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
        gripper_open, image_time_cutoff = await self._open_gripper_for_box_image(goal_handle)
        if not gripper_open:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        side = goal_handle.request.side.strip().lower() or 'right'
        self.get_logger().info(f'Finding box grasp point on {side} edge')
        retry_count = max(1, int(self.get_parameter('box_detection_retry_count').value))
        retry_delay_sec = float(self.get_parameter('box_detection_retry_delay_sec').value)
        grasp_result = None

        for attempt in range(1, retry_count + 1):
            feedback.status = f'capturing box image ({attempt}/{retry_count})'
            goal_handle.publish_feedback(feedback)
            image_rgb, image_received_time = await self._wait_for_image_rgb(
                'box_test_image_path',
                image_time_cutoff,
            )
            if image_rgb is None:
                self.get_logger().warning('Timed out waiting for a box image')
                break
            capture_stem = self._save_box_capture_image(
                image_rgb,
                attempt,
                side,
                image_received_time,
            )
            if image_received_time is not None:
                self.get_logger().info(
                    'Using fresh box image received at '
                    f'{image_received_time.nanoseconds}'
                )
                image_time_cutoff = image_received_time

            feedback.status = f'detecting box grasp point ({attempt}/{retry_count})'
            goal_handle.publish_feedback(feedback)
            try:
                grasp_result = self.box_grasp_detector.find_grasp_point(image_rgb, side=side)
            except Exception as exc:
                self.get_logger().error(f'Box grasp detection failed: {exc}')
                goal_handle.abort()
                return self._set_pixel_result(result, False)
            self._save_box_overlay_image(image_rgb, grasp_result, attempt, side, capture_stem)
            if grasp_result is not None:
                self.get_logger().info(
                    'Box grasp point found at '
                    f'({grasp_result.grasp_pixel[0]}, {grasp_result.grasp_pixel[1]}) '
                    f'on {grasp_result.side} edge '
                    f'from {grasp_result.owl_detection.label} '
                    f'{grasp_result.owl_detection.confidence:.2f}'
                )
                break

            self.get_logger().warning(
                f'Box grasp detection failed on attempt {attempt}/{retry_count}'
            )
            if attempt < retry_count and retry_delay_sec > 0.0:
                await self._sleep(retry_delay_sec)

        if grasp_result is None:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        feedback.status = 'grasping box edge'
        goal_handle.publish_feedback(feedback)
        grasp_success, grasp_message = await self._grasp_box_at_pixel(grasp_result.grasp_pixel)
        if not grasp_success:
            self.get_logger().error(f'Box grasp failed: {grasp_message}')
            goal_handle.abort()
            return self._set_pixel_result(result, False)
        self.get_logger().info(f'Box grasp succeeded: {grasp_message}')

        goal_handle.succeed()
        return self._set_pixel_result(result, True, grasp_result.grasp_pixel)

    async def _execute_find_cabinet_handle(self, goal_handle):
        feedback = FindCabinetHandle.Feedback()
        feedback.status = 'finding cabinet handle'
        goal_handle.publish_feedback(feedback)

        result = FindCabinetHandle.Result()
        image_rgb, _ = self._get_image_rgb('cabinet_test_image_path')
        if image_rgb is None:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        handle_result = self.cabinet_handle_detector.detect(image_rgb)
        self._save_cabinet_handle_debug_image(image_rgb, handle_result)
        if handle_result is None:
            goal_handle.abort()
            return self._set_pixel_result(result, False)

        feedback.status = 'grasping cabinet handle'
        goal_handle.publish_feedback(feedback)
        grasp_success, grasp_message = await self._grasp_handle_at_pixel(
            handle_result.center_pixel
        )
        if not grasp_success:
            self.get_logger().error(f'Cabinet handle grasp failed: {grasp_message}')
            goal_handle.abort()
            return self._set_pixel_result(result, False)
        self.get_logger().info(f'Cabinet handle grasp succeeded: {grasp_message}')

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

        image_rgb, _ = self._get_image_rgb('object_test_image_path')
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
