import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class DepthToScanNode(Node):
    """Project a horizontal band from a depth image into a 2D LaserScan."""

    def __init__(self) -> None:
        super().__init__('depth_to_scan')

        self.declare_parameter('depth_topic', '/spot/depth/frontleft/image_raw')
        self.declare_parameter('camera_info_topic', '/spot/depth/frontleft/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_frame_id', 'base_link')
        self.declare_parameter('horizontal_fov', 1.35)
        self.declare_parameter('scan_height_pixels', 18)
        self.declare_parameter('row_center_fraction', 0.5)
        self.declare_parameter('range_min', 0.2)
        self.declare_parameter('range_max', 4.0)
        self.declare_parameter('depth_scale_16uc1', 0.001)
        self.declare_parameter('publish_rate_hz', 15.0)

        self._camera_info: Optional[CameraInfo] = None
        self._last_depth: Optional[Image] = None

        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self._scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.create_subscription(Image, depth_topic, self._depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, qos_profile_sensor_data)

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._publish_scan)

        self.get_logger().info(
            f'Depth-to-scan: {depth_topic} + {camera_info_topic} -> {scan_topic}'
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _depth_callback(self, msg: Image) -> None:
        self._last_depth = msg

    def _publish_scan(self) -> None:
        if self._last_depth is None:
            return

        depth = self._image_to_depth_array(self._last_depth)
        if depth is None:
            return

        height, width = depth.shape
        if height == 0 or width == 0:
            return

        range_min = float(self.get_parameter('range_min').value)
        range_max = float(self.get_parameter('range_max').value)
        band = self._extract_scan_band(depth)

        column_depth = np.nanmin(band, axis=0)
        column_depth[~np.isfinite(column_depth)] = np.inf
        column_depth[(column_depth < range_min) | (column_depth > range_max)] = np.inf

        # LaserScan ranges must be ordered from angle_min to angle_max. For a
        # front-facing camera, the rightmost image column is the most negative
        # bearing and the leftmost image column is the most positive bearing.
        column_depth = column_depth[::-1]
        angles = self._column_angles(width)
        cos_angles = np.cos(angles)
        ranges = column_depth / np.maximum(cos_angles, 0.01)
        ranges[(ranges < range_min) | (ranges > range_max)] = np.inf

        scan = LaserScan()
        scan.header.stamp = self._last_depth.header.stamp
        frame_id = str(self.get_parameter('scan_frame_id').value)
        scan.header.frame_id = frame_id if frame_id else self._last_depth.header.frame_id
        scan.angle_min = float(angles[0])
        scan.angle_max = float(angles[-1])
        scan.angle_increment = float((scan.angle_max - scan.angle_min) / max(width - 1, 1))
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = ranges.astype(np.float32).tolist()
        self._scan_pub.publish(scan)

    def _extract_scan_band(self, depth: np.ndarray) -> np.ndarray:
        height = depth.shape[0]
        scan_height = int(self.get_parameter('scan_height_pixels').value)
        scan_height = max(1, min(scan_height, height))
        center_fraction = float(self.get_parameter('row_center_fraction').value)
        center_fraction = max(0.0, min(center_fraction, 1.0))
        center = int(round(center_fraction * (height - 1)))
        start = max(0, center - scan_height // 2)
        end = min(height, start + scan_height)
        start = max(0, end - scan_height)
        return depth[start:end, :]

    def _column_angles(self, width: int) -> np.ndarray:
        if self._camera_info is not None and self._camera_info.k[0] > 0.0:
            fx = self._camera_info.k[0]
            cx = self._camera_info.k[2]
            columns = np.arange(width - 1, -1, -1, dtype=np.float32)
            return np.arctan2(cx - columns, fx).astype(np.float32)

        horizontal_fov = float(self.get_parameter('horizontal_fov').value)
        return np.linspace(-horizontal_fov / 2.0, horizontal_fov / 2.0, width, dtype=np.float32)

    def _image_to_depth_array(self, msg: Image) -> Optional[np.ndarray]:
        encoding = msg.encoding.upper()
        try:
            dtype, scale = self._encoding_dtype_and_scale(encoding)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return None

        item_size = np.dtype(dtype).itemsize
        row_items = msg.step // item_size
        try:
            raw = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, row_items))
        except ValueError as exc:
            self.get_logger().warning(f'Could not reshape depth image: {exc}', throttle_duration_sec=5.0)
            return None

        depth = raw[:, :msg.width].astype(np.float32) * scale
        depth[depth <= 0.0] = np.nan
        return depth

    def _encoding_dtype_and_scale(self, encoding: str) -> Tuple[np.dtype, float]:
        if encoding in ('32FC1', '32FC'):
            return np.float32, 1.0
        if encoding in ('64FC1', '64FC'):
            return np.float64, 1.0
        if encoding in ('16UC1', 'MONO16'):
            return np.uint16, float(self.get_parameter('depth_scale_16uc1').value)
        raise ValueError(f'Unsupported depth image encoding: {encoding}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthToScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
