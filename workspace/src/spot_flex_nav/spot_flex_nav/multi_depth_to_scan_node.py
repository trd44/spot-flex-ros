"""Fuse N depth cameras into a single 360°-capable LaserScan.

For each configured camera:
  * subscribe to depth image + camera_info
  * extract a horizontal band, back-project each pixel to a 3D point in the
    camera's optical frame
  * on a fixed cadence, transform all points to a common scan frame (usually
    ``spot/body`` or ``base_link``), convert each to (bearing, range), and bin
    into a single 360° LaserScan

This lets Nav2 + slam_toolbox see everything Spot's 5 stereo depth cameras see,
instead of only the front-left slice. Range/FOV caps follow slam_toolbox
params so the fused scan stays in the reliable band for stereo depth.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


@dataclass
class CameraState:
    depth_topic: str
    info_topic: str
    camera_info: Optional[CameraInfo] = None
    last_depth: Optional[Image] = None
    # Cache 3D points (x, y, z in camera optical frame) between depth updates.
    last_points: Optional[np.ndarray] = None
    last_frame_id: str = ''
    # A subscription handle list — kept so rclpy does not garbage-collect them.
    _subs: list = field(default_factory=list)


class MultiDepthToScanNode(Node):
    """Fuse N Spot depth cameras into a single 360° LaserScan."""

    def __init__(self) -> None:
        super().__init__('multi_depth_to_scan')

        # Cameras to subscribe to — one entry per Spot body camera. Omit any
        # camera whose topics are not published on your robot.
        self.declare_parameter(
            'cameras',
            ['frontleft', 'frontright', 'left', 'right', 'back'],
        )
        self.declare_parameter('depth_topic_template', '/spot/depth/{camera}/image')
        self.declare_parameter('info_topic_template', '/spot/depth/{camera}/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_frame_id', 'base_link')

        # Same meaning as depth_to_scan_node.
        self.declare_parameter('scan_height_pixels', 40)
        self.declare_parameter('row_center_fraction', 0.5)
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 2.5)
        self.declare_parameter('depth_scale_16uc1', 0.001)

        # Horizontal ray resolution of the output scan.
        self.declare_parameter('angular_resolution_deg', 1.0)
        # Output scan rate.
        self.declare_parameter('publish_rate_hz', 10.0)
        # Sub-sample depth columns to keep the transform math cheap.
        self.declare_parameter('column_stride', 4)
        # Drop points outside this vertical band (in the scan frame) so floor /
        # ceiling hits don't appear as obstacles.
        self.declare_parameter('z_min', -0.1)
        self.declare_parameter('z_max',  0.8)

        cameras: List[str] = list(self.get_parameter('cameras').value)
        depth_tpl = str(self.get_parameter('depth_topic_template').value)
        info_tpl = str(self.get_parameter('info_topic_template').value)
        scan_topic = str(self.get_parameter('scan_topic').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cameras: Dict[str, CameraState] = {}
        for camera in cameras:
            state = CameraState(
                depth_topic=depth_tpl.format(camera=camera),
                info_topic=info_tpl.format(camera=camera),
            )
            state._subs.append(self.create_subscription(
                Image, state.depth_topic,
                lambda msg, c=camera: self._on_depth(c, msg),
                qos_profile_sensor_data,
            ))
            state._subs.append(self.create_subscription(
                CameraInfo, state.info_topic,
                lambda msg, c=camera: self._on_info(c, msg),
                qos_profile_sensor_data,
            ))
            self._cameras[camera] = state
            self.get_logger().info(f'[{camera}] subscribing to {state.depth_topic} + {state.info_topic}')

        self._scan_pub = self.create_publisher(LaserScan, scan_topic, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._publish_scan)

    # --- callbacks ---------------------------------------------------------

    def _on_info(self, camera: str, msg: CameraInfo) -> None:
        self._cameras[camera].camera_info = msg

    def _on_depth(self, camera: str, msg: Image) -> None:
        state = self._cameras[camera]
        state.last_depth = msg
        state.last_frame_id = msg.header.frame_id
        state.last_points = self._depth_to_points(state, msg)

    # --- fusion ------------------------------------------------------------

    def _publish_scan(self) -> None:
        scan_frame = str(self.get_parameter('scan_frame_id').value)
        range_min = float(self.get_parameter('range_min').value)
        range_max = float(self.get_parameter('range_max').value)
        z_min = float(self.get_parameter('z_min').value)
        z_max = float(self.get_parameter('z_max').value)
        ang_res = math.radians(float(self.get_parameter('angular_resolution_deg').value))

        num_bins = max(8, int(round(2.0 * math.pi / ang_res)))
        bearings = np.linspace(-math.pi, math.pi, num_bins, endpoint=False, dtype=np.float32)
        scan_ranges = np.full(num_bins, np.inf, dtype=np.float32)
        latest_stamp = None

        for camera, state in self._cameras.items():
            if state.last_points is None or state.last_depth is None:
                continue
            points_cam = state.last_points  # (N,3) in camera optical frame
            if points_cam.size == 0:
                continue

            try:
                tf = self._tf_buffer.lookup_transform(
                    scan_frame, state.last_frame_id, rclpy.time.Time()
                )
            except TransformException as exc:
                self.get_logger().warning(
                    f'[{camera}] TF {scan_frame} <- {state.last_frame_id} unavailable: {exc}',
                    throttle_duration_sec=5.0,
                )
                continue

            points_scan = self._transform_points(points_cam, tf)

            # Drop vertical outliers (floor / ceiling / overhangs).
            z = points_scan[:, 2]
            m = (z >= z_min) & (z <= z_max)
            if not np.any(m):
                continue
            x = points_scan[m, 0]
            y = points_scan[m, 1]

            r = np.sqrt(x * x + y * y)
            ok = (r >= range_min) & (r <= range_max)
            if not np.any(ok):
                continue
            x = x[ok]; y = y[ok]; r = r[ok]

            theta = np.arctan2(y, x).astype(np.float32)
            # Bin into the global bearing table, keep min range per bin.
            bins = ((theta + math.pi) / ang_res).astype(np.int64) % num_bins
            # np.minimum.at is the scatter-min primitive.
            np.minimum.at(scan_ranges, bins, r.astype(np.float32))

            if latest_stamp is None or _stamp_gt(state.last_depth.header.stamp, latest_stamp):
                latest_stamp = state.last_depth.header.stamp

        if latest_stamp is None:
            return

        scan = LaserScan()
        scan.header.stamp = latest_stamp
        scan.header.frame_id = scan_frame
        scan.angle_min = float(bearings[0])
        scan.angle_max = float(bearings[-1] + ang_res)
        scan.angle_increment = float(ang_res)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = scan_ranges.tolist()
        self._scan_pub.publish(scan)

    # --- helpers -----------------------------------------------------------

    def _depth_to_points(self, state: CameraState, msg: Image) -> Optional[np.ndarray]:
        info = state.camera_info
        if info is None or info.k[0] <= 0.0:
            return None
        depth = self._image_to_depth_array(msg)
        if depth is None:
            return None

        height, width = depth.shape
        if height == 0 or width == 0:
            return None

        band_height = int(self.get_parameter('scan_height_pixels').value)
        band_height = max(1, min(band_height, height))
        center_frac = float(self.get_parameter('row_center_fraction').value)
        center_frac = max(0.0, min(center_frac, 1.0))
        center = int(round(center_frac * (height - 1)))
        start = max(0, min(height - band_height, center - band_height // 2))
        band = depth[start:start + band_height, :]

        stride = max(1, int(self.get_parameter('column_stride').value))
        band = band[:, ::stride]
        cols = np.arange(0, width, stride, dtype=np.float32)

        fx = float(info.k[0]); fy = float(info.k[4])
        cx = float(info.k[2]); cy = float(info.k[5])
        rows = np.arange(start, start + band_height, dtype=np.float32)

        # Build (h*w_sub, 3) point array in the camera optical frame:
        # z forward, x right, y down.
        z = band.astype(np.float32)
        u = np.broadcast_to(cols, z.shape)
        v = np.broadcast_to(rows[:, None], z.shape)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        pts = np.stack([x, y, z], axis=-1).reshape(-1, 3)
        # Drop invalid / zero depth rows.
        valid = np.isfinite(pts).all(axis=1) & (pts[:, 2] > 0.0)
        return pts[valid]

    def _image_to_depth_array(self, msg: Image) -> Optional[np.ndarray]:
        enc = msg.encoding.upper()
        if enc in ('32FC1', '32FC'):
            dtype, scale = np.float32, 1.0
        elif enc in ('64FC1', '64FC'):
            dtype, scale = np.float64, 1.0
        elif enc in ('16UC1', 'MONO16'):
            dtype = np.uint16
            scale = float(self.get_parameter('depth_scale_16uc1').value)
        else:
            self.get_logger().warning(
                f'Unsupported depth encoding: {enc}', throttle_duration_sec=5.0)
            return None
        item_size = np.dtype(dtype).itemsize
        row_items = msg.step // item_size
        try:
            raw = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, row_items))
        except ValueError:
            return None
        depth = raw[:, :msg.width].astype(np.float32) * scale
        depth[depth <= 0.0] = np.nan
        return depth

    @staticmethod
    def _transform_points(pts: np.ndarray, tf) -> np.ndarray:
        # Spot's *optical* frame convention is z-forward / x-right / y-down,
        # which matches ROS camera optical frames. The URDF ties each camera's
        # optical frame to its body link, so the TF we looked up already does
        # the right axis conversion.
        t = tf.transform.translation
        q = tf.transform.rotation
        # Build 3x3 rotation from quaternion (x, y, z, w).
        x, y, z, w = q.x, q.y, q.z, q.w
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        R = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
            [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
        ], dtype=np.float32)
        T = np.array([t.x, t.y, t.z], dtype=np.float32)
        return pts @ R.T + T


def _stamp_gt(a, b) -> bool:
    return (a.sec, a.nanosec) > (b.sec, b.nanosec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiDepthToScanNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
