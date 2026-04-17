import argparse
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from spot_flex_nav.location_store import default_locations_file, load_locations, save_locations


class LocationTagger(Node):
    def __init__(self) -> None:
        super().__init__('tag_location')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def lookup_pose(self, target_frame: str, source_frame: str):
        deadline = self.get_clock().now() + Duration(seconds=5.0)
        last_error = None
        while rclpy.ok() and self.get_clock().now() < deadline:
            try:
                return self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
            except TransformException as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f'Could not look up {target_frame} -> {source_frame}: {last_error}')


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Tag the current robot pose as a named map location.')
    parser.add_argument('name', help='Location name, for example cabinet')
    parser.add_argument('--file', default=str(default_locations_file()), help='YAML location file')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = LocationTagger()
    try:
        tf_msg = node.lookup_pose(args.map_frame, args.base_frame)
        path = Path(args.file).expanduser()
        data = load_locations(path)
        translation = tf_msg.transform.translation
        rotation = tf_msg.transform.rotation
        data['locations'][args.name] = {
            'frame_id': args.map_frame,
            'pose': {
                'position': {
                    'x': float(translation.x),
                    'y': float(translation.y),
                    'z': float(translation.z),
                },
                'orientation': {
                    'x': float(rotation.x),
                    'y': float(rotation.y),
                    'z': float(rotation.z),
                    'w': float(rotation.w),
                },
            },
        }
        save_locations(path, data)
        node.get_logger().info(f"Tagged location '{args.name}' in {path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
