import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from spot_msgs.srv import GraphNavGetLocalizationPose

from spot_flex_nav.location_store import default_locations_file, load_locations, save_locations


class GraphNavLocationTagger(Node):
    def __init__(self) -> None:
        super().__init__('tag_graphnav_location')

    def get_localization_pose(self, service_name: str):
        client = self.create_client(GraphNavGetLocalizationPose, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'service {service_name} unavailable')

        future = client.call_async(GraphNavGetLocalizationPose.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            raise RuntimeError(f'{service_name} returned no response')
        if not response.success:
            raise RuntimeError(response.message)
        return response.pose


def _pose_to_dict(pose_stamped):
    position = pose_stamped.pose.position
    orientation = pose_stamped.pose.orientation
    return {
        'frame_id': pose_stamped.header.frame_id,
        'pose': {
            'position': {
                'x': float(position.x),
                'y': float(position.y),
                'z': float(position.z),
            },
            'orientation': {
                'x': float(orientation.x),
                'y': float(orientation.y),
                'z': float(orientation.z),
                'w': float(orientation.w),
            },
        },
    }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Tag a semantic demo location with a GraphNav waypoint id or annotation name.',
    )
    parser.add_argument('name', help='Location name, for example box, cabinet, or dropoff')
    parser.add_argument('waypoint', help='GraphNav waypoint id, short code, or unique annotation name')
    parser.add_argument('--file', default=str(default_locations_file()), help='YAML location file')
    parser.add_argument(
        '--capture-localization-pose',
        action='store_true',
        help='Also store the current graph_nav_get_localization_pose pose for RViz/Nav2 fallback.',
    )
    parser.add_argument(
        '--localization-service',
        default='graph_nav_get_localization_pose',
        help='GraphNav localization pose service name',
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = GraphNavLocationTagger()
    try:
        path = Path(args.file).expanduser()
        data = load_locations(path)
        entry = {'graphnav_waypoint': args.waypoint}
        if args.capture_localization_pose:
            entry.update(_pose_to_dict(node.get_localization_pose(args.localization_service)))
        data['locations'][args.name] = entry
        save_locations(path, data)
        node.get_logger().info(
            f"Tagged GraphNav location '{args.name}' as '{args.waypoint}' in {path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
