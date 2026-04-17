import argparse
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from spot_flex_nav.location_store import default_locations_file, load_locations


class LocationNavigator(Node):
    def __init__(self) -> None:
        super().__init__('go_to_location')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def go_to(self, name: str, locations_file: Path) -> None:
        data = load_locations(locations_file)
        if name not in data['locations']:
            known = ', '.join(sorted(data['locations'].keys())) or '<none>'
            raise RuntimeError(f"Unknown location '{name}'. Known locations: {known}")

        entry = data['locations'][name]
        pose_data = entry['pose']

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = entry.get('frame_id', 'map')
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose_data['position']['x'])
        goal.pose.pose.position.y = float(pose_data['position']['y'])
        goal.pose.pose.position.z = float(pose_data['position'].get('z', 0.0))
        goal.pose.pose.orientation.x = float(pose_data['orientation'].get('x', 0.0))
        goal.pose.pose.orientation.y = float(pose_data['orientation'].get('y', 0.0))
        goal.pose.pose.orientation.z = float(pose_data['orientation'].get('z', 0.0))
        goal.pose.pose.orientation.w = float(pose_data['orientation'].get('w', 1.0))

        self.get_logger().info(f"Waiting for Nav2 action server, then navigating to '{name}'")
        self._client.wait_for_server()
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('Nav2 rejected the goal')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"Navigation finished with status {result.status}")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Send Nav2 to a named saved location.')
    parser.add_argument('name', help='Location name, for example cabinet')
    parser.add_argument('--file', default=str(default_locations_file()), help='YAML location file')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = LocationNavigator()
    try:
        node.go_to(args.name, Path(args.file).expanduser())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
