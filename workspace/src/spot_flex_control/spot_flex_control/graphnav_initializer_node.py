"""One-shot GraphNav setup node.

Uploads the configured GraphNav map and sets initial localization before the
task conductor sends navigation goals.
"""

import sys
import time

import rclpy
from rclpy.node import Node

from spot_msgs.srv import GraphNavSetLocalization, GraphNavUploadGraph, ListGraph


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('1', 'true', 'yes', 'on')


class GraphNavInitializerNode(Node):
    def __init__(self):
        super().__init__('graphnav_initializer_node')

        self.declare_parameter('upload_graph', True)
        self.declare_parameter('refresh_waypoint_names', True)
        self.declare_parameter('map_path', '/repo/workspace/maps/demo.walk')
        self.declare_parameter('set_localization', True)
        self.declare_parameter('localization_method', 'fiducial')
        self.declare_parameter('localization_waypoint', '')
        self.declare_parameter('upload_graph_service', '/spot/graph_nav_upload_graph')
        self.declare_parameter('list_graph_service', '/spot/list_graph')
        self.declare_parameter('set_localization_service', '/spot/graph_nav_set_localization')
        self.declare_parameter('service_wait_timeout_sec', 120.0)
        self.declare_parameter('localization_retry_count', 3)
        self.declare_parameter('localization_retry_delay_sec', 2.0)

    def run(self) -> bool:
        if _as_bool(self.get_parameter('upload_graph').value):
            if not self._upload_graph():
                return False

        if _as_bool(self.get_parameter('refresh_waypoint_names').value):
            if not self._refresh_waypoint_names():
                return False

        if _as_bool(self.get_parameter('set_localization').value):
            if not self._set_localization():
                return False

        self.get_logger().info('GraphNav initialization complete')
        return True

    def _upload_graph(self) -> bool:
        service = str(self.get_parameter('upload_graph_service').value)
        map_path = str(self.get_parameter('map_path').value)
        client = self.create_client(GraphNavUploadGraph, service)

        if not self._wait_for_service(client, service):
            return False

        self.get_logger().info(f'Uploading GraphNav map from {map_path}')
        request = GraphNavUploadGraph.Request()
        request.upload_filepath = map_path
        response = self._call(client, request)
        if response is None or not response.success:
            message = getattr(response, 'message', 'no response')
            self.get_logger().error(f'GraphNav map upload failed: {message}')
            return False

        self.get_logger().info('GraphNav map upload complete')
        return True

    def _refresh_waypoint_names(self) -> bool:
        service = str(self.get_parameter('list_graph_service').value)
        map_path = str(self.get_parameter('map_path').value)
        client = self.create_client(ListGraph, service)

        if not self._wait_for_service(client, service):
            return False

        self.get_logger().info('Refreshing GraphNav waypoint name cache')
        request = ListGraph.Request()
        request.upload_filepath = map_path
        response = self._call(client, request)
        if response is None:
            self.get_logger().error('GraphNav waypoint refresh failed: no response')
            return False

        self.get_logger().info(
            f'GraphNav waypoint refresh complete: {list(response.waypoint_ids)}')
        return True

    def _set_localization(self) -> bool:
        service = str(self.get_parameter('set_localization_service').value)
        method = str(self.get_parameter('localization_method').value).strip() or 'fiducial'
        waypoint = str(self.get_parameter('localization_waypoint').value).strip()
        retries = max(1, int(self.get_parameter('localization_retry_count').value))
        delay = max(0.0, float(self.get_parameter('localization_retry_delay_sec').value))

        client = self.create_client(GraphNavSetLocalization, service)
        if not self._wait_for_service(client, service):
            return False

        for attempt in range(1, retries + 1):
            self.get_logger().info(
                f'Setting GraphNav localization by {method} '
                f'(attempt {attempt}/{retries})')
            request = GraphNavSetLocalization.Request()
            request.method = method
            request.waypoint_id = waypoint
            response = self._call(client, request)
            if response is not None and response.success:
                self.get_logger().info('GraphNav localization complete')
                return True

            message = getattr(response, 'message', 'no response')
            self.get_logger().warn(f'GraphNav localization failed: {message}')
            if attempt < retries and delay > 0.0:
                time.sleep(delay)

        return False

    def _wait_for_service(self, client, service_name: str) -> bool:
        timeout = float(self.get_parameter('service_wait_timeout_sec').value)
        self.get_logger().info(f'Waiting for {service_name}')
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service unavailable: {service_name}')
            return False
        return True

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = GraphNavInitializerNode()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
