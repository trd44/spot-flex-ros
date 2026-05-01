"""Navigation wrapper.

Hosts /go_to (NavigateToPose) and dispatches to one of three backends:

  trajectory  - spot_msgs/Trajectory (PoseStamped, no map needed)            [default]
  graphnav    - spot_msgs/NavigateTo (graph_nav waypoint_id from YAML)
  nav2        - nav2_msgs/NavigateToPose passed through to /navigate_to_pose

The location-name sentinel travels in NavigateToPose.Goal.behavior_tree (a
string field). For graphnav this is the lookup key into a YAML map of
location_name -> waypoint_id; the other backends ignore it.
"""

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from spot_msgs.action import NavigateTo, Trajectory


BACKENDS = ('trajectory', 'graphnav', 'nav2')


class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('backend', 'trajectory')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('trajectory_action', 'trajectory')
        self.declare_parameter('graphnav_action', 'navigate_to')
        self.declare_parameter('nav2_action', 'navigate_to_pose')
        self.declare_parameter('trajectory_duration_sec', 30.0)
        self.declare_parameter('trajectory_precise', True)

        backend = self.get_parameter('backend').value
        if backend not in BACKENDS:
            raise ValueError(f'backend must be one of {BACKENDS}, got {backend!r}')
        self._backend = backend
        self._graphnav = self._load_graphnav() if backend == 'graphnav' else {}

        if backend == 'trajectory':
            name = self.get_parameter('trajectory_action').value
            self._client = ActionClient(self, Trajectory, name, callback_group=self.cb_group)
        elif backend == 'graphnav':
            name = self.get_parameter('graphnav_action').value
            self._client = ActionClient(self, NavigateTo, name, callback_group=self.cb_group)
        else:
            name = self.get_parameter('nav2_action').value
            self._client = ActionClient(self, NavigateToPose, name, callback_group=self.cb_group)

        self._server = ActionServer(
            self, NavigateToPose, 'go_to', self._execute,
            callback_group=self.cb_group,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info(f'nav_node ready (backend={backend}, downstream={name})')

    def _load_graphnav(self):
        path = self.get_parameter('waypoints_file').value
        if not path:
            self.get_logger().warn('graphnav backend selected but waypoints_file is empty')
            return {}
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f'failed to read {path}: {e}')
            return {}
        entries = data.get('locations', data)
        out = {}
        for name, entry in entries.items():
            if not isinstance(entry, dict):
                continue
            wid = (entry.get('graphnav_waypoint')
                   or entry.get('waypoint_id')
                   or entry.get('graphnav_waypoint_id')
                   or entry.get('graphnav_annotation')
                   or '')
            if wid:
                out[name] = str(wid)

        # Let the plan call the delivery location dropoff while older files call it table.
        if 'dropoff' in out and 'table' not in out:
            out['table'] = out['dropoff']
        if 'table' in out and 'dropoff' not in out:
            out['dropoff'] = out['table']
        if 'drop_off' in out and 'dropoff' not in out:
            out['dropoff'] = out['drop_off']

        self.get_logger().info(f'graphnav waypoints: {out}')
        return out

    async def _execute(self, goal_handle):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{self._backend} backend server unavailable')
            goal_handle.abort()
            return NavigateToPose.Result()

        inner_goal = self._build_inner_goal(goal_handle)
        if inner_goal is None:
            goal_handle.abort()
            return NavigateToPose.Result()

        send_future = self._client.send_goal_async(inner_goal)
        inner_handle = await send_future
        if not inner_handle.accepted:
            self.get_logger().error('downstream rejected goal')
            goal_handle.abort()
            return NavigateToPose.Result()

        result_wrap = await inner_handle.get_result_async()
        if result_wrap.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return NavigateToPose.Result()

    def _build_inner_goal(self, goal_handle):
        req = goal_handle.request
        if self._backend == 'trajectory':
            g = Trajectory.Goal()
            g.target_pose = req.pose
            if not g.target_pose.header.frame_id:
                g.target_pose.header.frame_id = 'body'
            g.duration = Duration(
                seconds=self.get_parameter('trajectory_duration_sec').value).to_msg()
            g.precise_positioning = bool(self.get_parameter('trajectory_precise').value)
            return g

        if self._backend == 'graphnav':
            name = req.behavior_tree
            wid = self._graphnav.get(name, '')
            if not wid:
                self.get_logger().error(
                    f'no graphnav waypoint for location {name!r}; '
                    f'known: {list(self._graphnav)}')
                return None
            return NavigateTo.Goal(waypoint_id=wid)

        # nav2 pass-through
        g = NavigateToPose.Goal()
        g.pose = req.pose
        return g


def main():
    rclpy.init()
    node = NavNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
