"""Conductor: hosts /fetch_item and runs the demo FSM on each goal."""

import math
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from spot_flex_msgs.action import FetchItem
from yasmin import Blackboard

from spot_flex_plan.demo_fsm import build_demo_fsm


DEFAULT_WAYPOINTS = 'demo_waypoints.yaml'
POLL_HZ = 10.0


def _pose_from_xy_yaw(x, y, yaw, frame='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _entry(data, key):
    locations = data.get('locations', {}) or {}
    aliases = (key,)
    if key == 'table':
        aliases = ('table', 'dropoff', 'drop_off')

    for alias in aliases:
        if alias in locations:
            return locations[alias] or {}
        if alias in data:
            return data[alias] or {}
    return {}


def _pose_entry(entry):
    pose = entry.get('pose', entry)
    position = pose.get('position', pose)
    orientation = pose.get('orientation', {})
    return position, orientation


def _pose_from_entry(entry, default_frame):
    position, orientation = _pose_entry(entry)
    frame = entry.get('frame_id', default_frame)
    if orientation:
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = float(position.get('x', 0.0))
        pose.pose.position.y = float(position.get('y', 0.0))
        pose.pose.position.z = float(position.get('z', 0.0))
        pose.pose.orientation.x = float(orientation.get('x', 0.0))
        pose.pose.orientation.y = float(orientation.get('y', 0.0))
        pose.pose.orientation.z = float(orientation.get('z', 0.0))
        pose.pose.orientation.w = float(orientation.get('w', 1.0))
        return pose

    return _pose_from_xy_yaw(
        position.get('x', 0.0), position.get('y', 0.0),
        entry.get('yaw', 0.0), frame)


def _load_waypoints(path: str, blackboard: Blackboard) -> None:
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    frame = data.get('frame_id', 'map')
    for key in ('box', 'cabinet', 'table', 'dock'):
        wp = _entry(data, key)
        blackboard[f'{key}_pose'] = _pose_from_entry(wp, frame)
    blackboard['dock_id'] = int(data.get('dock_id', 0))


class ConductorNode(Node):
    def __init__(self):
        super().__init__('conductor_node')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter(
            'waypoints_file',
            os.path.join(get_package_share_directory('spot_flex_plan'),
                         'config', DEFAULT_WAYPOINTS))
        self.declare_parameter('box_grasp_side', 'right')
        self.declare_parameter('skip_box_push', False)

        self._fetch_server = ActionServer(
            self, FetchItem, 'fetch_item',
            execute_callback=self._execute_fetch,
            callback_group=self.cb_group,
        )
        self.get_logger().info('conductor_node ready')

    def _execute_fetch(self, goal_handle):
        blackboard = Blackboard()
        blackboard['target_item'] = goal_handle.request.item_name or 'item'

        try:
            _load_waypoints(self.get_parameter('waypoints_file').value, blackboard)
        except Exception as e:
            self.get_logger().error(f'failed to load waypoints: {e}')
            goal_handle.abort()
            return FetchItem.Result(success=False, message=f'waypoint load failed: {e}')

        box_grasp_side = self.get_parameter('box_grasp_side').value
        skip_box_push = bool(self.get_parameter('skip_box_push').value)
        sm = build_demo_fsm(box_grasp_side=box_grasp_side, skip_box_push=skip_box_push)
        self.get_logger().info(
            f'starting FSM for item={blackboard["target_item"]}, '
            f'box_grasp_side={box_grasp_side}, skip_box_push={skip_box_push}'
        )

        result = {}
        def run():
            try:
                result['outcome'] = sm(blackboard)
            except Exception as e:
                result['error'] = str(e)

        worker = threading.Thread(target=run, daemon=True)
        worker.start()

        last_state = None
        period = 1.0 / POLL_HZ
        while worker.is_alive():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('cancel requested; canceling FSM')
                try:
                    sm.cancel_state()
                except Exception:
                    pass
                worker.join(timeout=5.0)
                goal_handle.canceled()
                return FetchItem.Result(success=False, message='canceled')

            current = sm.get_current_state() if hasattr(sm, 'get_current_state') else None
            if current and current != last_state:
                last_state = current
                fb = FetchItem.Feedback()
                fb.current_step = current
                fb.progress = 0.0
                goal_handle.publish_feedback(fb)
                self.get_logger().info(f'state -> {current}')

            time.sleep(period)

        if 'error' in result:
            self.get_logger().error(f'FSM crashed: {result["error"]}')
            goal_handle.abort()
            return FetchItem.Result(success=False, message=result['error'])

        outcome = result.get('outcome', 'ABORTED')
        if outcome == 'DONE':
            goal_handle.succeed()
            return FetchItem.Result(success=True, message='DONE')

        goal_handle.abort()
        return FetchItem.Result(success=False, message=outcome)


def main():
    rclpy.init()
    node = ConductorNode()
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
