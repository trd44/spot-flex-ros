"""Best-effort policy server for the hardware demo.

The real learned policies are not wired in yet. This node gives the planner a
stable /execute_policy action and maps a few policy names to conservative Spot
services or short body velocity commands. Keep dry_run=true until the motions
have been tested on hardware.
"""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from spot_flex_msgs.action import ExecutePolicy


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('1', 'true', 'yes', 'on')


class PolicyServerNode(Node):
    """Executes named demo policies through currently available Spot controls."""

    def __init__(self):
        super().__init__('policy_server_node')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('dry_run', True)
        self.declare_parameter('policy_delay_sec', 1.0)
        self.declare_parameter('push_speed_mps', 0.12)
        self.declare_parameter('push_duration_sec', 2.0)

        self._server = ActionServer(
            self,
            ExecutePolicy,
            'execute_policy',
            self._execute_policy,
            callback_group=self.cb_group,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._trigger_clients = {}
        self.get_logger().info(
            f'policy_server_node ready (dry_run={self._dry_run()})')

    async def _execute_policy(self, goal_handle):
        """Run the named policy."""
        policy = goal_handle.request.policy_name
        self.get_logger().info(f'execute_policy: {policy}')
        self._publish_feedback(goal_handle, 0.1, f'starting {policy}')

        try:
            if self._dry_run():
                await self._sleep(self.get_parameter('policy_delay_sec').value)
                goal_handle.succeed()
                return ExecutePolicy.Result(
                    success=True,
                    message=f'{policy} dry-run complete',
                )

            success, message = await self._run_policy(policy, goal_handle)
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return ExecutePolicy.Result(success=success, message=message)
        except Exception as exc:
            self.get_logger().error(f'policy {policy} failed: {exc}')
            goal_handle.abort()
            return ExecutePolicy.Result(success=False, message=str(exc))

    async def _run_policy(self, policy: str, goal_handle) -> tuple[bool, str]:
        if policy == 'push_box':
            return await self._push_box(goal_handle)
        if policy == 'open_cabinet':
            return await self._call_sequence(['arm_unstow', 'open_gripper'], goal_handle)
        if policy == 'position_for_grasp':
            return await self._call_sequence(['arm_unstow'], goal_handle)
        if policy == 'grasp':
            return await self._call_sequence(['close_gripper', 'arm_carry'], goal_handle)
        if policy == 'place':
            return await self._call_sequence(['open_gripper'], goal_handle)
        return False, f'unknown policy: {policy}'

    async def _push_box(self, goal_handle) -> tuple[bool, str]:
        duration = float(self.get_parameter('push_duration_sec').value)
        speed = float(self.get_parameter('push_speed_mps').value)
        twist = Twist()
        twist.linear.x = speed

        start = self.get_clock().now()
        while (self.get_clock().now() - start) < Duration(seconds=duration):
            if goal_handle.is_cancel_requested:
                self._stop_body()
                goal_handle.canceled()
                return False, 'canceled'
            self._cmd_vel_pub.publish(twist)
            self._publish_feedback(goal_handle, 0.5, 'pushing box')
            await self._sleep(0.1)

        self._stop_body()
        return True, 'push_box complete'

    async def _call_sequence(self, names: list[str], goal_handle) -> tuple[bool, str]:
        for index, name in enumerate(names):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False, 'canceled'
            self._publish_feedback(goal_handle, index / max(len(names), 1), name)
            success, message = await self._call_trigger(name)
            if not success:
                return False, f'{name} failed: {message}'
        self._publish_feedback(goal_handle, 1.0, 'complete')
        return True, ', '.join(names) + ' complete'

    async def _call_trigger(self, name: str) -> tuple[bool, str]:
        client = self._trigger_clients.get(name)
        if client is None:
            client = self.create_client(Trigger, name, callback_group=self.cb_group)
            self._trigger_clients[name] = client

        if not client.wait_for_service(timeout_sec=3.0):
            return False, f'service {name} unavailable'

        response = await client.call_async(Trigger.Request())
        return bool(response.success), response.message

    async def _sleep(self, seconds: float) -> None:
        time.sleep(float(seconds))

    def _dry_run(self) -> bool:
        return _as_bool(self.get_parameter('dry_run').value)

    def _stop_body(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _publish_feedback(self, goal_handle, progress: float, status: str) -> None:
        feedback = ExecutePolicy.Feedback()
        feedback.progress = float(progress)
        feedback.status = status
        goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
