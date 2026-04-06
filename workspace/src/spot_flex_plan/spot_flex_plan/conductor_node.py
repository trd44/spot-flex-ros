"""
Conductor node — task planning and execution

Receives high-level goals (e.g., fetch item X from cabinet Y),
generates a plan, and executes each step by calling action servers
on the perception, navigation, and arm control nodes.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from spot_flex_msgs.action import FetchItem, FindObject, ExecutePolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from spot_flex_plan.task_planner import TaskPlanner, ActionType


class ConductorNode(Node):
    """
    Central orchestrator that coordinates all subsystems.
    """

    def __init__(self):
        super().__init__('conductor_node')
        self.cb_group = ReentrantCallbackGroup()

        # Task planner (swappable — replace internals without changing this node)
        self.planner = TaskPlanner()

        # Current state tracking
        self.current_step_index = 0
        self.current_plan = []
        self.robot_state = {
            'arm_stowed': True,
            'holding_item': None,
            'current_pose': None,
        }

        # --- Action server: receives goals from UI ---
        self._fetch_server = ActionServer(
            self,
            FetchItem,
            'fetch_item',
            self._execute_fetch,
            callback_group=self.cb_group,
        )

        # --- Action clients: calls out to other subsystems ---
        self._nav_client = ActionClient(
            self, NavigateToPose, 'go_to',
            callback_group=self.cb_group,
        )
        self._find_object_client = ActionClient(
            self, FindObject, 'find_object',
            callback_group=self.cb_group,
        )
        self._policy_client = ActionClient(
            self, ExecutePolicy, 'execute_policy',
            callback_group=self.cb_group,
        )

        self.get_logger().info('Conductor node started')

    async def _execute_fetch(self, goal_handle):
        """Execute a FetchItem goal by planning and stepping through actions."""
        pass

    async def _execute_step(self, step):
        """
        """
        pass

    async def _do_navigate(self, step):
        """Send a navigation goal to the nav node."""
        pass

    async def _do_perceive(self, step):
        """Send a perception request to find an object or feature."""
        pass

    async def _do_policy(self, step):
        """Execute a learned policy (cabinet open, push, etc.)."""
        pass

    async def _do_grasp(self, step):
        """Grasp the target object."""
        pass

    async def _do_place(self, step):
        """Place the held object at the target location."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ConductorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
