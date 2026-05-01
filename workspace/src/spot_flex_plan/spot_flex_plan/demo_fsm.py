"""YASMIN state machine for the scripted Spot demo.

States are wired to whichever action/service is reachable on the bus, so the
same FSM runs against mock servers (spot_flex_mocks) or the real subsystem
nodes without changes.
"""

from yasmin import StateMachine
from yasmin_ros import ActionState, ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from spot_msgs.srv import Dock

from spot_flex_msgs.action import (
    ExecutePolicy,
    FindBoxGraspPoint,
    FindCabinetHandle,
    FindObject,
)


def _goto(blackboard_key, location_name=None):
    # location_name rides in goal.behavior_tree as a sentinel; nav_node's
    # graphnav backend uses it to look up a waypoint_id. Other backends ignore.
    # If the caller does not provide one, derive it from keys like box_pose.
    if location_name is None:
        location_name = blackboard_key[:-5] if blackboard_key.endswith('_pose') else blackboard_key

    def build(blackboard):
        goal = NavigateToPose.Goal()
        goal.pose = blackboard[blackboard_key]
        goal.behavior_tree = location_name
        return goal
    return build


def _trigger_request(_blackboard):
    return Trigger.Request()


def _dock_request(blackboard):
    req = Dock.Request()
    req.dock_id = int(blackboard['dock_id'])
    return req


def _policy(name):
    def build(_blackboard):
        return ExecutePolicy.Goal(policy_name=name)
    return build


def _find_object(item_key):
    def build(blackboard):
        return FindObject.Goal(object_name=blackboard[item_key])
    return build


def _find_box(side):
    def build(_blackboard):
        return FindBoxGraspPoint.Goal(side=side)
    return build


def _find_handle(_blackboard):
    return FindCabinetHandle.Goal()


def _trigger_service(name):
    return ServiceState(Trigger, name, _trigger_request)


def _action(action_type, name, goal_builder):
    return ActionState(action_type, name, goal_builder)


def _action_to(next_label):
    return {SUCCEED: next_label, ABORT: 'RECOVERY_ARM_STOW', CANCEL: 'RECOVERY_ARM_STOW'}


def _service_to(next_label):
    return {SUCCEED: next_label, ABORT: 'RECOVERY_ARM_STOW'}


def _terminal_action_to(next_label):
    return {SUCCEED: next_label, ABORT: 'ABORTED', CANCEL: 'ABORTED'}


def _terminal_service_to(next_label):
    return {SUCCEED: next_label, ABORT: 'ABORTED'}


def _recovery_action_to(next_label):
    return {SUCCEED: next_label, ABORT: next_label, CANCEL: next_label}


def build_demo_fsm(box_grasp_side='right', skip_box_push=False) -> StateMachine:
    """Build the demo FSM. Outcomes: 'DONE', 'ABORTED'.

    When ``skip_box_push`` is True, the FSM's first state is MOVE_TO_CABINET
    and the undock + box-push prelude is omitted.
    """
    box_grasp_side = (box_grasp_side or 'right').strip().lower()
    sm = StateMachine(outcomes=['DONE', 'ABORTED'])

    if not skip_box_push:
        sm.add_state('UNDOCK', _trigger_service('undock'),
                     transitions=_service_to('MOVE_TO_BOX'))

        sm.add_state('MOVE_TO_BOX', _action(NavigateToPose, 'go_to', _goto('box_pose', 'box')),
                     transitions=_action_to('PERCEIVE_BOX_GRASP'))

        sm.add_state('PERCEIVE_BOX_GRASP',
                     _action(FindBoxGraspPoint, 'find_box_grasp_point',
                             _find_box(box_grasp_side)),
                     transitions=_action_to('PUSH_BOX'))

        sm.add_state('PUSH_BOX', _action(ExecutePolicy, 'execute_policy', _policy('push_box')),
                     transitions=_action_to('STOW_AFTER_PUSH'))

        sm.add_state('STOW_AFTER_PUSH', _trigger_service('arm_stow'),
                     transitions=_service_to('MOVE_TO_CABINET'))

    sm.add_state('MOVE_TO_CABINET', _action(NavigateToPose, 'go_to', _goto('cabinet_pose')),
                 transitions=_action_to('PERCEIVE_HANDLE'))

    sm.add_state('PERCEIVE_HANDLE',
                 _action(FindCabinetHandle, 'find_cabinet_handle', _find_handle),
                 transitions=_action_to('OPEN_CABINET'))

    sm.add_state('OPEN_CABINET',
                 _action(ExecutePolicy, 'execute_policy', _policy('open_cabinet')),
                 transitions=_action_to('POSITION_AT_CABINET'))

    sm.add_state('POSITION_AT_CABINET',
                 _action(ExecutePolicy, 'execute_policy', _policy('position_for_grasp')),
                 transitions=_action_to('PERCEIVE_ITEM'))

    sm.add_state('PERCEIVE_ITEM',
                 _action(FindObject, 'find_object', _find_object('target_item')),
                 transitions=_action_to('GRASP_ITEM'))

    sm.add_state('GRASP_ITEM', _action(ExecutePolicy, 'execute_policy', _policy('grasp')),
                 transitions=_action_to('ARM_CARRY'))

    sm.add_state('ARM_CARRY', _trigger_service('arm_carry'),
                 transitions=_service_to('MOVE_TO_TABLE'))

    sm.add_state('MOVE_TO_TABLE', _action(NavigateToPose, 'go_to', _goto('table_pose', 'dropoff')),
                 transitions=_action_to('PERCEIVE_DROPOFF'))

    sm.add_state('PERCEIVE_DROPOFF',
                 _action(FindObject, 'find_object', lambda b: FindObject.Goal(object_name='table')),
                 transitions=_action_to('PLACE_ITEM'))

    sm.add_state('PLACE_ITEM', _action(ExecutePolicy, 'execute_policy', _policy('place')),
                 transitions=_action_to('ARM_STOW'))

    sm.add_state('ARM_STOW', _trigger_service('arm_stow'),
                 transitions=_service_to('MOVE_TO_DOCK'))

    sm.add_state('MOVE_TO_DOCK', _action(NavigateToPose, 'go_to', _goto('dock_pose')),
                 transitions=_terminal_action_to('DOCK'))

    sm.add_state('DOCK', ServiceState(Dock, 'dock', _dock_request),
                 transitions={SUCCEED: 'DONE', ABORT: 'ABORTED'})

    sm.add_state('RECOVERY_ARM_STOW', _trigger_service('arm_stow'),
                 transitions={SUCCEED: 'RECOVERY_MOVE_TO_DOCK',
                              ABORT: 'RECOVERY_MOVE_TO_DOCK'})

    sm.add_state('RECOVERY_MOVE_TO_DOCK',
                 _action(NavigateToPose, 'go_to', _goto('dock_pose')),
                 transitions=_recovery_action_to('RECOVERY_DOCK'))

    sm.add_state('RECOVERY_DOCK', ServiceState(Dock, 'dock', _dock_request),
                 transitions=_terminal_service_to('ABORTED'))

    return sm
