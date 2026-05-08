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
from spot_msgs.srv import Dock, SetGripperAngle

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


HANDLE_LOOSEN_DELTA_DEG = 1
GRIPPER_MAX_ANGLE_DEG = 90.0


def _loosen_handle_grip_service():
    def build(blackboard):
        current_angle = 0.0
        if blackboard.contains('gripper_open_percentage'):
            open_pct = float(blackboard['gripper_open_percentage'])
            current_angle = max(0.0, min(GRIPPER_MAX_ANGLE_DEG, open_pct * 0.01 * GRIPPER_MAX_ANGLE_DEG))
        target_angle = min(GRIPPER_MAX_ANGLE_DEG, current_angle + HANDLE_LOOSEN_DELTA_DEG)
        blackboard['handle_loosen_grip_angle'] = target_angle
        req = SetGripperAngle.Request()
        req.gripper_angle = float(target_angle)
        return req
    return ServiceState(SetGripperAngle, 'set_gripper_angle', build)


def _action_feedback(action_name):
    def handle(blackboard, feedback):
        status = str(getattr(feedback, 'status', '') or '').strip()
        if not status:
            status = str(getattr(feedback, 'current_step', '') or '').strip()
        if not status:
            return

        try:
            progress = float(getattr(feedback, 'progress', 0.0) or 0.0)
        except (TypeError, ValueError):
            progress = 0.0

        seq = 1
        if blackboard.contains('_action_feedback_seq'):
            seq = int(blackboard.get('_action_feedback_seq') or 0) + 1
        blackboard['_action_feedback_seq'] = seq
        blackboard['_action_feedback'] = {
            'action': action_name,
            'status': status,
            'progress': progress,
            'seq': seq,
        }
    return handle


def _action(action_type, name, goal_builder):
    return ActionState(action_type, name, goal_builder, feedback_handler=_action_feedback(name))


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


def _add_post_open_states(sm: StateMachine) -> None:
    """Shared sequence following OPEN_CABINET, identical for both demos.

    RELEASE_HANDLE -> REPOSITION_AT_CABINET -> STOW_ARM_OPEN ->
    PERCEIVE_AND_GRASP_ITEM -> CARRY_AFTER_GRASP -> MOVE_TO_TABLE ->
    PLACE_ITEM -> MOVE_TO_DOCK -> DOCK.
    """
    sm.add_state('RELEASE_HANDLE', _trigger_service('open_gripper'),
                 transitions=_service_to('REPOSITION_AT_CABINET'))

    sm.add_state('REPOSITION_AT_CABINET',
                 _action(NavigateToPose, 'go_to', _goto('cabinet_pose')),
                 transitions=_action_to('STOW_ARM_OPEN'))

    sm.add_state('STOW_ARM_OPEN', _trigger_service('arm_stow'),
                 transitions=_service_to('PERCEIVE_AND_GRASP_ITEM'))

    sm.add_state('PERCEIVE_AND_GRASP_ITEM',
                 _action(FindObject, 'find_object', _find_object('target_item')),
                 transitions=_action_to('CARRY_AFTER_GRASP'))

    sm.add_state('CARRY_AFTER_GRASP', _trigger_service('arm_carry'),
                 transitions=_service_to('MOVE_TO_TABLE'))

    sm.add_state('MOVE_TO_TABLE',
                 _action(NavigateToPose, 'go_to', _goto('table_pose', 'table')),
                 transitions=_action_to('PLACE_ITEM'))

    sm.add_state('PLACE_ITEM',
                 _action(ExecutePolicy, 'execute_policy', _policy('place')),
                 transitions=_action_to('MOVE_TO_DOCK'))

    sm.add_state('MOVE_TO_DOCK',
                 _action(NavigateToPose, 'go_to', _goto('dock_pose')),
                 transitions=_terminal_action_to('DOCK'))

    sm.add_state('DOCK', ServiceState(Dock, 'dock', _dock_request),
                 transitions={SUCCEED: 'DONE', ABORT: 'ABORTED'})


def _add_open_cabinet_item_fetch_states(sm: StateMachine) -> None:
    """Shared sequence for a cabinet that is already open.

    STOW_ARM_OPEN -> PERCEIVE_AND_GRASP_ITEM -> CARRY_AFTER_GRASP ->
    MOVE_TO_TABLE -> PLACE_ITEM -> MOVE_TO_DOCK -> DOCK.
    """
    sm.add_state('STOW_ARM_OPEN', _trigger_service('arm_stow'),
                 transitions=_service_to('PERCEIVE_AND_GRASP_ITEM'))

    sm.add_state('PERCEIVE_AND_GRASP_ITEM',
                 _action(FindObject, 'find_object', _find_object('target_item')),
                 transitions=_action_to('CARRY_AFTER_GRASP'))

    sm.add_state('CARRY_AFTER_GRASP', _trigger_service('arm_carry'),
                 transitions=_service_to('MOVE_TO_TABLE'))

    sm.add_state('MOVE_TO_TABLE',
                 _action(NavigateToPose, 'go_to', _goto('table_pose', 'table')),
                 transitions=_action_to('PLACE_ITEM'))

    sm.add_state('PLACE_ITEM',
                 _action(ExecutePolicy, 'execute_policy', _policy('place')),
                 transitions=_action_to('MOVE_TO_DOCK'))

    sm.add_state('MOVE_TO_DOCK',
                 _action(NavigateToPose, 'go_to', _goto('dock_pose')),
                 transitions=_terminal_action_to('DOCK'))

    sm.add_state('DOCK', ServiceState(Dock, 'dock', _dock_request),
                 transitions={SUCCEED: 'DONE', ABORT: 'ABORTED'})


def _add_recovery_states(sm: StateMachine) -> None:
    sm.add_state('RECOVERY_ARM_STOW', _trigger_service('arm_stow'),
                 transitions={SUCCEED: 'RECOVERY_MOVE_TO_DOCK',
                              ABORT: 'RECOVERY_MOVE_TO_DOCK'})

    sm.add_state('RECOVERY_MOVE_TO_DOCK',
                 _action(NavigateToPose, 'go_to', _goto('dock_pose')),
                 transitions=_recovery_action_to('RECOVERY_DOCK'))

    sm.add_state('RECOVERY_DOCK', ServiceState(Dock, 'dock', _dock_request),
                 transitions=_terminal_service_to('ABORTED'))


def build_demo_fsm(box_grasp_side='right') -> StateMachine:
    """Full demo FSM including the box-push prelude. Outcomes: 'DONE', 'ABORTED'.

    UNDOCK -> MOVE_TO_BOX -> PERCEIVE_BOX_GRASP -> PUSH_BOX -> STOW_AFTER_PUSH ->
    MOVE_TO_CABINET -> PERCEIVE_HANDLE -> GRASP_HANDLE -> OPEN_CABINET ->
    [shared post-open sequence].
    """
    box_grasp_side = (box_grasp_side or 'right').strip().lower()
    sm = StateMachine(outcomes=['DONE', 'ABORTED'])

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

    sm.add_state('MOVE_TO_CABINET',
                 _action(NavigateToPose, 'go_to', _goto('cabinet_pose')),
                 transitions=_action_to('PERCEIVE_HANDLE'))

    sm.add_state('PERCEIVE_HANDLE',
                 _action(FindCabinetHandle, 'find_cabinet_handle', _find_handle),
                 transitions=_action_to('LOOSEN_HANDLE_GRIP'))

    sm.add_state('LOOSEN_HANDLE_GRIP', _loosen_handle_grip_service(),
                 transitions=_service_to('OPEN_CABINET'))

    sm.add_state('OPEN_CABINET',
                 _action(ExecutePolicy, 'execute_policy', _policy('open_cabinet')),
                 transitions=_action_to('RELEASE_HANDLE'))

    _add_post_open_states(sm)
    _add_recovery_states(sm)

    return sm


def build_fetch_from_cabinet_fsm() -> StateMachine:
    """Fetch demo without the box-push prelude. Outcomes: 'DONE', 'ABORTED'.

    UNDOCK -> MOVE_TO_CABINET -> PERCEIVE_HANDLE -> GRASP_HANDLE ->
    OPEN_CABINET -> [shared post-open sequence].
    """
    sm = StateMachine(outcomes=['DONE', 'ABORTED'])

    sm.add_state('UNDOCK', _trigger_service('undock'),
                 transitions=_service_to('MOVE_TO_CABINET'))

    sm.add_state('MOVE_TO_CABINET',
                 _action(NavigateToPose, 'go_to', _goto('cabinet_pose')),
                 transitions=_action_to('PERCEIVE_HANDLE'))

    sm.add_state('PERCEIVE_HANDLE',
                 _action(FindCabinetHandle, 'find_cabinet_handle', _find_handle),
                 transitions=_action_to('LOOSEN_HANDLE_GRIP'))

    sm.add_state('LOOSEN_HANDLE_GRIP', _loosen_handle_grip_service(),
                 transitions=_service_to('OPEN_CABINET'))

    sm.add_state('OPEN_CABINET',
                 _action(ExecutePolicy, 'execute_policy', _policy('open_cabinet')),
                 transitions=_action_to('RELEASE_HANDLE'))

    _add_post_open_states(sm)
    _add_recovery_states(sm)

    return sm


def build_fetch_from_open_cabinet_fsm() -> StateMachine:
    """Fetch demo starting with the cabinet door already open.

    UNDOCK -> MOVE_TO_CABINET -> STOW_ARM_OPEN ->
    PERCEIVE_AND_GRASP_ITEM -> CARRY_AFTER_GRASP -> MOVE_TO_TABLE ->
    PLACE_ITEM -> MOVE_TO_DOCK -> DOCK.
    """
    sm = StateMachine(outcomes=['DONE', 'ABORTED'])

    sm.add_state('UNDOCK', _trigger_service('undock'),
                 transitions=_service_to('MOVE_TO_CABINET'))

    sm.add_state('MOVE_TO_CABINET',
                 _action(NavigateToPose, 'go_to', _goto('cabinet_pose')),
                 transitions=_action_to('STOW_ARM_OPEN'))

    _add_open_cabinet_item_fetch_states(sm)
    _add_recovery_states(sm)

    return sm
