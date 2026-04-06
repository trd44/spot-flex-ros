"""
Task planner that generates action sequences for high-level goals.

Currently state machine but may be upgraded
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Dict, Any


class ActionType(Enum):
    """Types of actions the robot can execute."""
    NAVIGATE = "navigate"
    PERCEIVE_OBJECT = "perceive_object"
    PERCEIVE_CABINET = "perceive_cabinet"
    PERCEIVE_DROPOFF = "perceive_dropoff"
    OPEN_CABINET = "open_cabinet"
    GRASP = "grasp"
    PLACE = "place"
    PUSH = "push"
    ADJUST_POSITION = "adjust_position"
    ARM_STOW = "arm_stow"
    ARM_CARRY = "arm_carry"


@dataclass
class ActionStep:
    """A single step in a task plan."""
    pass


class TaskPlanner:
    """
    Generates a sequence of ActionSteps for a given high-level task.

    Currently uses hardcoded sequences.
    """

    def __init__(self):
        self.known_locations: Dict[str, Any] = {}

    def plan(self, task: str, item: str, location: str) -> List[ActionStep]:
        """
        Generate an action sequence for the given task.
        """
        pass

    def _plan_fetch_from_cabinet(self, item: str, location: str) -> List[ActionStep]:
        """Plan: go to cabinet, open it, find item, grab it, bring it back."""
        pass

    def _plan_push_obstacle(self, item: str, location: str) -> List[ActionStep]:
        """Plan: go to obstacle, push it out of the way."""
        pass
        
