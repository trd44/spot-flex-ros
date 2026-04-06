"""
Policy executor

Loads and runs trained manipulation policies (cabinet opening,
box pushing). Outputs end-effector and/or body poses that
the policy server node sends to the arm.
"""

import numpy as np
from typing import List, Optional
from dataclasses import dataclass


@dataclass
class PolicyOutput:
    """Output from one step of a policy."""
    eef_pose = None     
    body_pose = None
    done = False


class PolicyExecutor:
    """
    Loads and steps through a trained manipulation policy.
    Maybe just use the other code directly somehow
    """
    pass
