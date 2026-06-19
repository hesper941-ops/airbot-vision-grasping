"""Small planning result object used by the open-loop preflight check."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class PlanningResult:
    approach_mode: str
    target: Optional[list] = None
    pre_grasp: Optional[list] = None
    grasp: Optional[list] = None
    lift_goal: Optional[list] = None
    reason: str = ''
    ok: bool = False
