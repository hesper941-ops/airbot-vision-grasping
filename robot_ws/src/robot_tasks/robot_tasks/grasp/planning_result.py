"""Small planning result object used by the open-loop preflight check."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class PlanningResult:
    approach_mode: str
    plan_id: int = 0
    created_time_sec: float = 0.0
    target_snapshot: Optional[list] = None
    target: Optional[list] = None
    pre_grasp: Optional[list] = None
    grasp: Optional[list] = None
    lift_goal: Optional[list] = None
    reason: str = ''
    ok: bool = False
