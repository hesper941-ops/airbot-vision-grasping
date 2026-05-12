"""抓取状态机各组件之间共享的运行时上下文。"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class GraspContext:
    """抓取任务的可变运行时状态。

    各组件直接读写字段；controller 负责协调状态迁移。
    """

    task_state: str = "IDLE"
    executor_status: str = "UNKNOWN"

    selected_target_type: str = "duck"

    latest_target: Optional[object] = None
    stable_target: Optional[object] = None
    active_target: Optional[object] = None
    fixed_target_snapshot: Optional[object] = None
    last_seen_target: Optional[object] = None

    current_stage_name: str = ""
    current_search_pose_name: str = ""
    search_cycle_count: int = 0

    last_target_time: float = 0.0
    last_executor_status_time: float = 0.0
    stage_start_time: float = 0.0
    recovery_start_time: float = 0.0

    last_end_pose: Optional[list] = None
    grasp_attempt_count: int = 0

    # 每次 timer tick 前由节点设置。
    now_sec: float = 0.0
