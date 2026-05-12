"""抓取状态机使用的状态枚举与结果对象。"""

from dataclasses import dataclass
from enum import Enum, auto


# ---------------------------------------------------------------------------
# 任务状态
# ---------------------------------------------------------------------------

class TaskState(Enum):
    """抓取任务高层状态。"""
    IDLE = "IDLE"
    WAIT_TARGET = "WAIT_TARGET"
    SEARCH_TARGET = "SEARCH_TARGET"
    WAIT_STABLE_TARGET = "WAIT_STABLE_TARGET"
    SET_GRIPPER_ORIENTATION = "SET_GRIPPER_ORIENTATION"
    PRE_OPEN_GRIPPER = "PRE_OPEN_GRIPPER"
    MOVE_PRE_GRASP = "MOVE_PRE_GRASP"
    MOVE_GRASP = "MOVE_GRASP"
    CLOSE_GRIPPER = "CLOSE_GRIPPER"
    MOVE_LIFT = "MOVE_LIFT"
    MOVE_RETREAT = "MOVE_RETREAT"
    RETURN_INIT_POSE = "RETURN_INIT_POSE"
    DONE = "DONE"
    RECOVER = "RECOVER"
    FAILED = "FAILED"


# ---------------------------------------------------------------------------
# 执行器状态（兼容现有 /robot_arm/executor_status）
# ---------------------------------------------------------------------------

class ExecutorStatus(Enum):
    UNKNOWN = "UNKNOWN"
    IDLE = "IDLE"
    BUSY = "BUSY"
    DONE = "DONE"
    ERROR = "ERROR"
    RESETTING = "RESETTING"
    RESET_DONE = "RESET_DONE"
    RESET_FAILED = "RESET_FAILED"
    REJECTED_BUSY = "REJECTED_BUSY"
    REJECTED_INVALID_JOINT_LIMIT = "REJECTED_INVALID_JOINT_LIMIT"
    REJECTED_INVALID_TARGET = "REJECTED_INVALID_TARGET"
    REJECTED_INVALID_GRIPPER = "REJECTED_INVALID_GRIPPER"
    SDK_DISCONNECTED = "SDK_DISCONNECTED"
    TIMEOUT = "TIMEOUT"


# ---------------------------------------------------------------------------
# 阶段结果
# ---------------------------------------------------------------------------

class StageStatus(Enum):
    RUNNING = auto()
    DONE = auto()
    FAILED = auto()
    TIMEOUT = auto()
    SKIPPED = auto()


@dataclass
class StageResult:
    """单个运动/动作阶段的输出。"""
    status: StageStatus
    stage_name: str = ""
    reason: str = ""

    @classmethod
    def running(cls, stage_name: str = "") -> "StageResult":
        return cls(status=StageStatus.RUNNING, stage_name=stage_name)

    @classmethod
    def done(cls, stage_name: str = "") -> "StageResult":
        return cls(status=StageStatus.DONE, stage_name=stage_name)

    @classmethod
    def failed(cls, stage_name: str = "", reason: str = "") -> "StageResult":
        return cls(status=StageStatus.FAILED, stage_name=stage_name, reason=reason)

    @classmethod
    def timeout(cls, stage_name: str = "") -> "StageResult":
        return cls(status=StageStatus.TIMEOUT, stage_name=stage_name)

    @classmethod
    def skipped(cls, stage_name: str = "") -> "StageResult":
        return cls(status=StageStatus.SKIPPED, stage_name=stage_name)

    @property
    def is_terminal(self) -> bool:
        return self.status in (
            StageStatus.DONE,
            StageStatus.FAILED,
            StageStatus.TIMEOUT,
            StageStatus.SKIPPED,
        )


# ---------------------------------------------------------------------------
# 恢复触发原因
# ---------------------------------------------------------------------------

class RecoveryTrigger(Enum):
    EXECUTOR_ERROR = auto()
    MOTION_TIMEOUT = auto()
    TARGET_LOST = auto()
    REJECTED_BUSY = auto()
    REJECTED_INVALID_TARGET = auto()
    REJECTED_INVALID_JOINT = auto()
    MANUAL_RESET = auto()


# ---------------------------------------------------------------------------
# 恢复结果
# ---------------------------------------------------------------------------

@dataclass
class RecoveryResult:
    active: bool = False
    is_done: bool = False
    is_failed: bool = False
    stage: str = ""
    reason: str = ""

    @classmethod
    def idle(cls) -> "RecoveryResult":
        return cls()

    @classmethod
    def running(cls, stage: str = "") -> "RecoveryResult":
        return cls(active=True, stage=stage)

    @classmethod
    def done(cls) -> "RecoveryResult":
        return cls(active=False, is_done=True)

    @classmethod
    def failed(cls, reason: str = "") -> "RecoveryResult":
        return cls(active=False, is_failed=True, reason=reason)


# ---------------------------------------------------------------------------
# 搜索结果
# ---------------------------------------------------------------------------

class SearchResultStatus(Enum):
    RUNNING = auto()
    TARGET_FOUND = auto()
    SEARCH_FAILED = auto()
    EXECUTOR_ERROR = auto()
    IDLE = auto()


@dataclass
class SearchResult:
    status: SearchResultStatus = SearchResultStatus.IDLE
    detail: str = ""

    @classmethod
    def idle(cls) -> "SearchResult":
        return cls(status=SearchResultStatus.IDLE)

    @classmethod
    def running(cls, detail: str = "") -> "SearchResult":
        return cls(status=SearchResultStatus.RUNNING, detail=detail)

    @classmethod
    def target_found(cls) -> "SearchResult":
        return cls(status=SearchResultStatus.TARGET_FOUND)

    @classmethod
    def failed(cls, detail: str = "") -> "SearchResult":
        return cls(status=SearchResultStatus.SEARCH_FAILED, detail=detail)

    @classmethod
    def executor_error(cls) -> "SearchResult":
        return cls(status=SearchResultStatus.EXECUTOR_ERROR)
