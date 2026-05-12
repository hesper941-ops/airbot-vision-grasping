"""运动阶段抽象：FixedTargetMotionStage 在进入时冻结目标快照并分段执行。"""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Optional

from .status_model import StageResult, StageStatus
from .target_manager import FixedTargetSnapshot


# ---------------------------------------------------------------------------
# 运动阶段规格
# ---------------------------------------------------------------------------

@dataclass
class MotionStageSpec:
    """单个运动阶段的配置。"""
    name: str                          # 阶段名（如 MOVE_PRE_GRASP）
    command_type: str                  # "joint" / "cartesian"
    timeout_sec: float                 # 超时时间 (s)
    tolerance_m: float                 # 到位容差 (m)
    fixed_target_required: bool = True
    freeze_target_on_start: bool = True
    max_step_m: float = 0.08           # 单步最大步长


# ---------------------------------------------------------------------------
# 抽象基类
# ---------------------------------------------------------------------------

class MotionStage(ABC):
    """运动阶段抽象基类。"""

    def __init__(self, spec: MotionStageSpec):
        self.spec = spec
        self.enter_time_sec: float = 0.0

    @abstractmethod
    def on_enter(self, context, now_sec: float) -> StageResult:
        """进入阶段时调用一次。"""
        ...

    @abstractmethod
    def tick(self, context, now_sec: float) -> StageResult:
        """每个 timer tick 调用。"""
        ...

    def on_exit(self, context):
        """退出阶段时调用。"""
        pass

    def elapsed_sec(self, now_sec: float) -> float:
        return now_sec - self.enter_time_sec

    def is_timed_out(self, now_sec: float) -> bool:
        return self.elapsed_sec(now_sec) > self.spec.timeout_sec


# ---------------------------------------------------------------------------
# 固定目标运动阶段
# ---------------------------------------------------------------------------

class FixedTargetMotionStage(MotionStage):
    """进入时从 TargetManager 冻结一个 FixedTargetSnapshot，然后分段向 full_goal 移动。

    分段逻辑：
    - on_enter() 时创建快照、调用 planner 生成 full_goal
    - 每个 tick 发布一步 cartesian step_goal，直到到达 full_goal
    - 到达 full_goal 并 settle 后返回 DONE
    - 超时或 executor ERROR 返回 FAILED / TIMEOUT
    """

    def __init__(
        self,
        spec: MotionStageSpec,
        planner,                # GraspPlanner
        command_port,           # ArmCommandPort
        target_manager,         # TargetManager
    ):
        super().__init__(spec)
        self._planner = planner
        self._command_port = command_port
        self._target_manager = target_manager

        # 冻结的目标
        self.snapshot: Optional[FixedTargetSnapshot] = None
        # 运动全目标
        self.full_goal: Optional[list] = None
        # 当前步目标
        self.step_goal: Optional[list] = None
        # 命令是否已发送
        self.command_sent: bool = False
        # settle 开始时间
        self.settle_start_sec: Optional[float] = None

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def on_enter(self, context, now_sec: float) -> StageResult:
        self.enter_time_sec = now_sec

        if self.spec.freeze_target_on_start:
            self.snapshot = self._target_manager.make_fixed_snapshot(now_sec)
            if self.snapshot is None:
                return StageResult.failed(
                    self.spec.name, reason="cannot create fixed snapshot")
            context.fixed_target_snapshot = self.snapshot

        return StageResult.running(self.spec.name)

    def tick(self, context, now_sec: float) -> StageResult:
        if self.is_timed_out(now_sec):
            return StageResult.timeout(self.spec.name)

        if context.executor_status in ("ERROR", "TIMEOUT"):
            return StageResult.failed(
                self.spec.name, reason=f"executor_{context.executor_status}")

        # ---- 构建 full_goal ----
        if self.full_goal is None:
            goal = self._build_goal()
            if goal is None:
                return StageResult.failed(
                    self.spec.name, reason="goal computation failed")
            self.full_goal = goal

        # ---- 检查是否已到达 ----
        if self._is_at_goal(now_sec):
            return StageResult.done(self.spec.name)

        # ---- 发布下一步 ----
        if not self.command_sent:
            if context.executor_status not in ("IDLE", "DONE", ""):
                return StageResult.running(self.spec.name)

            step = self._compute_step(context)
            if step is None:
                return StageResult.failed(
                    self.spec.name, reason="step computation failed")
            self.step_goal = step
            self._command_port.publish_cart_target(step, reason=self.spec.name)
            self.command_sent = True
            self.settle_start_sec = None
            return StageResult.running(self.spec.name)

        # ---- 等待步完成 ----
        if self.step_goal is None:
            return StageResult.failed(
                self.spec.name, reason="step_goal missing")

        if not self._step_reached(context, now_sec):
            return StageResult.running(self.spec.name)

        # 步完成，准备下一步
        self.command_sent = False
        self.step_goal = None
        self.settle_start_sec = None
        return StageResult.running(self.spec.name)

    def on_exit(self, context):
        pass

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _build_goal(self) -> Optional[list]:
        """由子类覆盖以生成特定阶段的全目标。"""
        raise NotImplementedError

    def _compute_step(self, context) -> Optional[list]:
        """从当前位姿向 full_goal 走一步。"""
        if context.last_end_pose is None:
            return None
        dist = math.hypot(
            self.full_goal[0] - context.last_end_pose[0],
            self.full_goal[1] - context.last_end_pose[1],
            self.full_goal[2] - context.last_end_pose[2],
        )
        if dist <= self.spec.max_step_m:
            return list(self.full_goal)
        ratio = self.spec.max_step_m / dist
        return [
            context.last_end_pose[0] + (self.full_goal[0] - context.last_end_pose[0]) * ratio,
            context.last_end_pose[1] + (self.full_goal[1] - context.last_end_pose[1]) * ratio,
            context.last_end_pose[2] + (self.full_goal[2] - context.last_end_pose[2]) * ratio,
        ]

    def _is_at_goal(self, now_sec: float) -> bool:
        """检查是否已达到 full_goal 并完成 settle。"""
        if self.full_goal is None or self.step_goal is not None:
            return False  # 还有未完成的步
        # 最后一步已到达但尚未 settle
        if self.settle_start_sec is None:
            self.settle_start_sec = now_sec
            return False
        return (now_sec - self.settle_start_sec) >= 0.5  # settle_time

    def _step_reached(self, context, now_sec: float) -> bool:
        """当前步是否到达。"""
        if self.step_goal is None or context.last_end_pose is None:
            return False
        dist = math.hypot(
            self.step_goal[0] - context.last_end_pose[0],
            self.step_goal[1] - context.last_end_pose[1],
            self.step_goal[2] - context.last_end_pose[2],
        )
        if dist <= self.spec.tolerance_m:
            if self.settle_start_sec is None:
                self.settle_start_sec = now_sec
                return False
            return (now_sec - self.settle_start_sec) >= 0.3
        self.settle_start_sec = None
        return False


# ---------------------------------------------------------------------------
# 具体阶段：PreGraspStage
# ---------------------------------------------------------------------------

class PreGraspStage(FixedTargetMotionStage):
    """MOVE_PRE_GRASP：从快照生成预抓取点并分段移动。"""

    def _build_goal(self) -> Optional[list]:
        return self._planner.compute_safe_pre_grasp(
            [self.snapshot.x, self.snapshot.y, self.snapshot.z])


# ---------------------------------------------------------------------------
# 具体阶段：GraspStage
# ---------------------------------------------------------------------------

class GraspStage(FixedTargetMotionStage):
    """MOVE_GRASP：从快照生成抓取点并分段下降。"""

    def _build_goal(self) -> Optional[list]:
        return self._planner.compute_safe_grasp(
            [self.snapshot.x, self.snapshot.y, self.snapshot.z])


# ---------------------------------------------------------------------------
# 具体阶段：LiftStage
# ---------------------------------------------------------------------------

class LiftStage(FixedTargetMotionStage):
    """MOVE_LIFT：从快照或当前位姿生成抬升点。"""

    def _build_goal(self) -> Optional[list]:
        if self.snapshot:
            return self._planner.compute_safe_lift(
                [self.snapshot.x, self.snapshot.y, self.snapshot.z])
        return None
