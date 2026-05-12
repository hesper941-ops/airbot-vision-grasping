"""抓取序列控制器：编排主状态机。

这是重构后的核心控制器，取代原先 grasp_task_open_loop.py 中的 step_loop() 逻辑。
控制器协调 TargetManager、SearchStrategy、RecoveryManager 和 MotionStage 各模块。
"""

from typing import Optional

from .context import GraspContext
from .status_model import TaskState, RecoveryTrigger, SearchResultStatus, StageResult, StageStatus


class GraspSequenceController:
    """抓取任务主控制器。

    负责：
    - 主状态机迁移
    - 将感知数据分发给 TargetManager
    - 触发 SearchStrategy / RecoveryManager
    - 控制 MotionStage 执行
    - 保持与现有 topic 接口兼容
    """

    def __init__(self, config, context: GraspContext,
                 target_manager, target_source_manager,
                 command_port, search_strategy, recovery,
                 planner):
        self._config = config
        self._ctx = context
        self._target_manager = target_manager
        self._target_source = target_source_manager
        self._cmd = command_port
        self._search = search_strategy
        self._recovery = recovery
        self._planner = planner

        # 当前活跃的运动阶段
        self._active_stage = None
        # 已选择的最终阶段序列
        self._stage_sequence = []

    # ------------------------------------------------------------------
    # 对外接口（由 ROS Node 调用）
    # ------------------------------------------------------------------

    def on_target(self, msg):
        """收到视觉目标消息。"""
        # 兼容现有 VisualTarget msg 格式
        try:
            obs = self._msg_to_observation(msg)
        except Exception:
            return

        self._ctx.now_sec = self._ctx.now_sec  # 由节点在 tick 前设置

        accepted = self._target_manager.accept_observation(
            obs, self._ctx.now_sec,
            active_target=self._ctx.active_target,
            task_state=self._ctx.task_state,
        )
        if accepted:
            self._ctx.last_seen_target = self._target_manager.last_seen
            self._ctx.last_target_time = self._ctx.now_sec

    def on_executor_status(self, status: str):
        """收到执行器状态更新。"""
        self._ctx.executor_status = status
        self._ctx.last_executor_status_time = self._ctx.now_sec

        if status in ("ERROR", "TIMEOUT"):
            if self._ctx.task_state in ("RECOVER", "RETURN_INIT_POSE"):
                return
            self._recovery.start(
                RecoveryTrigger.EXECUTOR_ERROR, self._ctx,
                keep_gripper_closed=getattr(self._ctx, "grasp_closed", False),
                reason=f"executor_{status}",
            )

        if status == "REJECTED_BUSY":
            self._ctx.grasp_attempt_count += 1

    def on_end_pose(self, pose: list):
        """收到末端位姿更新。"""
        self._ctx.last_end_pose = pose

    def tick(self):
        """每个 timer tick 调用一次。"""
        ctx = self._ctx
        ctx.now_sec = self._ctx.now_sec  # 节点应在 tick 前设置

        # ---- RECOVER 优先 ----
        if ctx.task_state == "RECOVER":
            result = self._recovery.tick(ctx)
            if result.is_done:
                # 恢复完成，清状态回 IDLE
                ctx.task_state = "IDLE"
                ctx.grasp_attempt_count = 0
            return

        # ---- SEARCH_TARGET ----
        if ctx.task_state == "SEARCH_TARGET":
            result = self._search.tick(ctx)
            if result.status == SearchResultStatus.TARGET_FOUND:
                ctx.task_state = "SET_GRIPPER_ORIENTATION"
            elif result.status in (SearchResultStatus.SEARCH_FAILED,
                                   SearchResultStatus.EXECUTOR_ERROR):
                ctx.task_state = "FAILED"
            return

        # ---- 主状态机 ----
        self._tick_main_fsm()

    # ------------------------------------------------------------------
    # 主状态机
    # ------------------------------------------------------------------

    def _tick_main_fsm(self):
        state = self._ctx.task_state

        if state == "IDLE":
            self._ctx.task_state = "WAIT_TARGET"
            self._ctx.stage_start_time = self._ctx.now_sec
            self._target_manager.reset_stability()

        elif state == "WAIT_TARGET":
            self._tick_wait_target()

        elif state == "PRE_OPEN_GRIPPER":
            self._tick_pre_open_gripper()

        elif state == "SET_GRIPPER_ORIENTATION":
            self._tick_set_gripper_orientation()

        elif state in ("MOVE_PRE_GRASP", "MOVE_GRASP", "MOVE_LIFT",
                       "MOVE_RETREAT", "RETURN_INIT_POSE"):
            self._tick_active_stage()

        elif state == "CLOSE_GRIPPER":
            self._tick_close_gripper()

        elif state == "DONE":
            self._ctx.task_state = "IDLE"

    # ------------------------------------------------------------------
    # 各状态处理器（骨架，后续逐步填充）
    # ------------------------------------------------------------------

    def _tick_wait_target(self):
        timeout = self._config.active_search.target_acquire_timeout_sec
        elapsed = self._ctx.now_sec - self._ctx.stage_start_time

        if self._target_manager.is_stable():
            self._ctx.task_state = "PRE_OPEN_GRIPPER"
            self._ctx.stage_start_time = self._ctx.now_sec
            return

        if elapsed > timeout and self._config.active_search.enabled:
            self._ctx.task_state = "SEARCH_TARGET"
            self._search.start(self._ctx)

    def _tick_pre_open_gripper(self):
        # 骨架：后续接入完整 PRE_OPEN_GRIPPER 逻辑
        self._ctx.task_state = "SET_GRIPPER_ORIENTATION"

    def _tick_set_gripper_orientation(self):
        # 骨架：后续接入完整 J6 旋转逻辑
        self._ctx.task_state = "MOVE_PRE_GRASP"

    def _tick_active_stage(self):
        # 骨架：后续接入 FixedTargetMotionStage
        self._ctx.task_state = "DONE"

    def _tick_close_gripper(self):
        # 骨架：后续接入完整 CLOSE_GRIPPER 逻辑
        self._ctx.task_state = "DONE"

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------

    @staticmethod
    def _msg_to_observation(msg):
        """将 ROS msg 转为 TargetObservation。兼容现有格式。"""
        from .target_manager import TargetObservation

        x = float(getattr(msg, "x", 0.0))
        y = float(getattr(msg, "y", 0.0))
        z = float(getattr(msg, "z", 0.0))
        depth = float(getattr(msg, "depth", 0.0))
        confidence = float(getattr(msg, "confidence", 0.85))
        frame_id = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")
        stamp_sec = 0.0
        try:
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception:
            pass
        object_name = str(getattr(msg, "object_name", ""))
        return TargetObservation(
            x=x, y=y, z=z, depth=depth, confidence=confidence,
            frame_id=frame_id, stamp_sec=stamp_sec, object_name=object_name,
        )
