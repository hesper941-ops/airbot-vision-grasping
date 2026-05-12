"""独立的 RECOVER 模块：负责错误恢复，不负责目标不可见（目标不可见走 SEARCH_TARGET）。

RecoveryManager 内部管理子状态机：
  CLEAR_EXECUTOR_ERROR → OPEN_GRIPPER / KEEP_GRIPPER_CLOSED → SAFE_RETURN_INIT → DONE / FAILED
"""

import math
from typing import Optional

from .status_model import RecoveryResult, RecoveryTrigger, StageResult


class RecoveryManager:
    """错误恢复管理器。

    适用场景：executor error, motion timeout, reset failure, rejected 等。
    不适用场景：detector 看不到目标、目标不稳定——这些应进入 SEARCH_TARGET。
    """

    def __init__(self, config, command_port):
        self._config = config
        self._command_port = command_port

        self.active: bool = False
        self.stage: str = "IDLE"
        self.trigger: Optional[RecoveryTrigger] = None
        self.start_time_sec: float = 0.0

        # 子阶段状态
        self._command_sent: bool = False
        self._settle_start_sec: Optional[float] = None
        self._last_clear_error_sec: Optional[float] = None
        self._gripper_open_done: bool = False

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------

    def start(
        self, trigger, context,
        keep_gripper_closed: bool = False,
        reason: str = "",
    ) -> RecoveryResult:
        """进入 RECOVER。"""
        self.active = True
        self.trigger = trigger
        self.start_time_sec = context.now_sec
        self._command_sent = False
        self._settle_start_sec = None
        self._last_clear_error_sec = None
        self._gripper_open_done = False

        if keep_gripper_closed:
            self.stage = "KEEP_GRIPPER_CLOSED"
        else:
            self.stage = "CLEAR_EXECUTOR_ERROR"

        return RecoveryResult.running(self.stage)

    def tick(self, context) -> RecoveryResult:
        """每个 timer tick 调用一次。"""
        if not self.active:
            return RecoveryResult.idle()

        if self._is_timeout(context):
            self.active = False
            return RecoveryResult.failed("recover_timeout")

        # ---- CLEAR_EXECUTOR_ERROR ----
        if self.stage == "CLEAR_EXECUTOR_ERROR":
            return self._tick_clear_error(context)

        # ---- KEEP_GRIPPER_CLOSED（夹爪已闭合，不能打开） ----
        if self.stage == "KEEP_GRIPPER_CLOSED":
            self.stage = "SAFE_RETURN_INIT"
            self._reset_stage_vars()
            return RecoveryResult.running(self.stage)

        # ---- OPEN_GRIPPER ----
        if self.stage == "OPEN_GRIPPER":
            return self._tick_open_gripper(context)

        # ---- SAFE_RETURN_INIT ----
        if self.stage == "SAFE_RETURN_INIT":
            return self._tick_safe_return_init(context)

        return RecoveryResult.running(self.stage)

    def is_active(self) -> bool:
        return self.active

    def is_done(self) -> bool:
        return not self.active

    @property
    def result(self) -> RecoveryResult:
        if self.active:
            return RecoveryResult.running(self.stage)
        if self.stage == "RECOVER_DONE":
            return RecoveryResult.done()
        if self.stage == "RECOVER_FAILED":
            return RecoveryResult.failed("recover_failed")
        return RecoveryResult.idle()

    # ------------------------------------------------------------------
    # 子阶段
    # ------------------------------------------------------------------

    def _tick_clear_error(self, context) -> RecoveryResult:
        now = context.now_sec
        if context.executor_status not in ("ERROR", "TIMEOUT"):
            # 已清除，进入下一步
            if self._gripper_open_done or self.trigger == RecoveryTrigger.MANUAL_RESET:
                self.stage = "SAFE_RETURN_INIT"
            else:
                self.stage = "OPEN_GRIPPER"
            self._reset_stage_vars()
            return RecoveryResult.running(self.stage)

        if self._last_clear_error_sec is None or \
           now - self._last_clear_error_sec >= self._config.recover_clear_error_interval_sec:
            self._command_port.publish_reset(
                "clear_error", reason="recover:clear_executor_error")
            self._last_clear_error_sec = now

        return RecoveryResult.running("clear_executor_error")

    def _tick_open_gripper(self, context) -> RecoveryResult:
        if not self._command_sent:
            if context.executor_status not in ("IDLE", "DONE", ""):
                return RecoveryResult.running(self.stage)
            self._command_port.publish_gripper("open", reason="recover:open_gripper")
            self._command_sent = True
            self._settle_start_sec = context.now_sec
            return RecoveryResult.running(self.stage)

        if context.executor_status in ("BUSY",):
            return RecoveryResult.running(self.stage)

        settle = self._config.gripper_settle_sec
        if context.now_sec - (self._settle_start_sec or context.now_sec) >= settle:
            self._gripper_open_done = True
            self.stage = "SAFE_RETURN_INIT"
            self._reset_stage_vars()
            return RecoveryResult.running(self.stage)

        return RecoveryResult.running(self.stage)

    def _tick_safe_return_init(self, context) -> RecoveryResult:
        if not self._command_sent:
            if context.executor_status not in ("IDLE", "DONE", ""):
                return RecoveryResult.running(self.stage)
            init_deg = list(self._config.final_init_joint_pos_deg)
            init_rad = [math.radians(float(v)) for v in init_deg]
            self._command_port.publish_joint_target(
                init_rad, reason="recover:return_init_pose")
            self._command_sent = True
            return RecoveryResult.running(self.stage)

        if context.executor_status in ("BUSY",):
            return RecoveryResult.running(self.stage)

        # 等待关节速度安全
        # （由外部通过 last_joint_vel 判断；这里简单 settle）
        if self._settle_start_sec is None:
            self._settle_start_sec = context.now_sec
            return RecoveryResult.running(self.stage)

        if context.now_sec - self._settle_start_sec >= 1.0:
            self.stage = "RECOVER_DONE"
            self.active = False
            return RecoveryResult.done()

        return RecoveryResult.running(self.stage)

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _is_timeout(self, context) -> bool:
        return (context.now_sec - self.start_time_sec) > self._config.recover_timeout_sec

    def _reset_stage_vars(self):
        self._command_sent = False
        self._settle_start_sec = None
