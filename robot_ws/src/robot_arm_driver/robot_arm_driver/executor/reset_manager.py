"""Reset 管理器：扩展 clear_error 为更完整的 reset 语义。

支持的 reset 命令：
  clear_error      - 仅清除 ERROR 状态，不移动机械臂，不重连 SDK
  soft_reset       - 清除内部状态和 active command，不移动机械臂
  reconnect        - 重连 SDK，不主动运动
  reinit           - 重连 SDK 并移动到 init_joint_pos（会运动机械臂）
  recover_safe     - 清错误后尝试回到安全位姿
  state_only_reset - 仅开发调试用，强制状态回 IDLE，不调用 SDK
  stop_motion      - 如 SDK 支持则停止运动，不支持则返回 unsupported
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


# ---------------------------------------------------------------------------
# Reset 类型
# ---------------------------------------------------------------------------

class ResetCommandType(Enum):
    CLEAR_ERROR = auto()
    SOFT_RESET = auto()
    RECONNECT = auto()
    REINIT = auto()
    RECOVER_SAFE = auto()
    STATE_ONLY_RESET = auto()
    STOP_MOTION = auto()


@dataclass
class ResetResult:
    action: str            # resetting / done / failed / rejected / unsupported
    detail: str = ""
    new_status: str = ""


# ---------------------------------------------------------------------------
# ResetManager
# ---------------------------------------------------------------------------

class ResetManager:
    """管理所有 reset 命令的分发与状态转换。"""

    SUPPORTED_COMMANDS = {
        "clear_error": ResetCommandType.CLEAR_ERROR,
        "soft_reset": ResetCommandType.SOFT_RESET,
        "reconnect": ResetCommandType.RECONNECT,
        "reinit": ResetCommandType.REINIT,
        "recover_safe": ResetCommandType.RECOVER_SAFE,
        "state_only_reset": ResetCommandType.STATE_ONLY_RESET,
        "stop_motion": ResetCommandType.STOP_MOTION,
    }

    def __init__(self, allow_state_only_reset: bool = True,
                 allow_reinit_reset: bool = True,
                 init_joint_pos_deg: Optional[list] = None):
        self._allow_state_only_reset = allow_state_only_reset
        self._allow_reinit_reset = allow_reinit_reset
        self._init_joint_pos_deg = init_joint_pos_deg or [
            0.0, -45.0, 110.0, -90.0, 90.0, 90.0]

    def handle(
        self, command_str: str, current_status: str,
        sdk_available: bool = True,
    ) -> ResetResult:
        """处理一条 reset 命令字符串。返回 ResetResult。"""
        cmd = command_str.strip().lower()

        if cmd not in self.SUPPORTED_COMMANDS:
            return ResetResult(
                action="unsupported",
                detail=f"unknown reset command: {cmd!r}",
                new_status=current_status,
            )

        cmd_type = self.SUPPORTED_COMMANDS[cmd]

        # ---- clear_error ----
        if cmd_type == ResetCommandType.CLEAR_ERROR:
            if current_status == "ERROR":
                return ResetResult(
                    action="done",
                    detail="executor ERROR cleared to IDLE",
                    new_status="IDLE",
                )
            return ResetResult(
                action="done",
                detail=f"clear_error ignored; executor is {current_status}",
                new_status=current_status,
            )

        # ---- soft_reset ----
        if cmd_type == ResetCommandType.SOFT_RESET:
            return ResetResult(
                action="done",
                detail="soft reset: internal state cleared",
                new_status="IDLE",
            )

        # ---- reconnect ----
        if cmd_type == ResetCommandType.RECONNECT:
            if not sdk_available:
                return ResetResult(
                    action="resetting",
                    detail="reconnect: SDK reconnecting",
                    new_status="RESETTING",
                )
            return ResetResult(
                action="done",
                detail="reconnect: SDK reconnected",
                new_status="RESET_DONE",
            )

        # ---- reinit ----
        if cmd_type == ResetCommandType.REINIT:
            if not self._allow_reinit_reset:
                return ResetResult(
                    action="rejected",
                    detail="reinit reset disabled by config",
                    new_status=current_status,
                )
            return ResetResult(
                action="resetting",
                detail="reinit: reconnecting and moving to init pose",
                new_status="RESETTING",
            )

        # ---- recover_safe ----
        if cmd_type == ResetCommandType.RECOVER_SAFE:
            return ResetResult(
                action="resetting",
                detail="recover_safe: returning to safe pose",
                new_status="RESETTING",
            )

        # ---- state_only_reset ----
        if cmd_type == ResetCommandType.STATE_ONLY_RESET:
            if not self._allow_state_only_reset:
                return ResetResult(
                    action="rejected",
                    detail="state_only_reset disabled by config",
                    new_status=current_status,
                )
            return ResetResult(
                action="done",
                detail="state_only_reset: forced IDLE (debug only, no SDK call)",
                new_status="IDLE",
            )

        # ---- stop_motion ----
        if cmd_type == ResetCommandType.STOP_MOTION:
            return ResetResult(
                action="unsupported",
                detail="stop_motion not supported by current SDK",
                new_status=current_status,
            )

        return ResetResult(
            action="unsupported",
            detail=f"unhandled reset command: {cmd}",
            new_status=current_status,
        )
