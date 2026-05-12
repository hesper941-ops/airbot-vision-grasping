"""机械臂内部命令模型。"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class JointMotionCommand:
    """关节运动命令。"""
    joint_pos: list
    source: str = "ros_topic"
    reason: str = ""


@dataclass
class CartesianMotionCommand:
    """笛卡尔运动命令。"""
    x: float
    y: float
    z: float
    frame_id: str = "base_link"
    source: str = "ros_topic"
    reason: str = ""


@dataclass
class GripperCommand:
    """夹爪命令。"""
    command: str  # "open" / "close"
    target_width: float = 0.0
    speed: Optional[float] = None
    source: str = "ros_topic"
    reason: str = ""


@dataclass
class SpeedProfileCommand:
    """速度档位命令。"""
    profile: str  # "slow" / "default" / "fast"
    source: str = "ros_topic"
    reason: str = ""


@dataclass
class ResetCommand:
    """执行器 reset 命令。"""
    command: str   # clear_error / soft_reset / reconnect / reinit / ...
    reason: str = ""
    source: str = "ros_topic"
