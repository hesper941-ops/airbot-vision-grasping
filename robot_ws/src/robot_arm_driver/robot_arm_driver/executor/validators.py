"""命令校验器：关节限位、笛卡尔 frame、夹爪参数、速度档位。"""

import math
from typing import Tuple


class JointLimitValidator:
    """全 6 关节限位检查。"""

    def __init__(self, joint_min_rad: list, joint_max_rad: list):
        if len(joint_min_rad) != 6 or len(joint_max_rad) != 6:
            raise ValueError("joint_min_rad and joint_max_rad must each have 6 values.")
        self._min = [float(v) for v in joint_min_rad]
        self._max = [float(v) for v in joint_max_rad]

    def validate(self, target: list) -> Tuple[bool, str]:
        """返回 (is_valid, error_message)。"""
        if len(target) != 6:
            return False, f"joint target length is {len(target)}, expected 6"

        for i in range(6):
            value = float(target[i])
            lo = self._min[i]
            hi = self._max[i]
            if value < lo or value > hi:
                deg = value * 180.0 / math.pi
                lo_deg = lo * 180.0 / math.pi
                hi_deg = hi * 180.0 / math.pi
                return False, (
                    f"J{i+1} target={value:.4f} rad ({deg:.2f} deg) "
                    f"outside [{lo:.4f}, {hi:.4f}] rad "
                    f"([{lo_deg:.2f}, {hi_deg:.2f}] deg)"
                )
        return True, ""


class CartesianFrameValidator:
    """笛卡尔 target frame 检查。"""

    VALID_FRAMES = {"base_link", ""}

    @classmethod
    def validate(cls, frame_id: str) -> Tuple[bool, str]:
        fid = frame_id.strip()
        if fid and fid not in cls.VALID_FRAMES:
            return False, f"cart target frame_id={fid!r}; expected base_link"
        return True, ""


class GripperCommandValidator:
    """夹爪命令检查。"""

    VALID_COMMANDS = {"open", "close"}

    @classmethod
    def validate(cls, command: str) -> Tuple[bool, str]:
        cmd = command.strip().lower()
        if cmd not in cls.VALID_COMMANDS:
            return False, f"unknown gripper command: {cmd!r}; expected open or close"
        return True, ""


class SpeedProfileValidator:
    """速度档位检查。"""

    VALID_PROFILES = {"slow", "default", "fast"}

    @classmethod
    def validate(cls, profile: str) -> Tuple[bool, str]:
        p = profile.strip().lower()
        if p not in cls.VALID_PROFILES:
            return False, f"unknown speed profile: {p!r}; expected slow/default/fast"
        return True, ""
