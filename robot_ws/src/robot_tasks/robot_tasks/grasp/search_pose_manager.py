"""搜索姿态管理器：从配置中读取搜索姿态，管理循环搜索序列。"""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class SearchPose:
    """单个安全观察姿态。"""
    name: str
    joint_pos: List[float]        # rad
    speed_scale: float
    settle_time_sec: float
    detect_window_sec: float


class SearchPoseManager:
    """管理搜索姿态队列与循环。"""

    def __init__(self, poses: List[SearchPose], max_cycles: int):
        self._poses = poses
        self._max_cycles = max_cycles
        self._index: int = 0
        self._cycle_count: int = 0

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------

    def reset(self):
        """重置回第一个搜索姿态。"""
        self._index = 0
        self._cycle_count = 0

    def get_next_pose(self) -> Optional[SearchPose]:
        """返回下一个搜索姿态；搜索完所有姿态后返回 None。"""
        if self.is_finished():
            return None

        if not self._poses:
            return None

        pose = self._poses[self._index]
        self._index += 1

        if self._index >= len(self._poses):
            self._index = 0
            self._cycle_count += 1

        return pose

    def is_finished(self) -> bool:
        return self._cycle_count >= self._max_cycles

    @property
    def current_index(self) -> int:
        return self._index

    @property
    def cycle_count(self) -> int:
        return self._cycle_count

    @property
    def max_cycles(self) -> int:
        return self._max_cycles

    @property
    def pose_count(self) -> int:
        return len(self._poses)
