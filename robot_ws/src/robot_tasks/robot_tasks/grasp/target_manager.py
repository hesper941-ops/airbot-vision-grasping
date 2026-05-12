"""目标管理器：负责目标合法性检查、稳定判断、跳变拒绝、last-seen fallback 和目标冻结。"""

import math
import statistics
from collections import deque
from dataclasses import dataclass
from typing import List, Optional


# ---------------------------------------------------------------------------
# 数据对象
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TargetObservation:
    """单帧观测。"""
    x: float
    y: float
    z: float
    depth: float
    confidence: float
    frame_id: str
    stamp_sec: float
    object_name: str = ""


@dataclass(frozen=True)
class FixedTargetSnapshot:
    """冻结的目标快照，运动阶段开始后不再受视觉更新影响。"""
    x: float
    y: float
    z: float
    frame_id: str
    source_stamp_sec: float
    object_name: str
    snapshot_stamp_sec: float


# ---------------------------------------------------------------------------
# TargetManager
# ---------------------------------------------------------------------------

class TargetManager:
    """管理视觉目标的接收、稳定判断、last-seen fallback 与冻结。

    所有判断条件均可通过 GraspTaskConfig 配置。
    """

    def __init__(self, config):
        self._config = config

        # 最新一帧
        self.latest_observation: Optional[TargetObservation] = None
        # 稳定目标（窗口内所有帧的 median）
        self.stable_target: Optional[TargetObservation] = None
        # 最后一次看到的有效目标
        self.last_seen: Optional[TargetObservation] = None
        self.last_seen_time_sec: Optional[float] = None
        # 当前活跃的运动目标（可能等于 latest / stable / last_seen）
        self.active_target: Optional[TargetObservation] = None
        # 是否已冻结
        self.frozen: bool = False

        # 稳定性窗口
        self._window = deque(
            maxlen=max(1, config.stable_frame_count_required))

    # ------------------------------------------------------------------
    # 接收新帧
    # ------------------------------------------------------------------

    def accept_observation(
        self, obs: TargetObservation, now_sec: float,
        active_target: Optional[list] = None,
        task_state: str = "",
    ) -> bool:
        """处理一帧新观测。返回 True 表示帧被接受并更新了内部状态。"""
        # 合法性
        if not self._is_valid(obs):
            self.reset_stability()
            return False

        # 跳变拒绝
        if self._is_large_jump(obs, active_target, task_state):
            self.reset_stability()
            self.latest_observation = obs
            self.last_seen = obs
            self.last_seen_time_sec = now_sec
            return False

        self.latest_observation = obs
        self.last_seen = obs
        self.last_seen_time_sec = now_sec

        # 稳定窗口更新
        self._window.append({
            "x": obs.x, "y": obs.y, "z": obs.z,
            "depth": obs.depth, "confidence": obs.confidence,
        })
        return True

    # ------------------------------------------------------------------
    # 稳定判断
    # ------------------------------------------------------------------

    def is_stable(self) -> bool:
        """当前窗口内的目标是否已经稳定。"""
        required = self._config.stable_frame_count_required
        if len(self._window) < required:
            return False

        median_xyz = self._stable_median()
        max_dist = max(
            math.hypot(
                e["x"] - median_xyz[0],
                e["y"] - median_xyz[1],
                e["z"] - median_xyz[2],
            )
            for e in self._window
        )

        depth_vals = [e["depth"] for e in self._window if math.isfinite(e["depth"])]
        max_depth_delta = 0.0
        if depth_vals:
            med_depth = statistics.median(depth_vals)
            max_depth_delta = max(abs(d - med_depth) for d in depth_vals)

        ok_pos = max_dist <= self._config.stable_position_threshold_m
        ok_depth = max_depth_delta <= self._config.stable_depth_threshold_m
        return ok_pos and ok_depth

    def get_stable_target(self) -> Optional[TargetObservation]:
        """返回当前稳定目标的 median 坐标。"""
        if not self.is_stable():
            return None
        median_xyz = self._stable_median()
        ref = self._window[-1]
        return TargetObservation(
            x=median_xyz[0], y=median_xyz[1], z=median_xyz[2],
            depth=ref.get("depth", 0.0),
            confidence=ref.get("confidence", self._config.confidence_threshold),
            frame_id=(self.latest_observation.frame_id if self.latest_observation else ""),
            stamp_sec=(self.latest_observation.stamp_sec if self.latest_observation else 0.0),
            object_name=(self.latest_observation.object_name if self.latest_observation else ""),
        )

    # ------------------------------------------------------------------
    # Last-seen fallback
    # ------------------------------------------------------------------

    def get_active_or_last_seen(
        self, now_sec: float, task_state: str = "",
        current_active: Optional[list] = None,
        is_frozen: bool = False,
    ) -> Optional[list]:
        """返回当前应使用的目标坐标（active 或 last_seen）。

        视觉目标可用时优先用 active；短暂丢失时 fallback 到 last_seen；
        last_seen 过期后返回 None。
        """
        max_age = self._config.last_seen_target_max_age_sec
        grace = self._config.visual_lost_grace_sec

        if self.last_seen is None or self.last_seen_time_sec is None:
            return None

        age = now_sec - self.last_seen_time_sec
        if age > max_age:
            return None

        use_last_seen = (
            self._config.use_last_seen_target_on_loss
        )
        if task_state in ("MOVE_PRE_GRASP", "MOVE_GRASP", "MOVE_LIFT"):
            use_last_seen = (
                use_last_seen
                and self._config.continue_with_last_seen_during_motion
            )

        if current_active is not None and not is_frozen:
            if age <= grace:
                return list(current_active)
            if not use_last_seen:
                return None
            return list(current_active)

        if not use_last_seen:
            return None

        return [self.last_seen.x, self.last_seen.y, self.last_seen.z]

    # ------------------------------------------------------------------
    # 目标冻结
    # ------------------------------------------------------------------

    def make_fixed_snapshot(self, now_sec: float) -> Optional[FixedTargetSnapshot]:
        """基于当前最新有效目标创建冻结快照。"""
        src = self.stable_target or self.latest_observation or self.last_seen
        if src is None:
            return None
        return FixedTargetSnapshot(
            x=src.x, y=src.y, z=src.z,
            frame_id=src.frame_id,
            source_stamp_sec=src.stamp_sec,
            object_name=src.object_name,
            snapshot_stamp_sec=now_sec,
        )

    def freeze(self):
        """冻结目标，后续不再更新 active_target。"""
        self.frozen = True

    def unfreeze(self):
        self.frozen = False

    # ------------------------------------------------------------------
    # 重置
    # ------------------------------------------------------------------

    def reset_stability(self):
        """清空稳定窗口。"""
        self._window.clear()
        self.stable_target = None

    def reset(self):
        """完全重置。"""
        self.latest_observation = None
        self.stable_target = None
        self.last_seen = None
        self.last_seen_time_sec = None
        self.active_target = None
        self.frozen = False
        self._window.clear()

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _is_valid(self, obs: TargetObservation) -> bool:
        if not all(math.isfinite(v) for v in (obs.x, obs.y, obs.z)):
            return False
        if not self._in_workspace([obs.x, obs.y, obs.z]):
            return False
        if math.isfinite(obs.confidence) and obs.confidence < self._config.confidence_threshold:
            return False
        return True

    def _is_large_jump(
        self, obs: TargetObservation,
        active_target: Optional[list],
        task_state: str,
    ) -> bool:
        if active_target is None:
            return False
        if task_state not in ("MOVE_PRE_GRASP", "MOVE_GRASP"):
            return False
        dist = math.hypot(
            obs.x - active_target[0],
            obs.y - active_target[1],
            obs.z - active_target[2],
        )
        z_jump = abs(obs.z - active_target[2])
        return (
            dist > self._config.max_target_jump_m
            or z_jump > self._config.max_target_z_jump_m
        )

    def _in_workspace(self, xyz: List[float]) -> bool:
        return (
            self._config.workspace_x_min <= xyz[0] <= self._config.workspace_x_max
            and self._config.workspace_y_min <= xyz[1] <= self._config.workspace_y_max
            and self._config.workspace_z_min <= xyz[2] <= self._config.workspace_z_max
        )

    def _stable_median(self) -> List[float]:
        return [
            statistics.median(e["x"] for e in self._window),
            statistics.median(e["y"] for e in self._window),
            statistics.median(e["z"] for e in self._window),
        ]

    def last_seen_age_sec(self, now_sec: float) -> Optional[float]:
        if self.last_seen_time_sec is None:
            return None
        return now_sec - self.last_seen_time_sec
