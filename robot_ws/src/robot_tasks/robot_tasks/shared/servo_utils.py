"""视觉伺服工具,供方案二(visual_servo)任务节点使用。

将像素空间误差转换为小步 Cartesian 修正量，包含死区、步长限幅
和连续稳定帧计数。
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ServoConfig:
    """视觉伺服闭环的可调参数。

    这些参数通过 YAML 配置文件注入，不同场景可独立调节。
    """

    gain_k: float = 0.0003          # 像素误差到米的比例系数
    max_step: float = 0.015         # 单次伺服修正最大步长（米）
    dead_zone_u: float = 5.0        # u 方向死区（像素），小于此值不动作
    dead_zone_v: float = 5.0        # v 方向死区（像素）
    threshold_u: float = 3.0        # u 方向稳定阈值（像素）
    threshold_v: float = 3.0        # v 方向稳定阈值（像素）
    stable_count_required: int = 5  # 需要连续多少帧稳定才允许下降
    history_size: int = 20
    z_descend_step: float = 0.03    # 最终下降步长（米）


@dataclass
class ServoState:
    """单次伺服对齐会话的运行时状态。"""

    consecutive_stable: int = 0     # 连续稳定帧计数
    aligned: bool = False           # 是否已对齐
    pixel_history: deque = field(default_factory=lambda: deque(maxlen=20))

    def reset(self):
        """重置状态，开始新一轮对齐。"""
        self.consecutive_stable = 0
        self.aligned = False
        self.pixel_history.clear()


class ServoCalculator:
    """根据像素误差计算小步 Cartesian 修正量。"""

    def __init__(self, config: Optional[ServoConfig] = None):
        self.cfg = config or ServoConfig()

    # ------------------------------------------------------------------
    # 像素误差计算
    # ------------------------------------------------------------------

    def compute_pixel_error(self, u: float, v: float,
                            cx: float, cy: float) -> tuple:
        """返回 (error_u, error_v)，单位为像素。

        error_u = u - cx （正值表示目标在图像中心右侧）
        error_v = v - cy （正值表示目标在图像中心下方）
        """
        return (u - cx, v - cy)

    def is_in_dead_zone(self, error_u: float, error_v: float) -> bool:
        """误差是否在死区内（不需要修正）。"""
        return (abs(error_u) < self.cfg.dead_zone_u
                and abs(error_v) < self.cfg.dead_zone_v)

    def is_stable(self, error_u: float, error_v: float) -> bool:
        """误差是否小于稳定阈值（可认为已对准）。"""
        return (abs(error_u) < self.cfg.threshold_u
                and abs(error_v) < self.cfg.threshold_v)

    # ------------------------------------------------------------------
    # 修正量计算
    # ------------------------------------------------------------------

    def compute_correction(self, error_u: float, error_v: float,
                           current_depth: Optional[float] = None) -> list:
        """将像素误差转换为 Cartesian 修正量 [dx, dy]。

        使用简化的小孔成像近似：
            dx ≈ gain * error_u * depth
            dy ≈ -gain * error_v * depth  （图像 v 轴与机械臂 y 轴方向相反）

        如果 depth 未知，使用默认值 0.5 m。
        修正量会被裁剪到 max_step。
        """
        depth = current_depth if current_depth and current_depth > 0.01 else 0.5

        dx = self.cfg.gain_k * error_u * depth
        dy = -self.cfg.gain_k * error_v * depth

        # 限幅
        step_norm = math.sqrt(dx * dx + dy * dy)
        if step_norm > self.cfg.max_step:
            scale = self.cfg.max_step / step_norm
            dx *= scale
            dy *= scale

        return [dx, dy]

    # ------------------------------------------------------------------
    # 下降条件判断
    # ------------------------------------------------------------------

    def should_descend(self, state: ServoState) -> bool:
        """连续稳定帧数是否已达到下降阈值。"""
        return state.consecutive_stable >= self.cfg.stable_count_required

    def update_stability(self, state: ServoState,
                         error_u: float, error_v: float):
        """根据当前帧的像素误差更新连续稳定计数器。

        如果当前帧误差在稳定阈值内，consecutive_stable +1；
        否则清零，需重新累积。
        """
        state.pixel_history.append((error_u, error_v))
        if self.is_stable(error_u, error_v):
            state.consecutive_stable += 1
        else:
            state.consecutive_stable = 0
