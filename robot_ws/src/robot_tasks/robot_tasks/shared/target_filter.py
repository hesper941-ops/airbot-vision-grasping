"""视觉目标过滤与验证，供两个抓取任务节点共用。

该模块提供视觉目标的预处理和验证功能，包括：
- 工作空间边界检查
- 置信度阈值过滤
- 时间稳定性判断
- 历史轨迹跟踪

主要用于开环抓取和视觉伺服任务，确保目标可靠后再执行抓取。

过滤逻辑：
- 工作空间：检查目标是否在允许的 3D 范围内
- 置信度：高于阈值才接受
- 稳定性：连续多帧位置变化小于漂移阈值

配置参数：
- min_confidence_start: 启动最小置信度
- confidence_low: 低置信度阈值
- stable_count_required: 稳定性计数要求
- drift_threshold: 漂移阈值
- workspace_limits: 工作空间边界 {x_min, x_max, y_min, y_max, z_min, z_max}
"""

from collections import deque

from robot_msgs.msg import VisualTarget

from collections import deque

from robot_msgs.msg import VisualTarget


class TargetFilter:
    """校验 VisualTarget 的工作空间、置信度和时间稳定性。"""

    def __init__(self, config: dict):
        # 置信度阈值
        self.min_confidence = config.get('min_confidence_start', 0.7)
        self.confidence_low = config.get('confidence_low', 0.3)
        # 稳定性判断
        self.stable_count_required = config.get('stable_count_required', 3)
        self.drift_threshold = config.get('drift_threshold', 0.05)

        # 工作空间边界（base_link 坐标系）
        limits = config.get('workspace_limits', {})
        self.x_min = limits.get('x_min', 0.10)
        self.x_max = limits.get('x_max', 1.00)
        self.y_min = limits.get('y_min', -0.45)
        self.y_max = limits.get('y_max', 0.50)
        self.z_min = limits.get('z_min', 0.02)
        self.z_max = limits.get('z_max', 0.70)

        # 目标历史缓冲区，用于抖动检测
        self.history: deque = deque(maxlen=config.get('history_size', 10))

    # ------------------------------------------------------------------
    # 工作空间
    # ------------------------------------------------------------------

    def in_workspace(self, x: float, y: float, z: float) -> bool:
        """检查三维点是否在机械臂安全工作空间内。"""
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
        )

    def clamp_to_workspace(self, xyz: list) -> list:
        """将点裁剪到工作空间边界内。"""
        return [
            min(max(float(xyz[0]), self.x_min), self.x_max),
            min(max(float(xyz[1]), self.y_min), self.y_max),
            min(max(float(xyz[2]), self.z_min), self.z_max),
        ]

    # ------------------------------------------------------------------
    # 置信度
    # ------------------------------------------------------------------

    def has_min_confidence(self, target: VisualTarget) -> bool:
        """置信度是否满足启动抓取的要求。"""
        return target.confidence >= self.min_confidence

    def has_any_confidence(self, target: VisualTarget) -> bool:
        """置信度是否至少高于最低阈值（不过低）。"""
        return target.confidence >= self.confidence_low

    # ------------------------------------------------------------------
    # 稳定性
    # ------------------------------------------------------------------

    def update_history(self, target: VisualTarget):
        """将最新目标加入历史队列。"""
        self.history.append(target)

    def is_target_stable(self) -> bool:
        """最近 N 帧目标位置漂移是否在阈值内且置信度足够。"""
        if len(self.history) < self.stable_count_required:
            return False
        recent = list(self.history)[-self.stable_count_required:]
        # 所有帧置信度都必须高于最低阈值
        confidences = [t.confidence for t in recent]
        if min(confidences) < self.confidence_low:
            return False
        # 相邻帧间最大漂移必须小于阈值
        drift = max(
            self._distance([recent[i].x, recent[i].y, recent[i].z],
                           [recent[i + 1].x, recent[i + 1].y, recent[i + 1].z])
            for i in range(len(recent) - 1)
        )
        return drift < self.drift_threshold

    def target_drift(self, a: VisualTarget, b: VisualTarget) -> float:
        """计算两个目标之间的欧氏距离。"""
        return self._distance([a.x, a.y, a.z], [b.x, b.y, b.z])

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------

    @staticmethod
    def _distance(a: list, b: list) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5
