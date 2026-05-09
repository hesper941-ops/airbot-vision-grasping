"""抓取路径点规划器，供两个抓取任务节点共用。

该模块负责根据 3D 视觉目标生成分阶段 Cartesian 路径点。
主要用于开环抓取和视觉伺服的粗定位阶段。

规划的路径点包括：
- pre_grasp: 目标上方预抓取点
- grasp: 实际抓取点
- lift: 抓取后抬升点
- safe: 安全撤退位姿

特性：
- Z 轴偏移控制抓取深度
- 工作空间边界检查
- Joint6 旋转补偿调整夹爪朝向
- 距离和到达判定计算

配置参数：
- pre_grasp_z_offset: 预抓取 Z 偏移
- grasp_z_offset: 抓取 Z 偏移
- lift_z_offset: 抬升 Z 偏移
- safe_pose: 安全位姿 [x, y, z]
- joint6_compensation_deg: Joint6 补偿角度
- workspace_limits: 工作空间边界
"""

import math
from typing import Optional

import math
from typing import Optional

# 该模块依赖于 AirbotWrapper 来获取当前状态和执行路径点命令，确保生成的路径点在机械臂的工作空间内，并且每步移动都在安全范围内。
class GraspPlanner:
    """根据 3D 视觉目标生成分阶段 Cartesian 路径点。

    两个方案的粗定位阶段共用同一套路径点生成逻辑。
    """

    def __init__(self, config: dict):
        # Z 方向偏移量
        self.pre_grasp_z_offset = config.get('pre_grasp_z_offset', 0.12)
        self.grasp_z_offset = config.get('grasp_z_offset', 0.0)
        self.lift_z_offset = config.get('lift_z_offset', 0.10)

        # 安全撤退位姿
        self.safe_pose = config.get('safe_pose', [0.35, 0.00, 0.35])

        # Joint6 末端旋转补偿（度），用于调整夹爪朝向
        self.joint6_compensation_deg = config.get('joint6_compensation_deg', 90.0)

        # 工作空间边界
        limits = config.get('workspace_limits', {})
        self.x_min = limits.get('x_min', 0.10)
        self.x_max = limits.get('x_max', 1.00)
        self.y_min = limits.get('y_min', -0.45)
        self.y_max = limits.get('y_max', 0.50)
        self.z_min = limits.get('z_min', 0.02)
        self.z_max = limits.get('z_max', 0.70)

    # ------------------------------------------------------------------
    # 路径点生成
    # ------------------------------------------------------------------

    def compute_pre_grasp(self, target_xyz: list) -> list:
        """pre_grasp 点：目标正上方 offset 处。"""
        return self._clamp([
            target_xyz[0],
            target_xyz[1],
            target_xyz[2] + self.pre_grasp_z_offset,
        ])

    def compute_grasp(self, target_xyz: list) -> list:
        """抓取点：目标位置加可选的 Z 偏移。"""
        return self._clamp([
            target_xyz[0],
            target_xyz[1],
            target_xyz[2] + self.grasp_z_offset,
        ])

    def compute_lift(self, current_xyz: list) -> list:
        """抬升点：从当前位置垂直向上。"""
        return self._clamp([
            current_xyz[0],
            current_xyz[1],
            min(current_xyz[2] + self.lift_z_offset, self.z_max),
        ])

    def get_safe_pose(self) -> list:
        """返回配置的安全位姿（浅拷贝）。"""
        return list(self.safe_pose)

    # ------------------------------------------------------------------
    # 步长控制
    # ------------------------------------------------------------------

    @staticmethod
    def distance(a: list, b: list) -> float:
        """两点欧氏距离。"""
        return math.sqrt(
            (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
        )

    @staticmethod
    def limit_step(current: list, target: list, max_step: float) -> list:
        """单步限幅：确保每步移动不超过 max_step（米）。"""
        d = GraspPlanner.distance(current, target)
        if d <= max_step:
            return target
        ratio = max_step / d
        return [
            current[0] + (target[0] - current[0]) * ratio,
            current[1] + (target[1] - current[1]) * ratio,
            current[2] + (target[2] - current[2]) * ratio,
        ]

    @staticmethod
    def reached(current: list, target: list, threshold: float = 0.03) -> bool:
        """判断当前位置是否已到达目标（默认 3 cm 内）。"""
        return GraspPlanner.distance(current, target) < threshold

    # ------------------------------------------------------------------
    # Joint6 末端补偿
    # ------------------------------------------------------------------

    @property
    def joint6_compensation_rad(self) -> float:
        """Joint6 补偿角的弧度值。"""
        return math.radians(self.joint6_compensation_deg)

    def compute_joint6_target(self, current_joint_pos: list) -> Optional[list]:
        """生成仅旋转 joint6 的关节目标，joint1-5 保持不变。

        用于在 pre_grasp 前统一末端朝向。
        补偿方向在 pre_grasp 之前确定，pre_grasp 与 grasp 共用。
        """
        if len(current_joint_pos) < 6:
            return None
        target = list(current_joint_pos[:])
        target[5] += self.joint6_compensation_rad
        return target

    # ------------------------------------------------------------------
    # 内部方法
    # ------------------------------------------------------------------

    def _clamp(self, xyz: list) -> list:
        """裁剪到工作空间内。"""
        return [
            min(max(float(xyz[0]), self.x_min), self.x_max),
            min(max(float(xyz[1]), self.y_min), self.y_max),
            min(max(float(xyz[2]), self.z_min), self.z_max),
        ]
