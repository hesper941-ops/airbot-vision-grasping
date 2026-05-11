# -*- coding: utf-8 -*-
"""AIRBOT 机械臂包装器。

该模块提供 AIRBOT Python SDK 的轻量级包装层，实现：
- 安全检查和软件限位
- 坐标变换和单位转换
- 速度档位管理
- 错误处理和状态监控

主要类：
- ArmSafetyConfig: 安全配置数据类
- AirbotWrapper: SDK 包装器主类

功能：
- 连接和断开机械臂
- 执行 Cartesian 和关节运动
- 获取状态反馈
- 夹爪控制
- 速度档位切换

安全特性：
- 工作空间限位检查
- 关节角度限位检查
- 最大步长限制
- 速度限制

依赖：
- airbot_py: AIRBOT Python SDK
"""

from contextlib import redirect_stderr, redirect_stdout
from dataclasses import dataclass, field
import io
import math
import time
from airbot_py.arm import AIRBOTPlay, RobotMode, SpeedProfile

# 机械臂安全配置数据类，定义软件限位参数和工作空间边界。
@dataclass
class ArmSafetyConfig:
    # 机械臂工作空间边界（单位：米）
    workspace_min: list = field(default_factory=lambda: [0.10, -0.45, 0.02])
    workspace_max: list = field(default_factory=lambda: [1.00, 0.50, 0.70])
    max_cartesian_step: float = 0.10
    max_joint_speed: float = 1.20
    joint_limit_margin: float = 0.08
    joint_limits: list = field(default_factory=lambda: [
        [-3.1416, 3.1416],
        [-2.6000, 2.6000],
        [-2.6000, 2.6000],
        [-3.1416, 3.1416],
        [-2.6000, 2.6000],
        [-3.1416, 3.1416],
    ])

# AIRBOT Python SDK 包装器，提供安全检查和简化接口。
class AirbotWrapper:
    FAILURE_TEXT = (
        "grpc error",
        "move group planning ptp failed",
        "planning failed",
        "motion failed",
        "not connected",
    )

    def __init__(self, url="localhost", port=50001, safety_config=None):
        self.url = url
        self.port = port
        self.robot = None
        self.safety = safety_config or ArmSafetyConfig()

    def connect(self, speed_profile="default"):
        """Connect to the AIRBOT server and set the initial speed profile."""
        if self.robot is None:
            self.robot = AIRBOTPlay(url=self.url, port=self.port)
            self.robot.connect()
        self.set_speed_profile(speed_profile)

    def disconnect(self):
        """Disconnect cleanly from the robot service."""
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None

    def set_speed_profile(self, speed_profile="default"):
        """Translate a simple speed label into SDK speed profile values."""
        if self.robot is None:
            raise RuntimeError("Robot is not connected.")

        if speed_profile == "slow":
            self.robot.set_speed_profile(SpeedProfile.SLOW)
        elif speed_profile == "fast":
            # 快速档位，适合非精细操作和大范围移动
            self.robot.set_speed_profile(SpeedProfile.FAST)
        else:
            # 默认档位，适合大多数操作
            self.robot.set_speed_profile(SpeedProfile.DEFAULT)

    def get_state(self):
        # 获取当前机械臂状态，包括关节位置、速度和末端位姿等信息。
        return self.robot.get_state()

    def get_joint_pos(self):
        # 获取当前关节位置
        return self.robot.get_joint_pos()

    def get_joint_vel(self):
        # 获取当前关节速度
        return self.robot.get_joint_vel()

    def get_end_pose(self):
        # 获取当前末端位姿，返回值格式为 [position, quaternion]，其中 position 是 [x, y, z]，quaternion 是 [x, y, z, w]。
        return self.robot.get_end_pose()

    def move_joints(self, joint_target):
        # 发送一个关节目标到机械臂，执行前进行安全检查。
        self.validate_joint_target(joint_target)
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self._call_sdk_checked(
            "move_to_joint_pos",
            self.robot.move_to_joint_pos,
            joint_target,
        )

    def move_cart_waypoints(self, waypoints):
        # 发送一系列笛卡尔路径点到机械臂，执行前进行安全检查。
        self.validate_cart_waypoints(waypoints)
        self.robot.switch_mode(RobotMode.PLANNING_WAYPOINTS)
        self._call_sdk_checked(
            "move_with_cart_waypoints",
            self.robot.move_with_cart_waypoints,
            waypoints,
        )

    def move_to_cart_target_with_current_orientation(self, target_xyz):
        # 以当前末端位姿的方向，移动到指定的笛卡尔位置，执行前进行安全检查。
        pose = self.get_end_pose()
        if pose is None or len(pose) < 2:
            raise RuntimeError("Failed to read current end pose.")
        current_quat = list(pose[1])
        self.move_cart_waypoints([
            [list(target_xyz), current_quat],
        ])
        return True

    def servo_joints(self, joint_target):
        # 以关节速度模式执行一个关节目标，适合需要快速响应的操作，执行前进行安全检查。
        self.validate_joint_target(joint_target)
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        self.robot.servo_joint_pos(joint_target)

    def go_home(self):
        # 走到初始工作位姿，通常在启动时执行，确保机械臂处于已知安全位置。
        home_joint = [0.0, -0.785398, 0.785398, 0.0, 0.0, 0.0]
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.robot.move_to_joint_pos(home_joint)

    def validate_joint_target(self, joint_target):
        #限制关节目标在安全范围内，拒绝任何超出软件限位的命令。
        if joint_target is None or len(joint_target) != len(self.safety.joint_limits):
            raise ValueError("Joint target must contain 6 values.")

        self._validate_current_joint_state()

        for index, value in enumerate(joint_target):
            low, high = self.safety.joint_limits[index]
            if not low <= float(value) <= high:
                raise ValueError(
                    f"Joint {index + 1} target {value:.3f} is outside "
                    f"limit [{low:.3f}, {high:.3f}]."
                )

    def validate_cart_target(self, target_xyz):
        # 验证笛卡尔目标的合法性，包括工作空间检查、步长限制和当前状态检查，拒绝任何不安全的命令。
        if target_xyz is None or len(target_xyz) != 3:
            raise ValueError("Cartesian target must contain x, y, z.")

        self._validate_current_joint_state()
        self._validate_workspace(target_xyz)

        pose = self.get_end_pose()
        if pose is None or len(pose) < 2:
            raise RuntimeError("Failed to read current end pose.")

        current_xyz = list(pose[0])
        step = self._distance(current_xyz, target_xyz)
        if step > self.safety.max_cartesian_step:
            raise ValueError(
                f"Cartesian step {step:.3f} m exceeds safe limit "
                f"{self.safety.max_cartesian_step:.3f} m."
            )

        quat = list(pose[1])
        self._validate_quaternion(quat)

    def validate_cart_waypoints(self, waypoints):
        # 验证一系列笛卡尔路径点的合法性，逐点检查工作空间、步长和当前状态，拒绝任何包含不安全点的路径。
        if not waypoints:
            raise ValueError("Cartesian waypoint list is empty.")

        previous_xyz = list(self.get_end_pose()[0])
        self._validate_current_joint_state()

        for waypoint in waypoints:
            if waypoint is None or len(waypoint) < 2:
                raise ValueError("Each waypoint must contain position and orientation.")
            target_xyz = list(waypoint[0])
            target_quat = list(waypoint[1])

            self._validate_workspace(target_xyz)
            self._validate_quaternion(target_quat)

            step = self._distance(previous_xyz, target_xyz)
            if step > self.safety.max_cartesian_step:
                raise ValueError(
                    f"Cartesian waypoint step {step:.3f} m exceeds safe limit "
                    f"{self.safety.max_cartesian_step:.3f} m."
                )
            previous_xyz = target_xyz

    def _validate_current_joint_state(self):
        # 在接受新命令前检查当前关节位姿和速度，确保机械臂处于安全状态。
        joint_pos = list(self.get_joint_pos())
        joint_vel = list(self.get_joint_vel())

        if len(joint_pos) != len(self.safety.joint_limits):
            raise RuntimeError("Current joint position does not contain 6 values.")

        for index, value in enumerate(joint_pos):
            low, high = self.safety.joint_limits[index]
            margin = self.safety.joint_limit_margin
            if value < low + margin or value > high - margin:
                raise RuntimeError(
                    f"Current joint {index + 1} is too close to its limit: {value:.3f}."
                )

        if joint_vel:
            max_speed = max(abs(float(v)) for v in joint_vel)
            if max_speed > self.safety.max_joint_speed:
                raise RuntimeError(
                    f"Current joint speed {max_speed:.3f} rad/s exceeds safe limit "
                    f"{self.safety.max_joint_speed:.3f} rad/s."
                )

    def _validate_workspace(self, target_xyz):
        # 检查目标位置是否在配置的工作空间范围内，拒绝任何超出边界的命令。
        for index, value in enumerate(target_xyz):
            low = self.safety.workspace_min[index]
            high = self.safety.workspace_max[index]
            if not low <= float(value) <= high:
                axis = "xyz"[index]
                raise ValueError(
                    f"Target {axis}={value:.3f} is outside workspace "
                    f"[{low:.3f}, {high:.3f}]."
                )

    def _validate_quaternion(self, quat):
        # 在接受笛卡尔命令前验证目标姿态的四元数是否合法，拒绝任何格式错误或归一化不正确的命令。
        if len(quat) != 4:
            raise ValueError("Orientation quaternion must contain 4 values.")

        norm = math.sqrt(sum(float(value) ** 2 for value in quat))
        if norm < 0.95 or norm > 1.05:
            raise ValueError(f"Orientation quaternion norm {norm:.3f} is invalid.")

    def _distance(self, a, b):
        # 计算两个3D位置之间的欧几里得距离。
        return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))

    def set_gripper_width(self, width, speed=None):
        """Command the end-effector opening width in meters."""
        if self.robot is None:
            raise RuntimeError("Robot is not connected.")

        target_width = float(width)
        if target_width < 0.0:
            raise ValueError("Gripper width must be non-negative.")

        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        for _ in range(50):
            self._call_sdk_checked(
                "servo_eef_pos",
                self.robot.servo_eef_pos,
                [target_width],
            )
            time.sleep(0.02)
        return True

    def command_gripper(self, command, target_width=0.0, speed=None):
        """Handle structured gripper commands while keeping open/close aliases."""
        command = str(command).strip().lower()
        if command == "open":
            return self.set_gripper_width(0.07, speed)
        elif command == "close":
            return self.set_gripper_width(target_width, speed)
        elif command in ("width", "set_width", "set"):
            return self.set_gripper_width(target_width, speed)
        else:
            raise ValueError(f"Unknown gripper command: {command}")

    def open_gripper(self):
        self.command_gripper("open", 0.07)

    def close_gripper(self):
        self.command_gripper("close", 0.0)

    def _call_sdk_checked(self, name, func, *args, **kwargs):
        """Call an SDK command and convert silent failures into exceptions."""
        if self.robot is None:
            raise RuntimeError("Robot is not connected.")

        stdout = io.StringIO()
        stderr = io.StringIO()
        try:
            with redirect_stdout(stdout), redirect_stderr(stderr):
                result = func(*args, **kwargs)
        except Exception as exc:
            output = f"{stdout.getvalue()} {stderr.getvalue()}".strip()
            detail = f"; sdk output: {output}" if output else ""
            raise RuntimeError(f"SDK call {name} raised {exc}{detail}") from exc

        output = f"{stdout.getvalue()} {stderr.getvalue()}".strip()
        lower_output = output.lower()
        if any(token in lower_output for token in self.FAILURE_TEXT):
            raise RuntimeError(f"SDK call {name} reported failure: {output}")

        if result is False or result is None:
            detail = f"; sdk output: {output}" if output else ""
            raise RuntimeError(
                f"SDK call {name} failed: returned {result!r}{detail}")

        return result
