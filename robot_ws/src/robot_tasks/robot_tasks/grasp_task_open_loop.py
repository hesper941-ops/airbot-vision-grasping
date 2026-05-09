#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""方案一：开环抓取任务节点。

该节点实现了一个完整的开环抓取流程，通过 ROS 2 话题与执行层交互。
主要功能包括：
- 接收视觉目标 (VisualTarget 消息)
- 过滤和验证目标（置信度、工作空间检查）
- 连续 N 帧稳定后锁定 frozen_target
- 规划分阶段抓取路径 (J6_ROTATE → pre_grasp → grasp → lift → retreat)
- 执行阶段绝不重复发命令，只用 frozen_target
- 新视觉结果只缓存为 latest_target，不修改当前阶段目标
- 监控执行状态并切换状态机

流程：
  1. WAIT_TARGET: 持续接收视觉目标，连续 N 帧稳定后锁定为 frozen_target
  2. J6_ROTATE: 关节6旋转补偿，等待速度回落 + post_joint_rotate_settle
  3. MOVE_PRE_GRASP: 移动到预抓取点（基于 frozen_target），到位停稳后恢复视觉参与
  4. MOVE_DESCEND: 下降到抓取点
  5. CLOSE_GRIPPER: 闭合夹爪
  6. MOVE_LIFT: 抬升
  7. MOVE_RETREAT: 返回安全位姿
  8. DONE → IDLE

特点：
  - 最简单、最容易先跑通
  - 适合作为保底方案
  - 不直接冲最终抓取点，必须分阶段执行
  - 抓取前 joint6 有固定角度补偿
  - 支持命令串行化，避免重复下发
  - 到位判定 + 稳定停留，减少抖动
  - 抓取阶段自动切换慢速档位

依赖：
  - robot_msgs/VisualTarget: 视觉目标输入
  - robot_msgs/ArmJointState: 机械臂状态反馈（含 joint_vel）
  - geometry_msgs/PointStamped: Cartesian 目标输出
  - std_msgs/Float64MultiArray: 关节目标输出
  - std_msgs/String: 夹爪指令输出

参数：
  - stable_frame_count_required: 连续稳定帧数要求
  - stable_position_threshold_m: 位置稳定阈值 (m)
  - stable_depth_threshold_m: 深度稳定阈值 (m)
  - confidence_threshold: 置信度阈值
  - position_tolerance_m: 到位判定容差 (m)
  - settle_time_sec: 到位后稳定停留时间 (s)
  - joint_speed_safe_threshold: J6 安全速度阈值 (rad/s)
  - post_joint_rotate_settle_sec: J6 旋转后额外稳定时间 (s)
  - pre_grasp_z_offset / grasp_z_offset / lift_z_offset: Z 偏移
  - gripper_settle_sec: 夹爪稳定时间
  - cmd_timeout_sec: 命令超时
  - loop_hz: 主循环频率
"""

from datetime import datetime, timedelta
import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from robot_msgs.msg import ArmJointState, VisualTarget
from robot_tasks.shared.grasp_planner import GraspPlanner
from robot_tasks.shared.target_filter import TargetFilter


class GraspTaskOpenLoop(Node):
    """开环抓取任务节点 —— 一次识别 + 坐标转换 + 分阶段抓取。

    本节点只通过 ROS topic 与执行层交互，不直接触碰 AIRBOT SDK。
    """

    def __init__(self):
        super().__init__('grasp_task_open_loop')

        # ---- 加载参数（默认值 + YAML 覆盖） ----
        self._declare_parameters()

        # ---- 共享工具 ----
        self.filter = TargetFilter(self._config_dict())
        self.planner = GraspPlanner(self._config_dict())

        # ---- 状态机 ----
        # IDLE → WAIT_TARGET → J6_ROTATE → MOVE_PRE_GRASP → MOVE_DESCEND
        #   → CLOSE_GRIPPER → MOVE_LIFT → MOVE_RETREAT → DONE → IDLE
        self.task_state = 'IDLE'
        self.current_stage = None

        # ---- 三层目标分离 ----
        self.latest_target: VisualTarget = None     # 最新接收到的视觉目标（始终更新）
        self.frozen_target: VisualTarget = None      # 锁定后的目标（WAIT_TARGET 结束后冻结）
        self.active_stage_target: list = None        # 当前执行阶段的 Cartesian 路径点

        # ---- 稳定性跟踪（WAIT_TARGET 阶段） ----
        self._stable_frame_count = 0
        self._last_valid_target: VisualTarget = None

        # ---- 命令串行化：每阶段只发一次命令 ----
        self.state_command_sent = False
        self.stage_motion_started = False
        self.settle_start_time = None               # datetime, 到位后开始稳定的时刻

        # ---- J6 旋转后稳定 ----
        self._j6_speed_safe = False
        self._j6_settle_start_time = None

        # ---- 运行时变量 ----
        self.last_end_pose = None                   # [x, y, z]
        self.last_joint_pos = None                  # [j1..j6]
        self.last_joint_vel = None                  # [j1..j6]
        self.last_cmd_target = None                 # 最后一次下发的 Cartesian 目标
        self.last_cmd_time = None                   # 最后一次下发时间
        self.gripper_closed = False
        self.gripper_settle_until = None            # 夹爪闭合后等待的截止时间
        self.lift_target = None                     # 抬升目标（抓取后动态计算）
        self.joint6_compensated = False             # joint6 补偿是否已执行

        # ---- 超时 ----
        self.cmd_timeout = timedelta(seconds=self.get_parameter(
            'cmd_timeout_sec').value)

        # ---- 速度档位 ----
        self.speed_profile_active = 'default'

        # ---- 订阅 ----
        self.target_sub = self.create_subscription(
            VisualTarget,
            '/visual_target_base',
            self.target_callback,
            10,
        )
        self.state_sub = self.create_subscription(
            ArmJointState,
            '/robot_arm/joint_state',
            self.state_callback,
            10,
        )

        # ---- 发布 ----
        self.cart_pub = self.create_publisher(
            PointStamped, '/robot_arm/cart_target', 10)
        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/robot_arm/target_joint', 10)
        self.gripper_pub = self.create_publisher(
            String, '/robot_arm/gripper_cmd', 10)

        # ---- 主循环定时器 ----
        loop_hz = self.get_parameter('loop_hz').value
        self.timer = self.create_timer(1.0 / loop_hz, self.step_loop)

        self.get_logger().info('GraspTaskOpenLoop 启动，状态: IDLE')

    # ==================================================================
    # 参数声明
    # ==================================================================

    def _declare_parameters(self):
        """声明所有可调参数及其默认值。"""
        # ---- 路径点参数 ----
        self.declare_parameter('pre_grasp_z_offset', 0.12)
        self.declare_parameter('grasp_z_offset', 0.0)
        self.declare_parameter('lift_z_offset', 0.10)

        # ---- 置信度与稳定性 ----
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('stable_frame_count_required', 5)
        self.declare_parameter('stable_position_threshold_m', 0.015)
        self.declare_parameter('stable_depth_threshold_m', 0.03)

        # ---- 工作空间 ----
        self.declare_parameter('workspace_limits.x_min', 0.10)
        self.declare_parameter('workspace_limits.x_max', 1.00)
        self.declare_parameter('workspace_limits.y_min', -0.45)
        self.declare_parameter('workspace_limits.y_max', 0.50)
        self.declare_parameter('workspace_limits.z_min', 0.02)
        self.declare_parameter('workspace_limits.z_max', 0.70)

        # ---- 安全位姿 ----
        self.declare_parameter('safe_pose', [0.35, 0.0, 0.35])

        # ---- Joint6 末端补偿（度） ----
        self.declare_parameter('joint6_compensation_deg', 90.0)

        # ---- 到位判定与稳定停留 ----
        self.declare_parameter('position_tolerance_m', 0.02)
        self.declare_parameter('settle_time_sec', 0.5)

        # ---- J6 旋转后安全条件 ----
        self.declare_parameter('joint_speed_safe_threshold', 0.1)
        self.declare_parameter('post_joint_rotate_settle_sec', 0.5)

        # ---- 夹爪 ----
        self.declare_parameter('gripper_settle_sec', 1.0)

        # ---- 超时与频率 ----
        self.declare_parameter('cmd_timeout_sec', 10.0)
        self.declare_parameter('loop_hz', 4)

    def _config_dict(self) -> dict:
        """将 ROS 参数组装为共享模块所需的 dict 格式。"""
        return {
            'pre_grasp_z_offset': self.get_parameter('pre_grasp_z_offset').value,
            'grasp_z_offset': self.get_parameter('grasp_z_offset').value,
            'lift_z_offset': self.get_parameter('lift_z_offset').value,
            'min_confidence_start': self.get_parameter('confidence_threshold').value,
            'confidence_low': self.get_parameter('confidence_threshold').value * 0.5,
            'stable_count_required': self.get_parameter('stable_frame_count_required').value,
            'drift_threshold': self.get_parameter('stable_position_threshold_m').value,
            'safe_pose': self.get_parameter('safe_pose').value,
            'joint6_compensation_deg': self.get_parameter('joint6_compensation_deg').value,
            'workspace_limits': {
                'x_min': self.get_parameter('workspace_limits.x_min').value,
                'x_max': self.get_parameter('workspace_limits.x_max').value,
                'y_min': self.get_parameter('workspace_limits.y_min').value,
                'y_max': self.get_parameter('workspace_limits.y_max').value,
                'z_min': self.get_parameter('workspace_limits.z_min').value,
                'z_max': self.get_parameter('workspace_limits.z_max').value,
            },
        }

    # ==================================================================
    # 回调
    # ==================================================================

    def target_callback(self, msg: VisualTarget):
        """Receive visual target and handle according to current state.

        - WAIT_TARGET: validate, check stability, freeze after N stable frames
        - Execution states: only cache as latest_target, never modify frozen_target
        """
        try:
            # 校验 frame_id
            frame_id = msg.header.frame_id.strip()
            if frame_id and frame_id != 'base_link':
                self.get_logger().error(
                    f'frame_id mismatch: {frame_id}, expected base_link')
                return

            # 校验工作空间
            if not self.filter.in_workspace(msg.x, msg.y, msg.z):
                self.get_logger().error(
                    f'Target out of workspace: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')
                return

            # 校验置信度
            confidence_threshold = self.get_parameter('confidence_threshold').value
            if msg.confidence < confidence_threshold:
                self.get_logger().warning(
                    f'Target confidence too low ({msg.confidence:.2f} < {confidence_threshold:.2f})')
                return

            # ---- 始终更新 latest_target ----
            self.latest_target = msg

            self.get_logger().info(
                f'Received target: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) '
                f'confidence={msg.confidence:.2f} depth={msg.depth:.3f}')

            # ---- WAIT_TARGET 阶段：做稳定性判定并冻结 ----
            if self.task_state == 'WAIT_TARGET':
                self._check_stability_and_freeze(msg)
            else:
                # 执行阶段：只缓存，不修改 frozen_target / active_stage_target
                self.get_logger().debug(
                    f'Target received during {self.task_state}, cached as latest_target only')

        except Exception as e:
            self.get_logger().error(f'target_callback exception: {e}', exc_info=True)

    def state_callback(self, msg: ArmJointState):
        """Store the latest arm end pose, joint positions, and joint velocities."""
        try:
            if msg.end_pose and len(msg.end_pose) >= 3:
                self.last_end_pose = [msg.end_pose[0],
                                      msg.end_pose[1],
                                      msg.end_pose[2]]
            if msg.joint_pos and len(msg.joint_pos) >= 6:
                self.last_joint_pos = list(msg.joint_pos)
            if msg.joint_vel and len(msg.joint_vel) >= 6:
                self.last_joint_vel = list(msg.joint_vel)
        except Exception as e:
            self.get_logger().error(f'state_callback exception: {e}', exc_info=True)

    # ==================================================================
    # 稳定性判定与目标冻结（仅在 WAIT_TARGET 阶段调用）
    # ==================================================================

    def _check_stability_and_freeze(self, msg: VisualTarget):
        """Check if the incoming target is stable vs the previous one.

        Stability criteria:
        - Position drift < stable_position_threshold_m
        - Depth drift < stable_depth_threshold_m

        When stable_frame_count_required consecutive frames pass, freeze the target.
        """
        stable_pos_th = self.get_parameter('stable_position_threshold_m').value
        stable_depth_th = self.get_parameter('stable_depth_threshold_m').value
        required_count = self.get_parameter('stable_frame_count_required').value

        if self._last_valid_target is not None:
            pos_drift = self._distance_xyz(
                msg.x, msg.y, msg.z,
                self._last_valid_target.x,
                self._last_valid_target.y,
                self._last_valid_target.z,
            )
            depth_drift = abs(msg.depth - self._last_valid_target.depth)

            if pos_drift <= stable_pos_th and depth_drift <= stable_depth_th:
                self._stable_frame_count += 1
                self.get_logger().info(
                    f'Stable frame {self._stable_frame_count}/{required_count} '
                    f'(pos_drift={pos_drift:.4f}m, depth_drift={depth_drift:.4f}m)')
            else:
                # 漂移过大，重置计数
                self.get_logger().info(
                    f'Target drifted, resetting stability counter '
                    f'(pos_drift={pos_drift:.4f}m, depth_drift={depth_drift:.4f}m)')
                self._stable_frame_count = 1
        else:
            self._stable_frame_count = 1

        self._last_valid_target = msg

        if self._stable_frame_count >= required_count:
            self.frozen_target = msg
            self.get_logger().info(
                f'***** TARGET FROZEN after {self._stable_frame_count} stable frames *****\n'
                f'  position=({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})\n'
                f'  confidence={msg.confidence:.2f}  depth={msg.depth:.3f}')
            self._stable_frame_count = 0           # 重置供下次使用
            self._last_valid_target = None
            self._transition('J6_ROTATE')

    # ==================================================================
    # 主状态机
    # ==================================================================

    def step_loop(self):
        """Main state machine, called at loop_hz frequency."""
        try:
            if self.task_state == 'IDLE':
                self._transition('WAIT_TARGET')

            elif self.task_state == 'WAIT_TARGET':
                pass  # target_callback handles transition to J6_ROTATE

            elif self.task_state == 'J6_ROTATE':
                self._handle_j6_rotate()

            elif self.task_state == 'MOVE_PRE_GRASP':
                self._handle_move_pre_grasp()

            elif self.task_state == 'MOVE_DESCEND':
                self._handle_move_descend()

            elif self.task_state == 'CLOSE_GRIPPER':
                self._handle_close_gripper()

            elif self.task_state == 'MOVE_LIFT':
                self._handle_move_lift()

            elif self.task_state == 'MOVE_RETREAT':
                self._handle_move_retreat()

            elif self.task_state == 'DONE':
                self.get_logger().info('抓取流程完成，返回 IDLE')
                self._set_speed_profile('default')
                self._reset_task()
                self._transition('IDLE')

            elif self.task_state == 'ABORT':
                self.get_logger().error('任务中止，返回 WAIT_TARGET')
                self._reset_task()
                self._transition('WAIT_TARGET')

        except Exception as e:
            self.get_logger().error(f'step_loop exception: {e}', exc_info=True)
            self._reset_task()
            self._transition('WAIT_TARGET')

    # ------------------------------------------------------------------
    # J6_ROTATE：关节6旋转补偿
    # ------------------------------------------------------------------

    def _handle_j6_rotate(self):
        """J6_ROTATE: send joint6 rotation command once, then wait for
        joint speed to drop below safe threshold + extra settle time."""
        if self.last_joint_pos is None:
            return

        # ---- 首次进入：下发 J6 旋转指令 ----
        if not self.state_command_sent:
            if self.joint6_compensated:
                # 已补偿过，直接跳过
                self._finish_j6_and_proceed()
                return

            j6_target = self.planner.compute_joint6_target(self.last_joint_pos)
            if j6_target is None:
                self.get_logger().error('无法计算 joint6 目标')
                self._transition('ABORT')
                return

            self._publish_joint_target(j6_target)
            self.state_command_sent = True
            self.joint6_compensated = True
            self.get_logger().info('已下发 joint6 补偿指令，等待速度回落')
            return

        # ---- 等待关节速度降至安全阈值 ----
        joint_speed_th = self.get_parameter('joint_speed_safe_threshold').value
        if not self._j6_speed_safe:
            if self.last_joint_vel is None:
                return
            max_speed = max(abs(v) for v in self.last_joint_vel)
            if max_speed < joint_speed_th:
                self._j6_speed_safe = True
                settle_sec = self.get_parameter('post_joint_rotate_settle_sec').value
                self._j6_settle_start_time = datetime.now() + timedelta(seconds=settle_sec)
                self.get_logger().info(
                    f'J6 速度已安全 (max={max_speed:.4f} < {joint_speed_th}), '
                    f'等待额外稳定 {settle_sec:.1f}s')
            return

        # ---- 额外稳定等待 ----
        if self._j6_settle_start_time and datetime.now() >= self._j6_settle_start_time:
            self._finish_j6_and_proceed()

    def _finish_j6_and_proceed(self):
        """Clean up J6_ROTATE state and transition to MOVE_PRE_GRASP."""
        self.get_logger().info('J6_ROTATE 完成，进入 MOVE_PRE_GRASP')

        # 切换到慢速档位
        self._set_speed_profile('slow')

        # 重置阶段变量
        self._reset_stage_vars()
        self.current_stage = 'MOVE_PRE_GRASP'
        self._transition('MOVE_PRE_GRASP')

    # ------------------------------------------------------------------
    # MOVE_PRE_GRASP：预抓取阶段
    # ------------------------------------------------------------------

    def _handle_move_pre_grasp(self):
        """Move to pre_grasp position based on frozen_target.

        One command only, then wait for position reached + settle.
        After settlement, resume visual participation for verification.
        """
        if self.frozen_target is None:
            self.get_logger().error('frozen_target is None in MOVE_PRE_GRASP')
            self._transition('ABORT')
            return

        self._handle_cartesian_stage(
            stage_name='MOVE_PRE_GRASP',
            next_state='MOVE_DESCEND',
            waypoint_fn=lambda: self._compute_pre_grasp_waypoint(),
            on_settled_fn=self._on_pre_grasp_settled,
        )

    def _compute_pre_grasp_waypoint(self) -> list:
        """Compute pre_grasp waypoint from frozen_target. Called once at stage entry."""
        target_xyz = [self.frozen_target.x,
                      self.frozen_target.y,
                      self.frozen_target.z]
        return self.planner.compute_pre_grasp(target_xyz)

    def _on_pre_grasp_settled(self):
        """Called after pre_grasp position is reached and settled.

        Resume visual participation: verify latest_target is still near frozen_target.
        """
        if self.latest_target is None:
            self.get_logger().warning(
                'No latest_target available after pre_grasp settle, '
                'proceeding with frozen_target')
            return True

        drift = self._distance_xyz(
            self.latest_target.x, self.latest_target.y, self.latest_target.z,
            self.frozen_target.x, self.frozen_target.y, self.frozen_target.z,
        )
        max_drift = self.get_parameter('stable_position_threshold_m').value * 3.0
        if drift > max_drift:
            self.get_logger().error(
                f'Target drifted significantly after pre_grasp: {drift:.4f}m > {max_drift:.3f}m')
            return False
        self.get_logger().info(
            f'Visual verification passed after pre_grasp (drift={drift:.4f}m)')
        return True

    # ------------------------------------------------------------------
    # MOVE_DESCEND：下降阶段
    # ------------------------------------------------------------------

    def _handle_move_descend(self):
        """Move to grasp position based on frozen_target.

        One command only, then wait for position reached + settle.
        """
        if self.frozen_target is None:
            self.get_logger().error('frozen_target is None in MOVE_DESCEND')
            self._transition('ABORT')
            return

        self._handle_cartesian_stage(
            stage_name='MOVE_DESCEND',
            next_state='CLOSE_GRIPPER',
            waypoint_fn=lambda: self._compute_descend_waypoint(),
        )

    def _compute_descend_waypoint(self) -> list:
        """Compute grasp waypoint from frozen_target."""
        target_xyz = [self.frozen_target.x,
                      self.frozen_target.y,
                      self.frozen_target.z]
        return self.planner.compute_grasp(target_xyz)

    # ------------------------------------------------------------------
    # CLOSE_GRIPPER：夹爪闭合
    # ------------------------------------------------------------------

    def _handle_close_gripper(self):
        """Send gripper close command once and wait for settle time."""
        # ---- 首次进入：下发夹爪指令 ----
        if not self.state_command_sent:
            self._publish_gripper_command('close')
            self.state_command_sent = True
            settle_sec = self.get_parameter('gripper_settle_sec').value
            self.gripper_settle_until = datetime.now() + timedelta(seconds=settle_sec)
            self.get_logger().info(f'夹爪闭合指令已下发，等待 {settle_sec:.1f}s 稳定')
            return

        # ---- 等待稳定 ----
        if self.gripper_settle_until and datetime.now() >= self.gripper_settle_until:
            # 动态计算抬升目标（基于当前末端位置）
            if self.last_end_pose is not None:
                self.lift_target = self.planner.compute_lift(self.last_end_pose)
            else:
                self.lift_target = self.planner.get_safe_pose()
            self.get_logger().info(
                f'夹爪稳定完成，抬升目标: ({self.lift_target[0]:.3f}, '
                f'{self.lift_target[1]:.3f}, {self.lift_target[2]:.3f})')

            self._reset_stage_vars()
            self.gripper_closed = True
            self.current_stage = 'MOVE_LIFT'
            self._transition('MOVE_LIFT')

    # ------------------------------------------------------------------
    # MOVE_LIFT：抬升阶段
    # ------------------------------------------------------------------

    def _handle_move_lift(self):
        """Lift after grasping. One command only."""
        if self.lift_target is None:
            self.get_logger().error('lift_target is None')
            self._transition('ABORT')
            return

        self._handle_cartesian_stage(
            stage_name='MOVE_LIFT',
            next_state='MOVE_RETREAT',
            waypoint_fn=lambda: self.lift_target,
        )

    # ------------------------------------------------------------------
    # MOVE_RETREAT：撤退阶段
    # ------------------------------------------------------------------

    def _handle_move_retreat(self):
        """Retreat to safe pose. One command only."""
        self._handle_cartesian_stage(
            stage_name='MOVE_RETREAT',
            next_state='DONE',
            waypoint_fn=lambda: self.planner.get_safe_pose(),
        )

    # ------------------------------------------------------------------
    # 通用 Cartesian 阶段处理
    # ------------------------------------------------------------------

    def _handle_cartesian_stage(
        self,
        stage_name: str,
        next_state: str,
        waypoint_fn,
        on_settled_fn=None,
    ):
        """Generic handler for Cartesian movement stages.

        Guarantees:
        - Command is published exactly once per stage
        - active_stage_target is set once at entry and never modified
        - Position reached check uses position_tolerance_m
        - After reaching, waits for settle_time_sec
        - Optionally runs a post-settle verification callback

        Args:
            stage_name: Name of the current stage for logging
            next_state: State to transition to after completion
            waypoint_fn: Callable that returns the Cartesian target [x, y, z]
            on_settled_fn: Optional callable after settle, return True to proceed
        """
        if self.last_end_pose is None:
            return

        # ---- 首次进入：计算目标并发送命令 ----
        if not self.state_command_sent:
            waypoint = waypoint_fn()
            if waypoint is None:
                self.get_logger().error(f'{stage_name}: 无法计算路径点')
                self._transition('ABORT')
                return

            # 锁定 active_stage_target，整个阶段不再变动
            self.active_stage_target = list(waypoint)

            self._publish_cart_target(self.active_stage_target)
            self.state_command_sent = True
            self.stage_motion_started = True
            self.last_cmd_time = datetime.now()
            self.get_logger().info(
                f'{stage_name} 启动运动，目标: '
                f'({self.active_stage_target[0]:.3f}, '
                f'{self.active_stage_target[1]:.3f}, '
                f'{self.active_stage_target[2]:.3f})')
            return

        # ---- 运动已开始，等待到位 ----
        pos_tol = self.get_parameter('position_tolerance_m').value
        distance = self._distance_xyz(
            self.last_end_pose[0], self.last_end_pose[1], self.last_end_pose[2],
            self.active_stage_target[0],
            self.active_stage_target[1],
            self.active_stage_target[2],
        )

        # 尚未到位
        if distance > pos_tol:
            if self.settle_start_time is not None:
                # 曾经到位过但又漂移出去了，重置稳定计时
                self.get_logger().debug(
                    f'{stage_name}: drifted out of tolerance ({distance:.4f}m), resetting settle')
                self.settle_start_time = None
            return

        # 已进入容差范围，开始计时稳定
        if self.settle_start_time is None:
            settle_sec = self.get_parameter('settle_time_sec').value
            self.settle_start_time = datetime.now() + timedelta(seconds=settle_sec)
            self.get_logger().info(
                f'{stage_name} 已到位 (dist={distance:.4f}m), '
                f'等待稳定 {settle_sec:.1f}s')
            return

        # 等待稳定时间到达
        if datetime.now() < self.settle_start_time:
            return

        # ---- 到位且稳定：可选的后置验证 ----
        if on_settled_fn is not None:
            if not on_settled_fn():
                self.get_logger().error(f'{stage_name}: post-settle verification failed')
                self._transition('ABORT')
                return

        self.get_logger().info(f'{stage_name} 完成，切换到 {next_state}')

        # ---- 进入下一阶段 ----
        self._reset_stage_vars()
        self.current_stage = next_state if next_state != 'DONE' else None
        self._transition(next_state)

    # ==================================================================
    # 辅助方法
    # ==================================================================

    def _reset_stage_vars(self):
        """Reset per-stage tracking variables."""
        self.state_command_sent = False
        self.stage_motion_started = False
        self.settle_start_time = None
        self.active_stage_target = None
        self._j6_speed_safe = False
        self._j6_settle_start_time = None

    def _transition(self, new_state: str):
        """状态切换并记录日志。"""
        old = self.task_state
        self.task_state = new_state
        if old != new_state:
            self.get_logger().info(f'状态切换: {old} → {new_state}')

    def _reset_task(self):
        """重置任务运行时变量，准备下一次抓取。"""
        self.current_stage = None
        self.frozen_target = None
        self.latest_target = None
        self.active_stage_target = None
        self.last_cmd_target = None
        self.last_cmd_time = None
        self.gripper_closed = False
        self.gripper_settle_until = None
        self.lift_target = None
        self.joint6_compensated = False
        self._stable_frame_count = 0
        self._last_valid_target = None
        self._reset_stage_vars()
        self.filter.history.clear()

    # ------------------------------------------------------------------
    # 指令发布
    # ------------------------------------------------------------------

    def _set_speed_profile(self, profile: str):
        """设置执行层速度档位: 'slow' 或 'default'。"""
        if self.speed_profile_active == profile:
            return
        self.speed_profile_active = profile
        msg = String()
        msg.data = profile
        self.get_logger().info(f'切换速度档位: {profile}')

    def _publish_cart_target(self, xyz: list):
        """发布 Cartesian 目标指令到执行层。"""
        msg = PointStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        self.cart_pub.publish(msg)
        self.last_cmd_target = list(xyz)
        self.get_logger().info(
            f'下发 Cartesian: ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})')

    def _publish_joint_target(self, joint_pos: list):
        """发布关节空间指令（用于 joint6 补偿等）。"""
        msg = Float64MultiArray()
        msg.data = [float(v) for v in joint_pos]
        self.joint_pub.publish(msg)
        self.get_logger().info(f'下发 Joint: {[f"{v:.3f}" for v in joint_pos]}')

    def _publish_gripper_command(self, command: str):
        """发布夹爪指令。"""
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'下发夹爪指令: {command}')

    # ------------------------------------------------------------------
    # 数学工具
    # ------------------------------------------------------------------

    @staticmethod
    def _distance_xyz(x1: float, y1: float, z1: float,
                      x2: float, y2: float, z2: float) -> float:
        """3D Euclidean distance."""
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskOpenLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
