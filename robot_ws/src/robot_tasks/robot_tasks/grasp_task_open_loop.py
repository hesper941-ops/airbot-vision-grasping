#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""方案一：开环抓取任务节点。

流程：
  1. 等待 VisualTarget（一次识别 + 坐标转换，已转到 base_link）
  2. 置信度 / 工作空间检查
  3. 生成阶段路径点：safe → pre_grasp → grasp → lift
  4. 分阶段下发 Cartesian 指令到执行层
  5. 通过 /robot_arm/state 确认每步完成
  6. 抓取完成后返回 safe 并进入 DONE

特点：
  - 最简单、最容易先跑通
  - 适合作为保底方案
  - 不直接冲最终抓取点，必须分阶段执行
  - 抓取前 joint6 有固定 90° 补偿
"""

from datetime import datetime, timedelta

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
        # IDLE → WAIT_TARGET → PLAN_OPEN_LOOP → MOVE_PRE_GRASP → MOVE_DESCEND
        #   → CLOSE_GRIPPER → MOVE_LIFT → MOVE_RETREAT → DONE → IDLE
        self.task_state = 'IDLE'
        self.current_stage = None          # 当前执行的阶段名
        self.latest_target: VisualTarget = None  # 最新接收到的目标
        self.accepted_target: VisualTarget = None  # 被接受用于本次抓取的目标

        # ---- 运行时变量 ----
        self.last_end_pose = None           # [x, y, z]
        self.last_joint_pos = None          # [j1..j6]
        self.last_cmd_target = None         # 最后一次下发的 Cartesian 目标
        self.last_cmd_time = None           # 最后一次下发时间
        self.gripper_closed = False
        self.gripper_settle_until = None    # 夹爪闭合后等待的截止时间
        self.lift_target = None             # 抬升目标（抓取后动态计算）
        self.joint6_compensated = False     # joint6 补偿是否已执行

        # ---- 命令串行化机制 ----
        self.command_in_flight = False      # 当前是否有命令还在执行
        self.stage_motion_started = False   # 当前阶段是否已启动运动
        self.stage_settled = False          # 当前阶段是否已稳定到位
        self.stage_settle_until = None      # 当前阶段稳定停留的截止时间
        self.speed_profile_active = 'default'  # 当前速度档位

        # ---- 超时 ----
        self.cmd_timeout = timedelta(seconds=self.get_parameter(
            'cmd_timeout_sec').value)

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
        # 路径点参数
        self.declare_parameter('pre_grasp_z_offset', 0.12)
        self.declare_parameter('grasp_z_offset', 0.0)
        self.declare_parameter('lift_z_offset', 0.10)
        # 置信度
        self.declare_parameter('min_confidence_start', 0.7)
        self.declare_parameter('confidence_low', 0.3)
        # 稳定性
        self.declare_parameter('stable_count_required', 3)
        self.declare_parameter('drift_threshold', 0.05)
        # 工作空间
        self.declare_parameter('workspace_limits.x_min', 0.10)
        self.declare_parameter('workspace_limits.x_max', 1.00)
        self.declare_parameter('workspace_limits.y_min', -0.45)
        self.declare_parameter('workspace_limits.y_max', 0.50)
        self.declare_parameter('workspace_limits.z_min', 0.02)
        self.declare_parameter('workspace_limits.z_max', 0.70)
        # 安全位姿
        self.declare_parameter('safe_pose', [0.35, 0.0, 0.35])
        # joint6 补偿
        self.declare_parameter('joint6_compensation_deg', 90.0)
        # 步长控制
        self.declare_parameter('pre_grasp_step', 0.10)
        self.declare_parameter('descend_step', 0.05)
        self.declare_parameter('lift_step', 0.08)
        self.declare_parameter('retreat_step', 0.10)
        # 到达判定阈值
        self.declare_parameter('reach_threshold', 0.03)
        # 夹爪
        self.declare_parameter('gripper_settle_sec', 1.0)
        # 超时与频率
        self.declare_parameter('cmd_timeout_sec', 10.0)
        self.declare_parameter('loop_hz', 4)
        # ---- 命令串行化与到位判定 ----
        self.declare_parameter('position_tolerance_m', 0.02)
        self.declare_parameter('min_command_interval_sec', 1.0)
        self.declare_parameter('settle_time_sec', 0.5)

    def _config_dict(self) -> dict:
        """将 ROS 参数组装为共享模块所需的 dict 格式。"""
        return {
            'pre_grasp_z_offset': self.get_parameter('pre_grasp_z_offset').value,
            'grasp_z_offset': self.get_parameter('grasp_z_offset').value,
            'lift_z_offset': self.get_parameter('lift_z_offset').value,
            'min_confidence_start': self.get_parameter('min_confidence_start').value,
            'confidence_low': self.get_parameter('confidence_low').value,
            'stable_count_required': self.get_parameter('stable_count_required').value,
            'drift_threshold': self.get_parameter('drift_threshold').value,
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
        """Receive visual target, validate it, and update the task state."""
        try:
            frame_id = msg.header.frame_id.strip()
            if frame_id and frame_id != 'base_link':
                self.get_logger().error(
                    f'frame_id mismatch: {frame_id}, expected base_link')
                return

            if not self.filter.in_workspace(msg.x, msg.y, msg.z):
                self.get_logger().error(
                    f'Target out of workspace: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')
                return

            self.latest_target = msg
            self.filter.update_history(msg)

            if not self.filter.has_any_confidence(msg):
                self.get_logger().warning(
                    f'Target confidence too low ({msg.confidence:.2f}), waiting for better input')
                return

            self.get_logger().info(
                f'Received target: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) '
                f'confidence={msg.confidence:.2f} stable={msg.is_stable}')

            if self.task_state in ('IDLE', 'WAIT_TARGET'):
                self._transition('PLAN_OPEN_LOOP')
            else:
                self.get_logger().info(
                    f'目标接收于忙碌状态 {self.task_state}，仅缓存不转换')
        except Exception as e:
            self.get_logger().error(f'target_callback exception: {e}', exc_info=True)

    def state_callback(self, msg: ArmJointState):
        """Store the latest arm end pose and joint positions."""
        try:
            if msg.end_pose and len(msg.end_pose) >= 3:
                self.last_end_pose = [msg.end_pose[0],
                                      msg.end_pose[1],
                                      msg.end_pose[2]]
            if msg.joint_pos and len(msg.joint_pos) >= 6:
                self.last_joint_pos = list(msg.joint_pos)
        except Exception as e:
            self.get_logger().error(f'state_callback exception: {e}', exc_info=True)

    # ==================================================================
    # 主状态机
    # ==================================================================

    def step_loop(self):
        """Timer-driven state machine main loop."""
        try:
            if self.task_state == 'IDLE':
                self._transition('WAIT_TARGET')

            elif self.task_state == 'WAIT_TARGET':
                pass  # wait for target_callback to trigger

            elif self.task_state == 'PLAN_OPEN_LOOP':
                self._handle_plan_open_loop()

            elif self.task_state == 'MOVE_PRE_GRASP':
                self._handle_move_stage('MOVE_DESCEND')

            elif self.task_state == 'MOVE_DESCEND':
                self._handle_move_stage('CLOSE_GRIPPER')

            elif self.task_state == 'CLOSE_GRIPPER':
                self._handle_close_gripper()

            elif self.task_state == 'MOVE_LIFT':
                self._handle_move_stage('MOVE_RETREAT')

            elif self.task_state == 'MOVE_RETREAT':
                self._handle_move_stage('DONE')

            elif self.task_state == 'DONE':
                self.get_logger().info('抓取流程完成，返回 IDLE')
                self._set_speed_profile('default')
                self._reset_task()
                self._transition('IDLE')

            elif self.task_state == 'RECOVERY':
                self._handle_recovery()

            elif self.task_state == 'ABORT':
                self.get_logger().error('任务中止，返回 WAIT_TARGET')
                self._reset_task()
                self._transition('WAIT_TARGET')
        except Exception as e:
            self.get_logger().error(f'step_loop exception: {e}', exc_info=True)
            self._reset_task()
            self._transition('WAIT_TARGET')

    # ------------------------------------------------------------------
    # 各状态处理
    # ------------------------------------------------------------------

    def _handle_plan_open_loop(self):
        """PLAN_OPEN_LOOP：确认目标稳定后规划路径点，开始执行。"""
        if self.latest_target is None:
            self.get_logger().warning('无可用目标，退回等待')
            self._transition('WAIT_TARGET')
            return

        # 检查目标稳定性
        if not self.filter.is_target_stable():
            self.get_logger().info('目标尚不稳定，继续等待')
            self._transition('WAIT_TARGET')
            return

        # 检查启动置信度
        if not self.filter.has_min_confidence(self.latest_target):
            self.get_logger().warning('目标置信度不满足启动条件')
            self._transition('WAIT_TARGET')
            return

        # 锁定本次抓取目标
        self.accepted_target = self.latest_target
        target_xyz = [self.accepted_target.x,
                      self.accepted_target.y,
                      self.accepted_target.z]

        # 预计算关键路径点（日志输出用）
        pre_grasp = self.planner.compute_pre_grasp(target_xyz)
        grasp = self.planner.compute_grasp(target_xyz)
        safe = self.planner.get_safe_pose()

        self.get_logger().info(
            f'开环路径已规划:\n'
            f'  target     = ({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f})\n'
            f'  pre_grasp  = ({pre_grasp[0]:.3f}, {pre_grasp[1]:.3f}, {pre_grasp[2]:.3f})\n'
            f'  grasp      = ({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f})\n'
            f'  safe       = ({safe[0]:.3f}, {safe[1]:.3f}, {safe[2]:.3f})')

        # 先执行 joint6 补偿（如果尚未执行）
        if not self.joint6_compensated and self.last_joint_pos is not None:
            j6_target = self.planner.compute_joint6_target(
                self.last_joint_pos)
            if j6_target is not None:
                self._publish_joint_target(j6_target)
                self.joint6_compensated = True
                self.get_logger().info('已下发 joint6 补偿指令')
                # joint6 补偿后不立即走 Cartesian，等下一周期确认
                return

        # 切换到慢速档位
        self._set_speed_profile('slow')
        
        self.current_stage = 'MOVE_PRE_GRASP'
        self.stage_motion_started = False
        self.stage_settled = False
        self._transition('MOVE_PRE_GRASP')

    def _handle_move_stage(self, next_state: str):
        """处理移动阶段：首次发送命令，然后等待到位+稳定。
        
        核心逻辑：
        1. 进入阶段时发送一次命令（stage_motion_started = False）
        2. 检查末端位置是否在 position_tolerance 内
        3. 到位后等待 settle_time_sec
        4. 稳定后切换到下一个阶段
        """
        if self.last_end_pose is None:
            return

        # 获取当前阶段的目标路径点
        waypoint = self._current_waypoint()
        if waypoint is None:
            self.get_logger().error('无法获取当前阶段路径点，中止')
            self._transition('ABORT')
            return

        # 如果还没有启动这个阶段的运动，则发送初始命令
        if not self.stage_motion_started:
            self._publish_cart_target(waypoint)
            self.stage_motion_started = True
            self.last_cmd_time = datetime.now()
            self.get_logger().info(
                f'{self.current_stage} 启动运动，目标: '
                f'({waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f})')
            return

        # 检查是否已到位（使用 position_tolerance）
        pos_tol = self.get_parameter('position_tolerance_m').value
        dx = self.last_end_pose[0] - waypoint[0]
        dy = self.last_end_pose[1] - waypoint[1]
        dz = self.last_end_pose[2] - waypoint[2]
        distance = (dx * dx + dy * dy + dz * dz) ** 0.5
        
        if not self.stage_settled and distance > pos_tol:
            # 未到位，周期性检查，但不继续发送命令（等执行层反馈）
            self.get_logger().debug(
                f'{self.current_stage} 运动中... 距离: {distance:.4f} m')
            return

        # 已到位，进入稳定阶段
        if not self.stage_settled:
            self.stage_settled = True
            settle_sec = self.get_parameter('settle_time_sec').value
            self.stage_settle_until = datetime.now() + timedelta(seconds=settle_sec)
            self.get_logger().info(
                f'{self.current_stage} 已到位，等待稳定 {settle_sec:.1f} 秒')
            return

        # 检查稳定停留时间
        if self.stage_settle_until and datetime.now() >= self.stage_settle_until:
            self.get_logger().info(
                f'{self.current_stage} 完成，切换到 {next_state}')
            
            # 重置阶段状态
            self.stage_motion_started = False
            self.stage_settled = False
            self.stage_settle_until = None
            
            self._transition(next_state)
            
            # 为下一阶段更新 current_stage
            if next_state != 'DONE':
                stage_map = {
                    'MOVE_DESCEND': 'MOVE_DESCEND',
                    'CLOSE_GRIPPER': 'CLOSE_GRIPPER',
                    'MOVE_LIFT': 'MOVE_LIFT',
                    'MOVE_RETREAT': 'MOVE_RETREAT',
                }
                self.current_stage = stage_map.get(next_state, self.current_stage)

    def _handle_close_gripper(self):
        """CLOSE_GRIPPER：闭合夹爪并等待稳定。"""
        if not self.gripper_closed:
            self._publish_gripper_command('close')
            self.gripper_closed = True
            settle_sec = self.get_parameter('gripper_settle_sec').value
            self.gripper_settle_until = datetime.now() + timedelta(seconds=settle_sec)
            self.get_logger().info('夹爪闭合指令已下发，等待稳定')
            return

        if self.gripper_settle_until and datetime.now() >= self.gripper_settle_until:
            # 动态计算抬升目标（基于当前末端位置）
            self.lift_target = self.planner.compute_lift(self.last_end_pose)
            self.get_logger().info(
                f'夹爪稳定完成，抬升目标: ({self.lift_target[0]:.3f}, '
                f'{self.lift_target[1]:.3f}, {self.lift_target[2]:.3f})')
            self.current_stage = 'MOVE_LIFT'
            self.stage_motion_started = False
            self.stage_settled = False
            self._transition('MOVE_LIFT')

    def _handle_recovery(self):
        """RECOVERY：尝试回到安全位姿。"""
        if self.last_end_pose is None:
            self._transition('ABORT')
            return
        safe = self.planner.get_safe_pose()
        if self.planner.reached(self.last_end_pose, safe, 0.05):
            self.get_logger().info('已回到安全位姿')
            self._transition('ABORT')
            return
        step = self.planner.limit_step(
            self.last_end_pose, safe,
            self.get_parameter('retreat_step').value)
        self._publish_cart_target(step)

    # ------------------------------------------------------------------
    # 辅助方法
    # ------------------------------------------------------------------

    def _current_waypoint(self) -> list:
        """根据当前阶段返回对应路径点。"""
        if self.accepted_target is None:
            return None
        target_xyz = [self.accepted_target.x,
                      self.accepted_target.y,
                      self.accepted_target.z]

        if self.current_stage == 'MOVE_PRE_GRASP':
            return self.planner.compute_pre_grasp(target_xyz)
        elif self.current_stage == 'MOVE_DESCEND':
            # 从 pre_grasp 线性下降到 grasp
            return self.planner.compute_grasp(target_xyz)
        elif self.current_stage == 'MOVE_LIFT':
            return self.lift_target or self.planner.compute_lift(
                self.last_end_pose)
        elif self.current_stage == 'MOVE_RETREAT':
            return self.planner.get_safe_pose()
        return None

    def _stage_step_size(self) -> float:
        """返回当前阶段的单步最大步长。"""
        mapping = {
            'MOVE_PRE_GRASP': 'pre_grasp_step',
            'MOVE_DESCEND': 'descend_step',
            'MOVE_LIFT': 'lift_step',
            'MOVE_RETREAT': 'retreat_step',
        }
        key = mapping.get(self.current_stage, 'pre_grasp_step')
        return self.get_parameter(key).value

    def _transition(self, new_state: str):
        """状态切换并记录日志。"""
        old = self.task_state
        self.task_state = new_state
        if old != new_state:
            self.get_logger().info(f'状态切换: {old} → {new_state}')

    def _reset_task(self):
        """重置任务运行时变量，准备下一次抓取。"""
        self.current_stage = None
        self.accepted_target = None
        self.latest_target = None
        self.last_cmd_target = None
        self.last_cmd_time = None
        self.gripper_closed = False
        self.gripper_settle_until = None
        self.lift_target = None
        self.joint6_compensated = False
        self.command_in_flight = False
        self.stage_motion_started = False
        self.stage_settled = False
        self.stage_settle_until = None
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
        # 发布到一个假想的速度控制 topic（实际可能需要适配）
        # 如果没有独立的速度 topic，可以通过夹爪或其他 topic 携带
        # 这里先留作示例，实际集成时根据 executor 接口适配

    def _publish_cart_target(self, xyz: list):
        """发布 Cartesian 步进指令到执行层。"""
        msg = PointStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        self.cart_pub.publish(msg)
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
