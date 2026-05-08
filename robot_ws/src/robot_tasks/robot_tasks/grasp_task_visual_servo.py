#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""方案二：粗定位 + 眼在手上视觉闭环微调 + 抓取任务节点。

流程：
  1. 接收 VisualTarget，基于 x,y,z 粗定位到 pre_grasp
  2. 到达 pre_grasp 后进入视觉闭环阶段
  3. 根据目标像素坐标 (u,v) 与图像中心 (cx,cy) 的误差做小步修正
  4. 连续 N 帧满足稳定阈值后，才允许下降抓取
  5. 下降 → 闭合夹爪 → 抬升 → 撤退

特点：
  - 方案一负责粗定位，方案二负责精修正
  - 适合作为最终主力方案
  - 步长限幅 + 死区 + 连续稳定帧判断
  - 每一步之后必须重新等待新的 VisualTarget，不允许连续多步
"""

from datetime import datetime, timedelta

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from robot_msgs.msg import ArmJointState, VisualTarget
from robot_tasks.shared.grasp_planner import GraspPlanner
from robot_tasks.shared.servo_utils import (ServoCalculator, ServoConfig,
                                             ServoState)
from robot_tasks.shared.target_filter import TargetFilter


class GraspTaskVisualServo(Node):
    """视觉伺服抓取任务节点 —— 粗定位 + 眼在手上闭环微调。

    本节点只通过 ROS topic 与执行层交互，不直接触碰 AIRBOT SDK。
    """

    def __init__(self):
        super().__init__('grasp_task_visual_servo')

        # ---- 加载参数 ----
        self._declare_parameters()

        # ---- 共享工具 ----
        self.filter = TargetFilter(self._config_dict())
        self.planner = GraspPlanner(self._config_dict())
        self.servo_cfg = ServoConfig(
            gain_k=self.get_parameter('servo_gain_k').value,
            max_step=self.get_parameter('servo_max_step').value,
            dead_zone_u=self.get_parameter('servo_dead_zone_u').value,
            dead_zone_v=self.get_parameter('servo_dead_zone_v').value,
            threshold_u=self.get_parameter('servo_threshold_u').value,
            threshold_v=self.get_parameter('servo_threshold_v').value,
            stable_count_required=self.get_parameter('servo_stable_count_required').value,
            history_size=self.get_parameter('servo_history_size').value,
            z_descend_step=self.get_parameter('servo_z_descend_step').value,
        )
        self.servo_calc = ServoCalculator(self.servo_cfg)
        self.servo_state = ServoState()

        # ---- 状态机 ----
        # IDLE → WAIT_TARGET → PLAN_COARSE_PATH → MOVE_PRE_GRASP
        #   → WAIT_SERVO_TARGET → SERVO_ALIGN → WAIT_AFTER_SERVO_STEP
        #   → FINAL_DESCEND → CLOSE_GRIPPER → MOVE_LIFT → MOVE_RETREAT → DONE
        self.task_state = 'IDLE'
        self.current_stage = None
        self.latest_target: VisualTarget = None
        self.accepted_target: VisualTarget = None  # 粗定位锁定的目标

        # ---- 运行时变量 ----
        self.last_end_pose = None
        self.last_joint_pos = None
        self.last_cmd_target = None
        self.last_cmd_time = None
        self.gripper_closed = False
        self.gripper_settle_until = None
        self.lift_target = None
        self.joint6_compensated = False
        self.servo_step_count = 0            # 当前伺服周期已执行的步数
        self.max_servo_steps = 30            # 伺服最大步数，超时保护

        # ---- 图像参数（可通过参数配置） ----
        self.image_width = 640
        self.image_height = 480

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

        # ---- 主循环 ----
        loop_hz = self.get_parameter('loop_hz').value
        self.timer = self.create_timer(1.0 / loop_hz, self.step_loop)

        self.get_logger().info('GraspTaskVisualServo 启动，状态: IDLE')

    # ==================================================================
    # 参数声明
    # ==================================================================

    def _declare_parameters(self):
        """声明所有可调参数。"""
        # 粗定位参数（共用 GraspPlanner）
        self.declare_parameter('pre_grasp_z_offset', 0.12)
        self.declare_parameter('grasp_z_offset', 0.0)
        self.declare_parameter('lift_z_offset', 0.10)
        # 置信度
        self.declare_parameter('min_confidence_start', 0.7)
        self.declare_parameter('confidence_low', 0.3)
        # 稳定性（粗定位阶段用）
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
        # 粗定位步长
        self.declare_parameter('coarse_step', 0.10)
        # 视觉伺服参数
        self.declare_parameter('servo_gain_k', 0.0003)
        self.declare_parameter('servo_max_step', 0.015)
        self.declare_parameter('servo_dead_zone_u', 5.0)
        self.declare_parameter('servo_dead_zone_v', 5.0)
        self.declare_parameter('servo_threshold_u', 3.0)
        self.declare_parameter('servo_threshold_v', 3.0)
        self.declare_parameter('servo_stable_count_required', 5)
        self.declare_parameter('servo_history_size', 20)
        self.declare_parameter('servo_z_descend_step', 0.03)
        # 下降 / 抬升 / 撤退步长
        self.declare_parameter('descend_step', 0.03)
        self.declare_parameter('lift_step', 0.08)
        self.declare_parameter('retreat_step', 0.10)
        # 到达 / 超时 / 频率
        self.declare_parameter('reach_threshold', 0.03)
        self.declare_parameter('gripper_settle_sec', 1.0)
        self.declare_parameter('cmd_timeout_sec', 10.0)
        self.declare_parameter('loop_hz', 4)

    def _config_dict(self) -> dict:
        """组装共享模块所需配置。"""
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
        """接收视觉目标。

        粗定位阶段：校验工作空间和置信度。
        伺服阶段：还额外需要像素坐标 (u,v) 和图像尺寸。
        """
        frame_id = msg.header.frame_id.strip()
        if frame_id and frame_id != 'base_link':
            self.get_logger().error(
                f'frame_id 不匹配: {frame_id}，期望 base_link')
            return

        if not self.filter.in_workspace(msg.x, msg.y, msg.z):
            self.get_logger().error(
                f'目标超出工作空间: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')
            return

        self.latest_target = msg
        self.filter.update_history(msg)

        if not self.filter.has_any_confidence(msg):
            self.get_logger().warning(
                f'目标置信度过低 ({msg.confidence:.2f})')
            return

        # 更新图像尺寸（如果消息携带了有效值）
        if msg.image_width > 0:
            self.image_width = msg.image_width
        if msg.image_height > 0:
            self.image_height = msg.image_height

        self.get_logger().info(
            f'收到目标: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) '
            f'pixel=({msg.u:.1f}, {msg.v:.1f}) '
            f'confidence={msg.confidence:.2f} stable={msg.is_stable}')

        # 空闲状态 → 开始粗定位规划
        if self.task_state in ('IDLE', 'WAIT_TARGET'):
            self.task_state = 'PLAN_COARSE_PATH'

        # 伺服阶段：收到新帧，触发新一轮对齐评估
        if self.task_state == 'WAIT_SERVO_TARGET':
            self.task_state = 'SERVO_ALIGN'

    def state_callback(self, msg: ArmJointState):
        """存储机械臂状态。"""
        if msg.end_pose and len(msg.end_pose) >= 3:
            self.last_end_pose = [msg.end_pose[0],
                                  msg.end_pose[1],
                                  msg.end_pose[2]]
        if msg.joint_pos and len(msg.joint_pos) >= 6:
            self.last_joint_pos = list(msg.joint_pos)

    # ==================================================================
    # 主状态机
    # ==================================================================

    def step_loop(self):
        """定时器驱动的状态机主循环。"""
        if self.task_state == 'IDLE':
            self._transition('WAIT_TARGET')

        elif self.task_state == 'WAIT_TARGET':
            pass

        elif self.task_state == 'PLAN_COARSE_PATH':
            self._handle_plan_coarse_path()

        elif self.task_state == 'MOVE_PRE_GRASP':
            self._handle_move_pre_grasp()

        elif self.task_state == 'WAIT_SERVO_TARGET':
            self._handle_wait_servo_target()

        elif self.task_state == 'SERVO_ALIGN':
            self._handle_servo_align()

        elif self.task_state == 'WAIT_AFTER_SERVO_STEP':
            self._handle_wait_after_servo_step()

        elif self.task_state == 'FINAL_DESCEND':
            self._handle_final_descend()

        elif self.task_state == 'CLOSE_GRIPPER':
            self._handle_close_gripper()

        elif self.task_state == 'MOVE_LIFT':
            self._handle_move_lift()

        elif self.task_state == 'MOVE_RETREAT':
            self._handle_move_retreat()

        elif self.task_state == 'DONE':
            self.get_logger().info('视觉伺服抓取流程完成')
            self._reset_task()
            self._transition('IDLE')

        elif self.task_state == 'RECOVERY':
            self._handle_recovery()

        elif self.task_state == 'ABORT':
            self.get_logger().error('任务中止')
            self._reset_task()
            self._transition('WAIT_TARGET')

    # ------------------------------------------------------------------
    # 各状态处理
    # ------------------------------------------------------------------

    def _handle_plan_coarse_path(self):
        """PLAN_COARSE_PATH：目标稳定后，规划粗定位路径到 pre_grasp。

        这一步与方案一的开环规划逻辑完全相同，共用 GraspPlanner。
        """
        if self.latest_target is None:
            self._transition('WAIT_TARGET')
            return

        if not self.filter.is_target_stable():
            self.get_logger().info('目标尚未稳定')
            self._transition('WAIT_TARGET')
            return

        if not self.filter.has_min_confidence(self.latest_target):
            self._transition('WAIT_TARGET')
            return

        # 锁定粗定位目标
        self.accepted_target = self.latest_target
        t = self.accepted_target
        pre_grasp = self.planner.compute_pre_grasp([t.x, t.y, t.z])
        self.get_logger().info(
            f'粗定位路径已规划: target=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) '
            f'pre_grasp=({pre_grasp[0]:.3f}, {pre_grasp[1]:.3f}, {pre_grasp[2]:.3f})')

        # joint6 补偿
        if not self.joint6_compensated and self.last_joint_pos is not None:
            j6_target = self.planner.compute_joint6_target(
                self.last_joint_pos)
            if j6_target is not None:
                self._publish_joint_target(j6_target)
                self.joint6_compensated = True
                self.get_logger().info('已下发 joint6 补偿')
                return

        self.current_stage = 'MOVE_PRE_GRASP'
        self._transition('MOVE_PRE_GRASP')

    def _handle_move_pre_grasp(self):
        """MOVE_PRE_GRASP：分步移动到 pre_grasp 点。"""
        if self.last_end_pose is None or self.accepted_target is None:
            return

        t = self.accepted_target
        waypoint = self.planner.compute_pre_grasp([t.x, t.y, t.z])
        reach = self.get_parameter('reach_threshold').value

        if self.planner.reached(self.last_end_pose, waypoint, reach):
            self.get_logger().info('已到达 pre_grasp，进入视觉伺服阶段')
            self.servo_state.reset()
            self.servo_step_count = 0
            self._transition('WAIT_SERVO_TARGET')
            return

        coarse_step = self.get_parameter('coarse_step').value
        step_target = self.planner.limit_step(
            self.last_end_pose, waypoint, coarse_step)
        self._publish_cart_target(step_target)
        self.last_cmd_target = step_target
        self.last_cmd_time = datetime.now()

    def _handle_wait_servo_target(self):
        """WAIT_SERVO_TARGET：等待新的 VisualTarget（含像素坐标）。

        伺服阶段必须逐帧驱动——每收到一帧新图像，才执行一次修正评估。
        """
        if self.latest_target is None:
            return
        # 检查最新帧是否携带有效的像素信息
        if self.latest_target.image_width <= 0 or self.latest_target.image_height <= 0:
            self.get_logger().warning('目标缺少图像尺寸信息，无法伺服')
            return
        # target_callback 会将状态切到 SERVO_ALIGN

    def _handle_servo_align(self):
        """SERVO_ALIGN：根据当前帧的像素误差计算并执行一步微调。

        核心逻辑：
          error_u = u - cx, error_v = v - cy
          基于误差生成 dx, dy
          每次只允许一小步，之后必须等待新的 VisualTarget
        """
        if self.latest_target is None:
            self._transition('WAIT_SERVO_TARGET')
            return

        # 计算像素误差
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0
        error_u, error_v = self.servo_calc.compute_pixel_error(
            self.latest_target.u, self.latest_target.v, cx, cy)

        self.get_logger().info(
            f'像素误差: error_u={error_u:.1f}, error_v={error_v:.1f}')

        # 更新连续稳定计数
        self.servo_calc.update_stability(self.servo_state, error_u, error_v)
        self.get_logger().info(
            f'连续稳定帧: {self.servo_state.consecutive_stable}/'
            f'{self.servo_cfg.stable_count_required}')

        # 检查是否满足下降条件
        if self.servo_calc.should_descend(self.servo_state):
            self.get_logger().info('连续稳定达标，允许最终下降！')
            self.current_stage = 'FINAL_DESCEND'
            self._transition('FINAL_DESCEND')
            return

        # 检查是否在死区（误差极小，本帧不动作）
        if self.servo_calc.is_in_dead_zone(error_u, error_v):
            self.get_logger().info('像素误差在死区内，本帧不修正')
            self._transition('WAIT_SERVO_TARGET')
            return

        # 检查最大步数保护
        self.servo_step_count += 1
        if self.servo_step_count > self.max_servo_steps:
            self.get_logger().error('伺服步数超限，进入恢复')
            self._transition('RECOVERY')
            return

        # 计算修正量并下发
        correction = self.servo_calc.compute_correction(
            error_u, error_v,
            current_depth=self.latest_target.depth)

        if self.last_end_pose is None:
            self._transition('WAIT_SERVO_TARGET')
            return

        # 修正后的目标 = 当前末端位置 + 修正量（仅 XY）
        new_target = [
            self.last_end_pose[0] + correction[0],
            self.last_end_pose[1] + correction[1],
            self.last_end_pose[2],  # Z 保持不变
        ]
        new_target = self.filter.clamp_to_workspace(new_target)

        self.get_logger().info(
            f'伺服修正: correction=({correction[0]:.4f}, {correction[1]:.4f}) '
            f'new_target=({new_target[0]:.4f}, {new_target[1]:.4f}, {new_target[2]:.4f})')

        self._publish_cart_target(new_target)
        self.last_cmd_target = new_target
        self.last_cmd_time = datetime.now()

        # 步后等待：不连续发步，必须等新帧
        self._transition('WAIT_AFTER_SERVO_STEP')

    def _handle_wait_after_servo_step(self):
        """WAIT_AFTER_SERVO_STEP：等待机械臂完成上一步伺服修正。

        确认到达后再切回 WAIT_SERVO_TARGET 等待新帧。
        不在这里直接发下一步，保证"一步一帧"。
        """
        if self.last_cmd_target is None:
            self._transition('WAIT_SERVO_TARGET')
            return

        reach = self.get_parameter('reach_threshold').value
        if self.last_end_pose is not None and self.planner.reached(
                self.last_end_pose, self.last_cmd_target, reach):
            self.get_logger().info('伺服步到达，等待新帧')
            self._transition('WAIT_SERVO_TARGET')
            return

        # 超时保护
        if self.last_cmd_time and datetime.now() - self.last_cmd_time > self.cmd_timeout:
            self.get_logger().warning('伺服步超时未到达，进入恢复')
            self._transition('RECOVERY')
            return

    def _handle_final_descend(self):
        """FINAL_DESCEND：最终直线下降到抓取点。

        基于当前末端 XY，下降到 grasp Z 坐标。
        """
        if self.last_end_pose is None or self.accepted_target is None:
            self._transition('RECOVERY')
            return

        grasp = self.planner.compute_grasp(
            [self.accepted_target.x,
             self.accepted_target.y,
             self.accepted_target.z])

        # 使用当前末端的 XY + 目标 Z，保证垂直下降
        descend_target = [self.last_end_pose[0],
                          self.last_end_pose[1],
                          grasp[2]]
        descend_target = self.filter.clamp_to_workspace(descend_target)

        reach = self.get_parameter('reach_threshold').value
        if self.planner.reached(self.last_end_pose, descend_target, reach):
            self.get_logger().info('已到达抓取点')
            self._transition('CLOSE_GRIPPER')
            return

        descend_step = self.get_parameter('descend_step').value
        step_target = self.planner.limit_step(
            self.last_end_pose, descend_target, descend_step)
        self._publish_cart_target(step_target)
        self.last_cmd_target = step_target
        self.last_cmd_time = datetime.now()

    def _handle_close_gripper(self):
        """CLOSE_GRIPPER：闭合夹爪。"""
        if not self.gripper_closed:
            self._publish_gripper_command('close')
            self.gripper_closed = True
            settle_sec = self.get_parameter('gripper_settle_sec').value
            self.gripper_settle_until = datetime.now() + timedelta(seconds=settle_sec)
            self.get_logger().info('夹爪闭合中')
            return
        if self.gripper_settle_until and datetime.now() >= self.gripper_settle_until:
            self.lift_target = self.planner.compute_lift(self.last_end_pose)
            self.get_logger().info('夹爪稳定，准备抬升')
            self.current_stage = 'MOVE_LIFT'
            self._transition('MOVE_LIFT')

    def _handle_move_lift(self):
        """MOVE_LIFT：抬升。"""
        self._handle_generic_move_stage(
            lambda: self.lift_target, 'MOVE_RETREAT', 'lift_step')

    def _handle_move_retreat(self):
        """MOVE_RETREAT：撤退到安全位姿。"""
        self._handle_generic_move_stage(
            self.planner.get_safe_pose, 'DONE', 'retreat_step')

    def _handle_generic_move_stage(self, waypoint_fn, next_state: str,
                                    step_param: str):
        """通用移动阶段：朝路径点走一步，到达后切换状态。"""
        if self.last_end_pose is None:
            return
        waypoint = waypoint_fn() if callable(waypoint_fn) else waypoint_fn
        if waypoint is None:
            self._transition('RECOVERY')
            return

        reach = self.get_parameter('reach_threshold').value
        if self.planner.reached(self.last_end_pose, waypoint, reach):
            self.get_logger().info(f'已到达，切换 → {next_state}')
            self._transition(next_state)
            return

        step_size = self.get_parameter(step_param).value
        step_target = self.planner.limit_step(
            self.last_end_pose, waypoint, step_size)
        self._publish_cart_target(step_target)
        self.last_cmd_target = step_target
        self.last_cmd_time = datetime.now()

    def _handle_recovery(self):
        """RECOVERY：尝试回到安全位姿。"""
        if self.last_end_pose is None:
            self._transition('ABORT')
            return
        safe = self.planner.get_safe_pose()
        if self.planner.reached(self.last_end_pose, safe, 0.05):
            self._transition('ABORT')
            return
        step = self.planner.limit_step(
            self.last_end_pose, safe,
            self.get_parameter('retreat_step').value)
        self._publish_cart_target(step)

    # ------------------------------------------------------------------
    # 辅助方法
    # ------------------------------------------------------------------

    def _transition(self, new_state: str):
        old = self.task_state
        self.task_state = new_state
        if old != new_state:
            self.get_logger().info(f'状态切换: {old} → {new_state}')

    def _reset_task(self):
        self.current_stage = None
        self.accepted_target = None
        self.last_cmd_target = None
        self.last_cmd_time = None
        self.gripper_closed = False
        self.gripper_settle_until = None
        self.lift_target = None
        self.joint6_compensated = False
        self.servo_step_count = 0
        self.filter.history.clear()
        self.servo_state.reset()

    # ------------------------------------------------------------------
    # 指令发布
    # ------------------------------------------------------------------

    def _publish_cart_target(self, xyz: list):
        msg = PointStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        self.cart_pub.publish(msg)
        self.get_logger().info(
            f'Cartesian: ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})')

    def _publish_joint_target(self, joint_pos: list):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in joint_pos]
        self.joint_pub.publish(msg)

    def _publish_gripper_command(self, command: str):
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'夹爪: {command}')


def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskVisualServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
