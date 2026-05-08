#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动抓取任务节点（对接 robot_ws 执行层）。

从 camera_to_base_transform 接收 VisualTarget，通过统一执行层完成
"预抓取 → joint6 旋转 → 下降 → 闭合夹爪 → 抬升 → 回 home"。

重要变更（相比旧版）：
  - 不再直接连接 AIRBOT SDK，所有指令通过 /robot_arm/* topic 下发
  - 订阅 /visual_target_base (VisualTarget) 替代多物体分散 topic
  - 订阅 /robot_arm/joint_state (ArmJointState) 做末端位姿反馈
  - 物体专属参数保留（duck/apple/box 各自 offset 和夹爪闭合值）
"""

import math
import threading
from collections import deque
from datetime import datetime, timedelta
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from robot_msgs.msg import ArmJointState, VisualTarget


# ===================================================================
# 物体参数（从旧版 auto_pick_from_base 继承，已调优）
# ===================================================================
OBJECT_CONFIG = {
    'apple': {
        'gripper_close': 0.0,
        'grasp_z_offset': 0.00,
        'grasp_x_offset': 0.00,
        'grasp_y_offset': 0.00,
    },
    'duck': {
        'gripper_close': 0.02,
        'grasp_z_offset': -0.03,
        'grasp_x_offset': 0.00,
        'grasp_y_offset': 0.00,
    },
    'box': {
        'gripper_close': 0.03,
        'grasp_z_offset': -0.02,
        'grasp_x_offset': 0.00,
        'grasp_y_offset': 0.00,
    },
}


class GraspState(Enum):
    IDLE = 'IDLE'
    WAIT_TARGET = 'WAIT_TARGET'
    OPEN_GRIPPER = 'OPEN_GRIPPER'
    MOVE_PRE_GRASP = 'MOVE_PRE_GRASP'
    ROTATE_JOINT6 = 'ROTATE_JOINT6'
    DESCEND = 'DESCEND'
    CLOSE_GRIPPER = 'CLOSE_GRIPPER'
    LIFT = 'LIFT'
    RETURN_HOME = 'RETURN_HOME'
    DONE = 'DONE'
    ABORT = 'ABORT'


class AutoPickFromBase(Node):
    """自动抓取节点 —— 订阅 VisualTarget，通过执行层完成抓取。"""

    def __init__(self):
        super().__init__('auto_pick_from_base')

        # ---- 通用抓取参数 ----
        self.declare_parameter('pregrasp_z_offset', 0.08)
        self.declare_parameter('lift_z_offset', 0.10)
        self.declare_parameter('joint6_clockwise_delta_deg', -90.0)
        self.declare_parameter('gripper_open', 1.0)

        # ---- 工作空间 ----
        self.declare_parameter('workspace.x_min', 0.10)
        self.declare_parameter('workspace.x_max', 1.00)
        self.declare_parameter('workspace.y_min', -0.45)
        self.declare_parameter('workspace.y_max', 0.50)
        self.declare_parameter('workspace.z_min', 0.02)
        self.declare_parameter('workspace.z_max', 0.70)

        # ---- 目标滤波 ----
        self.declare_parameter('target_window_size', 8)
        self.declare_parameter('max_target_std', 0.01)

        # ---- 步长与阈值 ----
        self.declare_parameter('reach_threshold', 0.03)
        self.declare_parameter('motion_settle_sec', 0.5)
        self.declare_parameter('gripper_settle_sec', 0.8)
        self.declare_parameter('cmd_timeout_sec', 15.0)
        self.declare_parameter('loop_hz', 4)

        # ---- 运行时变量 ----
        self.state = GraspState.IDLE
        self.latest_target: VisualTarget = None
        self.accepted_target: VisualTarget = None
        self.last_end_pose = None       # [x, y, z]
        self.last_joint_pos = None      # [j1..j6]
        self.last_cmd_target = None
        self.last_cmd_time = None
        self.done_once = False

        # 目标滤波缓冲
        self.target_buffer = deque(
            maxlen=self.get_parameter('target_window_size').value)
        self.current_object_name = None

        # 各阶段路径点（在收到目标时计算）
        self._pregrasp = None
        self._grasp = None
        self._mid = None
        self._lift = None
        self._home_joint = None
        self._rotated_quat = None       # joint6 旋转后的末端姿态（保留供后续用）

        # 夹爪
        self._gripper_close_value = 0.0
        self._gripper_cmd_sent = False
        self._gripper_settle_until = None

        # ---- 订阅 ----
        self.target_sub = self.create_subscription(
            VisualTarget,
            '/visual_target_base',
            self.target_callback,
            10
        )
        self.state_sub = self.create_subscription(
            ArmJointState,
            '/robot_arm/joint_state',
            self.state_callback,
            10
        )

        # ---- 发布（执行层接口） ----
        self.cart_pub = self.create_publisher(
            PointStamped, '/robot_arm/cart_target', 10)
        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/robot_arm/target_joint', 10)
        self.gripper_pub = self.create_publisher(
            String, '/robot_arm/gripper_cmd', 10)

        # ---- 主循环 ----
        loop_hz = self.get_parameter('loop_hz').value
        self.timer = self.create_timer(1.0 / loop_hz, self.step_loop)

        self.get_logger().info('AutoPickFromBase 启动（执行层模式）。')
        self.get_logger().info('监听: /visual_target_base, /robot_arm/joint_state')
        self.get_logger().info('发布: /robot_arm/cart_target, /robot_arm/target_joint, /robot_arm/gripper_cmd')

    # ==================================================================
    # 回调
    # ==================================================================

    def target_callback(self, msg: VisualTarget):
        """接收 VisualTarget，滤波后触发抓取。"""
        if self.done_once or self.state != GraspState.IDLE:
            return

        # 工作空间检查
        if not self._in_workspace(msg.x, msg.y, msg.z):
            self.get_logger().warning(
                f'目标超出工作空间: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')
            return

        # 物体切换时清空缓冲
        if self.current_object_name != msg.object_name:
            self.current_object_name = msg.object_name
            self.target_buffer.clear()
            self.get_logger().info(f'切换目标物体: {msg.object_name}')

        self.target_buffer.append([msg.x, msg.y, msg.z])
        self.latest_target = msg

        window = self.get_parameter('target_window_size').value
        if len(self.target_buffer) < window:
            self.get_logger().info(
                f'目标积累中: {len(self.target_buffer)}/{window}')
            return

        # 滤波：计算均值和标准差
        pts = np.array(self.target_buffer, dtype=np.float64)
        mean_pt = pts.mean(axis=0)
        std_norm = float(np.linalg.norm(pts.std(axis=0)))
        max_std = self.get_parameter('max_target_std').value

        self.get_logger().info(
            f'{msg.object_name} 滤波: mean=({mean_pt[0]:.3f}, {mean_pt[1]:.3f}, {mean_pt[2]:.3f}), '
            f'std_norm={std_norm:.4f}')

        if std_norm > max_std:
            self.get_logger().info('目标还不够稳定，继续等待。')
            return

        # 目标稳定，锁定并开始抓取
        self.accepted_target = msg
        self._compute_waypoints(msg)
        self.state = GraspState.OPEN_GRIPPER
        self.get_logger().info(f'目标已锁定，开始抓取 {msg.object_name}')

    def state_callback(self, msg: ArmJointState):
        """存储末端位姿和关节角。"""
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
        """定时器驱动的主循环。"""
        if self.state == GraspState.IDLE:
            pass  # 等待 target_callback 触发

        elif self.state == GraspState.OPEN_GRIPPER:
            self._handle_open_gripper()

        elif self.state == GraspState.MOVE_PRE_GRASP:
            self._handle_move_to('MOVE_PRE_GRASP', self._pregrasp,
                                 GraspState.ROTATE_JOINT6)

        elif self.state == GraspState.ROTATE_JOINT6:
            self._handle_rotate_joint6()

        elif self.state == GraspState.DESCEND:
            self._handle_descend()

        elif self.state == GraspState.CLOSE_GRIPPER:
            self._handle_close_gripper()

        elif self.state == GraspState.LIFT:
            self._handle_move_to('LIFT', self._lift, GraspState.RETURN_HOME)

        elif self.state == GraspState.RETURN_HOME:
            self._handle_return_home()

        elif self.state == GraspState.DONE:
            self.get_logger().info('抓取流程完成！')
            self.done_once = True
            self.state = GraspState.IDLE

        elif self.state == GraspState.ABORT:
            self.get_logger().error('抓取中止。')
            self._reset()
            self.state = GraspState.IDLE

    # ------------------------------------------------------------------
    # 各阶段处理
    # ------------------------------------------------------------------

    def _handle_open_gripper(self):
        """张开夹爪。"""
        if not self._gripper_cmd_sent:
            open_val = self.get_parameter('gripper_open').value
            self.get_logger().info(f'Step: 张开夹爪 (target={open_val})')
            self._publish_gripper_command('open')
            self._gripper_cmd_sent = True
            settle = self.get_parameter('gripper_settle_sec').value
            self._gripper_settle_until = datetime.now() + timedelta(seconds=settle)
            return

        if self._gripper_settle_until and datetime.now() >= self._gripper_settle_until:
            self._gripper_cmd_sent = False
            self.state = GraspState.MOVE_PRE_GRASP

    def _handle_move_to(self, stage_name: str, waypoint: list,
                         next_state: GraspState):
        """通用移动阶段：朝 waypoint 步进，到达后切到 next_state。"""
        if self.last_end_pose is None or waypoint is None:
            return

        reach = self.get_parameter('reach_threshold').value
        if self._reached(waypoint, reach):
            self.get_logger().info(f'{stage_name} 已到达')
            self.state = next_state
            return

        self._step_toward(waypoint)

    def _handle_rotate_joint6(self):
        """顺时针旋转 joint6（-90°）。"""
        if self.last_joint_pos is None:
            return

        if not self._gripper_cmd_sent:
            target_joint = self._compute_joint6_clockwise()
            if target_joint is None:
                self.get_logger().error('joint6 旋转角度超限，中止')
                self.state = GraspState.ABORT
                return

            self.get_logger().info(
                f'Step: joint6 顺时针旋转 '
                f'({math.degrees(self.last_joint_pos[5]):.1f}° → '
                f'{math.degrees(target_joint[5]):.1f}°)')
            self._publish_joint_target(target_joint)
            self._gripper_cmd_sent = True
            self.last_cmd_time = datetime.now()
            return

        # 等待到位
        if self._gripper_cmd_sent:
            settle = self.get_parameter('motion_settle_sec').value
            if (self.last_cmd_time and
                    datetime.now() - self.last_cmd_time > timedelta(seconds=settle + 1.0)):
                self._gripper_cmd_sent = False
                self.get_logger().info('joint6 旋转完成')
                self.state = GraspState.DESCEND

    def _handle_descend(self):
        """直线下降：mid → grasp。"""
        if self.last_end_pose is None or self._grasp is None:
            return

        # 先到 mid，再到 grasp
        if self._mid and not self._reached(self._mid, self.get_parameter('reach_threshold').value):
            self._step_toward(self._mid)
            return

        reach = self.get_parameter('reach_threshold').value
        if self._reached(self._grasp, reach):
            self.get_logger().info('DESCEND 已到达抓取点')
            self.state = GraspState.CLOSE_GRIPPER
            return

        self._step_toward(self._grasp)

    def _handle_close_gripper(self):
        """闭合夹爪。"""
        if not self._gripper_cmd_sent:
            self.get_logger().info(
                f'Step: 闭合夹爪 (target={self._gripper_close_value})')
            self._publish_gripper_command('close')
            self._gripper_cmd_sent = True
            settle = self.get_parameter('gripper_settle_sec').value
            self._gripper_settle_until = datetime.now() + timedelta(seconds=settle)
            return

        if self._gripper_settle_until and datetime.now() >= self._gripper_settle_until:
            self._gripper_cmd_sent = False
            self.state = GraspState.LIFT

    def _handle_return_home(self):
        """回到初始关节位姿。"""
        if self._home_joint is None or self.last_joint_pos is None:
            self.get_logger().warning('无 home 关节角记录，跳过回 home')
            self.state = GraspState.DONE
            return

        if not self._gripper_cmd_sent:
            self.get_logger().info('Step: 回到初始关节位姿')
            self._publish_joint_target(self._home_joint)
            self._gripper_cmd_sent = True
            self.last_cmd_time = datetime.now()
            return

        # 等待到位
        settle = self.get_parameter('motion_settle_sec').value
        if (self.last_cmd_time and
                datetime.now() - self.last_cmd_time > timedelta(seconds=settle + 2.0)):
            self._gripper_cmd_sent = False
            self.get_logger().info('已回到 home')
            self.state = GraspState.DONE

    # ==================================================================
    # 辅助方法
    # ==================================================================

    def _in_workspace(self, x, y, z) -> bool:
        return (
            self.get_parameter('workspace.x_min').value <= x <= self.get_parameter('workspace.x_max').value and
            self.get_parameter('workspace.y_min').value <= y <= self.get_parameter('workspace.y_max').value and
            self.get_parameter('workspace.z_min').value <= z <= self.get_parameter('workspace.z_max').value
        )

    def _compute_waypoints(self, target: VisualTarget):
        """根据物体类型计算所有路径点。"""
        obj = target.object_name
        cfg = OBJECT_CONFIG.get(obj, OBJECT_CONFIG['duck'])

        pregrasp_z = self.get_parameter('pregrasp_z_offset').value
        lift_z = self.get_parameter('lift_z_offset').value

        bx, by, bz = target.x, target.y, target.z
        gx = bx + cfg['grasp_x_offset']
        gy = by + cfg['grasp_y_offset']
        gz = bz + cfg['grasp_z_offset']

        self._pregrasp = [gx, gy, bz + pregrasp_z]
        self._grasp = [gx, gy, gz]
        self._mid = [gx, gy, (self._pregrasp[2] + self._grasp[2]) * 0.5]
        self._lift = [gx, gy, bz + lift_z]
        self._gripper_close_value = cfg['gripper_close']

        # 记录当前关节角作为 home
        if self.last_joint_pos and self._home_joint is None:
            self._home_joint = list(self.last_joint_pos)
            self.get_logger().info(
                f'记录 home 关节角: {[f"{math.degrees(v):.1f}°" for v in self._home_joint]}')

        self.get_logger().info(
            f'{obj} 路径点:\n'
            f'  pre_grasp=({self._pregrasp[0]:.3f}, {self._pregrasp[1]:.3f}, {self._pregrasp[2]:.3f})\n'
            f'  mid=({self._mid[0]:.3f}, {self._mid[1]:.3f}, {self._mid[2]:.3f})\n'
            f'  grasp=({self._grasp[0]:.3f}, {self._grasp[1]:.3f}, {self._grasp[2]:.3f})\n'
            f'  lift=({self._lift[0]:.3f}, {self._lift[1]:.3f}, {self._lift[2]:.3f})\n'
            f'  gripper_close={self._gripper_close_value}')

    def _compute_joint6_clockwise(self):
        """计算 joint6 顺时针旋转目标，带限位保护。"""
        if self.last_joint_pos is None:
            return None

        delta_deg = self.get_parameter('joint6_clockwise_delta_deg').value
        j6_deg = math.degrees(self.last_joint_pos[5])
        target_deg = j6_deg + delta_deg

        j6_min = -170.0
        j6_max = 170.0
        margin = 12.0

        if j6_min + margin <= target_deg <= j6_max - margin:
            target = list(self.last_joint_pos)
            target[5] = math.radians(target_deg)
            return target
        if j6_min <= target_deg <= j6_max:
            target = list(self.last_joint_pos)
            target[5] = math.radians(target_deg)
            self.get_logger().warning('joint6 在硬限位边缘，请留意')
            return target

        self.get_logger().error(
            f'joint6 顺时针 -90° 超限: current={j6_deg:.1f}°, '
            f'target={target_deg:.1f}°')
        return None

    def _step_toward(self, waypoint: list):
        """朝 waypoint 发一小步。"""
        if self.last_end_pose is None:
            return

        d = self._distance(self.last_end_pose, waypoint)
        step = min(d, 0.08)  # 单步最大 8 cm
        ratio = step / d if d > 1e-6 else 0.0
        target = [
            self.last_end_pose[0] + (waypoint[0] - self.last_end_pose[0]) * ratio,
            self.last_end_pose[1] + (waypoint[1] - self.last_end_pose[1]) * ratio,
            self.last_end_pose[2] + (waypoint[2] - self.last_end_pose[2]) * ratio,
        ]
        self._publish_cart_target(target)
        self.last_cmd_target = target
        self.last_cmd_time = datetime.now()

    def _reached(self, waypoint: list, threshold: float) -> bool:
        if self.last_end_pose is None:
            return False
        return self._distance(self.last_end_pose, waypoint) < threshold

    def _distance(self, a: list, b: list) -> float:
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def _reset(self):
        self.accepted_target = None
        self._pregrasp = None
        self._grasp = None
        self._mid = None
        self._lift = None
        self._rotated_quat = None
        self._gripper_close_value = 0.0
        self._gripper_cmd_sent = False
        self._gripper_settle_until = None
        self.last_cmd_target = None
        self.last_cmd_time = None

    # ==================================================================
    # 话题发布
    # ==================================================================

    def _publish_cart_target(self, xyz: list):
        msg = PointStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        self.cart_pub.publish(msg)
        self.get_logger().info(
            f'→ cart: ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})')

    def _publish_joint_target(self, joint_pos: list):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in joint_pos]
        self.joint_pub.publish(msg)

    def _publish_gripper_command(self, command: str):
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoPickFromBase()
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
