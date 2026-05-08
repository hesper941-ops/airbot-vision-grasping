#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机坐标系 → base_link 坐标系变换节点（对接 robot_ws 新消息体系）。

订阅：
  /robot_arm/end_pose   (PoseStamped)   ← arm_executor_node 发布
  /duck_position         (PointStamped)  ← 检测器（相机系）
  /apple_position        (PointStamped)
  /box_position          (PointStamped)

发布：
  /visual_target_base    (VisualTarget)  ← 给 grasp_task_* 任务节点使用
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped

from robot_msgs.msg import VisualTarget


def quat_xyzw_to_rot(qx, qy, qz, qw):
    """四元数 (x,y,z,w) → 旋转矩阵 3x3。"""
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Quaternion norm is zero.")
    q /= n
    x, y, z, w = q

    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ], dtype=np.float64)
    return R


def make_transform(R, t_xyz):
    """旋转矩阵 + 平移 → 4x4 齐次变换矩阵。"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t_xyz, dtype=np.float64).reshape(3)
    return T


class CameraToBaseTransformer(Node):
    """订阅相机系目标 + 末端位姿，发布 base_link 系 VisualTarget。"""

    def __init__(self):
        super().__init__('camera_to_base_transformer')

        # ---- 手眼标定矩阵：gripper ← camera ----
        # 如果重新标定过，请修改这里的值
        self.t_cam2gripper = np.array(
            [-0.0830395307186257, 0.008112286716840913, 0.08580828291231507],
            dtype=np.float64
        )
        self.q_cam2gripper = np.array(
            [-0.49270434706957716, 0.5001884081237661,
             -0.49995706645552335, 0.5070472507158893],
            dtype=np.float64
        )

        R_gc = quat_xyzw_to_rot(
            self.q_cam2gripper[0],
            self.q_cam2gripper[1],
            self.q_cam2gripper[2],
            self.q_cam2gripper[3],
        )
        self.T_gc = make_transform(R_gc, self.t_cam2gripper)

        # ---- 图像尺寸（与 launch 参数保持一致） ----
        self.image_width = 640
        self.image_height = 480

        # ---- 订阅：末端位姿（由 arm_executor_node 统一发布） ----
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_arm/end_pose',
            self.pose_callback,
            10
        )

        # ---- 订阅：各物体在相机系下的位置（检测器输出） ----
        self.duck_sub = self.create_subscription(
            PointStamped,
            '/duck_position',
            lambda msg: self.object_callback(msg, 'duck'),
            10
        )

        self.apple_sub = self.create_subscription(
            PointStamped,
            '/apple_position',
            lambda msg: self.object_callback(msg, 'apple'),
            10
        )

        self.box_sub = self.create_subscription(
            PointStamped,
            '/box_position',
            lambda msg: self.object_callback(msg, 'box'),
            10
        )

        # ---- 发布：统一 VisualTarget（对接 robot_tasks 任务节点） ----
        self.target_pub = self.create_publisher(
            VisualTarget,
            '/visual_target_base',
            10
        )

        self.latest_pose_msg = None

        self.get_logger().info('CameraToBaseTransformer 启动。')
        self.get_logger().info(
            '监听: /robot_arm/end_pose, /duck_position, /apple_position, /box_position')
        self.get_logger().info('发布: /visual_target_base (VisualTarget)')

    def pose_callback(self, msg: PoseStamped):
        """缓存最新的末端位姿。"""
        self.latest_pose_msg = msg

    def object_callback(self, msg: PointStamped, object_name: str):
        """收到相机系目标 → 变换到 base_link → 发布 VisualTarget。"""
        if self.latest_pose_msg is None:
            self.get_logger().warning(
                f'尚未收到 /robot_arm/end_pose，无法变换 /{object_name}_position')
            return

        try:
            p = self.latest_pose_msg.pose.position
            q = self.latest_pose_msg.pose.orientation

            # base → gripper 变换矩阵
            R_bg = quat_xyzw_to_rot(q.x, q.y, q.z, q.w)
            T_bg = make_transform(R_bg, [p.x, p.y, p.z])

            # 相机系目标点
            Pc = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0],
                          dtype=np.float64)

            # gripper 系
            Pg = self.T_gc @ Pc

            # base_link 系
            Pb = T_bg @ Pg

            # ---- 构建 VisualTarget ----
            target = VisualTarget()
            target.header.stamp = msg.header.stamp
            target.header.frame_id = 'base_link'
            target.target_id = f'{object_name}_{self.get_clock().now().nanoseconds}'
            target.object_name = object_name
            target.x = float(Pb[0])
            target.y = float(Pb[1])
            target.z = float(Pb[2])
            target.confidence = 0.85       # 检测器暂不提供置信度，给默认值
            target.is_stable = True        # 坐标变换本身是稳定的

            # 像素坐标：检测器暂不提供，留空占位，等队友补充
            target.u = 0.0
            target.v = 0.0
            target.depth = float(Pc[2])    # 用相机系 Z 作为深度近似
            target.image_width = self.image_width
            target.image_height = self.image_height

            self.target_pub.publish(target)

            self.get_logger().info(
                f'{object_name}: '
                f'camera=({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}) m | '
                f'base=({target.x:.3f}, {target.y:.3f}, {target.z:.3f}) m',
                throttle_duration_sec=0.5   # 每秒最多打一条，减少刷屏
            )

        except Exception as e:
            self.get_logger().error(f'坐标变换失败 ({object_name}): {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraToBaseTransformer()
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
