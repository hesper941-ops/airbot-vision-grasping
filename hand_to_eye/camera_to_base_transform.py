#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped


def quat_xyzw_to_rot(qx, qy, qz, qw):
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
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t_xyz, dtype=np.float64).reshape(3)
    return T


class CameraToBaseTransformer(Node):
    def __init__(self):
        super().__init__('camera_to_base_transformer')

        # 手眼标定结果：gripper <- camera
        self.t_cam2gripper = np.array(
            [-0.0830395307186257, 0.008112286716840913, 0.08580828291231507],
            dtype=np.float64
        )
        self.q_cam2gripper = np.array(
            [-0.49270434706957716, 0.5001884081237661, -0.49995706645552335, 0.5070472507158893],
            dtype=np.float64
        )

        R_gc = quat_xyzw_to_rot(
            self.q_cam2gripper[0],
            self.q_cam2gripper[1],
            self.q_cam2gripper[2],
            self.q_cam2gripper[3],
        )
        self.T_gc = make_transform(R_gc, self.t_cam2gripper)

        # 订阅机械臂末端位姿
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_arm/end_pose',
            self.pose_callback,
            10
        )

        # 订阅三种目标在相机坐标系下的位置
        self.apple_sub = self.create_subscription(
            PointStamped,
            '/apple_position',
            lambda msg: self.object_callback(msg, 'apple'),
            10
        )

        self.duck_sub = self.create_subscription(
            PointStamped,
            '/duck_position',
            lambda msg: self.object_callback(msg, 'duck'),
            10
        )

        self.box_sub = self.create_subscription(
            PointStamped,
            '/box_position',
            lambda msg: self.object_callback(msg, 'box'),
            10
        )

        # 发布三种目标在 gripper / base 坐标系下的位置
        self.apple_base_pub = self.create_publisher(
            PointStamped,
            '/apple_position_base',
            10
        )
        self.apple_gripper_pub = self.create_publisher(
            PointStamped,
            '/apple_position_gripper',
            10
        )

        self.duck_base_pub = self.create_publisher(
            PointStamped,
            '/duck_position_base',
            10
        )
        self.duck_gripper_pub = self.create_publisher(
            PointStamped,
            '/duck_position_gripper',
            10
        )

        self.box_base_pub = self.create_publisher(
            PointStamped,
            '/box_position_base',
            10
        )
        self.box_gripper_pub = self.create_publisher(
            PointStamped,
            '/box_position_gripper',
            10
        )

        self.latest_pose_msg = None

        self.get_logger().info('CameraToBaseTransformer started.')
        self.get_logger().info(
            'Subscribing: /robot_arm/end_pose, /apple_position, /duck_position, /box_position'
        )
        self.get_logger().info('Publishing : /apple_position_gripper, /apple_position_base')
        self.get_logger().info('Publishing : /duck_position_gripper, /duck_position_base')
        self.get_logger().info('Publishing : /box_position_gripper, /box_position_base')

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose_msg = msg

    def object_callback(self, msg: PointStamped, object_name: str):
        if self.latest_pose_msg is None:
            self.get_logger().warning(
                f'No /robot_arm/end_pose received yet, cannot transform /{object_name}_position'
            )
            return

        try:
            p = self.latest_pose_msg.pose.position
            q = self.latest_pose_msg.pose.orientation

            # base -> gripper
            R_bg = quat_xyzw_to_rot(q.x, q.y, q.z, q.w)
            T_bg = make_transform(R_bg, [p.x, p.y, p.z])

            # 相机坐标系下目标点
            Pc = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0], dtype=np.float64)

            # gripper 坐标系
            Pg = self.T_gc @ Pc

            # base 坐标系
            Pb = T_bg @ Pg

            gripper_msg = PointStamped()
            gripper_msg.header.stamp = msg.header.stamp
            gripper_msg.header.frame_id = 'gripper_frame'
            gripper_msg.point.x = float(Pg[0])
            gripper_msg.point.y = float(Pg[1])
            gripper_msg.point.z = float(Pg[2])

            base_msg = PointStamped()
            base_msg.header.stamp = msg.header.stamp
            base_msg.header.frame_id = 'base_link'
            base_msg.point.x = float(Pb[0])
            base_msg.point.y = float(Pb[1])
            base_msg.point.z = float(Pb[2])

            if object_name == 'apple':
                self.apple_gripper_pub.publish(gripper_msg)
                self.apple_base_pub.publish(base_msg)
            elif object_name == 'duck':
                self.duck_gripper_pub.publish(gripper_msg)
                self.duck_base_pub.publish(base_msg)
            elif object_name == 'box':
                self.box_gripper_pub.publish(gripper_msg)
                self.box_base_pub.publish(base_msg)
            else:
                self.get_logger().warning(f'Unknown object_name: {object_name}')
                return

            self.get_logger().info(
                f'{object_name}: '
                f'camera=({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}) m | '
                f'gripper=({gripper_msg.point.x:.3f}, {gripper_msg.point.y:.3f}, {gripper_msg.point.z:.3f}) m | '
                f'base=({base_msg.point.x:.3f}, {base_msg.point.y:.3f}, {base_msg.point.z:.3f}) m'
            )

        except Exception as e:
            self.get_logger().error(f'Transform failed for {object_name}: {e}')


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