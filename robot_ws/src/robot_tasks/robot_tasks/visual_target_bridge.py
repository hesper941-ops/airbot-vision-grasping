#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node

from robot_msgs.msg import VisualTarget


def quat_xyzw_to_rot(qx: float, qy: float, qz: float, qw: float):
    """Convert quaternion to 3x3 rotation matrix."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        raise ValueError('Quaternion norm is zero.')
    x = qx / norm
    y = qy / norm
    z = qz / norm
    w = qw / norm

    return [
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
    ]


def make_transform_matrix(R, t_xyz):
    """Build a 4x4 homogeneous transformation matrix."""
    return [
        [R[0][0], R[0][1], R[0][2], t_xyz[0]],
        [R[1][0], R[1][1], R[1][2], t_xyz[1]],
        [R[2][0], R[2][1], R[2][2], t_xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def transform_point(T, point):
    """Transform a homogeneous point by a 4x4 matrix."""
    x = T[0][0] * point[0] + T[0][1] * point[1] + T[0][2] * point[2] + T[0][3] * point[3]
    y = T[1][0] * point[0] + T[1][1] * point[1] + T[1][2] * point[2] + T[1][3] * point[3]
    z = T[2][0] * point[0] + T[2][1] * point[1] + T[2][2] * point[2] + T[2][3] * point[3]
    w = T[3][0] * point[0] + T[3][1] * point[1] + T[3][2] * point[2] + T[3][3] * point[3]
    if abs(w) < 1e-12:
        raise ValueError('Invalid homogeneous transform result.')
    return [x / w, y / w, z / w]


class VisualTargetBridge(Node):
    """Bridge from detector PointStamped to robot_msgs/VisualTarget."""

    def __init__(self):
        super().__init__('visual_target_bridge')

        # Camera-to-gripper extrinsics (from existing hand-eye calibration).
        self.t_cam2gripper = [-0.0830395307186257,
                              0.008112286716840913,
                              0.08580828291231507]
        self.q_cam2gripper = [-0.49270434706957716,
                              0.5001884081237661,
                              -0.49995706645552335,
                              0.5070472507158893]

        R_gc = quat_xyzw_to_rot(*self.q_cam2gripper)
        self.T_gc = make_transform_matrix(R_gc, self.t_cam2gripper)

        self.image_width = 640
        self.image_height = 480

        self.latest_end_pose = None

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_arm/end_pose',
            self.pose_callback,
            10,
        )
        self.duck_sub = self.create_subscription(
            PointStamped,
            '/duck_position',
            self.duck_callback,
            10,
        )

        self.target_pub = self.create_publisher(
            VisualTarget,
            '/visual_target_base',
            10,
        )

        self.get_logger().info('VisualTargetBridge started.')
        self.get_logger().info('Listening: /robot_arm/end_pose, /duck_position')
        self.get_logger().info('Publishing: /visual_target_base (VisualTarget)')

    def pose_callback(self, msg: PoseStamped):
        """Cache the latest end-effector pose from the executor."""
        self.latest_end_pose = msg

    def duck_callback(self, msg: PointStamped):
        """Convert detector output from camera frame into base_link VisualTarget."""
        self.get_logger().info(
            f'Received detector target /duck_position: '
            f'({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}) '
            f'frame={msg.header.frame_id}')

        if self.latest_end_pose is None:
            self.get_logger().warning(
                'No /robot_arm/end_pose yet, cannot transform detector target.')
            return

        try:
            target_xyz = self._camera_to_base(msg.point)
        except Exception as e:
            self.get_logger().error(f'Failed to transform detector target: {e}')
            return

        target = VisualTarget()
        target.header.stamp = msg.header.stamp
        target.header.frame_id = 'base_link'
        target.target_id = f'duck_{self.get_clock().now().nanoseconds}'
        target.object_name = 'duck'
        target.x = float(target_xyz[0])
        target.y = float(target_xyz[1])
        target.z = float(target_xyz[2])
        target.confidence = 1.0
        target.is_stable = True
        target.u = 0.0
        target.v = 0.0
        target.depth = float(msg.point.z)
        target.image_width = self.image_width
        target.image_height = self.image_height

        self.target_pub.publish(target)
        self.get_logger().info(
            f'Published VisualTarget to /visual_target_base: '
            f'({target.x:.3f}, {target.y:.3f}, {target.z:.3f})')

    def _camera_to_base(self, point):
        """Transform a point from camera coordinates to base_link coordinates."""
        pose = self.latest_end_pose.pose
        R_bg = quat_xyzw_to_rot(pose.orientation.x,
                                 pose.orientation.y,
                                 pose.orientation.z,
                                 pose.orientation.w)
        T_bg = make_transform_matrix(R_bg,
                                    [pose.position.x,
                                     pose.position.y,
                                     pose.position.z])

        # camera frame point in homogeneous coordinates
        Pc = [point.x, point.y, point.z, 1.0]
        Pg = transform_point(self.T_gc, Pc)
        Pg_h = [Pg[0], Pg[1], Pg[2], 1.0]
        Pb = transform_point(T_bg, Pg_h)
        return Pb


def main(args=None):
    rclpy.init(args=args)
    node = VisualTargetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
