#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node

from robot_msgs.msg import VisualTarget


def quat_xyzw_to_rot(qx: float, qy: float, qz: float, qw: float):
    """Convert quaternion xyzw to a 3x3 rotation matrix."""
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


def make_transform_matrix(rotation, translation):
    """Build a 4x4 homogeneous transform matrix."""
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], translation[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], translation[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def invert_transform(transform):
    """Invert a rigid 4x4 transform matrix."""
    rotation_t = [
        [transform[0][0], transform[1][0], transform[2][0]],
        [transform[0][1], transform[1][1], transform[2][1]],
        [transform[0][2], transform[1][2], transform[2][2]],
    ]
    translation = [transform[0][3], transform[1][3], transform[2][3]]
    inv_translation = [
        -sum(rotation_t[row][col] * translation[col] for col in range(3))
        for row in range(3)
    ]
    return make_transform_matrix(rotation_t, inv_translation)


def transform_point(transform, point_xyz):
    """Apply a 4x4 transform to a 3D point."""
    point = [point_xyz[0], point_xyz[1], point_xyz[2], 1.0]
    x = sum(transform[0][i] * point[i] for i in range(4))
    y = sum(transform[1][i] * point[i] for i in range(4))
    z = sum(transform[2][i] * point[i] for i in range(4))
    w = sum(transform[3][i] * point[i] for i in range(4))
    if abs(w) < 1e-12:
        raise ValueError('Invalid homogeneous transform result.')
    return [x / w, y / w, z / w]


def distance(a, b):
    """Euclidean distance between two 3D points."""
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def fmt_vec(values):
    """Readable vector formatting for transform debug logs."""
    return '[' + ', '.join(f'{float(v): .6f}' for v in values) + ']'


def fmt_matrix(matrix):
    """Readable matrix formatting for transform debug logs."""
    return '\n'.join('    ' + fmt_vec(row) for row in matrix)


class VisualTargetBridge(Node):
    """Bridge /duck_position from camera optical frame to base_link VisualTarget.

    Transform convention used here:
      P_base = T_base_gripper * T_gripper_camera * P_camera

    The detector already publishes optical-frame coordinates:
      X right, Y down, Z forward/depth.
    Therefore this node must not apply another optical-axis conversion.
    """

    def __init__(self):
        super().__init__('visual_target_bridge')

        self._declare_parameters()

        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.expected_camera_frame = self.get_parameter('expected_camera_frame').value
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.max_camera_jump_m = float(self.get_parameter('max_camera_jump_m').value)
        self.max_base_jump_m = float(self.get_parameter('max_base_jump_m').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        self.T_gripper_camera = self._load_handeye_transform()

        self.latest_end_pose = None
        self.last_accepted_camera_xyz = None
        self.last_published_base_xyz = None

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
        self.get_logger().info(
            f'Input filter: depth=({self.min_depth_m:.3f}, {self.max_depth_m:.3f}) m, '
            f'max_camera_jump={self.max_camera_jump_m:.3f} m, '
            f'max_base_jump={self.max_base_jump_m:.3f} m')
        self.get_logger().info(
            'Using T_gripper_camera (^gT_c):\n' + fmt_matrix(self.T_gripper_camera))

    def _declare_parameters(self):
        """Declare bridge parameters so launch YAML can tune calibration/filtering."""
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('expected_camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('max_depth_m', 1.50)
        self.declare_parameter('max_camera_jump_m', 0.35)
        self.declare_parameter('max_base_jump_m', 0.60)
        self.declare_parameter('debug_log', True)

        # This calibration is the OpenCV calibrateHandEye result named
        # cam2gripper in the existing scripts, meaning ^gT_c.
        self.declare_parameter(
            'handeye.translation',
            [-0.0830395307186257, 0.008112286716840913, 0.08580828291231507],
        )
        self.declare_parameter(
            'handeye.quaternion_xyzw',
            [-0.49270434706957716,
             0.5001884081237661,
             -0.49995706645552335,
             0.5070472507158893],
        )
        self.declare_parameter('handeye.direction', 'camera_to_gripper')

    def _load_handeye_transform(self):
        """Load hand-eye extrinsic and return T_gripper_camera (^gT_c)."""
        translation = list(self.get_parameter('handeye.translation').value)
        quaternion = list(self.get_parameter('handeye.quaternion_xyzw').value)
        direction = self.get_parameter('handeye.direction').value.strip().lower()

        if len(translation) != 3:
            raise ValueError('handeye.translation must contain 3 values.')
        if len(quaternion) != 4:
            raise ValueError('handeye.quaternion_xyzw must contain 4 values.')

        rotation = quat_xyzw_to_rot(
            quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        transform = make_transform_matrix(rotation, translation)

        if direction == 'camera_to_gripper':
            return transform
        if direction == 'gripper_to_camera':
            return invert_transform(transform)

        raise ValueError(
            "handeye.direction must be 'camera_to_gripper' or 'gripper_to_camera'.")

    def pose_callback(self, msg: PoseStamped):
        """Cache the latest end-effector pose.

        Do not publish a visual target from this callback. Re-transforming a stale
        detector point every time the arm moves can create artificial target jumps.
        """
        self.latest_end_pose = msg

    def duck_callback(self, msg: PointStamped):
        """Filter detector point, transform it to base_link, then publish."""
        camera_xyz = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]
        self.get_logger().info(
            f'P_camera raw /duck_position: {fmt_vec(camera_xyz)} '
            f'frame={msg.header.frame_id}')

        if not self._valid_camera_input(msg, camera_xyz):
            return

        if self.latest_end_pose is None:
            self.get_logger().warning(
                'No /robot_arm/end_pose yet, skip current detector target.')
            return

        try:
            base_xyz, gripper_xyz = self._camera_to_base(camera_xyz)
        except Exception as e:
            self.get_logger().error(f'Failed to transform detector target: {e}')
            return

        if self.last_published_base_xyz is not None:
            base_jump = distance(self.last_published_base_xyz, base_xyz)
            if base_jump > self.max_base_jump_m:
                self.get_logger().warning(
                    f'Reject transformed target: base jump {base_jump:.3f} m '
                    f'> {self.max_base_jump_m:.3f} m. P_base={fmt_vec(base_xyz)}')
                return

        self._log_transform_debug(camera_xyz, gripper_xyz, base_xyz)
        self._publish_visual_target(msg, camera_xyz, base_xyz)

        self.last_accepted_camera_xyz = camera_xyz
        self.last_published_base_xyz = base_xyz

    def _valid_camera_input(self, msg: PointStamped, camera_xyz):
        """Reject invalid depth, wrong frame, and obvious detector jumps."""
        frame_id = msg.header.frame_id.strip()
        if self.expected_camera_frame and frame_id and frame_id != self.expected_camera_frame:
            self.get_logger().warning(
                f'Unexpected detector frame: {frame_id}, '
                f'expected {self.expected_camera_frame}. Skip target.')
            return False

        if not all(math.isfinite(v) for v in camera_xyz):
            self.get_logger().warning(f'Reject detector target: non-finite P_camera={fmt_vec(camera_xyz)}')
            return False

        depth = camera_xyz[2]
        if depth <= self.min_depth_m:
            self.get_logger().warning(
                f'Reject detector target: invalid depth {depth:.3f} m '
                f'<= {self.min_depth_m:.3f} m.')
            return False

        if depth > self.max_depth_m:
            self.get_logger().warning(
                f'Reject detector target: depth {depth:.3f} m '
                f'> {self.max_depth_m:.3f} m.')
            return False

        if self.last_accepted_camera_xyz is not None:
            camera_jump = distance(self.last_accepted_camera_xyz, camera_xyz)
            if camera_jump > self.max_camera_jump_m:
                self.get_logger().warning(
                    f'Reject detector target: camera jump {camera_jump:.3f} m '
                    f'> {self.max_camera_jump_m:.3f} m. P_camera={fmt_vec(camera_xyz)}')
                return False

        return True

    def _camera_to_base(self, camera_xyz):
        """Transform a point from camera optical frame to base_link."""
        pose = self.latest_end_pose.pose
        translation_bg = [pose.position.x, pose.position.y, pose.position.z]
        quaternion_bg = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        rotation_bg = quat_xyzw_to_rot(*quaternion_bg)
        T_base_gripper = make_transform_matrix(rotation_bg, translation_bg)

        gripper_xyz = transform_point(self.T_gripper_camera, camera_xyz)
        base_xyz = transform_point(T_base_gripper, gripper_xyz)
        return base_xyz, gripper_xyz

    def _log_transform_debug(self, camera_xyz, gripper_xyz, base_xyz):
        """Print enough transform detail to see where a bad value is introduced."""
        if not self.debug_log:
            return

        pose = self.latest_end_pose.pose
        translation_bg = [pose.position.x, pose.position.y, pose.position.z]
        quaternion_bg = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        rotation_bg = quat_xyzw_to_rot(*quaternion_bg)
        T_base_gripper = make_transform_matrix(rotation_bg, translation_bg)

        self.get_logger().info(
            'Transform debug:\n'
            f'  P_camera optical = {fmt_vec(camera_xyz)}\n'
            f'  end_pose translation base->gripper = {fmt_vec(translation_bg)}\n'
            f'  end_pose quaternion xyzw = {fmt_vec(quaternion_bg)}\n'
            f'  T_gripper_camera (^gT_c):\n{fmt_matrix(self.T_gripper_camera)}\n'
            f'  T_base_gripper (^bT_g):\n{fmt_matrix(T_base_gripper)}\n'
            f'  P_gripper = {fmt_vec(gripper_xyz)}\n'
            f'  P_base = {fmt_vec(base_xyz)}')

    def _publish_visual_target(self, source_msg: PointStamped, camera_xyz, base_xyz):
        """Publish a filtered VisualTarget in base_link frame."""
        target = VisualTarget()
        target.header.stamp = source_msg.header.stamp
        target.header.frame_id = 'base_link'
        target.target_id = f'duck_{self.get_clock().now().nanoseconds}'
        target.object_name = 'duck'
        target.x = float(base_xyz[0])
        target.y = float(base_xyz[1])
        target.z = float(base_xyz[2])
        target.confidence = 1.0
        target.is_stable = True
        target.u = 0.0
        target.v = 0.0
        target.depth = float(camera_xyz[2])
        target.image_width = self.image_width
        target.image_height = self.image_height

        self.target_pub.publish(target)
        self.get_logger().info(
            f'Published /visual_target_base: P_base={fmt_vec(base_xyz)} '
            f'depth={camera_xyz[2]:.3f} m')


def main(args=None):
    rclpy.init(args=args)
    node = VisualTargetBridge()
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
