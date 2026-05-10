#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert detector camera-frame points into base_link VisualTarget messages.

Main real-robot chain:
  /duck_position, /apple_position, /box_position (PointStamped, camera frame)
  + /robot_arm/end_pose (PoseStamped, base_link -> gripper)
  -> /visual_target_base (robot_msgs/VisualTarget, base_link)

The hand-eye transform below is ^gT_c, meaning camera-to-gripper.
"""

import math
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node

from robot_msgs.msg import VisualTarget


def quat_xyzw_to_rot(qx: float, qy: float, qz: float, qw: float):
    """Convert quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError('Quaternion norm is zero.')
    q /= norm
    x, y, z, w = q

    return np.array([
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
    ], dtype=np.float64)


def make_transform(rotation, translation_xyz):
    """Build a 4x4 rigid transform matrix."""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation_xyz, dtype=np.float64).reshape(3)
    return transform


def fmt_xyz(values) -> str:
    """Format a 3D vector for logs."""
    return f'({values[0]:.3f}, {values[1]:.3f}, {values[2]:.3f})'


@dataclass
class CachedTarget:
    msg: VisualTarget
    object_name: str
    base_xyz: list
    detected_time_sec: float


class CameraToBaseTransformer(Node):
    """Bridge detector output into the robot task VisualTarget topic."""

    def __init__(self):
        super().__init__('camera_to_base_transformer')
        self._declare_parameters()

        self.target_frame = self.get_parameter('target_frame').value
        allowed_frames_value = self.get_parameter('allowed_camera_frames').value
        if isinstance(allowed_frames_value, str):
            self.allowed_camera_frames = {allowed_frames_value}
        else:
            self.allowed_camera_frames = set(allowed_frames_value)
        self.max_end_pose_age_sec = float(self.get_parameter('max_end_pose_age_sec').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.default_confidence = float(self.get_parameter('default_confidence').value)
        self.assume_target_stable = bool(self.get_parameter('assume_target_stable').value)
        self.republish_rate_hz = float(self.get_parameter('republish_rate_hz').value)
        self.target_hold_sec = float(self.get_parameter('target_hold_sec').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)

        self.t_cam2gripper = np.array(
            self.get_parameter('cam_to_gripper_translation').value,
            dtype=np.float64,
        )
        self.q_cam2gripper = np.array(
            self.get_parameter('cam_to_gripper_quaternion_xyzw').value,
            dtype=np.float64,
        )
        if self.t_cam2gripper.shape[0] != 3:
            raise ValueError('cam_to_gripper_translation must contain 3 values.')
        if self.q_cam2gripper.shape[0] != 4:
            raise ValueError('cam_to_gripper_quaternion_xyzw must contain 4 values.')

        rotation_gripper_camera = quat_xyzw_to_rot(*self.q_cam2gripper)
        self.transform_gripper_camera = make_transform(
            rotation_gripper_camera,
            self.t_cam2gripper,
        )

        self.latest_pose_msg: Optional[PoseStamped] = None
        self.latest_pose_time_sec: Optional[float] = None
        self.latest_target: Optional[CachedTarget] = None
        self.last_detector_time_sec: Optional[float] = None
        self.last_log_time: Dict[str, float] = {}

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_arm/end_pose',
            self.pose_callback,
            10,
        )
        self.duck_sub = self.create_subscription(
            PointStamped,
            '/duck_position',
            lambda msg: self.object_callback(msg, 'duck', '/duck_position'),
            10,
        )
        self.apple_sub = self.create_subscription(
            PointStamped,
            '/apple_position',
            lambda msg: self.object_callback(msg, 'apple', '/apple_position'),
            10,
        )
        self.box_sub = self.create_subscription(
            PointStamped,
            '/box_position',
            lambda msg: self.object_callback(msg, 'box', '/box_position'),
            10,
        )
        self.target_pub = self.create_publisher(
            VisualTarget,
            '/visual_target_base',
            10,
        )
        self.hold_timer = self.create_timer(
            1.0 / max(self.republish_rate_hz, 0.1),
            self.republish_latest_target,
        )

        self.get_logger().info(
            'CameraToBaseTransformer started. Listening: /robot_arm/end_pose, '
            '/duck_position, /apple_position, /box_position')
        self.get_logger().info('Publishing: /visual_target_base (robot_msgs/msg/VisualTarget)')
        self.get_logger().info(
            f'Using ^gT_c translation={self.t_cam2gripper.tolist()}, '
            f'quaternion_xyzw={self.q_cam2gripper.tolist()}')
        self.get_logger().info(
            f'target_frame={self.target_frame}, allowed_camera_frames={sorted(self.allowed_camera_frames)}, '
            f'max_end_pose_age_sec={self.max_end_pose_age_sec:.2f}, '
            f'default_confidence={self.default_confidence:.2f}, '
            f'assume_target_stable={self.assume_target_stable}, '
            f'republish_rate_hz={self.republish_rate_hz:.2f}, '
            f'target_hold_sec={self.target_hold_sec:.2f}')

    def _declare_parameters(self):
        self.declare_parameter(
            'cam_to_gripper_translation',
            [-0.0830395307186257, 0.008112286716840913, 0.08580828291231507],
        )
        self.declare_parameter(
            'cam_to_gripper_quaternion_xyzw',
            [-0.49270434706957716, 0.5001884081237661, -0.49995706645552335, 0.5070472507158893],
        )
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('allowed_camera_frames', ['camera_color_optical_frame'])
        self.declare_parameter('max_end_pose_age_sec', 0.5)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('default_confidence', 0.85)
        self.declare_parameter('assume_target_stable', True)
        self.declare_parameter('republish_rate_hz', 10.0)
        self.declare_parameter('target_hold_sec', 0.8)
        self.declare_parameter('target_timeout_sec', 1.5)

    def pose_callback(self, msg: PoseStamped):
        """Cache the latest end-effector pose (^bT_g)."""
        self.latest_pose_msg = msg
        self.latest_pose_time_sec = self.now_sec()

    def object_callback(self, msg: PointStamped, object_name: str, source_topic: str):
        """Transform a detector point and publish it as a VisualTarget."""
        self.last_detector_time_sec = self.now_sec()

        if self.latest_pose_msg is None:
            self.warn_throttled(
                'missing_end_pose',
                'Waiting for /robot_arm/end_pose before transforming visual target.',
                1.0,
            )
            return

        pose_age = self.now_sec() - self.latest_pose_time_sec
        if pose_age > self.max_end_pose_age_sec:
            self.warn_throttled(
                'stale_end_pose',
                f'Reject {source_topic}: /robot_arm/end_pose is stale ({pose_age:.3f}s > '
                f'{self.max_end_pose_age_sec:.3f}s).',
                1.0,
            )
            return

        frame_id = msg.header.frame_id.strip()
        if frame_id and frame_id not in self.allowed_camera_frames:
            self.warn_throttled(
                f'bad_frame_{source_topic}',
                f'Unexpected {source_topic} frame_id={frame_id!r}; '
                f'expected one of {sorted(self.allowed_camera_frames)}.',
                1.0,
            )

        try:
            camera_xyz = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]
            base_xyz = self.camera_to_base(camera_xyz)
            target = self.make_visual_target(object_name, camera_xyz, base_xyz)
        except Exception as exc:
            self.get_logger().error(f'Failed to transform {source_topic}: {exc}')
            return

        self.target_pub.publish(target)
        self.latest_target = CachedTarget(
            msg=target,
            object_name=object_name,
            base_xyz=base_xyz,
            detected_time_sec=self.now_sec(),
        )
        self.info_throttled(
            f'publish_{object_name}',
            f'{object_name} from {source_topic}: camera={fmt_xyz(camera_xyz)} base={fmt_xyz(base_xyz)}',
            0.5,
        )

    def republish_latest_target(self):
        """Republish the last valid converted target during short detector gaps."""
        now = self.now_sec()
        if self.latest_target is None:
            if self.last_detector_time_sec is not None:
                age = now - self.last_detector_time_sec
                if age > self.target_timeout_sec:
                    self.warn_throttled(
                        'no_recent_detector_target',
                        f'No new visual target for {age:.2f}s.',
                        2.0,
                    )
            return

        age = now - self.latest_target.detected_time_sec
        if age <= self.target_hold_sec:
            self.latest_target.msg.header.stamp = self.get_clock().now().to_msg()
            self.target_pub.publish(self.latest_target.msg)
            self.info_throttled(
                'holding_target',
                f'Holding {self.latest_target.object_name}: '
                f'base={fmt_xyz(self.latest_target.base_xyz)} age={age:.2f}s',
                2.0,
            )
        elif age > self.target_timeout_sec:
            self.warn_throttled(
                'target_timeout',
                f'No new visual target for {age:.2f}s.',
                2.0,
            )

    def camera_to_base(self, camera_xyz: list) -> list:
        """Compute ^bP = ^bT_g * ^gT_c * ^cP."""
        pose = self.latest_pose_msg.pose
        rotation_base_gripper = quat_xyzw_to_rot(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        transform_base_gripper = make_transform(
            rotation_base_gripper,
            [pose.position.x, pose.position.y, pose.position.z],
        )
        point_camera = np.array([camera_xyz[0], camera_xyz[1], camera_xyz[2], 1.0], dtype=np.float64)
        point_base = transform_base_gripper @ self.transform_gripper_camera @ point_camera
        return [float(point_base[0]), float(point_base[1]), float(point_base[2])]

    def make_visual_target(self, object_name: str, camera_xyz: list, base_xyz: list):
        """Create the VisualTarget consumed by grasp_task_open_loop."""
        target = VisualTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.target_frame
        target.target_id = f'{object_name}_{self.get_clock().now().nanoseconds}'
        target.object_name = object_name
        target.x = float(base_xyz[0])
        target.y = float(base_xyz[1])
        target.z = float(base_xyz[2])
        target.confidence = self.default_confidence
        target.is_stable = self.assume_target_stable
        target.u = 0.0
        target.v = 0.0
        target.depth = float(camera_xyz[2])
        target.image_width = self.image_width
        target.image_height = self.image_height
        return target

    def now_sec(self) -> float:
        """Return ROS time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def info_throttled(self, key: str, message: str, period_sec: float):
        if self.should_log(key, period_sec):
            self.get_logger().info(message)

    def warn_throttled(self, key: str, message: str, period_sec: float):
        if self.should_log(key, period_sec):
            self.get_logger().warning(message)

    def should_log(self, key: str, period_sec: float) -> bool:
        now = self.now_sec()
        last = self.last_log_time.get(key)
        if last is None or now - last >= period_sec:
            self.last_log_time[key] = now
            return True
        return False


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
