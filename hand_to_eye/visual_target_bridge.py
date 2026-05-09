#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bridge detector PointStamped targets into base_link VisualTarget messages.

This script is intentionally kept outside robot_ws packages so it can be run
manually during real-camera tests without changing the robot task state machine
or launch files.
"""

import math
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node

from robot_msgs.msg import VisualTarget


def quat_xyzw_to_rot(qx: float, qy: float, qz: float, qw: float):
    """Convert quaternion xyzw into a 3x3 rotation matrix."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        raise ValueError('Quaternion norm is zero.')

    x = qx / norm
    y = qy / norm
    z = qz / norm
    w = qw / norm

    return [
        [1.0 - 2.0 * y * y - 2.0 * z * z, 2.0 * x * y - 2.0 * z * w, 2.0 * x * z + 2.0 * y * w],
        [2.0 * x * y + 2.0 * z * w, 1.0 - 2.0 * x * x - 2.0 * z * z, 2.0 * y * z - 2.0 * x * w],
        [2.0 * x * z - 2.0 * y * w, 2.0 * y * z + 2.0 * x * w, 1.0 - 2.0 * x * x - 2.0 * y * y],
    ]


def transform_point(rotation, translation, point):
    """Apply a rigid transform to a 3D point."""
    return [
        rotation[row][0] * point[0]
        + rotation[row][1] * point[1]
        + rotation[row][2] * point[2]
        + translation[row]
        for row in range(3)
    ]


def fmt_xyz(values) -> str:
    """Format a 3D vector for readable logs."""
    return f'({values[0]:.3f}, {values[1]:.3f}, {values[2]:.3f})'


@dataclass
class CachedTarget:
    msg: VisualTarget
    source_topic: str
    camera_xyz: list
    base_xyz: list
    detected_time_sec: float


class VisualTargetBridge(Node):
    """Convert held camera-frame detector targets into /visual_target_base."""

    def __init__(self):
        super().__init__('hand_to_eye_visual_target_bridge')

        self.declare_parameter('republish_rate_hz', 10.0)
        self.declare_parameter('target_hold_sec', 0.8)
        self.declare_parameter('target_timeout_sec', 1.5)
        self.declare_parameter('confidence', 0.85)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        self.republish_rate_hz = float(self.get_parameter('republish_rate_hz').value)
        self.target_hold_sec = float(self.get_parameter('target_hold_sec').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.confidence = float(self.get_parameter('confidence').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)

        self.t_cam2gripper = [
            -0.0830395307186257,
            0.008112286716840913,
            0.08580828291231507,
        ]
        self.q_cam2gripper_xyzw = [
            -0.49270434706957716,
            0.5001884081237661,
            -0.49995706645552335,
            0.5070472507158893,
        ]
        self.rotation_gripper_camera = quat_xyzw_to_rot(*self.q_cam2gripper_xyzw)

        self.latest_end_pose: Optional[PoseStamped] = None
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

        timer_period = 1.0 / max(self.republish_rate_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.republish_latest_target)

        self.get_logger().info(
            'Listening: /robot_arm/end_pose, /duck_position, /apple_position, /box_position')
        self.get_logger().info('Publishing: /visual_target_base (robot_msgs/msg/VisualTarget)')
        self.get_logger().info(
            f'Handeye ^gT_c translation={self.t_cam2gripper}, '
            f'quaternion_xyzw={self.q_cam2gripper_xyzw}')
        self.get_logger().info(
            f'republish_rate_hz={self.republish_rate_hz:.2f}, '
            f'target_hold_sec={self.target_hold_sec:.2f}, '
            f'target_timeout_sec={self.target_timeout_sec:.2f}')

    def pose_callback(self, msg: PoseStamped):
        """Cache the current base-to-gripper pose (^bT_g)."""
        self.latest_end_pose = msg

    def object_callback(self, msg: PointStamped, object_name: str, source_topic: str):
        """Transform one detector target and cache it for short-term republish."""
        self.last_detector_time_sec = self.now_sec()

        if self.latest_end_pose is None:
            self.warn_throttled(
                'waiting_end_pose',
                'Waiting for /robot_arm/end_pose before transforming visual target.',
                1.0,
            )
            return

        try:
            camera_xyz = [
                float(msg.point.x),
                float(msg.point.y),
                float(msg.point.z),
            ]
            base_xyz = self.camera_to_base(camera_xyz)
            target = self.make_visual_target(msg, object_name, camera_xyz, base_xyz)
        except Exception as exc:
            self.get_logger().error(f'Failed to transform {object_name} from {source_topic}: {exc}')
            return

        self.latest_target = CachedTarget(
            msg=target,
            source_topic=source_topic,
            camera_xyz=camera_xyz,
            base_xyz=base_xyz,
            detected_time_sec=self.now_sec(),
        )
        self.target_pub.publish(target)

        self.info_throttled(
            f'received_{object_name}',
            f'Received {object_name} from {source_topic}: '
            f'camera={fmt_xyz(camera_xyz)} base={fmt_xyz(base_xyz)}',
            0.5,
        )

    def camera_to_base(self, camera_xyz: list) -> list:
        """Compute ^bP = ^bT_g * ^gT_c * ^cP."""
        pose = self.latest_end_pose.pose
        translation_base_gripper = [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ]
        rotation_base_gripper = quat_xyzw_to_rot(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )

        gripper_xyz = transform_point(
            self.rotation_gripper_camera,
            self.t_cam2gripper,
            camera_xyz,
        )
        return transform_point(
            rotation_base_gripper,
            translation_base_gripper,
            gripper_xyz,
        )

    def make_visual_target(self, source_msg: PointStamped, object_name: str, camera_xyz: list, base_xyz: list):
        """Create the VisualTarget consumed by grasp_task_open_loop."""
        target = VisualTarget()
        target.header.stamp = source_msg.header.stamp
        target.header.frame_id = 'base_link'
        target.target_id = f'{object_name}_{self.get_clock().now().nanoseconds}'
        target.object_name = object_name
        target.x = float(base_xyz[0])
        target.y = float(base_xyz[1])
        target.z = float(base_xyz[2])
        target.confidence = self.confidence
        target.is_stable = True
        target.u = 0.0
        target.v = 0.0
        target.depth = float(camera_xyz[2])
        target.image_width = self.image_width
        target.image_height = self.image_height
        return target

    def republish_latest_target(self):
        """Republish a recent target during short detector gaps."""
        now = self.now_sec()

        if self.latest_target is None:
            if self.last_detector_time_sec is None:
                self.warn_throttled('no_input_ever', 'No visual detector input received yet.', 2.0)
            elif now - self.last_detector_time_sec > self.target_timeout_sec:
                self.warn_throttled(
                    'no_recent_input',
                    f'No new visual target for {now - self.last_detector_time_sec:.2f}s.',
                    2.0,
                )
            return

        age = now - self.latest_target.detected_time_sec
        if age <= self.target_hold_sec:
            self.target_pub.publish(self.latest_target.msg)
            self.info_throttled(
                'holding_target',
                f'Holding {self.latest_target.msg.object_name}: '
                f'base={fmt_xyz(self.latest_target.base_xyz)} age={age:.2f}s',
                2.0,
            )
            return

        if age > self.target_timeout_sec:
            self.warn_throttled(
                'target_timeout',
                f'No new visual target for {age:.2f}s.',
                2.0,
            )

    def now_sec(self) -> float:
        """Return ROS time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def info_throttled(self, key: str, message: str, period_sec: float):
        """Log an info message at most once per period."""
        if self.should_log(key, period_sec):
            self.get_logger().info(message)

    def warn_throttled(self, key: str, message: str, period_sec: float):
        """Log a warning message at most once per period."""
        if self.should_log(key, period_sec):
            self.get_logger().warning(message)

    def should_log(self, key: str, period_sec: float) -> bool:
        """Return True when a keyed throttle interval has elapsed."""
        now = self.now_sec()
        last = self.last_log_time.get(key)
        if last is None or now - last >= period_sec:
            self.last_log_time[key] = now
            return True
        return False


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
