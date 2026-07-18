#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from robot_msgs.msg import VisualTarget


class FakeVisualTargetPublisher(Node):
    """Publish a stable fake visual target in base_link for open-loop grasp testing."""

    def __init__(self, args):
        super().__init__('fake_visual_target_publisher')

        self.args = args
        self.last_end_pose: Optional[list] = None
        self.last_status: Optional[str] = None
        self.start_time = time.time()
        self.target_xyz: Optional[list] = None

        self.target_pub = self.create_publisher(
            VisualTarget,
            '/visual_target_base',
            10,
        )

        self.end_pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_arm/end_pose',
            self.end_pose_callback,
            10,
        )

        self.status_sub = self.create_subscription(
            String,
            '/robot_arm/executor_status',
            self.status_callback,
            10,
        )

        self.timer = self.create_timer(1.0 / self.args.rate, self.timer_callback)

        self.get_logger().info('Fake visual target publisher started.')
        self.get_logger().info(
            'This node publishes /visual_target_base only. The real arm still moves.'
        )

    def end_pose_callback(self, msg: PoseStamped):
        self.last_end_pose = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ]

    def status_callback(self, msg: String):
        status = msg.data.strip()
        if status != self.last_status:
            self.last_status = status
            self.get_logger().info(f'executor_status = {status}')

    def compute_target_once(self) -> bool:
        if self.target_xyz is not None:
            return True

        if self.args.x is not None and self.args.y is not None and self.args.z is not None:
            self.target_xyz = [float(self.args.x), float(self.args.y), float(self.args.z)]
            self.get_logger().info(
                f'Using manual fake target: {self.format_xyz(self.target_xyz)}'
            )
            return True

        if self.last_end_pose is None:
            self.get_logger().info('Waiting for /robot_arm/end_pose...')
            return False

        current = self.last_end_pose

        target_x = current[0] + self.args.dx
        target_y = current[1] + self.args.dy
        target_z = current[2] - self.args.drop

        if target_z < self.args.min_z:
            self.get_logger().error(
                f'Computed target_z={target_z:.3f} is lower than min_z={self.args.min_z:.3f}. '
                'Abort fake target publishing. Use a smaller --drop.'
            )
            return False

        self.target_xyz = [target_x, target_y, target_z]

        self.get_logger().info(f'Current end_pose: {self.format_xyz(current)}')
        self.get_logger().info(f'Auto fake target: {self.format_xyz(self.target_xyz)}')
        self.get_logger().info(
            f'Expected pre-grasp is roughly target_z + configured pre_grasp_z_offset. '
            f'Current test drop={self.args.drop:.3f} m.'
        )
        return True

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        if elapsed > self.args.duration:
            self.get_logger().info('Test duration reached. Stop publishing fake target.')
            rclpy.shutdown()
            return

        if not self.compute_target_once():
            return

        msg = VisualTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.target_id = 'fake_block_001'
        msg.object_name = 'fake_block'

        msg.x = float(self.target_xyz[0])
        msg.y = float(self.target_xyz[1])
        msg.z = float(self.target_xyz[2])

        msg.confidence = 0.95
        msg.is_stable = True

        msg.u = 320.0
        msg.v = 240.0
        msg.depth = max(0.01, float(self.target_xyz[2]))

        msg.image_width = 640
        msg.image_height = 480

        self.target_pub.publish(msg)

        self.get_logger().info(
            f'Published fake target: {self.format_xyz(self.target_xyz)} '
            f'elapsed={elapsed:.1f}s',
            throttle_duration_sec=2.0,
        )

    @staticmethod
    def format_xyz(xyz: list) -> str:
        return f'({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})'


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--x',
        type=float,
        default=None,
        help='Manual target x in base_link. If set, --y and --z must also be set.',
    )
    parser.add_argument(
        '--y',
        type=float,
        default=None,
        help='Manual target y in base_link. If set, --x and --z must also be set.',
    )
    parser.add_argument(
        '--z',
        type=float,
        default=None,
        help='Manual target z in base_link. If set, --x and --y must also be set.',
    )

    parser.add_argument(
        '--dx',
        type=float,
        default=0.0,
        help='Auto target x offset from current end_pose.',
    )
    parser.add_argument(
        '--dy',
        type=float,
        default=0.0,
        help='Auto target y offset from current end_pose.',
    )
    parser.add_argument(
        '--drop',
        type=float,
        default=0.08,
        help='Auto target z drop from current end_pose. Use 0.06~0.08 for first real-arm test.',
    )
    parser.add_argument(
        '--min-z',
        type=float,
        default=0.08,
        help='Safety minimum target z.',
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=10.0,
        help='Publish rate in Hz.',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=90.0,
        help='Publish duration in seconds.',
    )

    args = parser.parse_args()

    manual_values = [args.x is not None, args.y is not None, args.z is not None]
    if any(manual_values) and not all(manual_values):
        raise SystemExit('Manual mode requires --x, --y, and --z together.')

    return args


def main():
    args = parse_args()
    rclpy.init()
    node = FakeVisualTargetPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
