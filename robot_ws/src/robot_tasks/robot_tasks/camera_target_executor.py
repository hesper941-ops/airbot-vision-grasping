#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from robot_msgs.msg import ArmCommand, ArmState


class CameraTargetExecutor(Node):
    def __init__(self):
        super().__init__('camera_target_executor')

        self.x_min = 0.10
        self.x_max = 1.00
        self.y_min = -0.45
        self.y_max = 0.50
        self.z_min = 0.02
        self.z_max = 0.70

        self.waiting_for_result = False
        self.pending_target = None
        self.arm_busy = False

        self.cmd_pub = self.create_publisher(
            ArmCommand,
            '/robot_arm/cmd',
            10
        )

        self.target_sub = self.create_subscription(
            PointStamped,
            '/camera_target_base',
            self.target_callback,
            10
        )

        self.state_sub = self.create_subscription(
            ArmState,
            '/robot_arm/state',
            self.state_callback,
            10
        )

        self.get_logger().info('CameraTargetExecutor started.')
        self.get_logger().info('Subscribed topics: /camera_target_base, /robot_arm/state')
        self.get_logger().info('Publishing topic: /robot_arm/cmd')
        self.get_logger().info('Expected target frame_id: base_link')

    def in_workspace(self, x, y, z):
        return (
            self.x_min <= x <= self.x_max and
            self.y_min <= y <= self.y_max and
            self.z_min <= z <= self.z_max
        )

    def target_callback(self, msg: PointStamped):
        if self.waiting_for_result or self.arm_busy:
            self.get_logger().warning('Executor is waiting or arm is busy, skip this target.')
            return

        frame_id = msg.header.frame_id.strip()
        if frame_id and frame_id != 'base_link':
            self.get_logger().error(f'Invalid frame_id: {frame_id}, expected base_link')
            return

        target = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]

        if not self.in_workspace(target[0], target[1], target[2]):
            self.get_logger().error(
                'Target is out of workspace: '
                f'({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})'
            )
            return

        cmd = ArmCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.command_type = 'MOVE_CART_KEEP_ORI'
        cmd.cartesian_position = target
        cmd.keep_current_orientation = True
        cmd.source = 'camera_target_executor'

        self.cmd_pub.publish(cmd)
        self.pending_target = target
        self.waiting_for_result = True

        self.get_logger().info(
            f'Dispatched target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})'
        )

    def state_callback(self, msg: ArmState):
        self.arm_busy = bool(msg.busy)

        if not self.waiting_for_result:
            return

        if msg.last_command != 'MOVE_CART_KEEP_ORI':
            return

        if msg.busy:
            return

        target = self.pending_target
        if target is None:
            self.waiting_for_result = False
            return

        if msg.success:
            self.get_logger().info(
                f'Target move succeeded: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})'
            )
        else:
            self.get_logger().error(
                f'Target move failed: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) | '
                f'error={msg.error_message}'
            )

        self.pending_target = None
        self.waiting_for_result = False


def main(args=None):
    rclpy.init(args=args)
    node = CameraTargetExecutor()
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
