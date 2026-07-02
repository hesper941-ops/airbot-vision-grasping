#!/usr/bin/env python3
"""Publish configured standby or ready joint poses through the arm executor."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class PoseSwitchNode(Node):
    """Translate named pose commands into joint targets without calling the SDK."""

    def __init__(self):
        super().__init__('pose_switch_node')

        self.declare_parameter(
            'pose_switch_cmd_topic', '/robot_arm/pose_switch_cmd')
        self.declare_parameter(
            'standby_joint_pos_deg',
            [0.0, -45.0, 110.0, -90.0, 90.0, 0.0])
        self.declare_parameter(
            'ready_grasp_joint_pos_deg',
            [0.0, -45.0, 110.0, -90.0, 90.0, 0.0])

        self.executor_status = 'UNKNOWN'
        command_topic = str(
            self.get_parameter('pose_switch_cmd_topic').value)

        self.target_joint_pub = self.create_publisher(
            Float64MultiArray, '/robot_arm/target_joint', 10)
        self.command_sub = self.create_subscription(
            String, command_topic, self.command_callback, 10)
        self.executor_status_sub = self.create_subscription(
            String,
            '/robot_arm/executor_status',
            self.executor_status_callback,
            10,
        )

        self.get_logger().info(
            f'PoseSwitchNode ready: command_topic={command_topic}, '
            'target_topic=/robot_arm/target_joint.')

    def executor_status_callback(self, msg: String):
        self.executor_status = msg.data.strip().upper()

    def command_callback(self, msg: String):
        raw_command = msg.data
        command = raw_command.strip().lower()
        self.get_logger().info(
            f'Pose switch command received: raw={raw_command!r}.')

        resolved = self._resolve_pose(command)
        if resolved is None:
            self.get_logger().warning(
                f'Unsupported pose switch command: raw={raw_command!r}. '
                'Expected standby/idle or ready/pre_grasp.')
            return

        pose_name, parameter_name = resolved
        target_deg = [
            float(value)
            for value in self.get_parameter(parameter_name).value
        ]
        self.get_logger().info(
            f'Pose switch resolved: pose={pose_name}, '
            f'target_joint_deg={target_deg}.')

        if self.executor_status in ('BUSY', 'ERROR'):
            self.get_logger().warning(
                f'Pose switch rejected: executor_status={self.executor_status}, '
                f'pose={pose_name}, raw={raw_command!r}.')
            return

        if len(target_deg) != 6:
            self.get_logger().error(
                f'Pose switch rejected: {parameter_name} has '
                f'{len(target_deg)} values, expected 6.')
            return

        target_rad = [math.radians(value) for value in target_deg]
        command_msg = Float64MultiArray()
        command_msg.data = target_rad
        self.target_joint_pub.publish(command_msg)
        self.get_logger().info(
            'Pose switch published to /robot_arm/target_joint: '
            f'pose={pose_name}, target_joint_rad={target_rad}.')

    @staticmethod
    def _resolve_pose(command: str):
        if command in ('standby', 'idle'):
            return 'standby', 'standby_joint_pos_deg'
        if command in ('ready', 'pre_grasp'):
            return 'ready_grasp', 'ready_grasp_joint_pos_deg'
        return None


def main(args=None):
    rclpy.init(args=args)
    node = PoseSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
