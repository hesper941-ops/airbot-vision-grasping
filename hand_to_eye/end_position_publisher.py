#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from airbot_py.arm import AIRBOTPlay


class EndPosePublisher(Node):
    def __init__(self):
        super().__init__('end_pose_publisher')

        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/robot_arm/end_pose',
            10
        )

        self.robot = AIRBOTPlay(url="localhost", port=50001)
        self.connected = False

        try:
            self.robot.connect()
            self.connected = True
            self.get_logger().info('Connected to robot server.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot server: {e}')

        self.timer = self.create_timer(0.1, self.publish_end_pose)
        self.get_logger().info('End pose publisher started.')

    def publish_end_pose(self):
        if not self.connected:
            return

        try:
            pose = self.robot.get_end_pose()
            position = pose[0]
            quaternion = pose[1]

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            msg.pose.position.x = float(position[0])
            msg.pose.position.y = float(position[1])
            msg.pose.position.z = float(position[2])

            msg.pose.orientation.x = float(quaternion[0])
            msg.pose.orientation.y = float(quaternion[1])
            msg.pose.orientation.z = float(quaternion[2])
            msg.pose.orientation.w = float(quaternion[3])

            self.publisher_.publish(msg)

            self.get_logger().info(
                f'Published end pose: '
                f'pos=({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}), '
                f'quat=({msg.pose.orientation.x}, {msg.pose.orientation.y}, '
                f'{msg.pose.orientation.z}, {msg.pose.orientation.w})'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to get end pose: {e}')

    def destroy_node(self):
        if self.connected:
            try:
                self.robot.disconnect()
            except Exception as e:
                self.get_logger().warning(f'Failed to disconnect cleanly: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EndPosePublisher()
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