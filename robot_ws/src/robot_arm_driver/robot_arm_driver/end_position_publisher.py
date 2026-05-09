#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_arm_interface.airbot_wrapper import AirbotWrapper

# 机械臂末端位姿发布节点，发布 /robot_arm/end_pose (PoseStamped)，frame_id 是 base_link，频率 10Hz，供任务层和坐标变换使用。
class EndPosePublisher(Node):
    def __init__(self):
        super().__init__('end_pose_publisher')

        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/robot_arm/end_pose',
            10
        )

        self.robot = AirbotWrapper(url="localhost", port=50001)
        self.connected = False

        try:
            self.robot.connect(speed_profile='default')
            self.connected = True
            self.get_logger().info('Connected to robot server through AirbotWrapper.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot server: {e}')

        self.timer = self.create_timer(0.1, self.publish_end_pose)
        self.get_logger().info('End pose publisher started.')

    def publish_end_pose(self):
        if not self.connected:
            return

        try:
            pose = self.robot.get_end_pose()
            if pose is None or len(pose) < 2:
                raise RuntimeError('Failed to get valid end pose.')

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
