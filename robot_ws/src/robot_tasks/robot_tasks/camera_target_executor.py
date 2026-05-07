#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from airbot_py.arm import AIRBOTPlay, RobotMode, SpeedProfile


class CameraTargetExecutor(Node):
    def __init__(self):
        super().__init__('camera_target_executor')

        self.robot_url = 'localhost'
        self.robot_port = 50001
        self.busy = False
        self.motion_sleep = 0.35

        self.x_min = 0.10
        self.x_max = 1.00
        self.y_min = -0.45
        self.y_max = 0.50
        self.z_min = 0.02
        self.z_max = 0.70

        self.sub = self.create_subscription(
            PointStamped,
            '/camera_target_base',
            self.target_callback,
            10
        )

        self.get_logger().info('CameraTargetExecutor started.')
        self.get_logger().info('Subscribed topic: /camera_target_base')
        self.get_logger().info('Expected frame_id: base_link')

    def in_workspace(self, x, y, z):
        return (
            self.x_min <= x <= self.x_max and
            self.y_min <= y <= self.y_max and
            self.z_min <= z <= self.z_max
        )

    def target_callback(self, msg: PointStamped):
        if self.busy:
            self.get_logger().warning('Robot is busy, skip this target.')
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

        self.busy = True
        threading.Thread(
            target=self.execute_target,
            args=(target,),
            daemon=True
        ).start()

    def execute_target(self, target):
        robot = None
        try:
            self.get_logger().info(
                f'Move to target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})'
            )

            robot = AIRBOTPlay(url=self.robot_url, port=self.robot_port)
            robot.connect()
            robot.set_speed_profile(SpeedProfile.SLOW)

            pose = robot.get_end_pose()
            if pose is None or len(pose) < 2:
                raise RuntimeError('Failed to read current end pose.')

            current_quat = list(pose[1])

            robot.switch_mode(RobotMode.PLANNING_WAYPOINTS)
            robot.move_with_cart_waypoints([
                [target, current_quat],
            ])
            time.sleep(self.motion_sleep)

            self.get_logger().info('Target move executed.')

        except Exception as e:
            self.get_logger().error(f'Failed to execute target move: {e}')
        finally:
            if robot is not None:
                try:
                    robot.disconnect()
                except Exception:
                    pass
            self.busy = False


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
