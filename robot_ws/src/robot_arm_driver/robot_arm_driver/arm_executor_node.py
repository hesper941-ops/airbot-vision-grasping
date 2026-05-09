#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Single-owner AIRBOT executor node.

This node is the only place that calls the AIRBOT SDK. It accepts one command
only when the executor is idle. Commands received while a motion is running are
rejected immediately and are never queued or cached for later execution.
"""

import math
import threading
from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from robot_arm_interface.airbot_wrapper import AirbotWrapper
from robot_msgs.msg import ArmJointState


class ArmExecutorNode(Node):
    """ROS topic facade around the AIRBOT SDK."""

    IDLE = 'IDLE'
    BUSY = 'BUSY'
    DONE = 'DONE'
    ERROR = 'ERROR'
    REJECTED_BUSY = 'REJECTED_BUSY'

    def __init__(self):
        super().__init__('arm_executor_node')

        self.declare_parameter('do_init', True)
        self.declare_parameter(
            'init_joint_pos_deg',
            [0.0, -45.0, 120.0, -90.0, 90.0, 90.0],
        )

        self.arm = AirbotWrapper(url='localhost', port=50001)
        self.sdk_lock = threading.Lock()
        self.status_lock = threading.Lock()
        self.executor_state = self.IDLE
        self.active_thread = None
        self.last_state_msg = None
        self.last_pose_msg = None

        self.state_pub = self.create_publisher(
            ArmJointState, '/robot_arm/joint_state', 10)
        self.end_pose_pub = self.create_publisher(
            PoseStamped, '/robot_arm/end_pose', 10)
        self.executor_status_pub = self.create_publisher(
            String, '/robot_arm/executor_status', 10)

        self.arm.connect(speed_profile='slow')
        self._publish_executor_status(self.IDLE)

        if self.get_parameter('do_init').value:
            self._move_to_init_pose()

        with self.sdk_lock:
            self.arm.set_speed_profile('default')
        self.get_logger().info(
            'Arm initialized; executor is ready and speed profile is default.')

        self.state_timer = self.create_timer(0.1, self.publish_state)

        self.joint_sub = self.create_subscription(
            Float64MultiArray,
            '/robot_arm/target_joint',
            self.joint_target_callback,
            10,
        )
        self.cart_sub = self.create_subscription(
            PointStamped,
            '/robot_arm/cart_target',
            self.cart_target_callback,
            10,
        )
        self.gripper_sub = self.create_subscription(
            String,
            '/robot_arm/gripper_cmd',
            self.gripper_callback,
            10,
        )
        self.speed_sub = self.create_subscription(
            String,
            '/robot_arm/speed_profile',
            self.speed_callback,
            10,
        )
        self.reset_sub = self.create_subscription(
            String,
            '/robot_arm/reset_executor',
            self.reset_callback,
            10,
        )

        self.get_logger().info(
            'ArmExecutorNode started. Listening on /robot_arm/target_joint, '
            '/robot_arm/cart_target, /robot_arm/gripper_cmd, /robot_arm/speed_profile, '
            '/robot_arm/reset_executor.')
        self.get_logger().info(
            'Publishing /robot_arm/joint_state, /robot_arm/end_pose, '
            '/robot_arm/executor_status.')

    def _deg2rad(self, deg: float) -> float:
        return deg * math.pi / 180.0

    def _move_to_init_pose(self):
        deg = self.get_parameter('init_joint_pos_deg').value
        rad = [self._deg2rad(v) for v in deg]
        self.get_logger().info(f'Moving to init joint pose deg={deg}')
        try:
            with self.sdk_lock:
                self._publish_executor_status(self.BUSY)
                self.arm.get_state()
                self.arm.move_joints(rad)
                self._publish_executor_status(self.DONE)
            self.get_logger().info('Init pose reached.')
        except Exception as exc:
            self._set_error(f'Init pose failed: {exc}')
        finally:
            if self._get_executor_state() != self.ERROR:
                self._publish_executor_status(self.IDLE)

    def joint_target_callback(self, msg: Float64MultiArray):
        target = [float(v) for v in msg.data]
        if len(target) != 6:
            self.get_logger().error(
                f'Invalid joint target length: {len(target)} (expected 6).')
            return
        self._try_start_command('joint', target)

    def cart_target_callback(self, msg: PointStamped):
        frame_id = msg.header.frame_id.strip()
        if frame_id and frame_id != 'base_link':
            self.get_logger().error(
                f'Invalid cart target frame_id={frame_id}; expected base_link.')
            return
        target = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]
        self._try_start_command('cartesian', target)

    def gripper_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command not in ('open', 'close'):
            self.get_logger().error(f'Unknown gripper command: {command}')
            return
        self._try_start_command('gripper', command)

    def speed_callback(self, msg: String):
        profile = msg.data.strip().lower()
        if profile not in ('slow', 'default', 'fast'):
            self.get_logger().error(f'Unknown speed profile: {profile}')
            return
        self._try_start_command('speed_profile', profile)

    def reset_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command != 'clear_error':
            self.get_logger().warning(f'Unknown reset_executor command: {command}')
            return

        with self.status_lock:
            if self.executor_state != self.ERROR:
                self.get_logger().info(
                    f'clear_error received while executor is {self.executor_state}; no state change.')
                self._publish_executor_status_locked(self.executor_state)
                return

            self.get_logger().warning('clear_error received; executor ERROR cleared to IDLE.')
            self._publish_executor_status_locked(self.IDLE)

    def _try_start_command(self, command_type: str, payload: Any):
        with self.status_lock:
            if self.executor_state == self.ERROR:
                self.get_logger().error(
                    f'Executor is ERROR; reject {command_type} command.')
                self._publish_executor_status_locked(self.ERROR)
                return

            if self.executor_state == self.BUSY:
                self.get_logger().warning(
                    f'Executor busy; reject {command_type} command: {payload}')
                self.get_logger().warning(f'Executor status: REJECTED_BUSY {command_type}')
                self._publish_executor_status_locked(self.REJECTED_BUSY)
                self._publish_executor_status_locked(self.BUSY)
                return

            self.executor_state = self.BUSY
            self.get_logger().info(f'Executor status: BUSY {command_type}')
            self._publish_executor_status_locked(self.BUSY)

        self.get_logger().info(f'Start {command_type} command: {payload}')
        thread = threading.Thread(
            target=self._execute_command,
            args=(command_type, payload),
            daemon=True,
        )
        self.active_thread = thread
        thread.start()

    def _execute_command(self, command_type: str, payload: Any):
        try:
            with self.sdk_lock:
                if command_type == 'joint':
                    self.arm.move_joints(payload)
                elif command_type == 'cartesian':
                    self.arm.move_to_cart_target_with_current_orientation(payload)
                elif command_type == 'gripper':
                    if payload == 'open':
                        self.arm.open_gripper()
                    else:
                        self.arm.close_gripper()
                elif command_type == 'speed_profile':
                    self.arm.set_speed_profile(payload)
                else:
                    raise ValueError(f'Unsupported command type: {command_type}')

            self.get_logger().info(f'{command_type} command done.')
            self.get_logger().info(f'Executor status: DONE {command_type}')
            self._publish_executor_status(self.DONE)
        except Exception as exc:
            self._set_error(f'{command_type} command failed: {exc}')
            return

        self._publish_executor_status(self.IDLE)

    def publish_state(self):
        """Publish joint state and end-effector pose if the SDK can be sampled."""
        if not self.sdk_lock.acquire(blocking=False):
            if self.last_state_msg is not None:
                busy_msg = self._copy_state_msg(self.last_state_msg)
                busy_msg.state = self.BUSY
                self.state_pub.publish(busy_msg)
            if self.last_pose_msg is not None:
                self.end_pose_pub.publish(self.last_pose_msg)
            return

        try:
            msg = ArmJointState()
            msg.state = str(self.arm.get_state())
            msg.joint_pos = list(self.arm.get_joint_pos())
            msg.joint_vel = list(self.arm.get_joint_vel())

            end_pose = self.arm.get_end_pose()
            position = list(end_pose[0])
            quaternion = list(end_pose[1])
            msg.end_pose = position + quaternion

            self.last_state_msg = self._copy_state_msg(msg)
            self.state_pub.publish(msg)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            pose_msg.pose.position.x = float(position[0])
            pose_msg.pose.position.y = float(position[1])
            pose_msg.pose.position.z = float(position[2])
            pose_msg.pose.orientation.x = float(quaternion[0])
            pose_msg.pose.orientation.y = float(quaternion[1])
            pose_msg.pose.orientation.z = float(quaternion[2])
            pose_msg.pose.orientation.w = float(quaternion[3])
            self.last_pose_msg = pose_msg
            self.end_pose_pub.publish(pose_msg)
        except Exception as exc:
            self._set_error(f'Failed to publish arm state: {exc}')
        finally:
            self.sdk_lock.release()

    def _get_executor_state(self) -> str:
        with self.status_lock:
            return self.executor_state

    def _publish_executor_status(self, status: str):
        with self.status_lock:
            self._publish_executor_status_locked(status)

    def _publish_executor_status_locked(self, status: str):
        self.executor_state = status
        msg = String()
        msg.data = status
        self.executor_status_pub.publish(msg)

    def _set_error(self, message: str):
        self.get_logger().error(message)
        self._publish_executor_status(self.ERROR)

    def _copy_state_msg(self, msg: ArmJointState) -> ArmJointState:
        copied = ArmJointState()
        copied.state = msg.state
        copied.joint_pos = list(msg.joint_pos)
        copied.joint_vel = list(msg.joint_vel)
        copied.end_pose = list(msg.end_pose)
        return copied

    def destroy_node(self):
        if self.active_thread is not None and self.active_thread.is_alive():
            self.active_thread.join(timeout=1.0)
        if self.sdk_lock.acquire(timeout=1.0):
            try:
                self.arm.disconnect()
            except Exception:
                pass
            finally:
                self.sdk_lock.release()
        else:
            self.get_logger().warning(
                'SDK command still running; skip disconnect on shutdown.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmExecutorNode()
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
