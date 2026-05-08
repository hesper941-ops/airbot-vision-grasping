#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import deque
from datetime import datetime, timedelta

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from robot_msgs.msg import CameraTarget, ArmJointState


class CameraTargetExecutor(Node):
    """Task layer node for closed-loop camera-driven grasp steps.

    This node only builds and publishes commands to the execution layer.
    It must not directly call the AIRBOT SDK.
    """

    def __init__(self):
        super().__init__('camera_target_executor')

        # Task execution state machine variables
        self.task_state = 'WAIT_TARGET'
        self.current_stage = None
        self.latest_target = None
        self.current_target = None
        self.target_history = deque(maxlen=5)
        self.last_end_pose = None
        self.last_command_target = None
        self.last_command_time = None
        self.gripper_closed = False
        self.wait_timeout = timedelta(seconds=10)

        # Simple Cartesian workspace limits for incoming visual targets
        self.x_min = 0.10
        self.x_max = 1.00
        self.y_min = -0.45
        self.y_max = 0.50
        self.z_min = 0.02
        self.z_max = 0.70

        # Decision thresholds for confidence and drift
        self.confidence_high = 0.7
        self.confidence_low = 0.3
        self.drift_threshold = 0.05
        self.stable_count_required = 3
        self.max_cartesian_step = 0.05

        # Subscribe to vision target and arm state
        self.target_sub = self.create_subscription(
            CameraTarget,
            '/camera_target_base',
            self.target_callback,
            10
        )

        self.state_sub = self.create_subscription(
            ArmJointState,
            '/robot_arm/joint_state',
            self.state_callback,
            10
        )

        # Publish step commands and gripper commands
        self.cart_pub = self.create_publisher(
            PointStamped,
            '/robot_arm/cart_target',
            10
        )

        self.gripper_pub = self.create_publisher(
            String,
            '/robot_arm/gripper_cmd',
            10
        )

        self.timer = self.create_timer(0.5, self.step_loop)

        self.get_logger().info('CameraTargetExecutor started.')
        self.get_logger().info('Subscribed topic: /camera_target_base')
        self.get_logger().info('Expected frame_id: base_link')

    def in_workspace(self, x, y, z):
        """Check whether a target is within a safe Cartesian box."""
        return (
            self.x_min <= x <= self.x_max and
            self.y_min <= y <= self.y_max and
            self.z_min <= z <= self.z_max
        )

    def target_callback(self, msg: CameraTarget):
        """Handle incoming visual targets and update task state.

        This callback validates the frame and workspace, stores the target,
        and starts planning when the task is idle.
        """
        frame_id = msg.header.frame_id.strip()
        if frame_id and frame_id != 'base_link':
            self.get_logger().error(f'Invalid frame_id: {frame_id}, expected base_link')
            return

        if not self.in_workspace(msg.x, msg.y, msg.z):
            self.get_logger().error(
                'Target is out of workspace: '
                f'({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})'
            )
            return

        self.latest_target = msg
        self.target_history.append(msg)

        if msg.confidence < self.confidence_low:
            self.get_logger().warning(
                f'Low confidence target received ({msg.confidence:.2f}), waiting for better input.'
            )
            return

        self.get_logger().info(
            f'Received camera target: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) '
            f'confidence={msg.confidence:.2f} stable={msg.is_stable}'
        )

        if self.task_state in ['WAIT_TARGET', 'IDLE']:
            self.task_state = 'PLAN_PATH'

    def state_callback(self, msg: ArmJointState):
        """Store the most recent end pose from the arm state feed."""
        if msg.end_pose and len(msg.end_pose) >= 3:
            self.last_end_pose = [msg.end_pose[0], msg.end_pose[1], msg.end_pose[2]]

    def step_loop(self):
        """Scheduled state machine loop for planning and evaluating steps."""
        if self.task_state == 'WAIT_TARGET':
            return

        if self.task_state == 'PLAN_PATH':
            self.plan_path()
            return

        if self.task_state == 'WAIT_VISUAL_UPDATE':
            self.evaluate_visual_update()
            return

        if self.task_state == 'REPLAN':
            self.get_logger().info('Replanning based on latest visual input.')
            self.current_stage = None
            self.task_state = 'PLAN_PATH'
            return

        if self.task_state == 'ABORT':
            self.get_logger().error('Aborting current target and returning to WAIT_TARGET.')
            self.reset_task()
            return

    def plan_path(self):
        """Plan the next small step based on the latest stable target."""
        if self.latest_target is None:
            self.get_logger().info('No target available to plan.')
            self.task_state = 'WAIT_TARGET'
            return

        if not self._is_target_stable():
            self.get_logger().info('Target not yet stable, waiting for more visual updates.')
            self.task_state = 'WAIT_TARGET'
            return

        if self.latest_target.confidence < self.confidence_low:
            self.get_logger().warning('Target confidence too low for planning.')
            self.task_state = 'WAIT_TARGET'
            return

        if self.current_target is None or self._target_drift(self.current_target, self.latest_target) > self.drift_threshold:
            self.current_target = self.latest_target
            self.current_stage = 'PREGRASP'
            self.get_logger().info('Accepted new target and reset stage to PREGRASP.')

        step_target = self._choose_next_step(self.current_target)
        if self.task_state == 'WAIT_TARGET':
            return
        if step_target is None:
            self.get_logger().info('Current target reached or no step required.')
            self.task_state = 'WAIT_VISUAL_UPDATE'
            return

        self.publish_cart_step(step_target)
        self.last_command_target = step_target
        self.last_command_time = datetime.now()
        self.task_state = 'WAIT_VISUAL_UPDATE'

    def evaluate_visual_update(self):
        """Decide the next action after issuing a step command."""
        if self.last_command_target is None:
            self.task_state = 'PLAN_PATH'
            return

        if self.last_end_pose is not None and self._distance(self.last_end_pose, self.last_command_target) < 0.04:
            self.get_logger().info('Observed robot reached the last commanded step.')
            if self.current_stage == 'FINAL_APPROACH':
                self.current_stage = 'CLOSE_GRIPPER'
            else:
                self.current_stage = self._next_stage(self.current_stage)
            self.task_state = 'PLAN_PATH'
            return

        if self.latest_target is None:
            self.get_logger().info('No fresh visual input received yet.')
            if self.last_command_time and datetime.now() - self.last_command_time > self.wait_timeout:
                self.get_logger().warning('Visual update timeout, aborting current target.')
                self.task_state = 'ABORT'
            return

        if self._target_drift(self.current_target, self.latest_target) > self.drift_threshold:
            self.get_logger().warning('Target drift too large, replanning.')
            self.task_state = 'REPLAN'
            return

        if self.latest_target.confidence < self.confidence_low:
            self.get_logger().warning('Confidence dropped after step, waiting for better input.')
            self.task_state = 'WAIT_TARGET'
            return

        if self.latest_target.confidence < self.confidence_high:
            self.get_logger().info('Medium confidence, delaying next step until better visual feedback.')
            self.task_state = 'WAIT_TARGET'
            return

    def _choose_next_step(self, target: CameraTarget):
        """Choose the next small Cartesian target for the current stage."""
        if self.last_end_pose is None:
            self.get_logger().warning('No current end pose available, cannot choose next step.')
            return None

        current = self.last_end_pose
        pregrasp = [target.x, target.y, target.z + 0.12]
        align = [target.x, target.y, target.z + 0.06]
        final = [target.x, target.y, target.z]

        if self.current_stage == 'PREGRASP':
            if self._distance(current, pregrasp) > 0.03:
                return self._limit_step(current, pregrasp)
            self.get_logger().info('PREGRASP reached, moving to ALIGN stage.')
            self.current_stage = 'ALIGN'

        if self.current_stage == 'ALIGN':
            if self._distance(current, align) > 0.03:
                return self._limit_step(current, align)
            self.get_logger().info('ALIGN reached, moving to FINAL_APPROACH stage.')
            self.current_stage = 'FINAL_APPROACH'

        if self.current_stage == 'FINAL_APPROACH':
            if self._distance(current, final) > 0.02:
                return self._limit_step(current, final)
            self.get_logger().info('FINAL_APPROACH reached, target is in contact region.')
            return None

        if self.current_stage == 'CLOSE_GRIPPER':
            if not self.gripper_closed:
                self.publish_gripper_command('close')
                self.gripper_closed = True
                self.get_logger().info('Gripper close command published, task complete.')
            self.reset_task()
            return None

        return None

    def _limit_step(self, current, target):
        """Clamp a desired target to one small decision step."""
        distance = self._distance(current, target)
        if distance <= self.max_cartesian_step:
            return target

        ratio = self.max_cartesian_step / distance
        step = [
            current[0] + (target[0] - current[0]) * ratio,
            current[1] + (target[1] - current[1]) * ratio,
            current[2] + (target[2] - current[2]) * ratio,
        ]
        self.get_logger().info(
            f'Clamped cartesian step from {distance:.3f} m to {self.max_cartesian_step:.3f} m.'
        )
        return step

    def publish_cart_step(self, step_target):
        """Publish one small Cartesian step command to the execution layer."""
        msg = PointStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(step_target[0])
        msg.point.y = float(step_target[1])
        msg.point.z = float(step_target[2])
        self.cart_pub.publish(msg)
        self.get_logger().info(
            f'Published cartesian step to /robot_arm/cart_target: '
            f'({step_target[0]:.3f}, {step_target[1]:.3f}, {step_target[2]:.3f})'
        )

    def publish_gripper_command(self, command):
        """Publish a simple gripper command to the execution layer."""
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

    def _is_target_stable(self):
        """Check whether the recent target sequence is stable enough to start planning."""
        if len(self.target_history) < self.stable_count_required:
            return False

        confidences = [msg.confidence for msg in self.target_history]
        if min(confidences) < self.confidence_low:
            return False

        recent = list(self.target_history)
        drift = max(self._target_drift(recent[i], recent[i + 1]) for i in range(len(recent) - 1))
        return drift < self.drift_threshold

    def _target_drift(self, a: CameraTarget, b: CameraTarget):
        """Compute Euclidean drift between two target positions."""
        return self._distance([a.x, a.y, a.z], [b.x, b.y, b.z])

    def _distance(self, a, b):
        """Compute Euclidean distance between two 3D points."""
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5

    def _next_stage(self, current_stage):
        """Return the next logical grasp stage."""
        if current_stage == 'PREGRASP':
            return 'ALIGN'
        if current_stage == 'ALIGN':
            return 'FINAL_APPROACH'
        if current_stage == 'FINAL_APPROACH':
            return 'CLOSE_GRIPPER'
        return 'FINAL_APPROACH'

    def reset_task(self):
        """Reset the task state machine after aborting or finishing."""
        self.task_state = 'WAIT_TARGET'
        self.current_stage = None
        self.current_target = None
        self.last_command_target = None
        self.last_command_time = None
        self.gripper_closed = False
        self.target_history.clear()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTargetExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
