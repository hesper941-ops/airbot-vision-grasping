#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import json
import math
import time
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from robot_msgs.msg import VisualTarget


class GraspCommandBridge(Node):
    """
    Listen to high-level JSON command from /robot_command.

    Example:
    [{"actuator":"机械臂","action":"抓取","params":{"target":"小黄鸭"}}]

    Then use the latest /visual_target_base_candidate and publish /visual_target_base
    to trigger the existing grasp_task_open_loop.
    """

    def __init__(self):
        super().__init__('grasp_command_bridge')

        self.declare_parameter('command_topic', '/robot_command')
        self.declare_parameter('candidate_topic', '/visual_target_base_candidate')
        self.declare_parameter('trigger_topic', '/visual_target_base')
        self.declare_parameter('feedback_topic', '/robot_command_status')

        self.declare_parameter('min_confidence', 0.70)
        self.declare_parameter('target_timeout_sec', 1.0)
        self.declare_parameter('publish_count', 10)
        self.declare_parameter('publish_hz', 5.0)
        self.declare_parameter('cooldown_sec', 5.0)

        self.declare_parameter('front_grasp_x_offset', 0.065)
        self.declare_parameter('grasp_z_offset', 0.02)
        self.declare_parameter('official_reach_radius_m', 0.647)

        self.command_topic = self.get_parameter('command_topic').value
        self.candidate_topic = self.get_parameter('candidate_topic').value
        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value

        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)

        self.front_grasp_x_offset = float(self.get_parameter('front_grasp_x_offset').value)
        self.grasp_z_offset = float(self.get_parameter('grasp_z_offset').value)
        self.reach_radius = float(self.get_parameter('official_reach_radius_m').value)

        self.latest_target = None
        self.latest_target_time_sec = None
        self.executor_status = 'UNKNOWN'
        self.last_trigger_wall_time = 0.0

        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            10,
        )

        self.candidate_sub = self.create_subscription(
            VisualTarget,
            self.candidate_topic,
            self.candidate_callback,
            10,
        )

        self.status_sub = self.create_subscription(
            String,
            '/robot_arm/executor_status',
            self.executor_status_callback,
            10,
        )

        self.trigger_pub = self.create_publisher(
            VisualTarget,
            self.trigger_topic,
            10,
        )

        self.feedback_pub = self.create_publisher(
            String,
            self.feedback_topic,
            10,
        )

        self.get_logger().info('grasp_command_bridge started.')
        self.get_logger().info(f'Listening command topic: {self.command_topic}')
        self.get_logger().info(f'Listening candidate topic: {self.candidate_topic}')
        self.get_logger().info(f'Publishing trigger topic: {self.trigger_topic}')
        self.get_logger().info(f'Publishing feedback topic: {self.feedback_topic}')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def feedback(self, text: str, level: str = 'info'):
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)

        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def executor_status_callback(self, msg: String):
        self.executor_status = msg.data.strip()

    def candidate_callback(self, msg: VisualTarget):
        if msg.header.frame_id.strip() != 'base_link':
            self.feedback(
                f'Ignore candidate target because frame_id={msg.header.frame_id}, expected base_link.',
                'warn',
            )
            return

        self.latest_target = copy.deepcopy(msg)
        self.latest_target_time_sec = self.now_sec()

    def normalize_name(self, name: Any) -> str:
        name = str(name).strip()

        mapping = {
            '小黄鸭': 'duck',
            '黄鸭': 'duck',
            '鸭子': 'duck',
            'duck': 'duck',

            '红苹果': 'apple',
            '苹果': 'apple',
            'apple': 'apple',

            '绿色方块': 'box',
            '绿方块': 'box',
            '方块': 'box',
            'box': 'box',
            'green_box': 'box',
        }

        return mapping.get(name, name)

    def parse_commands(self, text: str) -> List[Dict[str, Any]]:
        data = json.loads(text)

        if isinstance(data, dict):
            return [data]

        if isinstance(data, list):
            return [x for x in data if isinstance(x, dict)]

        return []

    def command_callback(self, msg: String):
        raw = msg.data.strip()
        self.feedback(f'Received /robot_command: {raw}')

        try:
            commands = self.parse_commands(raw)
        except Exception as exc:
            self.feedback(f'Invalid JSON command: {exc}', 'error')
            return

        if not commands:
            self.feedback('No valid command object found in JSON.', 'warn')
            return

        for cmd in commands:
            self.handle_one_command(cmd)

    def handle_one_command(self, cmd: Dict[str, Any]):
        actuator = str(cmd.get('actuator', '')).strip()
        action = str(cmd.get('action', '')).strip()

        params = cmd.get('params', {})
        if not isinstance(params, dict):
            params = {}

        target_name = self.normalize_name(params.get('target', ''))

        if actuator not in ['机械臂', 'arm', 'robot_arm']:
            self.feedback(f'Ignore command because actuator={actuator}', 'warn')
            return

        if action not in ['抓取', 'grasp', 'pick']:
            self.feedback(f'Ignore command because action={action}', 'warn')
            return

        if target_name not in ['duck', 'apple', 'box']:
            self.feedback(f'Unsupported target: {target_name}', 'warn')
            return

        wall_now = time.time()
        if wall_now - self.last_trigger_wall_time < self.cooldown_sec:
            self.feedback('Ignore command because bridge is in cooldown.', 'warn')
            return

        if self.executor_status == 'BUSY':
            self.feedback('Ignore command because arm executor is BUSY.', 'warn')
            return

        if self.executor_status in ['ERROR', 'REJECTED_INVALID_JOINT_LIMIT']:
            self.feedback(
                f'Ignore command because arm executor status is {self.executor_status}.',
                'error',
            )
            return

        if self.latest_target is None or self.latest_target_time_sec is None:
            self.feedback('No /visual_target_base_candidate received yet.', 'warn')
            return

        target_age = self.now_sec() - self.latest_target_time_sec
        if target_age > self.target_timeout_sec:
            self.feedback(
                f'Candidate target is too old: {target_age:.2f}s > {self.target_timeout_sec:.2f}s.',
                'warn',
            )
            return

        target = self.latest_target
        candidate_name = self.normalize_name(target.object_name)

        if candidate_name != target_name:
            self.feedback(
                f'Candidate target mismatch: command wants {target_name}, '
                f'but latest candidate is {target.object_name}.',
                'warn',
            )
            return

        if target.confidence < self.min_confidence:
            self.feedback(
                f'Candidate confidence too low: {target.confidence:.2f} < {self.min_confidence:.2f}.',
                'warn',
            )
            return

        if not target.is_stable:
            self.feedback('Candidate target is not stable yet.', 'warn')
            return

        final_x = float(target.x) + self.front_grasp_x_offset
        final_y = float(target.y)
        final_z = float(target.z) + self.grasp_z_offset
        final_radius = math.sqrt(final_x * final_x + final_y * final_y + final_z * final_z)

        if final_radius > self.reach_radius:
            self.feedback(
                f'Target out of workspace: final_radius={final_radius:.3f} m '
                f'> {self.reach_radius:.3f} m. Need chassis adjustment first.',
                'error',
            )
            return

        self.feedback(
            f'Accept grasp command: target={target_name}, '
            f'x={target.x:.3f}, y={target.y:.3f}, z={target.z:.3f}, '
            f'final_radius={final_radius:.3f}, executor_status={self.executor_status}.'
        )

        self.last_trigger_wall_time = wall_now
        self.publish_trigger_target(target)

    def publish_trigger_target(self, target: VisualTarget):
        interval = 1.0 / max(self.publish_hz, 0.1)

        for i in range(self.publish_count):
            out = copy.deepcopy(target)
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'base_link'
            self.trigger_pub.publish(out)
            time.sleep(interval)

        self.feedback(
            f'Published {self.publish_count} frames to {self.trigger_topic}. '
            f'The existing grasp_task_open_loop should start grasping.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GraspCommandBridge()

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
