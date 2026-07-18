#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish chassis requests based on arm-side visual target state.

Inputs:
  /visual_target_base (robot_msgs/msg/VisualTarget, base_link)
  /arm_task/status (std_msgs/msg/String)
  /robot_arm/executor_status (std_msgs/msg/String)
  Graspable detector PointStamped topics

Outputs:
  /chassis/approach_request (std_msgs/msg/String, JSON)
  /chassis/no_graspable_objects (std_msgs/msg/String, JSON)

This node is intentionally separate from the arm grasp state machine. It keeps
the reach decision on the camera/arm X5, while the chassis X5 only receives a
small motion request such as "move forward 0.25 m".
"""

import json
import math
from typing import Dict

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String

from robot_msgs.msg import VisualTarget


class ChassisApproachPublisher(Node):
    """Convert far visual targets into chassis movement requests."""

    def __init__(self):
        super().__init__('chassis_approach_publisher')

        self._declare_parameters()
        self._load_parameters()

        self.last_publish_time_by_object: Dict[str, float] = {}
        self.last_graspable_seen_time_by_object: Dict[str, float] = {}
        self.last_done_status_time_sec = None
        self.no_object_reported_for_last_done = False
        self.executor_seen_busy = False
        self.sequence = 0
        self.no_object_sequence = 0

        self.target_sub = self.create_subscription(
            VisualTarget,
            self.input_topic,
            self.target_callback,
            10,
        )
        self.status_sub = self.create_subscription(
            String,
            self.arm_task_status_topic,
            self.arm_task_status_callback,
            10,
        )
        self.executor_status_sub = self.create_subscription(
            String,
            self.executor_status_topic,
            self.executor_status_callback,
            10,
        )
        self._create_graspable_detector_subscriptions()

        self.request_pub = self.create_publisher(
            String,
            self.output_topic,
            10,
        )
        self.no_object_pub = self.create_publisher(
            String,
            self.no_graspable_output_topic,
            10,
        )
        self.no_object_timer = self.create_timer(0.2, self.check_post_grasp_scene)

        self.get_logger().info(
            f'Listening: {self.input_topic} (robot_msgs/msg/VisualTarget)'
        )
        self.get_logger().info(
            f'Listening: {self.arm_task_status_topic} (std_msgs/msg/String)'
        )
        self.get_logger().info(
            f'Listening: {self.executor_status_topic} (std_msgs/msg/String)'
        )
        self.get_logger().info(
            f'Publishing: {self.output_topic} (std_msgs/msg/String JSON)'
        )
        self.get_logger().info(
            f'Publishing: {self.no_graspable_output_topic} (std_msgs/msg/String JSON)'
        )
        self.get_logger().info(
            'Reach config: '
            f'arm_reach_radius_m={self.arm_reach_radius_m:.3f}, '
            f'workspace_x=[{self.workspace_x_min_m:.3f}, {self.workspace_x_max_m:.3f}], '
            f'workspace_y_abs_m={self.workspace_y_abs_max_m:.3f}, '
            f'desired_target_x_m={self.desired_target_x_m:.3f}'
        )

    def _declare_parameters(self):
        self.declare_parameter('input_topic', '/visual_target_base')
        self.declare_parameter('output_topic', '/chassis/approach_request')
        self.declare_parameter('arm_task_status_topic', '/arm_task/status')
        self.declare_parameter('executor_status_topic', '/robot_arm/executor_status')
        self.declare_parameter('no_graspable_output_topic', '/chassis/no_graspable_objects')

        # Match the current open_loop_grasp.yaml defaults on the robot.
        self.declare_parameter('arm_reach_radius_m', 0.68)
        self.declare_parameter('workspace_x_min_m', 0.10)
        self.declare_parameter('workspace_x_max_m', 0.68)
        self.declare_parameter('workspace_y_abs_max_m', 0.38)
        self.declare_parameter('workspace_z_min_m', 0.02)
        self.declare_parameter('workspace_z_max_m', 0.75)

        # Where the target should end up after the chassis moves.
        self.declare_parameter('desired_target_x_m', 0.45)
        self.declare_parameter('too_far_margin_m', 0.03)
        self.declare_parameter('min_move_forward_m', 0.03)
        self.declare_parameter('max_move_forward_m', 0.50)
        self.declare_parameter('max_move_left_m', 0.30)

        self.declare_parameter('min_confidence', 0.70)
        self.declare_parameter('require_stable_target', True)
        self.declare_parameter('publish_cooldown_sec', 2.0)
        self.declare_parameter('include_lateral_request', True)

        self.declare_parameter('post_grasp_observe_delay_sec', 2.0)
        self.declare_parameter('no_graspable_absence_sec', 1.5)
        self.declare_parameter('graspable_detector_topics', [
            '/duck_position:duck',
            '/box_position:box',
            '/red_circle_position:red_circle',
            '/detect_yolo/apple_position:apple',
            '/detect_yolo/banana_position:banana',
            '/detect_yolo/bottle_position:bottle',
            '/detect_yolo/cake_position:cake',
        ])

    def _load_parameters(self):
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.arm_task_status_topic = str(self.get_parameter('arm_task_status_topic').value)
        self.executor_status_topic = str(self.get_parameter('executor_status_topic').value)
        self.no_graspable_output_topic = str(
            self.get_parameter('no_graspable_output_topic').value)
        self.arm_reach_radius_m = float(self.get_parameter('arm_reach_radius_m').value)
        self.workspace_x_min_m = float(self.get_parameter('workspace_x_min_m').value)
        self.workspace_x_max_m = float(self.get_parameter('workspace_x_max_m').value)
        self.workspace_y_abs_max_m = float(self.get_parameter('workspace_y_abs_max_m').value)
        self.workspace_z_min_m = float(self.get_parameter('workspace_z_min_m').value)
        self.workspace_z_max_m = float(self.get_parameter('workspace_z_max_m').value)
        self.desired_target_x_m = float(self.get_parameter('desired_target_x_m').value)
        self.too_far_margin_m = float(self.get_parameter('too_far_margin_m').value)
        self.min_move_forward_m = float(self.get_parameter('min_move_forward_m').value)
        self.max_move_forward_m = float(self.get_parameter('max_move_forward_m').value)
        self.max_move_left_m = float(self.get_parameter('max_move_left_m').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.require_stable_target = bool(self.get_parameter('require_stable_target').value)
        self.publish_cooldown_sec = float(self.get_parameter('publish_cooldown_sec').value)
        self.include_lateral_request = bool(self.get_parameter('include_lateral_request').value)
        self.post_grasp_observe_delay_sec = float(
            self.get_parameter('post_grasp_observe_delay_sec').value)
        self.no_graspable_absence_sec = float(
            self.get_parameter('no_graspable_absence_sec').value)
        raw_topics = self.get_parameter('graspable_detector_topics').value
        self.graspable_detector_topics = self._parse_graspable_detector_topics(raw_topics)

    def _create_graspable_detector_subscriptions(self):
        self.detector_subs = []
        for topic, object_name in self.graspable_detector_topics:
            self.detector_subs.append(
                self.create_subscription(
                    PointStamped,
                    topic,
                    lambda msg, name=object_name: self.graspable_detector_callback(msg, name),
                    10,
                )
            )

    @staticmethod
    def _parse_graspable_detector_topics(raw_topics):
        parsed = []
        for item in raw_topics:
            text = str(item).strip()
            if not text:
                continue
            if ':' in text:
                topic, object_name = text.split(':', 1)
            else:
                topic = text
                object_name = text.rsplit('/', 1)[-1].replace('_position', '')
            parsed.append((topic.strip(), object_name.strip()))
        return parsed

    def target_callback(self, msg: VisualTarget):
        if msg.object_name:
            self.last_graspable_seen_time_by_object[msg.object_name] = self.now_sec()
        if msg.confidence < self.min_confidence:
            return
        if self.require_stable_target and not bool(msg.is_stable):
            return

        x = float(msg.x)
        y = float(msg.y)
        z = float(msg.z)
        distance_xy = math.hypot(x, y)

        if self._target_is_in_arm_workspace(x, y, z, distance_xy):
            return

        move_forward_m = self._clamp(
            x - self.desired_target_x_m,
            0.0,
            self.max_move_forward_m,
        )
        if move_forward_m < self.min_move_forward_m:
            return

        object_key = msg.object_name or msg.target_id or 'unknown'
        now_sec = self.now_sec()
        last_publish = self.last_publish_time_by_object.get(object_key)
        if last_publish is not None and now_sec - last_publish < self.publish_cooldown_sec:
            return

        self.sequence += 1
        self.last_publish_time_by_object[object_key] = now_sec

        move_left_m = self._clamp(
            y,
            -self.max_move_left_m,
            self.max_move_left_m,
        ) if self.include_lateral_request else 0.0

        payload = {
            'version': 1,
            'request_id': f'chassis_approach_{self.sequence}',
            'target_id': msg.target_id,
            'object_name': msg.object_name,
            'frame_id': msg.header.frame_id,
            'target_x_m': x,
            'target_y_m': y,
            'target_z_m': z,
            'distance_xy_m': distance_xy,
            'arm_reach_radius_m': self.arm_reach_radius_m,
            'workspace_x_max_m': self.workspace_x_max_m,
            'workspace_y_abs_max_m': self.workspace_y_abs_max_m,
            'desired_target_x_m': self.desired_target_x_m,
            'move_forward_m': move_forward_m,
            'move_left_m': move_left_m,
            'confidence': float(msg.confidence),
            'reason': 'target_outside_arm_workspace',
        }

        out = String()
        out.data = json.dumps(payload, separators=(',', ':'))
        self.request_pub.publish(out)

        self.get_logger().info(
            f'Approach request: object={payload["object_name"]!r}, '
            f'target=({x:.3f}, {y:.3f}, {z:.3f}), '
            f'forward={move_forward_m:.3f} m, left={move_left_m:.3f} m'
        )

    def graspable_detector_callback(self, msg: PointStamped, object_name: str):
        del msg
        self.last_graspable_seen_time_by_object[object_name] = self.now_sec()

    def arm_task_status_callback(self, msg: String):
        status = msg.data.strip()
        if not status.startswith('DONE'):
            return
        self.mark_arm_done('/arm_task/status', status)

    def executor_status_callback(self, msg: String):
        status = msg.data.strip().upper()
        if status == 'BUSY':
            self.executor_seen_busy = True
            return

        if self.executor_seen_busy and status in ('DONE', 'IDLE'):
            self.executor_seen_busy = False
            self.mark_arm_done('/robot_arm/executor_status', status)

    def mark_arm_done(self, source_topic: str, status: str):
        self.last_done_status_time_sec = self.now_sec()
        self.no_object_reported_for_last_done = False
        self.get_logger().info(
            f'Arm task done; will check graspable objects after '
            f'{self.post_grasp_observe_delay_sec:.1f}s. source={source_topic}, status={status!r}'
        )

    def check_post_grasp_scene(self):
        if self.last_done_status_time_sec is None:
            return
        if self.no_object_reported_for_last_done:
            return

        now = self.now_sec()
        done_age = now - self.last_done_status_time_sec
        if done_age < self.post_grasp_observe_delay_sec:
            return

        latest_seen = self.latest_graspable_seen_time()
        if latest_seen is not None and latest_seen >= self.last_done_status_time_sec:
            latest_age = now - latest_seen
            if latest_age < self.no_graspable_absence_sec:
                return

        self.no_object_sequence += 1
        self.no_object_reported_for_last_done = True

        payload = {
            'version': 1,
            'event': 'no_graspable_objects_after_grasp',
            'request_id': f'no_graspable_{self.no_object_sequence}',
            'frame_id': 'camera',
            'arm_task_done_age_sec': round(done_age, 3),
            'no_graspable_absence_sec': self.no_graspable_absence_sec,
            'post_grasp_observe_delay_sec': self.post_grasp_observe_delay_sec,
            'known_graspable_objects': sorted(self.last_graspable_seen_time_by_object.keys()),
            'reason': 'post_grasp_camera_scene_empty',
        }

        out = String()
        out.data = json.dumps(payload, separators=(',', ':'))
        self.no_object_pub.publish(out)
        self.get_logger().info(f'No graspable objects after grasp: {out.data}')

    def latest_graspable_seen_time(self):
        if not self.last_graspable_seen_time_by_object:
            return None
        return max(self.last_graspable_seen_time_by_object.values())

    def _target_is_in_arm_workspace(self, x: float, y: float, z: float, distance_xy: float) -> bool:
        if distance_xy > self.arm_reach_radius_m + self.too_far_margin_m:
            return False
        if x < self.workspace_x_min_m or x > self.workspace_x_max_m + self.too_far_margin_m:
            return False
        if abs(y) > self.workspace_y_abs_max_m:
            return False
        if z < self.workspace_z_min_m or z > self.workspace_z_max_m:
            return False
        return True

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = ChassisApproachPublisher()
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
