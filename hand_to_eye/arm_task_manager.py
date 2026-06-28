#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the requested visual detector and grasp chain from /command.

Expected command payload:
  [{"actuator":"机械臂","action":"抓取","params":{"target":"苹果"}}]

The node is intended to run on the vision/arm RDK X5 after:
  1. airbot_server is running on can1
  2. the arm has been moved to the lower home pose
  3. the Orbbec camera is already running
"""

import json
import os
import shlex
import signal
import subprocess
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def shell_join(parts):
    return " && ".join(parts)


@dataclass(frozen=True)
class TargetSpec:
    label: str
    mode: str
    object_name: str
    topic: str
    yolo_class: str = ""
    detector_node: str = ""


TARGETS: Dict[str, TargetSpec] = {
    "苹果": TargetSpec(
        label="苹果",
        mode="yolo",
        object_name="apple",
        topic="/detect_yolo/apple_position",
        yolo_class="apple",
    ),
    "香蕉": TargetSpec(
        label="香蕉",
        mode="yolo",
        object_name="banana",
        topic="/detect_yolo/banana_position",
        yolo_class="banana",
    ),
    "瓶子": TargetSpec(
        label="瓶子",
        mode="yolo",
        object_name="bottle",
        topic="/detect_yolo/bottle_position",
        yolo_class="bottle",
    ),
    "蛋糕": TargetSpec(
        label="蛋糕",
        mode="yolo",
        object_name="cake",
        topic="/detect_yolo/cake_position",
        yolo_class="cake",
    ),
    "小黄鸭": TargetSpec(
        label="小黄鸭",
        mode="detector",
        object_name="duck",
        topic="/duck_position",
        detector_node="duck_detector_node",
    ),
    "绿色药盒": TargetSpec(
        label="绿色药盒",
        mode="detector",
        object_name="box",
        topic="/box_position",
        detector_node="box_detector_node",
    ),
    "大樱桃": TargetSpec(
        label="大樱桃",
        mode="detector",
        object_name="red_circle",
        topic="/red_circle_position",
        detector_node="red_circle_detector_node",
    ),
}


class ArmTaskManager(Node):
    def __init__(self):
        super().__init__("arm_task_manager")

        self.declare_parameter("robot_root", "/home/sunrise/robot")
        self.declare_parameter("task_timeout_sec", 90.0)
        self.declare_parameter("cleanup_detector_after_done", True)
        self.declare_parameter("launch_yolo_for_grasp", False)
        self.declare_parameter("command_topic", "/command")

        self.robot_root = str(self.get_parameter("robot_root").value)
        self.task_timeout_sec = float(self.get_parameter("task_timeout_sec").value)
        self.cleanup_detector_after_done = bool(
            self.get_parameter("cleanup_detector_after_done").value)
        self.launch_yolo_for_grasp = bool(
            self.get_parameter("launch_yolo_for_grasp").value)

        command_topic = str(self.get_parameter("command_topic").value)
        self.create_subscription(String, command_topic, self.command_callback, 10)
        self.create_subscription(
            String,
            "/robot_arm/executor_status",
            self.executor_status_callback,
            10,
        )
        self.status_pub = self.create_publisher(String, "/arm_task/status", 10)
        self.active_object_pub = self.create_publisher(
            String,
            "/arm_task/active_object",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.timer = self.create_timer(1.0, self.watchdog)

        self.open_loop_proc: Optional[subprocess.Popen] = None
        self.transform_proc: Optional[subprocess.Popen] = None
        self.detector_proc: Optional[subprocess.Popen] = None
        self.active_spec: Optional[TargetSpec] = None
        self.active_since: Optional[float] = None
        self.executor_seen_busy = False
        self.executor_status = "UNKNOWN"

        self.get_logger().info(
            f"arm_task_manager ready. Listening on {command_topic}. "
            f"Known targets: {', '.join(TARGETS.keys())}")

    def command_callback(self, msg: String):
        try:
            commands = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Ignore malformed /command JSON: {exc}")
            return

        if not isinstance(commands, list):
            self.get_logger().warning("Ignore /command because payload is not a JSON array.")
            return

        for command in commands:
            if not isinstance(command, dict):
                continue
            if command.get("actuator") != "机械臂" or command.get("action") != "抓取":
                continue
            target = command.get("params", {}).get("target")
            self.start_grasp_for_target(str(target))
            return

    def start_grasp_for_target(self, target_label: str):
        spec = TARGETS.get(target_label)
        if spec is None:
            self.publish_status(f"REJECTED unknown target={target_label}")
            self.get_logger().warning(
                f"Unknown grasp target {target_label!r}; known={list(TARGETS.keys())}")
            return

        if self.active_spec is not None and not self.task_finished():
            self.publish_status(
                f"BUSY active={self.active_spec.label} rejected={spec.label}")
            self.get_logger().warning(
                f"Reject {spec.label}: current task {self.active_spec.label} is still active.")
            return

        self.active_spec = spec
        self.active_since = time.monotonic()
        self.executor_seen_busy = False
        self.publish_active_object(spec.object_name)

        self.ensure_open_loop()
        self.ensure_transform()
        self.restart_detector(spec)

        self.publish_status(f"GRASPING target={spec.label} topic={spec.topic}")
        self.get_logger().info(
            f"Started grasp task: label={spec.label}, mode={spec.mode}, "
            f"object_name={spec.object_name}, topic={spec.topic}")

    def ensure_open_loop(self):
        if self.process_alive(self.open_loop_proc):
            return
        cmd = shell_join([
            "source /opt/ros/humble/setup.bash",
            f"source {shlex.quote(self.robot_root)}/robot_ws/install/setup.bash",
            "ros2 launch robot_bringup open_loop_grasp.launch.py "
            "task_log_level:=info executor_log_level:=warn",
        ])
        self.open_loop_proc = self.start_shell_process("open_loop_grasp", cmd)

    def ensure_transform(self):
        if self.process_alive(self.transform_proc):
            return
        script = f"{self.robot_root}/hand_to_eye/camera_to_base_transform.py"
        cmd = shell_join([
            "source /opt/ros/humble/setup.bash",
            f"source {shlex.quote(self.robot_root)}/Orbbec_ws/install/setup.bash",
            f"source {shlex.quote(self.robot_root)}/robot_ws/install/setup.bash",
            f"python3 {shlex.quote(script)} "
            "--ros-args -p active_object_filter_enabled:=true",
        ])
        self.transform_proc = self.start_shell_process("camera_to_base_transform", cmd)

    def restart_detector(self, spec: TargetSpec):
        self.stop_process(self.detector_proc, "detector")
        self.detector_proc = None

        if spec.mode == "yolo":
            if not self.launch_yolo_for_grasp:
                self.get_logger().info(
                    f"Reuse resident YOLO for {spec.label}; "
                    "set launch_yolo_for_grasp:=true to let arm_task_manager start it.")
                return
            classes = f"['{spec.yolo_class}']"
            cmd = shell_join([
                "source /opt/ros/humble/setup.bash",
                "source /opt/tros/humble/setup.bash",
                f"source {shlex.quote(self.robot_root)}/Orbbec_ws/install/setup.bash",
                "ros2 run detect_yolo detect_yolo_node "
                f"--ros-args -p forward_classes:={shlex.quote(classes)}",
            ])
            self.detector_proc = self.start_shell_process(
                f"detect_yolo_{spec.yolo_class}", cmd)
            return

        if spec.mode == "detector":
            cmd = shell_join([
                "source /opt/ros/humble/setup.bash",
                f"source {shlex.quote(self.robot_root)}/Orbbec_ws/install/setup.bash",
                f"ros2 run detector {shlex.quote(spec.detector_node)}",
            ])
            self.detector_proc = self.start_shell_process(spec.detector_node, cmd)
            return

        raise ValueError(f"Unsupported target mode: {spec.mode}")

    def start_shell_process(self, name: str, command: str):
        self.get_logger().info(f"Start {name}: {command}")
        return subprocess.Popen(
            ["bash", "-lc", command],
            start_new_session=True,
            stdout=None,
            stderr=None,
        )

    def executor_status_callback(self, msg: String):
        self.executor_status = msg.data.strip().upper()
        if self.active_spec is None:
            return

        if self.executor_status == "BUSY":
            self.executor_seen_busy = True
            return

        if self.executor_seen_busy and self.executor_status in ("DONE", "IDLE"):
            spec = self.active_spec
            self.publish_status(f"DONE target={spec.label}")
            self.get_logger().info(f"Grasp task done: {spec.label}")
            self.finish_active_task()

        if self.executor_status in ("ERROR", "TIMEOUT", "REJECTED_INVALID_JOINT_LIMIT"):
            spec = self.active_spec
            self.publish_status(f"ERROR target={spec.label} executor={self.executor_status}")
            self.get_logger().error(
                f"Grasp task failed: {spec.label}, executor_status={self.executor_status}")
            self.finish_active_task()

    def watchdog(self):
        if self.active_spec is None or self.active_since is None:
            return
        self.publish_active_object(self.active_spec.object_name)
        age = time.monotonic() - self.active_since
        if age <= self.task_timeout_sec:
            return
        spec = self.active_spec
        self.publish_status(f"TIMEOUT target={spec.label}")
        self.get_logger().error(f"Grasp task timeout: {spec.label}, age={age:.1f}s")
        self.finish_active_task()

    def finish_active_task(self):
        if self.cleanup_detector_after_done:
            self.stop_process(self.detector_proc, "detector")
            self.detector_proc = None
        self.active_spec = None
        self.active_since = None
        self.executor_seen_busy = False
        self.publish_active_object("")

    def task_finished(self) -> bool:
        return self.active_spec is None

    def publish_status(self, text: str):
        self.status_pub.publish(String(data=text))

    def publish_active_object(self, object_name: str):
        self.active_object_pub.publish(String(data=object_name))

    @staticmethod
    def process_alive(proc: Optional[subprocess.Popen]) -> bool:
        return proc is not None and proc.poll() is None

    def stop_process(self, proc: Optional[subprocess.Popen], name: str):
        if proc is None or proc.poll() is not None:
            return
        self.get_logger().info(f"Stop {name} pid={proc.pid}")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass

    def destroy_node(self):
        self.stop_process(self.detector_proc, "detector")
        self.stop_process(self.transform_proc, "camera_to_base_transform")
        self.stop_process(self.open_loop_proc, "open_loop_grasp")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmTaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
