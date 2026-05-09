#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from collections import deque
from datetime import datetime, timedelta

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from robot_msgs.msg import CameraTarget, ArmJointState

# 机械臂视觉目标执行节点，监听 /camera_target_base，要求 frame_id 是 base_link 或空（默认 base_link）
# 根据目标位置、置信度和稳定性分阶段规划小步移动命令，发布到 /robot_arm/cart_target 
# 和 /robot_arm/gripper_cmd，由执行层节点执行。
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
        self.gripper_settle_until = None
        self.lift_target = None
        self.wait_timeout = timedelta(seconds=10)
        self.gripper_settle_time = timedelta(seconds=1)

        #机械臂工作空间（待修改/更新）
        self.x_min = 0.10
        self.x_max = 1.00
        self.y_min = -0.45
        self.y_max = 0.50
        self.z_min = 0.02
        self.z_max = 0.80

        # 视觉目标评估参数
        self.confidence_high = 0.7
        self.confidence_low = 0.3
        self.drift_threshold = 0.05
        self.stable_count_required = 3

        # 不同阶段的步长参数（待调整/优化）
        # 预抓取阶段：目标正上方 10cm 处
        self.pregrasp_step = 0.10
        # 对齐阶段：目标正上方 7cm 处
        self.align_step = 0.07
        # 最终接近阶段：目标正上方 3cm 处
        self.final_approach_step = 0.03
        # 抬升阶段：从当前位置垂直向上 8cm
        self.lift_step = 0.08
        # 撤退阶段：直接回到安全位姿，或从当前位置垂直向上 10cm 后撤退
        self.retreat_step = 0.10
        # 抬升阶段的 Z 方向偏移量，确保物体被抬离后再横向移动，减少碰撞风险。
        self.post_grasp_lift = 0.10
        # 安全撤退位姿，位于工作空间内且远离视觉目标区域，确保在撤退阶段有一个明确的安全目标。
        self.safe_retreat_pose = [0.35, 0.00, 0.35]

        # 初始化 ROS 订阅和发布
        self.target_sub = self.create_subscription(
            CameraTarget,
            '/camera_target_base',
            self.target_callback,
            10
        )
        # 订阅机械臂状态以获取当前末端位姿，辅助规划和评估步骤执行效果。
        self.state_sub = self.create_subscription(
            ArmJointState,
            '/robot_arm/joint_state',
            self.state_callback,
            10
        )

        # 发布笛卡尔目标到执行层，要求 frame_id 是 base_link，执行前确保目标在工作空间内并且步长合理。
        self.cart_pub = self.create_publisher(
            PointStamped,
            '/robot_arm/cart_target',
            10
        )
        # 发布夹爪命令到执行层，命令格式简单（如 "open" 或 "close"），执行前确保机械臂处于安全状态。
        self.gripper_pub = self.create_publisher(
            String,
            '/robot_arm/gripper_cmd',
            10
        )

        self.timer = self.create_timer(0.25, self.step_loop)

        self.get_logger().info('CameraTargetExecutor started.')
        self.get_logger().info('Subscribed topic: /camera_target_base')
        self.get_logger().info('Expected frame_id: base_link')
    
    
    def in_workspace(self, x, y, z):
        # 检查三维点是否在机械臂安全工作空间内。
        return (
            self.x_min <= x <= self.x_max and
            self.y_min <= y <= self.y_max and
            self.z_min <= z <= self.z_max
        )

    def target_callback(self, msg: CameraTarget):
        # 验证输入消息的 frame_id 是否符合预期，检查目标位置是否在工作空间内，并根据置信度和稳定性更新当前任务状态。
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
        # 从机械臂状态消息中提取当前末端位姿，更新 last_end_pose 以辅助规划和评估步骤执行效果。
        if msg.end_pose and len(msg.end_pose) >= 3:
            self.last_end_pose = [msg.end_pose[0], msg.end_pose[1], msg.end_pose[2]]

    def step_loop(self):
        # 根据当前任务状态执行相应的逻辑，确保在每个阶段都进行必要的检查和评估，及时响应视觉输入的变化和执行结果。
        if self.task_state == 'WAIT_TARGET':#等待新的视觉目标输入，持续监测目标的置信度和稳定性，准备进入规划阶段。
            return

        if self.task_state == 'PLAN_PATH':#根据最新的稳定目标规划下一小步移动命令，确保每步命令都在工作空间内且步长合理，发布到执行层。
            self.plan_path()
            return

        if self.task_state == 'WAIT_VISUAL_UPDATE': #等待执行层反馈新的视觉输入，评估执行效果和目标状态，根据情况决定是否继续当前路径、重新规划或放弃当前目标。
            self.evaluate_visual_update()
            return

        if self.task_state == 'WAIT_GRIPPER_SETTLE':#等待夹爪动作完成后机械臂稳定，通常在发送夹爪命令后设置一个短暂的等待时间，确保机械臂在执行下一步前已经稳定。
            if self.gripper_settle_until and datetime.now() >= self.gripper_settle_until:
                self.current_stage = 'LIFT'
                self.task_state = 'PLAN_PATH'
            return

        if self.task_state == 'REPLAN':#当视觉输入发生较大变化（如目标漂移过大或置信度骤降）时，放弃当前路径规划，重新评估目标状态并进入规划阶段。
            self.get_logger().info('Replanning based on latest visual input.')
            self.current_stage = None
            self.task_state = 'PLAN_PATH'
            return

        if self.task_state == 'ABORT':  #当视觉输入长时间没有更新或目标状态变得不可靠时，放弃当前目标，重置任务状态机，等待新的目标输入。
            self.get_logger().error('Aborting current target and returning to WAIT_TARGET.')
            self.reset_task()
            return

    def plan_path(self):
        # 根据当前最新的视觉目标和机械臂状态，选择下一小步的笛卡尔目标，并发布到执行层，确保每步命令都经过安全检查和合理规划。
        if self.latest_target is None:
            self.get_logger().info('No target available to plan.')
            self.task_state = 'WAIT_TARGET'
            return

        post_grasp_stage = self.current_stage in ['LIFT', 'RETREAT']
        # 在抓取后阶段对目标漂移的容忍度更高，允许一定程度的目标变化而不立即放弃当前路径规划，但在抓取前阶段则要求目标稳定且置信度足够，任何较大变化都需要重新评估和规划。
        if not post_grasp_stage and not self._is_target_stable():
            self.get_logger().info('Target not yet stable, waiting for more visual updates.')
            self.task_state = 'WAIT_TARGET'
            return
        # 在抓取前阶段如果最新目标的置信度低于最低阈值，则放弃当前目标并等待新的视觉输入，确保机械臂不会基于不可靠的视觉信息进行规划和执行。
        if not post_grasp_stage and self.latest_target.confidence < self.confidence_low:
            self.get_logger().warning('Target confidence too low for planning.')
            self.task_state = 'WAIT_TARGET'
            return
        # 在抓取前阶段如果当前目标与最新目标的漂移超过阈值，则接受新的目标并重置阶段到 PREGRASP，确保机械臂始终基于最新且相对稳定的视觉信息进行规划和执行。
        if not post_grasp_stage and (
            self.current_target is None or
            self._target_drift(self.current_target, self.latest_target) > self.drift_threshold
        ):
            self.current_target = self.latest_target
            self.current_stage = 'PREGRASP'
            self.get_logger().info('Accepted new target and reset stage to PREGRASP.')

        step_target = self._choose_next_step(self.current_target)
        if self.task_state != 'PLAN_PATH':
            return
        if step_target is None:
            self.get_logger().info('Current target reached or no step required.')
            self.task_state = 'WAIT_VISUAL_UPDATE'
            return

        self.publish_cart_step(step_target)
        self.last_command_target = step_target
        self.last_command_time = datetime.now()
        self.task_state = 'WAIT_VISUAL_UPDATE'
    #
    def evaluate_visual_update(self):
        """Decide the next action after issuing a step command."""
        if self.last_command_target is None:
            self.task_state = 'PLAN_PATH'
            return

        if self.last_end_pose is not None and self._distance(self.last_end_pose, self.last_command_target) < 0.04:
            self.get_logger().info('Observed robot reached the last commanded step.')
            if self.current_stage == 'RETREAT':
                self.get_logger().info('RETREAT reached, grasp sequence finished.')
                self.reset_task()
                return
            elif self.current_stage == 'FINAL_APPROACH':
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

        if (
            self.current_stage not in ['LIFT', 'RETREAT'] and
            self._target_drift(self.current_target, self.latest_target) > self.drift_threshold
        ):
            self.get_logger().warning('Target drift too large, replanning.')
            self.task_state = 'REPLAN'
            return

        post_grasp_stage = self.current_stage in ['LIFT', 'RETREAT']

        if not post_grasp_stage and self.latest_target.confidence < self.confidence_low:
            self.get_logger().warning('Confidence dropped after step, waiting for better input.')
            self.task_state = 'WAIT_TARGET'
            return

        if not post_grasp_stage and self.latest_target.confidence < self.confidence_high:
            self.get_logger().info('Medium confidence, delaying next step until better visual feedback.')
            self.task_state = 'WAIT_TARGET'
            return
        
    #选择下一阶段的逻辑可以根据当前阶段进行简单的映射，例如 PREGRASP -> ALIGN -> FINAL_APPROACH -> CLOSE_GRIPPER -> LIFT -> RETREAT，确保每个阶段都有明确的目标和条件。
    def _choose_next_step(self, target: CameraTarget):
        if self.last_end_pose is None:
            self.get_logger().warning('No current end pose available, cannot choose next step.')
            return None

        current = self.last_end_pose
        pregrasp = self._clamp_workspace_target([target.x, target.y, target.z + 0.12])
        align = self._clamp_workspace_target([target.x, target.y, target.z + 0.06])
        final = self._clamp_workspace_target([target.x, target.y, target.z])
        retreat = self._clamp_workspace_target(self.safe_retreat_pose)

        if self.current_stage == 'PREGRASP':
            if self._distance(current, pregrasp) > 0.03:
                return self._limit_step(current, pregrasp, self.pregrasp_step)
            self.get_logger().info('PREGRASP reached, moving to ALIGN stage.')
            self.current_stage = 'ALIGN'

        if self.current_stage == 'ALIGN':
            if self._distance(current, align) > 0.03:
                return self._limit_step(current, align, self.align_step)
            self.get_logger().info('ALIGN reached, moving to FINAL_APPROACH stage.')
            self.current_stage = 'FINAL_APPROACH'

        if self.current_stage == 'FINAL_APPROACH':
            if self._distance(current, final) > 0.02:
                return self._limit_step(current, final, self.final_approach_step)
            self.get_logger().info('FINAL_APPROACH reached, target is in contact region.')
            return None

        if self.current_stage == 'CLOSE_GRIPPER':
            if not self.gripper_closed:
                self.publish_gripper_command('close')
                self.gripper_closed = True
                self.gripper_settle_until = datetime.now() + self.gripper_settle_time
                self.task_state = 'WAIT_GRIPPER_SETTLE'
                self.get_logger().info('Gripper close command published, waiting before lift.')
            return None

        if self.current_stage == 'LIFT':
            if self.lift_target is None:
                self.lift_target = [
                    current[0],
                    current[1],
                    min(current[2] + self.post_grasp_lift, self.z_max),
                ]
                self.lift_target = self._clamp_workspace_target(self.lift_target)
            lift = self.lift_target
            if self._distance(current, lift) > 0.03:
                return self._limit_step(current, lift, self.lift_step)
            self.get_logger().info('LIFT reached, moving to RETREAT stage.')
            self.current_stage = 'RETREAT'

        if self.current_stage == 'RETREAT':
            if self._distance(current, retreat) > 0.04:
                return self._limit_step(current, retreat, self.retreat_step)
            self.get_logger().info('RETREAT reached, grasp sequence finished.')
            self.reset_task()
            return None

        return None
    
    
    def _limit_step(self, current, target, max_step):
        distance = self._distance(current, target)
        if distance <= max_step:
            return target

        ratio = max_step / distance
        step = [
            current[0] + (target[0] - current[0]) * ratio,
            current[1] + (target[1] - current[1]) * ratio,
            current[2] + (target[2] - current[2]) * ratio,
        ]
        self.get_logger().info(
            f'Clamped {self.current_stage} step from {distance:.3f} m to {max_step:.3f} m.'
        )
        return step

    
    def _clamp_workspace_target(self, target):
        return [
            min(max(float(target[0]), self.x_min), self.x_max),
            min(max(float(target[1]), self.y_min), self.y_max),
            min(max(float(target[2]), self.z_min), self.z_max),
        ]

    def publish_cart_step(self, step_target):
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
        if current_stage == 'LIFT':
            return 'RETREAT'
        return 'FINAL_APPROACH'

    def reset_task(self):
        """Reset the task state machine after aborting or finishing."""
        self.task_state = 'WAIT_TARGET'
        self.current_stage = None
        self.current_target = None
        self.last_command_target = None
        self.last_command_time = None
        self.gripper_closed = False
        self.gripper_settle_until = None
        self.lift_target = None
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
