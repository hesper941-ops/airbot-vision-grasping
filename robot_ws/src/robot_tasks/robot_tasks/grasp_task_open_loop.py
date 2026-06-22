#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Single preflight-confirmed open-loop waypoint grasp task.

The task node consumes /visual_target_base in base_link coordinates and sends
explicit joint/cartesian/gripper/speed commands to arm_executor_node. It does
not call the AIRBOT SDK and does not perform camera-to-base transforms.
"""

import math
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray, String

from robot_msgs.msg import ArmJointState, VisualTarget
from robot_tasks.shared.grasp_planner import GraspPlanner

# 重构模块（已稳定，逐步接入中）
from robot_tasks.grasp.config import GraspTaskConfig
from robot_tasks.grasp.context import GraspContext
from robot_tasks.grasp.command_port import ArmCommandPort
from robot_tasks.grasp.target_manager import TargetManager, TargetObservation
from robot_tasks.grasp.diagnostics import (
    build_workspace_rejection_message,
    fmt_xyz,
    format_status_summary,
)
from robot_tasks.grasp.planning_result import PlanningResult
# 后续模块暂不接入，保持隔离:
# from robot_tasks.grasp.target_source_manager import TargetSourceManager
# from robot_tasks.grasp.search_pose_manager import SearchPoseManager
# from robot_tasks.grasp.search_strategy import SearchStrategy
# from robot_tasks.grasp.recovery import RecoveryManager
# from robot_tasks.grasp.grasp_sequence import GraspSequenceController


class GraspTaskOpenLoop(Node):
    """Preflight-confirmed open-loop waypoint grasp state machine."""

    def __init__(self):
        super().__init__('grasp_task_open_loop')
        self._declare_parameters()

        # ---- 已稳定的重构模块（与旧逻辑共存） ----
        self._cfg = GraspTaskConfig.from_ros_node(self)
        self._ctx = GraspContext()
        self._cmd_port = ArmCommandPort(self, base_frame=self._cfg.base_frame)
        self._target_mgr = TargetManager(self._cfg)
        # 后续模块（target_source / search / recovery / grasp_sequence）暂不接入

        # ---- 旧逻辑（当前主运行路径） ----
        self.planner = GraspPlanner(self._config_dict())

        self.task_state = 'IDLE'
        self.state_start_time = self._now_sec()

        self.latest_target: Optional[VisualTarget] = None
        self.latest_target_time: Optional[float] = None
        self.pre_target: Optional[list] = None
        self.grasp_target: Optional[list] = None
        self.last_seen_target_base = None
        self.last_seen_target_time = None
        self.active_target_base = None
        self.target_frozen = False
        self.active_motion_goal: Optional[list] = None
        self.last_visual_lost_warning_time: Optional[float] = None
        self.last_target_failure_reason = None
        self.current_approach_index = 0
        self.current_approach_mode = None
        self.approach_failed_modes = set()
        self.approach_retry_count = 0
        self.return_j6_home_sent = False
        self.grasp_closed = False
        self.lift_goal: Optional[list] = None
        self.stage_full_goal: Optional[list] = None
        self.blend_waypoints: Optional[list] = None
        self.recover_lift_goal: Optional[list] = None

        self.state_command_sent = False
        self.stage_motion_started = False
        self.settle_start_time: Optional[float] = None

        self.last_end_pose: Optional[list] = None
        self.last_end_pose_time: Optional[float] = None
        self.end_pose_stability_window = deque(maxlen=max(
            1, int(self.get_parameter('end_pose_stability_window').value)))
        self.target_collection_cooldown_until: Optional[float] = None
        self.target_collection_cooldown_logged = False
        self.last_end_pose_stability_log_time: Optional[float] = None
        self.last_joint_pos: Optional[list] = None
        self.last_joint_vel: Optional[list] = None
        self.executor_status = 'IDLE'

        self.speed_profile_active = 'unknown'
        self.pending_speed_profile: Optional[str] = None
        self.gripper_settle_start: Optional[float] = None
        self.recover_phase = 'OPEN_GRIPPER'
        self.recover_reason = 'idle'
        self.recover_detail = ''
        self.recover_reset_command: Optional[str] = None
        self.last_reset_executor_time: Optional[float] = None
        self.rejected_busy_count = 0
        self.blend_busy_retry_count = 0
        self.blend_busy_retry_after: Optional[float] = None
        self.cartesian_busy_retry_count = 0
        self.cartesian_busy_retry_after: Optional[float] = None
        self.recover_command_sent = False
        self.recover_command_type: Optional[str] = None
        self.recover_command_time: Optional[float] = None
        self.recover_timeout_logged = False
        self.last_status_log_time: Optional[float] = None
        self.selected_plan: Optional[PlanningResult] = None
        self.plan_id_counter = 0

        self.target_sub = self.create_subscription(
            VisualTarget,
            '/visual_target_base',
            self.target_callback,
            10,
        )
        self.joint_state_sub = self.create_subscription(
            ArmJointState,
            '/robot_arm/joint_state',
            self.joint_state_callback,
            10,
        )
        self.end_pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_arm/end_pose',
            self.end_pose_callback,
            10,
        )
        self.executor_status_sub = self.create_subscription(
            String,
            '/robot_arm/executor_status',
            self.executor_status_callback,
            10,
        )

        # 机械臂命令发布统一通过 ArmCommandPort，不再直接创建 publisher
        # 所有消息保持与原有 topic 和类型一致

        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('loop_hz').value),
            self.step_loop,
        )

        self.get_logger().info(
            'GraspTaskOpenLoop started: main_flow='
            'WAIT_PRE_TARGET -> PRE_OPEN_GRIPPER -> MOVE_APPROACH_BLEND -> '
            'CLOSE_GRIPPER -> MOVE_LIFT -> RETURN_INIT_POSE, '
            'sequential MOVE_PRE_GRASP -> MOVE_GRASP fallback enabled, '
            f'final_init_joint_pos_deg={list(self.get_parameter("final_init_joint_pos_deg").value)}.')

    def _declare_parameters(self):
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('allow_empty_target_frame', False)

        self.declare_parameter('pre_grasp_z_offset', 0.06)
        self.declare_parameter('grasp_z_offset', 0.02)
        self.declare_parameter('lift_z_offset', 0.04)
        self.declare_parameter('safe_pose', [0.35, 0.0, 0.35])
        self.declare_parameter('approach_mode', 'front')
        self.declare_parameter('approach_priority', ['front', 'top_down'])
        self.declare_parameter('front_first_then_top_down', True)
        self.declare_parameter('max_approach_mode_retries', 1)
        self.declare_parameter('blend_approach_enabled', True)
        self.declare_parameter('blend_approach_fallback_to_sequential', True)
        self.declare_parameter('blend_approach_retry_on_busy', True)
        self.declare_parameter('blend_approach_busy_retry_delay_sec', 1.5)
        self.declare_parameter('blend_approach_busy_max_retries', 3)
        self.declare_parameter('enable_top_down_fallback', True)
        self.declare_parameter('top_down_max_target_z', 0.49)
        self.declare_parameter('top_down_max_pre_grasp_radius_m', 0.64)
        self.declare_parameter('top_down_pre_grasp_z_offset', 0.035)
        self.declare_parameter('continue_with_last_seen_during_motion', True)
        self.declare_parameter('table_z', 0.0)
        self.declare_parameter('table_clearance', 0.04)
        self.declare_parameter('final_grasp_clearance', 0.015)
        self.declare_parameter('front_approach_x_offset', -0.10)
        self.declare_parameter('front_approach_z_offset', 0.05)
        self.declare_parameter('adaptive_front_pre_grasp', True)
        self.declare_parameter('workspace_soft_margin_m', 0.04)
        self.declare_parameter('min_front_pre_grasp_distance_m', 0.04)
        self.declare_parameter('front_grasp_x_offset', 0.0)
        self.declare_parameter('front_grasp_x_offset_max', 0.075)
        self.declare_parameter('min_safe_motion_z', 0.08)
        self.declare_parameter('reject_target_below_table', True)
        self.declare_parameter('official_reach_radius_m', 0.68)
        # Conservative J6 range based on AIRBOT Play official specs.
        # Confirm exact hardware model before widening this range.
        self.declare_parameter('joint6_min_rad', -2.9671)
        self.declare_parameter('joint6_max_rad', 2.9671)
        self.declare_parameter('j6_home_deg', 0.0)
        self.declare_parameter('j6_allowed_delta_deg', 90.0)
        self.declare_parameter('forbid_camera_upside_down', True)
        self.declare_parameter('return_j6_to_home_on_recover', True)
        self.declare_parameter('return_to_init_after_grasp', True)
        self.declare_parameter('keep_gripper_closed_after_grasp', True)
        self.declare_parameter('final_init_joint_pos_deg', [0.0, -45.0, 110.0, -90.0, 90.0, 0.0])
        self.declare_parameter('return_init_timeout_sec', 10.0)

        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('stable_frame_count', 5)
        self.declare_parameter('stable_frame_count_required', 5)
        self.declare_parameter('stable_position_threshold', 0.015)
        self.declare_parameter('stable_position_threshold_m', 0.015)
        self.declare_parameter('stable_depth_threshold_m', 0.03)
        self.declare_parameter('target_stability_window', 8)
        self.declare_parameter('target_stability_min_samples', 5)
        self.declare_parameter('target_stability_max_range_x', 0.015)
        self.declare_parameter('target_stability_max_range_y', 0.012)
        self.declare_parameter('target_stability_max_range_z', 0.015)
        self.declare_parameter('target_outlier_reject_distance', 0.04)
        self.declare_parameter('visual_sanity_max_radius_m', 0.64)
        self.declare_parameter('target_timeout_sec', 1.0)
        self.declare_parameter('end_pose_timeout_sec', 1.0)
        self.declare_parameter('use_last_seen_target_on_loss', True)
        self.declare_parameter('visual_lost_grace_sec', 0.5)
        self.declare_parameter('last_seen_target_max_age_sec', 8.0)
        self.declare_parameter('update_target_during_motion', False)
        self.declare_parameter('freeze_target_before_close', True)
        self.declare_parameter('ignore_visual_during_motion', True)
        self.declare_parameter('clear_target_window_on_cycle_start', True)
        self.declare_parameter('clear_target_window_on_cycle_end', True)
        self.declare_parameter('inter_cycle_cooldown_sec', 2.0)
        self.declare_parameter('require_idle_before_target_collection', True)
        self.declare_parameter('require_end_pose_stable_before_target_collection', True)
        self.declare_parameter('end_pose_stability_window', 5)
        self.declare_parameter('end_pose_stability_epsilon_m', 0.003)
        self.declare_parameter('max_target_jump_m', 0.08)
        self.declare_parameter('max_target_z_jump_m', 0.08)

        self.declare_parameter('workspace_limits.x_min', 0.10)
        self.declare_parameter('workspace_limits.x_max', 0.68)
        self.declare_parameter('workspace_limits.y_min', -0.38)
        self.declare_parameter('workspace_limits.y_max', 0.38)
        self.declare_parameter('workspace_limits.z_min', 0.02)
        self.declare_parameter('workspace_limits.z_max', 0.75)

        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('position_tolerance_m', 0.015)
        self.declare_parameter('settle_time_sec', 0.8)
        self.declare_parameter('joint_speed_safe_threshold', 0.1)
        self.declare_parameter('post_joint_rotate_settle_sec', 0.5)
        self.declare_parameter('gripper_settle_sec', 1.0)
        self.declare_parameter('open_gripper_before_grasp', True)
        self.declare_parameter('pre_grasp_open_timeout_sec', 4.0)
        self.declare_parameter('pre_grasp_open_settle_sec', 1.0)

        # Cartesian step-by-step: each command limited to this distance.
        # Keep waypoint max step below the configured executor safety limit.
        self.declare_parameter('max_cartesian_step', 0.06)
        self.declare_parameter('lift_cartesian_step_m', 0.03)
        self.declare_parameter('cart_waypoint_max_step_m', 0.10)
        self.declare_parameter('cart_waypoint_safe_limit_m', 0.12)

        self.declare_parameter('wait_pre_target_warn_sec', 15.0)
        self.declare_parameter('motion_timeout_sec', 16.0)
        self.declare_parameter('set_orientation_timeout_sec', 8.0)
        self.declare_parameter('close_gripper_timeout_sec', 4.0)
        self.declare_parameter('recover_timeout_sec', 15.0)
        self.declare_parameter('rejected_busy_recover_threshold', 2)
        self.declare_parameter('recover_clear_error_interval_sec', 0.5)
        self.declare_parameter('auto_recover_joint_limit', True)
        self.declare_parameter('post_motion_command_cooldown_sec', 1.5)
        self.declare_parameter('sequential_busy_max_retries', 3)
        self.declare_parameter('loop_hz', 4.0)
        self.declare_parameter('verbose_debug', False)
        self.declare_parameter('status_log_period_sec', 2.0)
        self.declare_parameter('log_waypoint_each_step', False)

    def _config_dict(self) -> dict:
        return {
            'pre_grasp_z_offset': self.get_parameter('pre_grasp_z_offset').value,
            'top_down_pre_grasp_z_offset': self.get_parameter('top_down_pre_grasp_z_offset').value,
            'grasp_z_offset': self.get_parameter('grasp_z_offset').value,
            'lift_z_offset': self.get_parameter('lift_z_offset').value,
            'safe_pose': self.get_parameter('safe_pose').value,
            'approach_mode': self.get_parameter('approach_mode').value,
            'approach_priority': self.get_parameter('approach_priority').value,
            'table_z': self.get_parameter('table_z').value,
            'table_clearance': self.get_parameter('table_clearance').value,
            'final_grasp_clearance': self.get_parameter('final_grasp_clearance').value,
            'front_approach_x_offset': self.get_parameter('front_approach_x_offset').value,
            'front_approach_z_offset': self.get_parameter('front_approach_z_offset').value,
            'adaptive_front_pre_grasp': self.get_parameter('adaptive_front_pre_grasp').value,
            'workspace_soft_margin_m': self.get_parameter('workspace_soft_margin_m').value,
            'min_front_pre_grasp_distance_m': self.get_parameter('min_front_pre_grasp_distance_m').value,
            'front_grasp_x_offset': self.get_parameter('front_grasp_x_offset').value,
            'front_grasp_x_offset_max': self.get_parameter('front_grasp_x_offset_max').value,
            'min_safe_motion_z': self.get_parameter('min_safe_motion_z').value,
            'reject_target_below_table': self.get_parameter('reject_target_below_table').value,
            'official_reach_radius_m': self.get_parameter('official_reach_radius_m').value,
            'joint6_min_rad': self.get_parameter('joint6_min_rad').value,
            'joint6_max_rad': self.get_parameter('joint6_max_rad').value,
            'j6_home_deg': self.get_parameter('j6_home_deg').value,
            'j6_allowed_delta_deg': self.get_parameter('j6_allowed_delta_deg').value,
            'forbid_camera_upside_down': self.get_parameter('forbid_camera_upside_down').value,
            'workspace_limits': {
                'x_min': self.get_parameter('workspace_limits.x_min').value,
                'x_max': self.get_parameter('workspace_limits.x_max').value,
                'y_min': self.get_parameter('workspace_limits.y_min').value,
                'y_max': self.get_parameter('workspace_limits.y_max').value,
                'z_min': self.get_parameter('workspace_limits.z_min').value,
                'z_max': self.get_parameter('workspace_limits.z_max').value,
            },
        }

    def target_callback(self, msg: VisualTarget):
        """接收视觉目标，经 TargetManager 判断稳定性后更新上下文。"""
        now_sec = self._now_sec()

        # 兼容旧字段赋值
        self.latest_target = msg
        self.latest_target_time = now_sec

        if bool(self.get_parameter('ignore_visual_during_motion').value) and self.task_state != 'WAIT_PRE_TARGET':
            self.get_logger().debug('Vision collection disabled during motion.')
            return

        if not self._target_collection_allowed():
            return

        if not self._valid_target(msg):
            return

        obs = TargetObservation(
            x=float(msg.x),
            y=float(msg.y),
            z=float(msg.z),
            depth=float(getattr(msg, "depth", 0.0)),
            confidence=float(getattr(msg, "confidence", 0.85)),
            frame_id=str(getattr(msg.header, "frame_id", "")),
            stamp_sec=now_sec,
            object_name=str(getattr(msg, "object_name", "duck")),
        )

        radius = self.planner.compute_radius([obs.x, obs.y, obs.z])
        max_radius = self._param_float('visual_sanity_max_radius_m')
        if radius > max_radius:
            self.get_logger().warning(
                f'Visual sanity check: radius={radius:.3f}, max={max_radius:.3f}, '
                'decision=rejected')
            self._target_mgr.reset_stability()
            return
        self.get_logger().debug(
            f'Visual sanity check: radius={radius:.3f}, max={max_radius:.3f}, '
            'decision=accepted')

        median_before = self._target_mgr.window_median_xyz()
        if median_before is not None:
            outlier_dist = self._distance([obs.x, obs.y, obs.z], median_before)
            if outlier_dist > self._param_float('target_outlier_reject_distance'):
                self.get_logger().warning(
                    f'Target stability window: samples={self._target_mgr.sample_count()}, '
                    f'range={self._fmt_range(self._target_mgr.window_range())}, '
                    f'median={self._fmt_xyz(median_before)}, decision=rejected, '
                    f'reason=outlier distance {outlier_dist:.3f}m.')

        accepted = self._target_mgr.accept_observation(
            obs,
            now_sec,
            active_target=self.active_target_base,
            task_state=self.task_state,
        )

        if not accepted:
            self._log_target_stability_window('rejected')
            return

        target_base = [obs.x, obs.y, obs.z]
        self.last_seen_target_base = target_base
        self.last_seen_target_time = now_sec

        # 更新 GraspContext
        self._ctx.latest_target = obs
        self._ctx.last_seen_target = self._target_mgr.last_seen
        self._ctx.last_target_time = now_sec

        stable = self._target_mgr.get_stable_target()
        if stable is not None:
            self._ctx.stable_target = stable
        self._log_target_stability_window(
            'accepted' if stable is not None else 'rejected')

        # 运动过程中通常不更新 active_target_base
        updatable_states = (
            'WAIT_PRE_TARGET',
        )
        can_update_during_motion = bool(
            self.get_parameter('update_target_during_motion').value)
        if (
            self.task_state in updatable_states
            and not self.target_frozen
            or (
                can_update_during_motion
                and self.task_state in ('MOVE_PRE_GRASP', 'MOVE_GRASP', 'MOVE_APPROACH_BLEND')
                and not self.target_frozen
            )
        ):
            self.active_target_base = target_base

    def _reject_target_jump(self, target_base: list) -> bool:
        if self.active_target_base is None:
            return False
        if self.task_state not in ('MOVE_PRE_GRASP', 'MOVE_GRASP', 'MOVE_APPROACH_BLEND'):
            return False

        distance = self._distance(self.active_target_base, target_base)
        z_jump = abs(float(target_base[2]) - float(self.active_target_base[2]))
        max_distance = self._param_float('max_target_jump_m')
        max_z_jump = self._param_float('max_target_z_jump_m')
        if distance <= max_distance and z_jump <= max_z_jump:
            return False

        self.get_logger().warning(
            f'Reject visual target jump: old={self._fmt_xyz(self.active_target_base)}, '
            f'new={self._fmt_xyz(target_base)}, dist={distance:.3f}m.')
        return True

    def joint_state_callback(self, msg: ArmJointState):
        if msg.joint_pos and len(msg.joint_pos) >= 6:
            self.last_joint_pos = list(msg.joint_pos)
        if msg.joint_vel and len(msg.joint_vel) >= 6:
            self.last_joint_vel = list(msg.joint_vel)
        if msg.end_pose and len(msg.end_pose) >= 3:
            self.last_end_pose = [float(msg.end_pose[0]), float(msg.end_pose[1]), float(msg.end_pose[2])]
            self.last_end_pose_time = self._now_sec()

    def end_pose_callback(self, msg: PoseStamped):
        self.last_end_pose = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ]
        self.last_end_pose_time = self._now_sec()
        self.end_pose_stability_window.append(list(self.last_end_pose))

    def executor_status_callback(self, msg: String):
        self.executor_status = msg.data.strip().upper()

        if self.executor_status in ('IDLE', 'DONE'):
            self.rejected_busy_count = 0

        if self.executor_status == 'REJECTED_BUSY':
            if self.task_state == 'RECOVER':
                return
            if self.task_state == 'MOVE_APPROACH_BLEND':
                self._schedule_blend_busy_retry('executor returned REJECTED_BUSY')
                return
            if self.task_state in ('MOVE_PRE_GRASP', 'MOVE_GRASP', 'MOVE_LIFT'):
                self._schedule_cartesian_busy_retry(
                    f'{self.task_state} executor returned REJECTED_BUSY')
                return
            self.rejected_busy_count += 1
            threshold = int(self.get_parameter('rejected_busy_recover_threshold').value)
            if self.rejected_busy_count >= threshold:
                self.get_logger().error(
                    f'REJECTED_BUSY {self.rejected_busy_count} times (>= {threshold}); handling approach failure.')
                if self.task_state == 'MOVE_APPROACH_BLEND':
                    self._fallback_blend_to_sequential(
                        f'REJECTED_BUSY {self.rejected_busy_count} times')
                    return
                self._handle_approach_failure(
                    f'REJECTED_BUSY {self.rejected_busy_count} times')
            else:
                self.get_logger().warning(
                    f'REJECTED_BUSY {self.rejected_busy_count}/{threshold}; '
                    f'waiting for executor to return IDLE before escalating to RECOVER.')
            return

        if self.executor_status in ('ERROR', 'TIMEOUT'):
            if self.task_state in ('RECOVER', 'RETURN_INIT_POSE'):
                self.get_logger().debug(
                    f'Executor status {self.executor_status} ignored '
                    f'because already in {self.task_state}.')
                return
            if self.task_state == 'MOVE_APPROACH_BLEND':
                self.get_logger().error(
                    f'Executor {self.executor_status} detected during MOVE_APPROACH_BLEND; '
                    'trying sequential fallback first.')
                self._fallback_blend_to_sequential(
                    f'executor_error: Executor status {self.executor_status}')
                return
            self.get_logger().error(
                f'Executor {self.executor_status} detected; entering RECOVER. '
                'recover_reason=executor_error')
            self._handle_approach_failure(
                f'executor_error: Executor status {self.executor_status}')

    def step_loop(self):
        try:
            self._maybe_log_status_summary()
            if self._handle_pending_speed_profile():
                return

            if self.task_state == 'IDLE':
                self._transition('WAIT_PRE_TARGET', clear_window=True)

            elif self.task_state == 'WAIT_PRE_TARGET':
                self._handle_wait_pre_target()

            elif self.task_state == 'PRE_OPEN_GRIPPER':
                self._handle_pre_open_gripper()

            elif self.task_state == 'MOVE_PRE_GRASP':
                self._handle_move_pre_grasp()

            elif self.task_state == 'MOVE_APPROACH_BLEND':
                self._handle_move_approach_blend()

            elif self.task_state == 'MOVE_GRASP':
                self._handle_move_grasp()

            elif self.task_state == 'CLOSE_GRIPPER':
                self._handle_close_gripper()

            elif self.task_state == 'MOVE_LIFT':
                self._handle_move_lift()

            elif self.task_state == 'MOVE_RETREAT':
                self._handle_move_retreat()

            elif self.task_state == 'RETURN_INIT_POSE':
                self._handle_return_init_pose()

            elif self.task_state == 'RECOVER':
                self._handle_recover()

        except Exception as exc:
            self.get_logger().error(f'step_loop exception: {exc}', exc_info=True)
            self._enter_recover()

    def _handle_wait_pre_target(self):
        """等待第一个稳定视觉目标。

        WAIT_PRE_TARGET 是抓取前等待阶段。没有目标只表示视觉管线未运行，
        周期性 warning 并持续等待，不进入 RECOVER。
        """
        if self.executor_status in ('ERROR', 'TIMEOUT'):
            self.get_logger().error(
                f'Executor status {self.executor_status} while waiting for target; '
                'entering RECOVER before new planning.')
            self._enter_recover(
                f'executor_error: Executor status {self.executor_status}')
            return

        warn_sec = self._param_float('wait_pre_target_warn_sec')
        if self._state_elapsed() > warn_sec:
            self.get_logger().warning(
                f'Still waiting for stable /visual_target_base in WAIT_PRE_TARGET '
                f'(elapsed {self._state_elapsed():.1f}s). '
                f'Clearing stale target stability and continuing to wait.')
            self._clear_selected_plan('WAIT_PRE_TARGET stale target stability reset')
            self._target_mgr.reset_stability()
            self.state_start_time = self._now_sec()
            return

        if self._target_mgr.has_stable_target():
            stable = self._target_mgr.get_stable_target()
            if stable is None:
                self.get_logger().warning(
                    'TargetManager reported stable target, but stable_target is None; '
                    'stay in WAIT_PRE_TARGET.')
                return
            try:
                self._ctx.fixed_target_snapshot = (
                    self._target_mgr.make_fixed_snapshot(self._now_sec()))
            except RuntimeError as exc:
                self.get_logger().warning(
                    f'Cannot create fixed target snapshot yet: {exc}; '
                    'stay in WAIT_PRE_TARGET.')
                return

            self.pre_target = [stable.x, stable.y, stable.z]
            if not self.target_frozen:
                self.active_target_base = list(self.pre_target)
            self._start_approach_sequence()
            plan = self._preflight_plan_approach_modes(self.pre_target)
            if plan is None or not plan.ok:
                self.get_logger().error(
                    'All approach modes rejected before opening gripper; stay in WAIT_PRE_TARGET.')
                self._clear_selected_plan('preflight failed')
                self._target_mgr.reset_stability()
                self.state_start_time = self._now_sec()
                return
            self._activate_selected_plan(plan, self._fixed_target_xyz() or self.pre_target)
            self.current_approach_mode = plan.approach_mode
            self.planner.set_approach_mode(plan.approach_mode)
            self.target_frozen = True
            self._target_mgr.freeze()
            self.stage_full_goal = None
            self.blend_waypoints = None
            self.get_logger().info(
                'Frozen target selected from stability window: '
                f'{self._fmt_xyz(self.pre_target)}; '
                f'fixed_target_snapshot={self._fmt_xyz(self._fixed_target_xyz())}')
            self.get_logger().info(
                f'Selected plan frozen: plan_id={plan.plan_id}, mode={plan.approach_mode}, '
                f'pre_grasp={self._fmt_xyz(plan.pre_grasp)}, '
                f'grasp={self._fmt_xyz(plan.grasp)}, lift_goal={self._fmt_xyz(plan.lift_goal)}')
            self._transition('PRE_OPEN_GRIPPER')

    def _handle_pre_open_gripper(self):
        """Open gripper before the grasp sequence starts.

        Only triggered when open_gripper_before_grasp=true.  Sends a
        single 'open' command, waits for executor to finish, then an
        additional settle period before advancing to MOVE_PRE_GRASP.
        """
        if self._state_elapsed() > self._param_float('pre_grasp_open_timeout_sec'):
            self.get_logger().error('PRE_OPEN_GRIPPER timeout.')
            self._enter_recover('PRE_OPEN_GRIPPER timeout')
            return

        if not bool(self.get_parameter('open_gripper_before_grasp').value):
            self.get_logger().info(
                'open_gripper_before_grasp=false; skipping PRE_OPEN_GRIPPER.')
            self._go_to_pre_grasp_after_open()
            return

        if self.pending_speed_profile is not None:
            return

        if not self.state_command_sent:
            if not self._executor_accepting():
                return
            self._publish_gripper_command('open')
            self.state_command_sent = True
            self.gripper_settle_start = None
            self.get_logger().info('Pre-grasp open gripper command sent once.')
            return

        # Wait for executor to finish the gripper command
        if not self._executor_accepting():
            return

        if self.gripper_settle_start is None:
            self.gripper_settle_start = self._now_sec()
            self.get_logger().info('Pre-grasp gripper open done; continue to MOVE_PRE_GRASP.')
            return

        if self._now_sec() - self.gripper_settle_start >= self._param_float('pre_grasp_open_settle_sec'):
            self._go_to_pre_grasp_after_open()

    def _start_approach_sequence(self):
        priority = self._approach_priority()
        self.current_approach_index = 0
        self.current_approach_mode = priority[0]
        self.approach_failed_modes.clear()
        self.approach_retry_count = 0
        self.planner.set_approach_mode(self.current_approach_mode)
        self.get_logger().info(
            f'Start approach sequence: current_approach_mode={self.current_approach_mode}, '
            f'approach_priority={priority}.')

    def _approach_priority(self):
        raw = list(self.get_parameter('approach_priority').value)
        priority = [str(item).strip().lower() for item in raw if str(item).strip()]
        if not priority:
            priority = [str(self.get_parameter('approach_mode').value).strip().lower()]
        clean = []
        for mode in priority:
            if mode in ('front', 'top_down') and mode not in clean:
                clean.append(mode)
        if not clean:
            clean = ['front', 'top_down']
        return clean

    def _validate_grasp_start_constraints(self, target: list, log_errors: bool = True) -> bool:
        self.last_target_failure_reason = None
        try:
            self.planner.validate_front_grasp_x_offset()
        except Exception as exc:
            self.last_target_failure_reason = f'invalid front_grasp_x_offset: {exc}'
            if log_errors:
                self.get_logger().error(
                    f'Grasp rejected before motion start: {exc}')
            return False

        try:
            target_point = self.planner.validate_official_workspace(
                target, label='target_pose_base')
            final_point = self.planner.compute_final_grasp_point(target)
            final_radius = self.planner.compute_radius(final_point)
            self.planner.validate_official_workspace(
                final_point, label='final_grasp_point')
            if log_errors:
                self.get_logger().info(
                    'Official workspace check passed: '
                    f'target_pose_base_radius={self.planner.compute_radius(target_point):.3f}m, '
                    f'final_grasp_radius={final_radius:.3f}m, '
                    f'official_reach_radius_m={self._param_float("official_reach_radius_m"):.3f}m.')
            return True
        except Exception as exc:
            self.last_target_failure_reason = (
                f'OUT_OF_OFFICIAL_WORKSPACE: {exc}. '
                'Base adjustment required before grasp.')
            if log_errors:
                self.get_logger().error(
                    'OUT_OF_OFFICIAL_WORKSPACE: final grasp point exceeds official reach. '
                    f'current_approach_mode={self.current_approach_mode}, '
                    f'target_pose_base={self._fmt_xyz(target)}, '
                    'Please adjust the mobile base before retrying.')
            return False

    def _log_plan_radius_check(self, result: PlanningResult) -> bool:
        official_radius = self._param_float('official_reach_radius_m')
        pre_radius = self.planner.compute_radius(result.pre_grasp)
        grasp_radius = self.planner.compute_radius(result.grasp)
        lift_radius = self.planner.compute_radius(result.lift_goal)
        self.get_logger().info(
            'Plan radius check: '
            f'approach_mode={result.approach_mode}, '
            f'pre_grasp radius={pre_radius:.3f}m, '
            f'grasp radius={grasp_radius:.3f}m, '
            f'lift_goal radius={lift_radius:.3f}m, '
            f'official_reach_radius_m={official_radius:.3f}m.')

        if pre_radius > official_radius + 0.05:
            result.reason = (
                f'pre_grasp radius={pre_radius:.3f}m exceeds '
                f'official_reach_radius_m + 0.05={official_radius + 0.05:.3f}m')
            self.last_target_failure_reason = result.reason
            self.get_logger().error(
                f'Plan rejected: {result.reason}, '
                f'pre_grasp={self._fmt_xyz(result.pre_grasp)}.')
            return False
        if pre_radius > official_radius:
            self.get_logger().warning(
                'pre_grasp is slightly outside official reach radius; '
                f'pre_grasp radius={pre_radius:.3f}m, '
                f'official_reach_radius_m={official_radius:.3f}m, '
                f'pre_grasp={self._fmt_xyz(result.pre_grasp)}.')
        return True

    def _preflight_plan_approach_modes(self, target: list) -> Optional[PlanningResult]:
        failures = []
        for mode in self._approach_priority():
            self.current_approach_mode = mode
            self.planner.set_approach_mode(mode)
            result = self._try_plan_mode(target, mode)
            if result.ok:
                return result
            failures.append(result)
            self.approach_failed_modes.add(mode)
            message = build_workspace_rejection_message(
                reason=result.reason,
                target_base=target,
                attempted_pre_grasp=result.pre_grasp,
                attempted_grasp=result.grasp,
                current_end_pose=self.last_end_pose,
                workspace_limits=self.planner.workspace_limits,
                approach_mode=mode,
                front_approach_x_offset=self._param_float('front_approach_x_offset'),
            )
            if mode == 'front' and 'top_down' in self._approach_priority():
                self.get_logger().warning(
                    f'{message}; trying next approach mode.')
            else:
                self.get_logger().error(message)

        self.last_target_failure_reason = '; '.join(
            f'{item.approach_mode}: {item.reason}' for item in failures)
        return PlanningResult(
            approach_mode='none',
            target=list(target),
            reason=self.last_target_failure_reason,
            ok=False,
        )

    def _activate_selected_plan(self, plan: PlanningResult, target_snapshot: list):
        self.plan_id_counter += 1
        plan.plan_id = self.plan_id_counter
        plan.created_time_sec = self._now_sec()
        plan.target_snapshot = list(target_snapshot)
        self.selected_plan = plan

    def _try_plan_mode(self, target: list, mode: str) -> PlanningResult:
        result = PlanningResult(approach_mode=mode, target=list(target), ok=False)
        self.planner.last_attempted_pre_grasp = None
        self.planner.last_attempted_grasp = None
        try:
            if mode == 'top_down':
                allowed, reason = self._top_down_fallback_allowed(target)
                if not allowed:
                    result.reason = reason
                    self.last_target_failure_reason = reason
                    return result
            if not self._validate_grasp_start_constraints(target, log_errors=False):
                result.reason = self.last_target_failure_reason or 'grasp start constraints failed'
                return result
            result.pre_grasp = self.planner.compute_safe_pre_grasp(target)
            if mode == 'top_down':
                pre_radius = self.planner.compute_radius(result.pre_grasp)
                max_radius = self._param_float('top_down_max_pre_grasp_radius_m')
                if pre_radius > max_radius:
                    result.reason = (
                        f'top_down pre_grasp radius={pre_radius:.3f}m exceeds '
                        f'top_down_max_pre_grasp_radius_m={max_radius:.3f}m')
                    self.last_target_failure_reason = result.reason
                    return result
            result.grasp = self.planner.compute_safe_grasp(target)
            result.lift_goal = self._compute_lift_goal_for_plan(result)
            if not self._log_plan_radius_check(result):
                return result
            result.ok = True
            return result
        except Exception as exc:
            result.pre_grasp = (
                list(self.planner.last_attempted_pre_grasp)
                if self.planner.last_attempted_pre_grasp is not None
                else result.pre_grasp
            )
            result.grasp = (
                list(self.planner.last_attempted_grasp)
                if self.planner.last_attempted_grasp is not None
                else result.grasp
            )
            result.reason = str(exc)
            self.last_target_failure_reason = result.reason
            return result

    def _top_down_fallback_allowed(self, target: list) -> tuple:
        if not bool(self.get_parameter('enable_top_down_fallback').value):
            return False, 'top_down fallback disabled by enable_top_down_fallback=false'
        max_z = self._param_float('top_down_max_target_z')
        if float(target[2]) > max_z:
            return (
                False,
                f'top_down rejected: target.z={float(target[2]):.3f} exceeds '
                f'top_down_max_target_z={max_z:.3f}',
            )
        return True, 'top_down fallback allowed'

    def _compute_lift_goal_for_plan(self, result: PlanningResult) -> list:
        lift_offset = self._param_float('lift_z_offset')
        grasp_z = float(result.grasp[2])
        pre_z = float(result.pre_grasp[2])
        raw_z = grasp_z + lift_offset
        if result.approach_mode == 'top_down':
            lift_z = min(pre_z, raw_z)
        else:
            lift_z = raw_z
        lift_goal = [float(result.grasp[0]), float(result.grasp[1]), float(lift_z)]
        if not self._in_workspace(lift_goal):
            raise ValueError(
                f'lift_goal outside workspace: {self._fmt_xyz(lift_goal)}, '
                f'workspace_limits={self.planner.workspace_limits}')
        if lift_z < self.planner.safe_motion_z:
            self.get_logger().warning(
                'MOVE_LIFT goal generation below safe_motion_z; later motion validation may clamp. '
                f'approach_mode={result.approach_mode}, grasp_z={grasp_z:.3f}, '
                f'pre_grasp_z={pre_z:.3f}, configured_lift_z_offset={lift_offset:.3f}, '
                f'lift_goal_z={lift_z:.3f}, safe_motion_z={self.planner.safe_motion_z:.3f}.')
        self.get_logger().info(
            'MOVE_LIFT goal generation: '
            f'approach_mode={result.approach_mode}, '
            f'grasp_z={grasp_z:.3f}, pre_grasp_z={pre_z:.3f}, '
            f'configured_lift_z_offset={lift_offset:.3f}, lift_goal_z={lift_z:.3f}.')
        return lift_goal

    def _go_to_pre_grasp_after_open(self):
        if self.selected_plan is None or not self.selected_plan.ok:
            self.get_logger().error(
                'Cannot enter approach motion: selected_plan missing or invalid; '
                'return to WAIT_PRE_TARGET without motion.')
            self._transition('WAIT_PRE_TARGET', clear_window=True)
            return
        self._set_speed_profile('default')
        self.get_logger().info('Approach speed_profile set to default for MOVE_APPROACH_BLEND / MOVE_PRE_GRASP.')
        if self._should_use_blend_approach():
            self._transition('MOVE_APPROACH_BLEND')
        else:
            self._transition('MOVE_PRE_GRASP')

    def _ensure_approach_mode_started(self):
        if self.current_approach_mode is None:
            self._start_approach_sequence()
        else:
            self.planner.set_approach_mode(self.current_approach_mode)

    def _should_use_blend_approach(self) -> bool:
        if not bool(self.get_parameter('blend_approach_enabled').value):
            return False
        return True

    def _fallback_blend_to_sequential(self, reason: str):
        if bool(self.get_parameter('blend_approach_fallback_to_sequential').value):
            self.get_logger().warning(
                f'BLEND_APPROACH: fallback to sequential MOVE_PRE_GRASP -> MOVE_GRASP; '
                f'reason={reason}; plan_id={self._selected_plan_id()}')
            if self.executor_status in ('ERROR', 'TIMEOUT'):
                self._publish_reset_executor('clear_error')
                self.last_reset_executor_time = self._now_sec()
            self._reset_stage_vars()
            self._transition('MOVE_PRE_GRASP')
            return

        self.get_logger().error(
            'BLEND_APPROACH: waypoint execution failed; fallback disabled; entering RECOVER')
        self._enter_recover(reason)

    def _handle_move_approach_blend(self):
        if self._state_elapsed() > self._param_float('motion_timeout_sec'):
            self.get_logger().error('MOVE_APPROACH_BLEND timeout.')
            self._fallback_blend_to_sequential('MOVE_APPROACH_BLEND timeout')
            return

        if self.blend_busy_retry_after is not None:
            if self._now_sec() < self.blend_busy_retry_after:
                return
            self.blend_busy_retry_after = None
            self.state_command_sent = False
            self.stage_motion_started = False

        if self.pending_speed_profile is not None:
            return

        if self.cartesian_busy_retry_after is not None:
            if self._now_sec() < self.cartesian_busy_retry_after:
                return
            self.cartesian_busy_retry_after = None
            self.state_command_sent = False
            self.stage_motion_started = False

        if not self._fresh_end_pose_available():
            return

        if self.blend_waypoints is None:
            try:
                self.blend_waypoints = self._build_blend_waypoints_from_selected_plan()
            except Exception as exc:
                reason = f'MOVE_APPROACH_BLEND selected_plan waypoint error: {exc}'
                self.get_logger().error(reason)
                if self.selected_plan is None or not self.selected_plan.ok:
                    self._transition('WAIT_PRE_TARGET', clear_window=True)
                else:
                    self._fallback_blend_to_sequential(reason)
                return

        if not self.state_command_sent:
            if not self._executor_accepting():
                return
            try:
                max_segment = self._max_segment_length(
                    self.last_end_pose, self.blend_waypoints)
                self.get_logger().info(
                    f'[GRASP] MOVE_APPROACH_BLEND plan_id={self._selected_plan_id()}, '
                    f'waypoint_count={len(self.blend_waypoints)}, '
                    f'max_segment={max_segment:.3f}, '
                    f'first={self._fmt_xyz(self.blend_waypoints[0])}, '
                    f'pre_grasp={self._fmt_xyz(self.selected_plan.pre_grasp)}, '
                    f'final={self._fmt_xyz(self.blend_waypoints[-1])}')
                self._publish_cart_waypoints(
                    self.blend_waypoints, reason='blend_approach')
            except Exception as exc:
                self.get_logger().error(
                    f'MOVE_APPROACH_BLEND publish failed: {exc}')
                self._fallback_blend_to_sequential(
                    f'MOVE_APPROACH_BLEND publish failed: {exc}')
                return
            self.state_command_sent = True
            self.stage_motion_started = False
            self.active_motion_goal = list(self.blend_waypoints[-1])
            return

        if self.executor_status == 'BUSY':
            self.stage_motion_started = True
            return

        if self.executor_status == 'DONE' or (
            self.stage_motion_started and self.executor_status == 'IDLE'
        ):
            self.get_logger().info(
                f'BLEND_APPROACH: cart_waypoints done; plan_id={self._selected_plan_id()}; '
                'entering CLOSE_GRIPPER')
            self._reset_stage_vars()
            self._after_move_grasp()

    def _build_blend_waypoints_from_selected_plan(self) -> list:
        if self.selected_plan is None or not self.selected_plan.ok:
            raise ValueError('selected_plan missing or invalid')
        if self.selected_plan.pre_grasp is None or self.selected_plan.grasp is None:
            raise ValueError('selected_plan.pre_grasp/grasp missing')

        pre_grasp = self.planner.validate_waypoint(
            list(self.selected_plan.pre_grasp),
            is_final_grasp=False,
            label='selected_plan.pre_grasp',
        )
        grasp = self.planner.validate_waypoint(
            list(self.selected_plan.grasp),
            is_final_grasp=True,
            label='selected_plan.grasp',
        )
        waypoints = self._build_blended_approach_waypoints(
            self.last_end_pose, pre_grasp, grasp)

        if len(waypoints) < 2:
            raise ValueError('cart_waypoints requires at least 2 points')
        return waypoints

    def _build_blended_approach_waypoints(
        self,
        current_xyz: list,
        pre_grasp: list,
        grasp: list,
    ) -> list:
        if current_xyz is None or len(current_xyz) != 3:
            raise ValueError('fresh current end_pose is required for cart_waypoints')

        max_step = self._param_float('cart_waypoint_max_step_m')
        safe_limit = self._param_float('cart_waypoint_safe_limit_m')
        if max_step <= 0.0:
            raise ValueError('cart_waypoint_max_step_m must be positive')
        if safe_limit <= 0.0:
            raise ValueError('cart_waypoint_safe_limit_m must be positive')
        if max_step > safe_limit:
            raise ValueError(
                f'cart_waypoint_max_step_m={max_step:.3f} exceeds '
                f'cart_waypoint_safe_limit_m={safe_limit:.3f}')

        current = [float(v) for v in current_xyz]
        pre = [float(v) for v in pre_grasp]
        final = [float(v) for v in grasp]

        waypoints = []
        waypoints.extend(self._interpolate_segment(current, pre, max_step))
        waypoints.extend(self._interpolate_segment(pre, final, max_step))

        if not waypoints or self._distance(waypoints[-1], final) > 1e-9:
            waypoints.append(final)

        max_segment = self._max_segment_length(current, waypoints)
        if max_segment > safe_limit + 1e-9:
            raise ValueError(
                f'cart_waypoints max_segment={max_segment:.3f} exceeds '
                f'safe_limit={safe_limit:.3f}')
        if not any(self._distance(point, pre) <= 1e-9 for point in waypoints):
            raise ValueError('pre_grasp is missing from cart_waypoints')
        if self._distance(waypoints[-1], final) > 1e-9:
            raise ValueError('final cart waypoint is not selected_plan.grasp')
        return waypoints

    def _interpolate_segment(self, start: list, end: list, max_step: float) -> list:
        distance = self._distance(start, end)
        if distance <= 1e-9:
            return []
        steps = max(1, int(math.ceil(distance / max_step)))
        points = []
        for index in range(1, steps + 1):
            ratio = float(index) / float(steps)
            points.append([
                float(start[0]) + (float(end[0]) - float(start[0])) * ratio,
                float(start[1]) + (float(end[1]) - float(start[1])) * ratio,
                float(start[2]) + (float(end[2]) - float(start[2])) * ratio,
            ])
        return points

    def _max_segment_length(self, current_xyz: list, waypoints: list) -> float:
        if current_xyz is None or not waypoints:
            return 0.0
        max_segment = 0.0
        previous = [float(v) for v in current_xyz]
        for point in waypoints:
            segment = self._distance(previous, point)
            if segment > max_segment:
                max_segment = segment
            previous = point
        return max_segment

    def _handle_move_pre_grasp(self):
        if self.stage_full_goal is not None:
            goal_fn = lambda: self.stage_full_goal
        else:
            def goal_fn():
                if self.selected_plan is None or not self.selected_plan.ok or self.selected_plan.pre_grasp is None:
                    self.last_target_failure_reason = 'selected_plan.pre_grasp missing'
                    self.get_logger().error(
                        'MOVE_PRE_GRASP requires selected_plan.pre_grasp; no dynamic recompute in main path.')
                    return None
                goal = list(self.selected_plan.pre_grasp)
                if goal is not None:
                    self.stage_full_goal = list(goal)
                    self.get_logger().info(
                        f'MOVE_PRE_GRASP fixed_goal={self._fmt_xyz(self.stage_full_goal)} '
                        f'from selected_plan={self.selected_plan.plan_id}.')
                return goal
        self._handle_cartesian_motion(
            'MOVE_PRE_GRASP',
            goal_fn,
            on_done=self._after_move_pre_grasp,
        )

    def _after_move_pre_grasp(self):
        self._set_speed_profile('slow')
        self._transition('MOVE_GRASP')

    def _handle_move_grasp(self):
        if self.stage_full_goal is not None:
            goal_fn = lambda: self.stage_full_goal
        else:
            def goal_fn():
                if self.selected_plan is None or not self.selected_plan.ok or self.selected_plan.grasp is None:
                    self.last_target_failure_reason = 'selected_plan.grasp missing'
                    self.get_logger().error(
                        'MOVE_GRASP requires selected_plan.grasp; no dynamic recompute in main path.')
                    return None
                goal = list(self.selected_plan.grasp)
                if goal is not None:
                    self.stage_full_goal = list(goal)
                    self.get_logger().info(
                        f'MOVE_GRASP fixed_goal={self._fmt_xyz(self.stage_full_goal)} '
                        f'from selected_plan={self.selected_plan.plan_id}.')
                return goal
        self._handle_cartesian_motion(
            'MOVE_GRASP',
            goal_fn,
            on_done=self._after_move_grasp,
        )

    def _after_move_grasp(self):
        if bool(self.get_parameter('freeze_target_before_close').value):
            target = self._get_fixed_snapshot_target()
            if target is None:
                self.get_logger().error(
                    'Cannot freeze target before close: no fixed target snapshot.')
                self._enter_recover()
                return
            self.active_target_base = list(target)
            self.target_frozen = True
            self.get_logger().info(
                f'Target frozen before close: {self._fmt_xyz(self.active_target_base)}')
        self._transition('CLOSE_GRIPPER')

    def _compute_pre_grasp_from_active_target(self):
        self._ensure_approach_mode_started()
        target = self._get_fixed_snapshot_target()
        if target is None:
            return None
        self.pre_target = list(target)

        try:
            if not self._validate_grasp_start_constraints(target):
                return None
            pre_grasp = self.planner.compute_safe_pre_grasp(target)
            if self.last_end_pose is not None:
                safe_z = max(float(self.planner.safe_motion_z), float(pre_grasp[2]))
                current_z = float(self.last_end_pose[2])
                if current_z < float(target[2]) or current_z < self.planner.safe_motion_z:
                    lift_first = [
                        float(self.last_end_pose[0]),
                        float(self.last_end_pose[1]),
                        safe_z,
                    ]
                    lift_first = self.planner.validate_waypoint(lift_first)
                    self._log_waypoint_safety(
                        'MOVE_PRE_GRASP', target, pre_grasp=lift_first)
                    return lift_first
                self.planner.validate_approach_direction(
                    self.last_end_pose, target, self.planner.approach_mode)
            self._log_waypoint_safety(
                'MOVE_PRE_GRASP', target, pre_grasp=pre_grasp)
            return pre_grasp
        except Exception as exc:
            self.get_logger().error(
                f'MOVE_PRE_GRASP safety validation failed: {exc}; '
                f'current_approach_mode={self.current_approach_mode}, '
                f'target_base={self._fmt_xyz(target)}, '
                f'active_target_base={self._fmt_xyz(self.active_target_base) if self.active_target_base else None}, '
                f'last_seen_target_base={self._fmt_xyz(self.last_seen_target_base) if self.last_seen_target_base else None}, '
                f'workspace_limits={self.planner.workspace_limits}.')
            return None

    def _compute_grasp_from_active_target(self):
        self._ensure_approach_mode_started()
        target = self._get_fixed_snapshot_target()
        if target is None:
            return None
        self.grasp_target = list(target)

        try:
            if not self._validate_grasp_start_constraints(target):
                return None
            grasp = self.planner.compute_safe_grasp(target)
            if self.last_end_pose is not None:
                self.planner.validate_approach_direction(
                    self.last_end_pose, target, self.planner.approach_mode)
                if self.planner.approach_mode == 'top_down':
                    xy_delta = self._distance(
                        [self.last_end_pose[0], self.last_end_pose[1], 0.0],
                        [grasp[0], grasp[1], 0.0],
                    )
                    if xy_delta > max(self._position_tolerance(), 0.03):
                        if self.approach_retry_count < self._param_int('max_approach_mode_retries'):
                            self.approach_retry_count += 1
                            self.get_logger().warning(
                                f'top_down XY not aligned (xy_delta={xy_delta:.3f}m); '
                                f'retry MOVE_PRE_GRASP {self.approach_retry_count}/'
                                f'{self._param_int("max_approach_mode_retries")}.')
                            self._transition('MOVE_PRE_GRASP')
                            return None
                        raise ValueError(
                            f'top_down grasp requires near-vertical descent; xy_delta={xy_delta:.3f}m.')
                    if float(grasp[2]) > float(self.last_end_pose[2]) + self._position_tolerance():
                        raise ValueError(
                            f'top_down grasp would approach from below: current_z={self.last_end_pose[2]:.3f}, grasp_z={grasp[2]:.3f}.')
            self._log_waypoint_safety('MOVE_GRASP', target, grasp=grasp)
            return grasp
        except Exception as exc:
            pre_grasp = None
            try:
                pre_grasp = self.planner.compute_safe_pre_grasp(target)
            except Exception:
                pass
            self.get_logger().error(
                f'MOVE_GRASP safety validation failed: {exc}; '
                f'current_approach_mode={self.current_approach_mode}, '
                f'target_base={self._fmt_xyz(target)}, '
                f'pre_grasp={self._fmt_xyz(pre_grasp) if pre_grasp else None}, '
                f'active_target_base={self._fmt_xyz(self.active_target_base) if self.active_target_base else None}, '
                f'last_seen_target_base={self._fmt_xyz(self.last_seen_target_base) if self.last_seen_target_base else None}, '
                f'workspace_limits={self.planner.workspace_limits}.')
            return None

    def _log_waypoint_safety(self, state_name: str, target: list, pre_grasp=None, grasp=None):
        parts = [
            f'{state_name} safety:',
            f'current_approach_mode={self.current_approach_mode}',
            f'approach_mode={self.planner.approach_mode}',
            f'target_base={self._fmt_xyz(target)}',
            f'active_target_base={self._fmt_xyz(self.active_target_base) if self.active_target_base else None}',
            f'last_seen_target_base={self._fmt_xyz(self.last_seen_target_base) if self.last_seen_target_base else None}',
            f'last_seen_age={self._last_seen_age_sec():.2f}s' if self._last_seen_age_sec() is not None else 'last_seen_age=None',
            f'table_z={self.planner.table_z:.3f}',
            f'min_safe_motion_z={self.planner.min_safe_motion_z:.3f}',
        ]
        if pre_grasp is not None:
            parts.append(f'pre_grasp={self._fmt_xyz(pre_grasp)}')
        if grasp is not None:
            parts.append(f'grasp={self._fmt_xyz(grasp)}')
        self._debug_or_info(', '.join(parts))

    def _fixed_target_xyz(self):
        snapshot = self._ctx.fixed_target_snapshot
        if snapshot is None:
            return None
        return [float(snapshot.x), float(snapshot.y), float(snapshot.z)]

    def _get_fixed_snapshot_target(self):
        target = self._fixed_target_xyz()
        if target is None:
            self.last_target_failure_reason = 'fixed_target_snapshot missing'
            self.get_logger().warning(
                'No fixed_target_snapshot available; motion stage cannot compute goal.')
            return None
        return target

    def _get_active_target_or_last_seen(self):
        now = self._now_sec()
        max_age = self._param_float('last_seen_target_max_age_sec')
        grace_sec = self._param_float('visual_lost_grace_sec')
        self.last_target_failure_reason = None

        if self.last_seen_target_base is None or self.last_seen_target_time is None:
            self.get_logger().error('No last-seen /visual_target_base available.')
            self.last_target_failure_reason = 'last_seen_target_base missing'
            return None

        age = now - self.last_seen_target_time
        if age > max_age:
            self.get_logger().error(
                f'Last-seen base target is too old: age={age:.2f}s > {max_age:.2f}s.')
            self.last_target_failure_reason = (
                f'last_seen_target_base age {age:.2f}s exceeds max {max_age:.2f}s')
            return None

        use_last_seen = bool(self.get_parameter('use_last_seen_target_on_loss').value)
        if self.task_state in ('MOVE_PRE_GRASP', 'MOVE_GRASP', 'MOVE_LIFT'):
            use_last_seen = (
                use_last_seen
                and bool(self.get_parameter('continue_with_last_seen_during_motion').value)
            )
        if self.active_target_base is not None and not self.target_frozen:
            if age <= grace_sec:
                self.get_logger().debug(
                    f'Using active_target_base={self._fmt_xyz(self.active_target_base)}, '
                    f'last_seen_age={age:.2f}s.')
                return list(self.active_target_base)
            if not use_last_seen:
                self.get_logger().error('Visual target lost and last-seen fallback is disabled.')
                self.last_target_failure_reason = 'last-seen fallback disabled'
                return None
            self._warn_visual_lost(age)
            self.get_logger().warning(
                f'No fresh visual target for {age:.2f}s; continue motion with '
                f'last_seen_target_base={self._fmt_xyz(self.last_seen_target_base)} '
                f'because age <= {max_age:.2f}s.')
            return list(self.active_target_base)

        if not use_last_seen:
            self.get_logger().error('Visual target lost and last-seen fallback is disabled.')
            self.last_target_failure_reason = 'last-seen fallback disabled'
            return None

        if age > grace_sec:
            self._warn_visual_lost(age)
            self.get_logger().warning(
                f'Using last_seen_target_base={self._fmt_xyz(self.last_seen_target_base)}, '
                f'last_seen_age={age:.2f}s; target is temporarily lost but within max age.')
        return list(self.last_seen_target_base)

    def _warn_visual_lost(self, age: float):
        now = self._now_sec()
        if (
            self.last_visual_lost_warning_time is None
            or now - self.last_visual_lost_warning_time >= 1.0
        ):
            self.get_logger().warning(
                'Visual target temporarily lost; using last seen base target.')
            self.last_visual_lost_warning_time = now

    def _handle_close_gripper(self):
        if self._state_elapsed() > self._param_float('close_gripper_timeout_sec'):
            self.get_logger().error('CLOSE_GRIPPER timeout.')
            self._enter_recover()
            return

        if not self.state_command_sent:
            if not self._executor_accepting():
                return
            self._publish_gripper_command('close')
            self.state_command_sent = True
            self.gripper_settle_start = self._now_sec()
            self.get_logger().info('Gripper close command sent once.')
            return

        if self.gripper_settle_start is None:
            self.gripper_settle_start = self._now_sec()

        if self._now_sec() - self.gripper_settle_start >= self._param_float('gripper_settle_sec'):
            if not self._fresh_end_pose_available():
                self.get_logger().error('No fresh end_pose for lift target.')
                self._enter_recover()
                return
            self.grasp_closed = True
            if (
                self.selected_plan is not None
                and self.selected_plan.ok
                and self.selected_plan.lift_goal is not None
            ):
                self.lift_goal = list(self.selected_plan.lift_goal)
                self.get_logger().info(
                    f'MOVE_LIFT fixed_goal={self._fmt_xyz(self.lift_goal)} '
                    f'from selected_plan={self.selected_plan.plan_id}.')
            else:
                self.lift_goal = self.planner.compute_safe_lift(self.last_end_pose)
                self.get_logger().warning(
                    'MOVE_LIFT selected_plan.lift_goal missing; fallback to current end_pose lift.')
            self.get_logger().info(
                f'MOVE_LIFT fixed_goal={self._fmt_xyz(self.lift_goal)}')
            self.get_logger().info(
                'Grasp closed; keep gripper closed during lift/return.')
            self._transition('MOVE_LIFT')

    def _handle_move_lift(self):
        if self.lift_goal is None:
            self.get_logger().error('MOVE_LIFT has no frozen lift_goal.')
            self._enter_post_grasp_recover('MOVE_LIFT missing lift_goal')
            return

        if not self._fresh_end_pose_available():
            if self._state_elapsed() > self._param_float('motion_timeout_sec'):
                self.get_logger().error('MOVE_LIFT has no fresh end_pose.')
                self._enter_post_grasp_recover('MOVE_LIFT no fresh end_pose')
            return

        self._handle_cartesian_motion(
            'MOVE_LIFT',
            lambda: list(self.lift_goal),
            on_done=self._after_move_lift,
            step_param='lift_cartesian_step_m',
            failure_handler=self._enter_post_grasp_recover,
        )

    def _after_move_lift(self):
        if bool(self.get_parameter('return_to_init_after_grasp').value):
            self.get_logger().info(
                'MOVE_LIFT done; entering RETURN_INIT_POSE. '
                'Grasp succeeded; keeping gripper closed. '
                f'{list(self.get_parameter("final_init_joint_pos_deg").value)}')
            self._transition('RETURN_INIT_POSE')
        else:
            self._transition('MOVE_RETREAT')

    def _handle_return_init_pose(self):
        if self._state_elapsed() > self._param_float('return_init_timeout_sec'):
            self.get_logger().error('RETURN_INIT_POSE timeout.')
            self._enter_post_grasp_recover('RETURN_INIT_POSE timeout')
            return

        if self.pending_speed_profile is not None:
            return

        if self.last_joint_pos is None:
            return

        if not self.state_command_sent:
            if not self._executor_accepting():
                return
            init_deg = list(self.get_parameter('final_init_joint_pos_deg').value)
            init_rad = [math.radians(float(v)) for v in init_deg]
            self._publish_joint_target(init_rad)
            self.state_command_sent = True
            self.stage_motion_started = True
            self.get_logger().info(
                f'RETURN_INIT_POSE: sent joint target [{[f"{v:.1f}" for v in init_deg]}] deg.')
            return

        if self.last_joint_vel is None:
            return

        max_speed = max(abs(float(v)) for v in self.last_joint_vel)
        if max_speed > self._param_float('joint_speed_safe_threshold'):
            self.settle_start_time = None
            return

        if self.settle_start_time is None:
            self.settle_start_time = self._now_sec()
            return

        if self._now_sec() - self.settle_start_time >= self._param_float('gripper_settle_sec'):
            self.get_logger().info('Return init pose reached; cycle complete.')
            self.grasp_closed = False
            self._clear_selected_plan('RETURN_INIT_POSE complete')
            self._finish_cycle()

    def _handle_move_retreat(self):
        self._set_speed_profile('default')
        self.get_logger().info('Retreat speed_profile set to default for MOVE_RETREAT.')
        self._handle_cartesian_motion(
            'MOVE_RETREAT',
            lambda: self._compute_safe_retreat_goal(),
            on_done=self._finish_cycle,
        )

    def _handle_recover(self):
        if self._state_elapsed() > self._param_float('recover_timeout_sec'):
            if not self.recover_timeout_logged:
                self.get_logger().error(
                    'RECOVER timeout; reset state to IDLE after best-effort recovery.')
                self.recover_timeout_logged = True
            self.grasp_closed = False
            self._reset_cycle()
            self._transition('IDLE')
            return

        if self.executor_status == 'BUSY':
            self.get_logger().warning(
                'RECOVER: executor BUSY, waiting for current recover command.')
            return

        if self.executor_status in ('ERROR', 'TIMEOUT'):
            if not self.recover_command_sent:
                reset_command = 'clear_error'
                if bool(self.get_parameter('auto_recover_joint_limit').value):
                    reset_command = 'recover_joint_limit'
                self._publish_reset_executor(reset_command)
                self.last_reset_executor_time = self._now_sec()
                self.recover_command_sent = True
                self.recover_command_type = reset_command
                self.recover_command_time = self.last_reset_executor_time
                if reset_command == 'recover_joint_limit':
                    self.get_logger().warning(
                        'RECOVER: request recover_joint_limit')
                else:
                    self.get_logger().warning(
                        'RECOVER: clear executor error.')
            else:
                now = self._now_sec()
                log_interval = max(
                    3.0, self._param_float('recover_clear_error_interval_sec'))
                if (
                    self.last_reset_executor_time is None
                    or now - self.last_reset_executor_time >= log_interval
                ):
                    self.last_reset_executor_time = now
                    self.get_logger().warning(
                        f'RECOVER: {self.recover_command_type} already sent; '
                        'not resending.')
            if self.recover_phase.startswith('KEEP_CLOSED'):
                return
            return

        # ---- Post-grasp recovery: keep gripper closed ----
        if self.recover_phase == 'KEEP_CLOSED_CLEAR_ERROR':
            self.get_logger().warning(
                'Post-grasp recovery: executor error cleared, keep gripper closed.')
            self.recover_phase = 'KEEP_CLOSED_RETURN_INIT'
            self._reset_stage_vars()
            return

        if self.recover_phase == 'KEEP_CLOSED_RETURN_INIT':
            if self.last_joint_pos is None:
                self._finish_recover()
                return
            if not self.state_command_sent:
                if not self._executor_accepting():
                    return
                init_deg = list(self.get_parameter('final_init_joint_pos_deg').value)
                init_rad = [math.radians(float(v)) for v in init_deg]
                self._publish_joint_target(init_rad)
                self.state_command_sent = True
                self.get_logger().info(
                    'Post-grasp recovery: return to init joint pose (gripper stays closed).')
                return
            if self.last_joint_vel is None:
                return
            max_speed = max(abs(float(v)) for v in self.last_joint_vel)
            if max_speed > self._param_float('joint_speed_safe_threshold'):
                return
            self.get_logger().info(
                'Post-grasp recovery complete; gripper stayed closed.')
            self._finish_recover()

        # ---- Normal RECOVER (pre-grasp or gripper not closed) ----
        if self.recover_phase == 'OPEN_GRIPPER':
            if not self.state_command_sent:
                if not self._executor_accepting():
                    return
                self._publish_gripper_command('open')
                self.state_command_sent = True
                self.gripper_settle_start = self._now_sec()
                self.get_logger().warning('RECOVER: open gripper command sent.')
                return
            if self._now_sec() - (self.gripper_settle_start or self._now_sec()) < self._param_float('gripper_settle_sec'):
                return
            self.get_logger().warning('RECOVER: executor error cleared.')
            self.recover_phase = 'SAFE_RETURN_INIT'
            self._reset_stage_vars()
            return

        if self.recover_phase == 'SAFE_RETURN_INIT':
            if self.last_joint_pos is None:
                self._finish_recover()
                return
            if not self.state_command_sent:
                if not self._executor_accepting():
                    return
                init_deg = list(self.get_parameter('final_init_joint_pos_deg').value)
                init_rad = [math.radians(float(v)) for v in init_deg]
                self._publish_joint_target(init_rad)
                self.state_command_sent = True
                self.get_logger().info(
                    'RECOVER: returning to init joint pose.')
                return
            if self.last_joint_vel is None:
                return
            max_speed = max(abs(float(v)) for v in self.last_joint_vel)
            if max_speed > self._param_float('joint_speed_safe_threshold'):
                return
            if self.settle_start_time is None:
                self.settle_start_time = self._now_sec()
                return
            if self._now_sec() - self.settle_start_time >= self._param_float('gripper_settle_sec'):
                self.get_logger().info(
                    'RECOVER: init pose reached; return to IDLE.')
                self._finish_recover()

    def _compute_safe_retreat_goal(self):
        if self.last_end_pose is not None and float(self.last_end_pose[2]) < self.planner.safe_motion_z:
            return self.planner.validate_waypoint([
                float(self.last_end_pose[0]),
                float(self.last_end_pose[1]),
                self.planner.safe_motion_z,
            ])
        return self.planner.get_safe_pose()

    def _handle_cartesian_motion(
        self,
        state_name: str,
        goal_fn,
        on_done,
        timeout_param='motion_timeout_sec',
        step_param='max_cartesian_step',
        failure_handler=None,
    ):
        """Step-by-step Cartesian movement with per-step settle gating.

        Each invocation publishes at most one step_goal (computed via limit_step
        from current end_pose toward full_goal).  The method never publishes
        the full final goal directly unless it is already within position_tolerance.

        When full_goal is reached and the settle timer expires, on_done() fires.
        """
        failure_handler = failure_handler or self._handle_approach_failure

        if self._state_elapsed() > self._param_float(timeout_param):
            self.get_logger().error(f'{state_name} timeout.')
            failure_handler(f'{state_name} timeout')
            return

        if self.pending_speed_profile is not None:
            return

        busy_retry_ready = False
        if self.cartesian_busy_retry_after is not None:
            if self._now_sec() < self.cartesian_busy_retry_after:
                return
            self.cartesian_busy_retry_after = None
            self.state_command_sent = False
            self.stage_motion_started = False
            busy_retry_ready = True

        if not self._fresh_end_pose_available():
            return

        # Resolve the full (final) goal every tick so dynamic goals (e.g. lift)
        # reflect the latest end_pose.
        try:
            full_goal = goal_fn()
        except Exception as exc:
            self.get_logger().error(f'{state_name}: failed to compute motion goal: {exc}')
            failure_handler(f'{state_name} goal exception: {exc}')
            return
        if self.task_state != state_name and state_name in ('MOVE_PRE_GRASP', 'MOVE_GRASP'):
            return
        if full_goal is None or len(full_goal) != 3:
            self.get_logger().error(f'{state_name}: invalid motion goal.')
            if self.last_target_failure_reason is not None:
                self._enter_recover(self.last_target_failure_reason)
                return
            failure_handler(f'{state_name} invalid motion goal')
            return
        full_goal = [float(full_goal[0]), float(full_goal[1]), float(full_goal[2])]
        try:
            full_goal = self.planner.validate_waypoint(
                full_goal,
                is_final_grasp=(state_name == 'MOVE_GRASP'),
            )
        except Exception as exc:
            self.get_logger().error(
                f'{state_name}: full_goal outside safety constraints: {exc}; '
                f'current_approach_mode={self.current_approach_mode}, '
                f'workspace_limits={self.planner.workspace_limits}.')
            failure_handler(f'{state_name} full_goal unsafe: {exc}')
            return

        tolerance = self._position_tolerance()
        distance_to_full = self._distance(self.last_end_pose, full_goal)

        # ---- Already at full goal → settle and finish ----
        if distance_to_full <= tolerance:
            if self.settle_start_time is None:
                self.settle_start_time = self._now_sec()
                self._log_waypoint_step(
                    f'{state_name}: full_goal {self._fmt_xyz(full_goal)} reached '
                    f'(dist={distance_to_full:.4f}m), settling.')
                return

            if self._now_sec() - self.settle_start_time >= self._param_float('settle_time_sec'):
                self._log_waypoint_step(
                    f'{state_name}: full_goal reached and settled.')
                self._reset_stage_vars()
                on_done()
            return

        # ---- Not yet sent the next step → compute and publish step_goal ----
        if not self.state_command_sent:
            if not self._executor_accepting() and not busy_retry_ready:
                return

            if self.active_motion_goal is None:
                max_step = min(
                    self._param_float('max_cartesian_step'),
                    self._param_float(step_param),
                ) if step_param != 'max_cartesian_step' else self._param_float('max_cartesian_step')
                step_goal = self.planner.limit_step(
                    self.last_end_pose, full_goal, max_step)
                is_final_step = self._distance(step_goal, full_goal) <= tolerance
                try:
                    if state_name == 'MOVE_GRASP' and not is_final_step:
                        step_goal = self.planner.validate_waypoint(
                            [
                                step_goal[0],
                                step_goal[1],
                                max(float(step_goal[2]), self.planner.safe_motion_z),
                            ],
                            is_final_grasp=False,
                        )
                    else:
                        step_goal = self.planner.validate_waypoint(
                            step_goal,
                            is_final_grasp=(state_name == 'MOVE_GRASP' and is_final_step),
                        )
                except Exception as exc:
                    self.get_logger().error(f'{state_name}: unsafe step_goal: {exc}')
                    failure_handler(f'{state_name} unsafe step_goal: {exc}')
                    return
                self.active_motion_goal = [
                    float(step_goal[0]),
                    float(step_goal[1]),
                    float(step_goal[2]),
                ]
                if state_name == 'MOVE_LIFT':
                    step_distance = self._distance(self.last_end_pose, self.active_motion_goal)
                    self.get_logger().info(
                        'MOVE_LIFT plan: '
                        f'current_end={self._fmt_xyz(self.last_end_pose)}, '
                        f'lift_goal={self._fmt_xyz(full_goal)}, '
                        f'lift_z_offset={self._param_float("lift_z_offset"):.3f}, '
                        f'lift_step_target={self._fmt_xyz(self.active_motion_goal)}, '
                        f'step_distance={step_distance:.4f}m.')

            self._publish_cart_target(self.active_motion_goal)
            self.state_command_sent = True
            self.stage_motion_started = True
            self.settle_start_time = None

            self._log_waypoint_step(
                f'{state_name}: step_goal={self._fmt_xyz(self.active_motion_goal)}, '
                f'full_goal={self._fmt_xyz(full_goal)}, '
                f'dist_to_full={distance_to_full:.4f}m.')
            return

        # ---- Step command sent; wait for step_goal ----
        if self.active_motion_goal is None:
            self.get_logger().error(f'{state_name}: active_motion_goal is missing.')
            failure_handler(f'{state_name} active_motion_goal missing')
            return

        distance_to_step = self._distance(self.last_end_pose, self.active_motion_goal)

        if distance_to_step > tolerance:
            self.settle_start_time = None
            return

        if self.settle_start_time is None:
            self.settle_start_time = self._now_sec()
            self._log_waypoint_step(
                f'{state_name}: step_goal {self._fmt_xyz(self.active_motion_goal)} reached '
                f'(dist={distance_to_step:.4f}m), settling before next step.')
            return

        if self._now_sec() - self.settle_start_time >= self._param_float('settle_time_sec'):
            self._log_waypoint_step(
                f'{state_name}: step_goal settled; ready for next Cartesian step.')
            self._reset_stage_vars()
            return

    def _schedule_blend_busy_retry(self, reason: str):
        if not bool(self.get_parameter('blend_approach_retry_on_busy').value):
            self._fallback_blend_to_sequential(reason)
            return

        max_retries = self._param_int('blend_approach_busy_max_retries')
        if self.blend_busy_retry_count >= max_retries:
            self._fallback_blend_to_sequential(
                f'{reason}; busy retries exceeded {max_retries}')
            return

        self.blend_busy_retry_count += 1
        delay = self._param_float('blend_approach_busy_retry_delay_sec')
        self.blend_busy_retry_after = self._now_sec() + delay
        self.state_command_sent = True
        self.stage_motion_started = False
        self.get_logger().warning(
            f'BLEND_APPROACH: executor busy / SDK not ready, retry '
            f'{self.blend_busy_retry_count}/{max_retries} after delay.')

    def _schedule_cartesian_busy_retry(self, reason: str):
        max_retries = self._param_int('sequential_busy_max_retries')
        if self.cartesian_busy_retry_count >= max_retries:
            failure = f'{reason}; busy retries exceeded {max_retries}'
            if self.task_state == 'MOVE_LIFT':
                self._enter_post_grasp_recover(failure)
            else:
                self._handle_approach_failure(failure)
            return

        self.cartesian_busy_retry_count += 1
        delay = self._param_float('post_motion_command_cooldown_sec')
        self.cartesian_busy_retry_after = self._now_sec() + delay
        self.state_command_sent = True
        self.stage_motion_started = False
        self.settle_start_time = None
        self.get_logger().warning(
            f'{self.task_state}: executor busy / SDK not ready, retry '
            f'{self.cartesian_busy_retry_count}/{max_retries} after cooldown.')

    def _valid_target(self, msg: VisualTarget) -> bool:
        frame_id = msg.header.frame_id.strip()
        base_frame = self.get_parameter('base_frame').value
        allow_empty = bool(self.get_parameter('allow_empty_target_frame').value)
        if frame_id != base_frame and not (allow_empty and frame_id == ''):
            self.get_logger().warning(
                f'Reject target frame_id={frame_id!r}; expected {base_frame!r}.')
            return False

        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            self.get_logger().warning('Reject target with non-finite coordinates.')
            return False

        if not self._in_workspace([msg.x, msg.y, msg.z]):
            self.get_logger().warning(
                f'Reject target outside workspace: {self._fmt_xyz([msg.x, msg.y, msg.z])}.')
            return False

        confidence = float(msg.confidence)
        if math.isfinite(confidence) and confidence < self._param_float('confidence_threshold'):
            self.get_logger().debug(
                f'Reject low confidence target: {confidence:.3f}.')
            return False

        stamp_age = self._message_age_sec(msg)
        if stamp_age is not None and stamp_age > self._param_float('target_timeout_sec'):
            self.get_logger().warning(
                f'Reject stale target: age={stamp_age:.3f}s.')
            return False

        return True

    def _target_collection_allowed(self) -> bool:
        now = self._now_sec()
        if self.task_state != 'WAIT_PRE_TARGET':
            return False

        if self.target_collection_cooldown_until is not None:
            if now < self.target_collection_cooldown_until:
                return False
            if not self.target_collection_cooldown_logged:
                self.target_collection_cooldown_logged = True
                self.get_logger().info(
                    'Inter-cycle cooldown finished; target collection enabled.')

        if (
            bool(self.get_parameter('require_idle_before_target_collection').value)
            and self.executor_status != 'IDLE'
        ):
            return False

        if bool(self.get_parameter('require_end_pose_stable_before_target_collection').value):
            ok, samples, ranges = self._end_pose_stable_for_collection()
            now = self._now_sec()
            if (
                self.last_end_pose_stability_log_time is None
                or now - self.last_end_pose_stability_log_time >= self._param_float('status_log_period_sec')
            ):
                self.last_end_pose_stability_log_time = now
                self.get_logger().info(
                    f'End pose stability: samples={samples}, '
                    f'range={self._fmt_range(ranges)}, '
                    f'decision={"accepted" if ok else "rejected"}.')
            if not ok:
                return False

        return True

    def _end_pose_stable_for_collection(self) -> tuple:
        required = self._param_int('end_pose_stability_window')
        samples = len(self.end_pose_stability_window)
        if samples < required:
            return False, samples, (0.0, 0.0, 0.0)
        xs = [pose[0] for pose in self.end_pose_stability_window]
        ys = [pose[1] for pose in self.end_pose_stability_window]
        zs = [pose[2] for pose in self.end_pose_stability_window]
        ranges = (
            max(xs) - min(xs),
            max(ys) - min(ys),
            max(zs) - min(zs),
        )
        epsilon = self._param_float('end_pose_stability_epsilon_m')
        return all(value <= epsilon for value in ranges), samples, ranges

    def _log_target_stability_window(self, decision_hint: str):
        ok, reason, samples, ranges, median = self._target_mgr.stability_decision()
        decision = 'accepted' if ok else decision_hint
        self.get_logger().info(
            f'Target stability window: samples={samples}, '
            f'range={self._fmt_range(ranges)}, '
            f'median={self._fmt_xyz(median)}, '
            f'decision={decision}, reason={reason}.')

    def _handle_approach_failure(self, reason: str):
        approach_states = ('MOVE_APPROACH_BLEND', 'MOVE_PRE_GRASP', 'MOVE_GRASP')
        if self.task_state not in approach_states:
            if self.grasp_closed:
                self._enter_post_grasp_recover(reason)
            else:
                self._enter_recover(reason)
            return

        current = self.current_approach_mode or self.planner.approach_mode
        self.approach_failed_modes.add(current)
        priority = self._approach_priority()

        if (
            bool(self.get_parameter('front_first_then_top_down').value)
            and current == 'front'
            and 'top_down' in priority
            and 'top_down' not in self.approach_failed_modes
        ):
            target = self._fixed_target_xyz() or self.pre_target
            if target is None:
                self._clear_selected_plan('front fallback has no target for top_down preflight')
                self._enter_recover('front fallback has no target for top_down preflight')
                return
            self.current_approach_mode = 'top_down'
            self.current_approach_index = priority.index('top_down')
            self.approach_retry_count = 0
            self.planner.set_approach_mode('top_down')
            fallback_plan = self._try_plan_mode(target, 'top_down')
            if not fallback_plan.ok:
                self._clear_selected_plan('top_down fallback preflight failed')
                self.get_logger().warning(
                    f'top_down fallback preflight failed: {fallback_plan.reason}; '
                    'return to WAIT_PRE_TARGET and collect a new stable target window.')
                self._target_mgr.reset_stability()
                self._transition('WAIT_PRE_TARGET', clear_window=True)
                return
            self._activate_selected_plan(fallback_plan, target)
            self.target_frozen = True
            self._target_mgr.freeze()
            self.active_motion_goal = None
            self._reset_stage_vars()
            if self.executor_status == 'ERROR':
                self._publish_reset_executor('clear_error')
                self.last_reset_executor_time = self._now_sec()
            self.get_logger().warning(
                f'Front approach failed; fallback to top_down approach. reason={reason}. '
                f'last_seen_target_base={self._fmt_xyz(self.last_seen_target_base) if self.last_seen_target_base else None}.')
            self._transition('MOVE_PRE_GRASP')
            return

        self.get_logger().error(
            f'Approach failed with no remaining fallback: mode={current}, '
            f'failed_modes={sorted(self.approach_failed_modes)}, reason={reason}.')
        self._enter_recover(reason)

    def _classify_recover_reason(self, reason: str) -> str:
        text = str(reason or '').strip().lower()
        if not text:
            return 'unknown'
        if 'return_init' in text or 'return init' in text:
            return 'return_init_failed'
        if 'visual' in text or 'last-seen' in text or 'last_seen' in text:
            return 'visual_lost'
        if 'timeout' in text:
            return 'motion_timeout'
        if 'joint_limit_recover' in text or 'recover_joint_limit' in text:
            return 'joint_limit_recover'
        if 'executor' in text or 'clear_error' in text:
            return 'executor_error'
        if 'out_of_official_workspace' in text:
            return 'visual_lost'
        return 'unknown'

    def _enter_recover(self, reason: str = ''):
        if self.task_state == 'RECOVER':
            return
        self._clear_selected_plan(f'enter RECOVER: {reason}')
        self.recover_reason = self._classify_recover_reason(reason)
        self.recover_detail = reason
        if reason:
            self.get_logger().error(
                f'Entering RECOVER: recover_reason={self.recover_reason}, detail={reason}')
        self.pre_target = None
        self.grasp_target = None
        self.active_target_base = None
        self.target_frozen = False
        self.active_motion_goal = None
        self._target_mgr.reset_stability()
        self.recover_phase = 'OPEN_GRIPPER'
        self.recover_reset_command = None
        self.pending_speed_profile = None
        self.last_reset_executor_time = None
        self.recover_command_sent = False
        self.recover_command_type = None
        self.recover_command_time = None
        self.recover_timeout_logged = False
        self.rejected_busy_count = 0
        self._transition('RECOVER')

    def _enter_post_grasp_recover(self, reason: str = ''):
        if self.task_state == 'RECOVER':
            return
        self._clear_selected_plan(f'enter post-grasp RECOVER: {reason}')
        self.recover_reason = self._classify_recover_reason(reason)
        self.recover_detail = reason
        if reason:
            self.get_logger().error(
                'Post-grasp recovery: keep gripper closed. '
                f'recover_reason={self.recover_reason}, detail={reason}')
        self.active_motion_goal = None
        self.lift_goal = None
        self._target_mgr.reset_stability()
        self.recover_phase = 'KEEP_CLOSED_CLEAR_ERROR'
        self.recover_reset_command = None
        self.pending_speed_profile = None
        self.last_reset_executor_time = None
        self.recover_command_sent = False
        self.recover_command_type = None
        self.recover_command_time = None
        self.recover_timeout_logged = False
        self.rejected_busy_count = 0
        self._transition('RECOVER')

    def _finish_recover(self):
        self.get_logger().info(
            'RECOVER complete; clearing approach state and returning to IDLE.')
        self._reset_cycle()
        self._transition('IDLE')

    def _finish_cycle(self):
        self.get_logger().info('Grasp cycle finished; returning to IDLE.')
        self._reset_cycle()
        self._set_speed_profile('default')
        self._transition('IDLE')

    def _reset_cycle(self):
        self._clear_selected_plan('reset cycle')
        self.latest_target = None
        self.latest_target_time = None
        self.pre_target = None
        self.grasp_target = None
        self.last_seen_target_base = None
        self.last_seen_target_time = None
        self.active_target_base = None
        self.target_frozen = False
        self.active_motion_goal = None
        if bool(self.get_parameter('clear_target_window_on_cycle_end').value):
            self._clear_target_window('after cycle')
            cooldown = self._param_float('inter_cycle_cooldown_sec')
            if cooldown > 0.0:
                self.target_collection_cooldown_until = self._now_sec() + cooldown
                self.target_collection_cooldown_logged = False
        self.pending_speed_profile = None
        self.recover_phase = 'OPEN_GRIPPER'
        self.recover_reason = 'idle'
        self.recover_detail = ''
        self.recover_reset_command = None
        self.gripper_settle_start = None
        self.last_reset_executor_time = None
        self.blend_busy_retry_count = 0
        self.blend_busy_retry_after = None
        self.cartesian_busy_retry_count = 0
        self.cartesian_busy_retry_after = None
        self.recover_command_sent = False
        self.recover_command_type = None
        self.recover_command_time = None
        self.recover_timeout_logged = False
        self.rejected_busy_count = 0
        self.last_visual_lost_warning_time = None
        self.last_target_failure_reason = None
        self.current_approach_index = 0
        self.current_approach_mode = None
        self.approach_failed_modes.clear()
        self.approach_retry_count = 0
        self.return_j6_home_sent = False
        self.grasp_closed = False
        self.lift_goal = None
        self.recover_lift_goal = None
        self._ctx.fixed_target_snapshot = None
        self._reset_stage_vars()

    def _clear_target_window(self, reason: str):
        self._target_mgr.reset_stability()
        self.get_logger().info(f'Target window cleared {reason}.')

    def _clear_selected_plan(self, reason: str):
        if self.selected_plan is not None:
            self.get_logger().debug(
                f'Clear selected_plan id={self.selected_plan.plan_id}: {reason}')
        self.selected_plan = None

    def _reset_stage_vars(self):
        self.state_command_sent = False
        self.stage_motion_started = False
        self.settle_start_time = None
        self.active_motion_goal = None
        self.stage_full_goal = None
        self.blend_waypoints = None

    def _transition(self, new_state: str, clear_window: bool = False):
        old_state = self.task_state
        self.task_state = new_state
        self.state_start_time = self._now_sec()
        self._reset_stage_vars()
        if clear_window:
            if new_state == 'WAIT_PRE_TARGET':
                self._clear_selected_plan('enter WAIT_PRE_TARGET with clear_window=true')
            if bool(self.get_parameter('clear_target_window_on_cycle_start').value):
                self._clear_target_window('after cycle' if old_state in ('IDLE', 'RECOVER') else 'on cycle start')
        if old_state != new_state:
            self.rejected_busy_count = 0
            if new_state == 'MOVE_APPROACH_BLEND':
                self.blend_busy_retry_count = 0
                self.blend_busy_retry_after = None
            if new_state in ('MOVE_PRE_GRASP', 'MOVE_GRASP', 'MOVE_LIFT'):
                self.cartesian_busy_retry_count = 0
                self.cartesian_busy_retry_after = None
            self.last_status_log_time = None
            self.get_logger().info(
                f'[GRASP] {old_state} -> {new_state}; '
                f'mode={self.current_approach_mode}; '
                f'plan_id={self._selected_plan_id()}; '
                f'cart_waypoints={new_state == "MOVE_APPROACH_BLEND"}')

    def _set_speed_profile(self, profile: str):
        profile = profile.lower()
        if self.speed_profile_active == profile:
            return False
        self.pending_speed_profile = profile
        return True

    def _handle_pending_speed_profile(self) -> bool:
        if self.pending_speed_profile is None:
            return False
        if not self._executor_accepting():
            return True
        self._publish_speed_profile(
            self.pending_speed_profile, reason="speed_change")
        self.get_logger().info(
            f'Published speed_profile: {self.pending_speed_profile}')
        self.pending_speed_profile = None
        return True

    def _publish_cart_target(self, xyz: list, reason: str = ""):
        self._cmd_port.publish_cart_target(xyz, reason=reason)
        self.rejected_busy_count = 0

    def _publish_cart_waypoints(self, points: list, reason: str = ""):
        self._cmd_port.publish_cart_waypoints(
            points, frame_id=self.get_parameter('base_frame').value)
        self.rejected_busy_count = 0

    def _publish_joint_target(self, joint_pos: list, reason: str = ""):
        self._cmd_port.publish_joint_target(joint_pos, reason=reason)
        self.rejected_busy_count = 0

    def _publish_gripper_command(self, command: str, reason: str = ""):
        self._cmd_port.publish_gripper(command, reason=reason)
        self.rejected_busy_count = 0

    def _publish_speed_profile(self, profile: str, reason: str = ""):
        self._cmd_port.publish_speed_profile(profile, reason=reason)
        self.speed_profile_active = profile
        self.rejected_busy_count = 0

    def _publish_reset_executor(self, command: str, reason: str = ""):
        self._cmd_port.publish_reset(command, reason=reason)

    def _debug_or_info(self, message: str):
        if bool(self.get_parameter('verbose_debug').value):
            self.get_logger().info(message)
        else:
            self.get_logger().debug(message)

    def _log_waypoint_step(self, message: str):
        if bool(self.get_parameter('log_waypoint_each_step').value):
            self.get_logger().info(message)
        else:
            self.get_logger().debug(message)

    def _maybe_log_status_summary(self):
        period = self._param_float('status_log_period_sec')
        if period <= 0.0:
            return
        now = self._now_sec()
        if self.last_status_log_time is not None and now - self.last_status_log_time < period:
            return
        self.last_status_log_time = now
        target = self._fixed_target_xyz() or self.active_target_base or self.last_seen_target_base
        goal = self.active_motion_goal or self.stage_full_goal or self.lift_goal
        extra = ''
        if self.task_state == 'MOVE_APPROACH_BLEND':
            waypoints_count = len(self.blend_waypoints) if self.blend_waypoints else 0
            final_goal = (
                list(self.selected_plan.grasp)
                if self.selected_plan is not None and self.selected_plan.grasp is not None
                else None
            )
            goal = final_goal
            extra = (
                f', plan_id={self._selected_plan_id()}, '
                f'waypoints_count={waypoints_count}, final_goal={self._fmt_xyz(final_goal)}')
        self.get_logger().info(format_status_summary(
            state=self.task_state,
            executor=self.executor_status,
            mode=self.current_approach_mode,
            target=target,
            end=self.last_end_pose,
            goal=goal,
        ) + extra)

    def _selected_plan_id(self):
        return self.selected_plan.plan_id if self.selected_plan is not None else None

    def _fmt_waypoints(self, waypoints: list) -> str:
        return '[' + ', '.join(self._fmt_xyz(point) for point in waypoints) + ']'

    def _executor_accepting(self) -> bool:
        return self.executor_status in ('IDLE', 'DONE', '')

    def _fresh_end_pose_available(self) -> bool:
        if self.last_end_pose is None or self.last_end_pose_time is None:
            return False
        return self._now_sec() - self.last_end_pose_time <= self._param_float('end_pose_timeout_sec')

    def _message_age_sec(self, msg: VisualTarget):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return None
        msg_time = Time.from_msg(msg.header.stamp)
        return (self.get_clock().now() - msg_time).nanoseconds / 1e9

    def _in_workspace(self, xyz: list) -> bool:
        return (
            self.get_parameter('workspace_limits.x_min').value <= xyz[0] <= self.get_parameter('workspace_limits.x_max').value
            and self.get_parameter('workspace_limits.y_min').value <= xyz[1] <= self.get_parameter('workspace_limits.y_max').value
            and self.get_parameter('workspace_limits.z_min').value <= xyz[2] <= self.get_parameter('workspace_limits.z_max').value
        )

    def _state_elapsed(self) -> float:
        return self._now_sec() - self.state_start_time

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _last_seen_age_sec(self):
        if self.last_seen_target_time is None:
            return None
        return self._now_sec() - self.last_seen_target_time

    def _position_tolerance(self) -> float:
        return self._param_float('position_tolerance_m')

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _param_int(self, name: str) -> int:
        if name == 'stable_frame_count':
            return self._stable_frame_count_required()
        return int(self.get_parameter(name).value)

    def _stable_frame_count_required(self) -> int:
        value = int(self.get_parameter('stable_frame_count').value)
        legacy = int(self.get_parameter('stable_frame_count_required').value)
        return max(1, value if value != 5 else legacy)

    @staticmethod
    def _distance(a: list, b: list) -> float:
        return math.sqrt(
            (float(a[0]) - float(b[0])) ** 2
            + (float(a[1]) - float(b[1])) ** 2
            + (float(a[2]) - float(b[2])) ** 2
        )

    @staticmethod
    def _fmt_xyz(xyz: list) -> str:
        return fmt_xyz(xyz)

    @staticmethod
    def _fmt_range(values) -> str:
        if values is None:
            return 'None'
        return f'({float(values[0]):.3f},{float(values[1]):.3f},{float(values[2]):.3f})'


def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskOpenLoop()
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
