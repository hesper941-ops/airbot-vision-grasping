#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Open-loop grasp task with two visual stability confirmations.

The task node consumes /visual_target_base in base_link coordinates and sends
explicit joint/cartesian/gripper/speed commands to arm_executor_node. It does
not call the AIRBOT SDK and does not perform camera-to-base transforms.
"""

from collections import deque
import math
import statistics
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray, String

from robot_msgs.msg import ArmJointState, VisualTarget
from robot_tasks.shared.grasp_planner import GraspPlanner


class GraspTaskOpenLoop(Node):
    """Two-stage visual-confirmed open-loop grasp state machine."""

    def __init__(self):
        super().__init__('grasp_task_open_loop')
        self._declare_parameters()

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
        self.target_window = deque(maxlen=self._stable_frame_count_required())
        self.last_visual_lost_warning_time: Optional[float] = None
        self.last_target_failure_reason = None
        self.current_approach_index = 0
        self.current_approach_mode = None
        self.approach_failed_modes = set()
        self.approach_retry_count = 0
        self.return_j6_home_sent = False

        self.state_command_sent = False
        self.stage_motion_started = False
        self.settle_start_time: Optional[float] = None

        self.last_end_pose: Optional[list] = None
        self.last_end_pose_time: Optional[float] = None
        self.last_joint_pos: Optional[list] = None
        self.last_joint_vel: Optional[list] = None
        self.executor_status = 'IDLE'

        self.speed_profile_active = 'unknown'
        self.pending_speed_profile: Optional[str] = None
        self.gripper_settle_start: Optional[float] = None
        self.recover_phase = 'OPEN_GRIPPER'
        self.last_reset_executor_time: Optional[float] = None
        self.rejected_busy_count = 0

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

        self.cart_pub = self.create_publisher(
            PointStamped, '/robot_arm/cart_target', 10)
        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/robot_arm/target_joint', 10)
        self.gripper_pub = self.create_publisher(
            String, '/robot_arm/gripper_cmd', 10)
        self.speed_pub = self.create_publisher(
            String, '/robot_arm/speed_profile', 10)
        self.reset_executor_pub = self.create_publisher(
            String, '/robot_arm/reset_executor', 10)

        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('loop_hz').value),
            self.step_loop,
        )

        self.get_logger().info(
            'GraspTaskOpenLoop started. /visual_target_base must already be in base_link.')

    def _declare_parameters(self):
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('allow_empty_target_frame', False)

        self.declare_parameter('pre_grasp_z_offset', 0.12)
        self.declare_parameter('grasp_z_offset', 0.0)
        self.declare_parameter('lift_z_offset', 0.10)
        self.declare_parameter('safe_pose', [0.35, 0.0, 0.35])
        self.declare_parameter('approach_mode', 'front')
        self.declare_parameter('approach_priority', ['front', 'top_down'])
        self.declare_parameter('front_first_then_top_down', True)
        self.declare_parameter('max_approach_mode_retries', 1)
        self.declare_parameter('continue_with_last_seen_during_motion', True)
        self.declare_parameter('table_z', 0.0)
        self.declare_parameter('table_clearance', 0.04)
        self.declare_parameter('final_grasp_clearance', 0.015)
        self.declare_parameter('front_approach_x_offset', -0.10)
        self.declare_parameter('front_approach_z_offset', 0.05)
        self.declare_parameter('min_safe_motion_z', 0.08)
        self.declare_parameter('reject_target_below_table', True)
        self.declare_parameter('joint6_compensation_deg', 90.0)
        # Conservative J6 range based on AIRBOT Play official specs.
        # Confirm exact hardware model before widening this range.
        self.declare_parameter('joint6_min_rad', -2.9671)
        self.declare_parameter('joint6_max_rad', 2.9671)
        self.declare_parameter('j6_home_deg', 90.0)
        self.declare_parameter('j6_allowed_delta_deg', 90.0)
        self.declare_parameter('j6_preferred_offsets_deg', [90.0, -90.0])
        self.declare_parameter('forbid_camera_upside_down', True)
        self.declare_parameter('return_j6_to_home_on_recover', True)
        self.declare_parameter('return_to_init_after_grasp', True)
        self.declare_parameter('keep_gripper_closed_after_grasp', True)
        self.declare_parameter('final_init_joint_pos_deg', [0.0, -45.0, 110.0, -90.0, 90.0, 90.0])
        self.declare_parameter('return_init_timeout_sec', 10.0)

        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('stable_frame_count', 5)
        self.declare_parameter('stable_frame_count_required', 5)
        self.declare_parameter('stable_position_threshold', 0.015)
        self.declare_parameter('stable_position_threshold_m', 0.015)
        self.declare_parameter('stable_depth_threshold_m', 0.03)
        self.declare_parameter('target_timeout_sec', 1.0)
        self.declare_parameter('end_pose_timeout_sec', 1.0)
        self.declare_parameter('use_last_seen_target_on_loss', True)
        self.declare_parameter('visual_lost_grace_sec', 0.5)
        self.declare_parameter('last_seen_target_max_age_sec', 5.0)
        self.declare_parameter('update_target_during_motion', True)
        self.declare_parameter('freeze_target_before_close', True)
        self.declare_parameter('require_second_visual_confirm', False)
        self.declare_parameter('max_target_jump_m', 0.08)
        self.declare_parameter('max_target_z_jump_m', 0.08)

        self.declare_parameter('workspace_limits.x_min', 0.10)
        self.declare_parameter('workspace_limits.x_max', 1.00)
        self.declare_parameter('workspace_limits.y_min', -0.45)
        self.declare_parameter('workspace_limits.y_max', 0.50)
        self.declare_parameter('workspace_limits.z_min', 0.02)
        self.declare_parameter('workspace_limits.z_max', 0.70)

        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('position_tolerance_m', 0.02)
        self.declare_parameter('settle_time_sec', 0.5)
        self.declare_parameter('joint_speed_safe_threshold', 0.1)
        self.declare_parameter('post_joint_rotate_settle_sec', 0.5)
        self.declare_parameter('gripper_settle_sec', 1.0)

        # Cartesian step-by-step: each command limited to this distance.
        # Must be smaller than AirbotWrapper's 0.100 m single-step safety limit.
        self.declare_parameter('max_cartesian_step', 0.08)

        self.declare_parameter('wait_pre_target_warn_sec', 15.0)
        self.declare_parameter('wait_grasp_target_timeout_sec', 8.0)
        self.declare_parameter('motion_timeout_sec', 12.0)
        self.declare_parameter('set_orientation_timeout_sec', 8.0)
        self.declare_parameter('close_gripper_timeout_sec', 4.0)
        self.declare_parameter('recover_timeout_sec', 15.0)
        self.declare_parameter('rejected_busy_recover_threshold', 2)
        self.declare_parameter('loop_hz', 4.0)

    def _config_dict(self) -> dict:
        return {
            'pre_grasp_z_offset': self.get_parameter('pre_grasp_z_offset').value,
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
            'min_safe_motion_z': self.get_parameter('min_safe_motion_z').value,
            'reject_target_below_table': self.get_parameter('reject_target_below_table').value,
            'joint6_compensation_deg': self.get_parameter('joint6_compensation_deg').value,
            'joint6_min_rad': self.get_parameter('joint6_min_rad').value,
            'joint6_max_rad': self.get_parameter('joint6_max_rad').value,
            'j6_home_deg': self.get_parameter('j6_home_deg').value,
            'j6_allowed_delta_deg': self.get_parameter('j6_allowed_delta_deg').value,
            'j6_preferred_offsets_deg': self.get_parameter('j6_preferred_offsets_deg').value,
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
        if not self._valid_target(msg):
            return

        self.latest_target = msg
        self.latest_target_time = self._now_sec()
        target_base = [float(msg.x), float(msg.y), float(msg.z)]

        if self._reject_target_jump(target_base):
            return

        self.last_seen_target_base = target_base
        self.last_seen_target_time = self.latest_target_time

        if self.task_state in ('WAIT_PRE_TARGET', 'WAIT_GRASP_TARGET'):
            self._update_target_window(msg)

        updatable_states = (
            'WAIT_PRE_TARGET',
            'SET_GRIPPER_ORIENTATION',
            'MOVE_PRE_GRASP',
            'MOVE_GRASP',
        )
        can_update_during_motion = bool(
            self.get_parameter('update_target_during_motion').value)
        if (
            self.task_state in updatable_states
            and not self.target_frozen
            and (
                can_update_during_motion
                or self.task_state in ('WAIT_PRE_TARGET', 'SET_GRIPPER_ORIENTATION')
            )
        ):
            self.active_target_base = target_base
        elif self.task_state not in ('WAIT_PRE_TARGET', 'WAIT_GRASP_TARGET'):
            self.get_logger().debug(
                f'Target cached during {self.task_state}; active target is unchanged.')

    def _reject_target_jump(self, target_base: list) -> bool:
        if self.active_target_base is None:
            return False
        if self.task_state not in ('MOVE_PRE_GRASP', 'MOVE_GRASP'):
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

    def executor_status_callback(self, msg: String):
        self.executor_status = msg.data.strip().upper()

        if self.executor_status in ('IDLE', 'DONE'):
            self.rejected_busy_count = 0

        if self.executor_status == 'REJECTED_BUSY':
            if self.task_state == 'RECOVER':
                return
            self.rejected_busy_count += 1
            threshold = int(self.get_parameter('rejected_busy_recover_threshold').value)
            if self.rejected_busy_count >= threshold:
                self.get_logger().error(
                    f'REJECTED_BUSY {self.rejected_busy_count} times (>= {threshold}); handling approach failure.')
                self._handle_approach_failure(
                    f'REJECTED_BUSY {self.rejected_busy_count} times')
            else:
                self.get_logger().warning(
                    f'REJECTED_BUSY {self.rejected_busy_count}/{threshold}; '
                    f'waiting for executor to return IDLE before escalating to RECOVER.')
            return

        if self.executor_status in ('ERROR', 'TIMEOUT'):
            self.get_logger().error(
                f'Executor status {self.executor_status}; handling approach failure.')
            self._handle_approach_failure(f'Executor status {self.executor_status}')

    def step_loop(self):
        try:
            if self._handle_pending_speed_profile():
                return

            if self.task_state == 'IDLE':
                self._transition('WAIT_PRE_TARGET', clear_window=True)

            elif self.task_state == 'WAIT_PRE_TARGET':
                self._handle_wait_pre_target()

            elif self.task_state == 'SET_GRIPPER_ORIENTATION':
                self._handle_set_gripper_orientation()

            elif self.task_state == 'MOVE_PRE_GRASP':
                self._handle_move_pre_grasp()

            elif self.task_state == 'WAIT_GRASP_TARGET':
                self._handle_wait_grasp_target()

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
        """Wait for the first stable visual target.

        WAIT_PRE_TARGET is before any grasp motion — no target just means the
        vision pipeline is not running yet.  We print a periodic warning and
        keep waiting rather than entering RECOVER.
        """
        warn_sec = self._param_float('wait_pre_target_warn_sec')
        if self._state_elapsed() > warn_sec:
            self.get_logger().warning(
                f'Still waiting for stable /visual_target_base in WAIT_PRE_TARGET '
                f'(elapsed {self._state_elapsed():.1f}s). '
                f'Clearing stale target window and continuing to wait.')
            self.target_window.clear()
            self.state_start_time = self._now_sec()
            return

        if self._is_target_stable():
            self.pre_target = self._get_stable_target()
            if not self.target_frozen:
                self.active_target_base = list(self.pre_target)
            self._start_approach_sequence()
            self.get_logger().info(
                f'Pre target stable: {self._fmt_xyz(self.pre_target)}')
            self._transition('SET_GRIPPER_ORIENTATION')

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

    def _ensure_approach_mode_started(self):
        if self.current_approach_mode is None:
            self._start_approach_sequence()
        else:
            self.planner.set_approach_mode(self.current_approach_mode)

    def _handle_set_gripper_orientation(self):
        if self._state_elapsed() > self._param_float('set_orientation_timeout_sec'):
            self.get_logger().error('SET_GRIPPER_ORIENTATION timeout.')
            self._enter_recover()
            return

        if self.pending_speed_profile is not None:
            return

        if self.last_joint_pos is None:
            return

        if not self.state_command_sent:
            if not self._executor_accepting():
                return
            joint_target = self.planner.compute_joint6_target(self.last_joint_pos)
            for line in self.planner.last_j6_debug:
                self.get_logger().info(line)
            if joint_target is None:
                self.get_logger().error('Cannot compute safe joint6 target; entering RECOVER.')
                self._enter_recover('Cannot compute safe joint6 target')
                return
            self._publish_joint_target(joint_target)
            self.state_command_sent = True
            self.stage_motion_started = True
            self.get_logger().info('Joint6 orientation command sent once.')
            return

        if self.last_joint_vel is None:
            return

        max_speed = max(abs(float(v)) for v in self.last_joint_vel)
        threshold = self._param_float('joint_speed_safe_threshold')
        if max_speed > threshold:
            self.settle_start_time = None
            return

        if self.settle_start_time is None:
            self.settle_start_time = self._now_sec()
            self.get_logger().info(
                f'Joint speed safe ({max_speed:.4f} < {threshold:.4f}); '
                f'wait {self._param_float("post_joint_rotate_settle_sec"):.2f}s before pre-grasp.')
            return

        if self._now_sec() - self.settle_start_time >= self._param_float('post_joint_rotate_settle_sec'):
            self._set_speed_profile('fast')
            self._transition('MOVE_PRE_GRASP')

    def _handle_move_pre_grasp(self):
        self._handle_cartesian_motion(
            'MOVE_PRE_GRASP',
            lambda: self._compute_pre_grasp_from_active_target(),
            on_done=self._after_move_pre_grasp,
        )

    def _handle_wait_grasp_target(self):
        if self._state_elapsed() > self._param_float('wait_grasp_target_timeout_sec'):
            self.get_logger().error('WAIT_GRASP_TARGET timeout.')
            self._handle_approach_failure('WAIT_GRASP_TARGET timeout')
            return

        if self._is_target_stable():
            stable = self._get_stable_target()
            if self.pre_target is not None:
                drift = self._distance(stable, self.pre_target)
                max_drift = self._param_float('stable_position_threshold_m') * 4.0
                if drift > max_drift:
                    self.get_logger().error(
                        f'Grasp target drift too large after pre-grasp: {drift:.4f}m > {max_drift:.4f}m.')
                    self._handle_approach_failure(
                        f'Grasp target drift too large after pre-grasp: {drift:.4f}m > {max_drift:.4f}m')
                    return
            self.grasp_target = stable
            if not self.target_frozen:
                self.active_target_base = list(self.grasp_target)
            self.get_logger().info(
                f'Grasp target stable: {self._fmt_xyz(self.grasp_target)}')
            self._set_speed_profile('slow')
            self._transition('MOVE_GRASP')

    def _after_move_pre_grasp(self):
        if bool(self.get_parameter('require_second_visual_confirm').value):
            self._transition('WAIT_GRASP_TARGET', clear_window=True)
            return

        self._set_speed_profile('slow')
        self._transition('MOVE_GRASP')

    def _handle_move_grasp(self):
        self._handle_cartesian_motion(
            'MOVE_GRASP',
            lambda: self._compute_grasp_from_active_target(),
            on_done=self._after_move_grasp,
        )

    def _after_move_grasp(self):
        if bool(self.get_parameter('freeze_target_before_close').value):
            target = self._get_active_target_or_last_seen()
            if target is None:
                self.get_logger().error(
                    'Cannot freeze target before close: no valid active or last-seen base target.')
                self._enter_recover()
                return
            self.active_target_base = list(target)
            self.target_frozen = True
            self.get_logger().info(
                f'Target frozen before close: {self._fmt_xyz(self.active_target_base)}')
        self._transition('CLOSE_GRIPPER')

    def _compute_pre_grasp_from_active_target(self):
        self._ensure_approach_mode_started()
        target = self._get_active_target_or_last_seen()
        if target is None:
            return None
        self.pre_target = list(target)

        try:
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
        target = self._get_active_target_or_last_seen()
        if target is None:
            return None
        self.grasp_target = list(target)

        try:
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
        self.get_logger().info(', '.join(parts))

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
            self._transition('MOVE_LIFT')

    def _handle_move_lift(self):
        if not self._fresh_end_pose_available():
            if self._state_elapsed() > self._param_float('motion_timeout_sec'):
                self.get_logger().error('MOVE_LIFT has no fresh end_pose.')
                self._enter_recover()
            return

        self._handle_cartesian_motion(
            'MOVE_LIFT',
            lambda: self.planner.compute_safe_lift(self.last_end_pose),
            on_done=self._after_move_lift,
        )

    def _after_move_lift(self):
        if bool(self.get_parameter('return_to_init_after_grasp').value):
            self.get_logger().info(
                'Grasp succeeded; keeping gripper closed. '
                'Returning to init joint pose after grasp: '
                f'{list(self.get_parameter("final_init_joint_pos_deg").value)}')
            self._transition('RETURN_INIT_POSE')
        else:
            self._transition('MOVE_RETREAT')

    def _handle_return_init_pose(self):
        if self._state_elapsed() > self._param_float('return_init_timeout_sec'):
            self.get_logger().error('RETURN_INIT_POSE timeout.')
            self._enter_recover('RETURN_INIT_POSE timeout')
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
            self._finish_cycle()

    def _handle_move_retreat(self):
        self._set_speed_profile('fast')
        self._handle_cartesian_motion(
            'MOVE_RETREAT',
            lambda: self._compute_safe_retreat_goal(),
            on_done=self._finish_cycle,
        )

    def _handle_recover(self):
        if self._state_elapsed() > self._param_float('recover_timeout_sec'):
            self.get_logger().error('RECOVER timeout; reset state to IDLE after best-effort recovery.')
            self._reset_cycle()
            self._transition('IDLE')
            return

        if self.executor_status == 'ERROR':
            now = self._now_sec()
            if self.last_reset_executor_time is None or now - self.last_reset_executor_time >= 0.5:
                self._publish_reset_executor('clear_error')
                self.last_reset_executor_time = now
                self.get_logger().warning(
                    'RECOVER: requested executor clear_error '
                    '(repeating every 0.5 s while executor is ERROR).')
            return

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
            self.recover_phase = 'LIFT'
            self._reset_stage_vars()
            self._set_speed_profile('fast')
            return

        if self.recover_phase == 'LIFT':
            if not self._fresh_end_pose_available():
                return
            self._handle_cartesian_motion(
                'RECOVER_LIFT',
                lambda: self.planner.compute_safe_lift(self.last_end_pose),
                on_done=self._advance_recover_to_retreat,
                timeout_param='recover_timeout_sec',
            )
            return

        if self.recover_phase == 'RETREAT':
            self._handle_cartesian_motion(
                'RECOVER_RETREAT',
                lambda: self._compute_safe_retreat_goal(),
                on_done=self._after_recover_retreat,
                timeout_param='recover_timeout_sec',
            )

        if self.recover_phase == 'RETURN_J6_HOME':
            self._handle_recover_return_j6_home()

    def _advance_recover_to_retreat(self):
        self.recover_phase = 'RETREAT'
        self._reset_stage_vars()

    def _after_recover_retreat(self):
        if bool(self.get_parameter('return_j6_to_home_on_recover').value):
            self.recover_phase = 'RETURN_J6_HOME'
            self._reset_stage_vars()
            self.get_logger().info('RECOVER: safe retreat reached; returning J6 to home.')
            return
        self._finish_recover()

    def _handle_recover_return_j6_home(self):
        if self.last_joint_pos is None:
            self._finish_recover()
            return
        if not self.state_command_sent:
            if not self._executor_accepting():
                return
            joint_target = self.planner.compute_j6_home_target(self.last_joint_pos)
            if joint_target is None:
                self.get_logger().warning(
                    'RECOVER: cannot compute legal J6 home target; finish recovery anyway.')
                self._finish_recover()
                return
            self._publish_joint_target(joint_target)
            self.state_command_sent = True
            self.get_logger().info(
                f'RECOVER: sent J6 home command target={math.degrees(joint_target[5]):.1f} deg.')
            return
        if self.last_joint_vel is None:
            return
        max_speed = max(abs(float(v)) for v in self.last_joint_vel)
        if max_speed > self._param_float('joint_speed_safe_threshold'):
            return
        self._finish_recover()

    def _compute_safe_retreat_goal(self):
        if self.last_end_pose is not None and float(self.last_end_pose[2]) < self.planner.safe_motion_z:
            return self.planner.validate_waypoint([
                float(self.last_end_pose[0]),
                float(self.last_end_pose[1]),
                self.planner.safe_motion_z,
            ])
        return self.planner.get_safe_pose()

    def _handle_cartesian_motion(self, state_name: str, goal_fn, on_done, timeout_param='motion_timeout_sec'):
        """Step-by-step Cartesian movement with per-step settle gating.

        Each invocation publishes at most one step_goal (computed via limit_step
        from current end_pose toward full_goal).  The method never publishes
        the full final goal directly unless it is already within position_tolerance.

        When full_goal is reached and the settle timer expires, on_done() fires.
        """
        if self._state_elapsed() > self._param_float(timeout_param):
            self.get_logger().error(f'{state_name} timeout.')
            self._handle_approach_failure(f'{state_name} timeout')
            return

        if self.pending_speed_profile is not None:
            return

        if not self._fresh_end_pose_available():
            return

        # Resolve the full (final) goal every tick so dynamic goals (e.g. lift)
        # reflect the latest end_pose.
        try:
            full_goal = goal_fn()
        except Exception as exc:
            self.get_logger().error(f'{state_name}: failed to compute motion goal: {exc}')
            self._handle_approach_failure(f'{state_name} goal exception: {exc}')
            return
        if self.task_state != state_name and state_name in ('MOVE_PRE_GRASP', 'MOVE_GRASP'):
            return
        if full_goal is None or len(full_goal) != 3:
            self.get_logger().error(f'{state_name}: invalid motion goal.')
            if self.last_target_failure_reason is not None:
                self._enter_recover(self.last_target_failure_reason)
                return
            self._handle_approach_failure(f'{state_name} invalid motion goal')
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
            self._handle_approach_failure(f'{state_name} full_goal unsafe: {exc}')
            return

        tolerance = self._position_tolerance()
        distance_to_full = self._distance(self.last_end_pose, full_goal)

        # ---- Already at full goal → settle and finish ----
        if distance_to_full <= tolerance:
            if self.settle_start_time is None:
                self.settle_start_time = self._now_sec()
                self.get_logger().info(
                    f'{state_name}: full_goal {self._fmt_xyz(full_goal)} reached '
                    f'(dist={distance_to_full:.4f}m), settling.')
                return

            if self._now_sec() - self.settle_start_time >= self._param_float('settle_time_sec'):
                self.get_logger().info(
                    f'{state_name}: full_goal reached and settled.')
                self._reset_stage_vars()
                on_done()
            return

        # ---- Not yet sent the next step → compute and publish step_goal ----
        if not self.state_command_sent:
            if not self._executor_accepting():
                return

            max_step = self._param_float('max_cartesian_step')
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
                self._handle_approach_failure(f'{state_name} unsafe step_goal: {exc}')
                return
            self.active_motion_goal = [
                float(step_goal[0]),
                float(step_goal[1]),
                float(step_goal[2]),
            ]

            self._publish_cart_target(self.active_motion_goal)
            self.state_command_sent = True
            self.stage_motion_started = True
            self.settle_start_time = None

            self.get_logger().info(
                f'{state_name}: step_goal={self._fmt_xyz(self.active_motion_goal)}, '
                f'full_goal={self._fmt_xyz(full_goal)}, '
                f'dist_to_full={distance_to_full:.4f}m.')
            return

        # ---- Step command sent; wait for step_goal ----
        if self.active_motion_goal is None:
            self.get_logger().error(f'{state_name}: active_motion_goal is missing.')
            self._handle_approach_failure(f'{state_name} active_motion_goal missing')
            return

        distance_to_step = self._distance(self.last_end_pose, self.active_motion_goal)

        if distance_to_step > tolerance:
            self.settle_start_time = None
            return

        if self.settle_start_time is None:
            self.settle_start_time = self._now_sec()
            self.get_logger().info(
                f'{state_name}: step_goal {self._fmt_xyz(self.active_motion_goal)} reached '
                f'(dist={distance_to_step:.4f}m), settling before next step.')
            return

        if self._now_sec() - self.settle_start_time >= self._param_float('settle_time_sec'):
            self.get_logger().info(
                f'{state_name}: step_goal settled; ready for next Cartesian step.')
            self._reset_stage_vars()
            return

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

    def _update_target_window(self, msg: VisualTarget):
        self.target_window.append({
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z),
            'depth': float(msg.depth),
            'confidence': float(msg.confidence),
        })

    def _is_target_stable(self) -> bool:
        required = self._param_int('stable_frame_count')
        if len(self.target_window) < required:
            return False

        stable = self._get_stable_target()
        max_distance = max(
            self._distance([entry['x'], entry['y'], entry['z']], stable)
            for entry in self.target_window
        )
        depth_values = [entry['depth'] for entry in self.target_window if math.isfinite(entry['depth'])]
        max_depth_delta = 0.0
        if depth_values:
            median_depth = statistics.median(depth_values)
            max_depth_delta = max(abs(depth - median_depth) for depth in depth_values)

        stable_position = max_distance <= self._param_float('stable_position_threshold_m')
        stable_depth = max_depth_delta <= self._param_float('stable_depth_threshold_m')
        if stable_position and stable_depth:
            self.get_logger().info(
                f'Stable target window {len(self.target_window)}/{required}: '
                f'max_pos={max_distance:.4f}m, max_depth={max_depth_delta:.4f}m.')
            return True
        return False

    def _get_stable_target(self) -> list:
        return [
            statistics.median(entry['x'] for entry in self.target_window),
            statistics.median(entry['y'] for entry in self.target_window),
            statistics.median(entry['z'] for entry in self.target_window),
        ]

    def _handle_approach_failure(self, reason: str):
        approach_states = ('MOVE_PRE_GRASP', 'MOVE_GRASP', 'WAIT_GRASP_TARGET')
        if self.task_state not in approach_states:
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
            self.current_approach_mode = 'top_down'
            self.current_approach_index = priority.index('top_down')
            self.approach_retry_count = 0
            self.planner.set_approach_mode('top_down')
            self.target_frozen = False
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

    def _enter_recover(self, reason: str = ''):
        if self.task_state == 'RECOVER':
            return
        if reason:
            self.get_logger().error(f'Entering RECOVER: {reason}')
        self.pre_target = None
        self.grasp_target = None
        self.active_target_base = None
        self.target_frozen = False
        self.active_motion_goal = None
        self.target_window.clear()
        self.recover_phase = 'OPEN_GRIPPER'
        self.pending_speed_profile = None
        self.last_reset_executor_time = None
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
        self.latest_target = None
        self.latest_target_time = None
        self.pre_target = None
        self.grasp_target = None
        self.last_seen_target_base = None
        self.last_seen_target_time = None
        self.active_target_base = None
        self.target_frozen = False
        self.active_motion_goal = None
        self.target_window.clear()
        self.pending_speed_profile = None
        self.recover_phase = 'OPEN_GRIPPER'
        self.gripper_settle_start = None
        self.last_reset_executor_time = None
        self.rejected_busy_count = 0
        self.last_visual_lost_warning_time = None
        self.last_target_failure_reason = None
        self.current_approach_index = 0
        self.current_approach_mode = None
        self.approach_failed_modes.clear()
        self.approach_retry_count = 0
        self.return_j6_home_sent = False
        self._reset_stage_vars()

    def _reset_stage_vars(self):
        self.state_command_sent = False
        self.stage_motion_started = False
        self.settle_start_time = None
        self.active_motion_goal = None

    def _transition(self, new_state: str, clear_window: bool = False):
        old_state = self.task_state
        self.task_state = new_state
        self.state_start_time = self._now_sec()
        self._reset_stage_vars()
        if clear_window:
            self.target_window.clear()
        if old_state != new_state:
            self.rejected_busy_count = 0
            self.get_logger().info(
                f'State transition: {old_state} -> {new_state}, '
                f'current_approach_mode={self.current_approach_mode}')

    def _set_speed_profile(self, profile: str):
        profile = profile.lower()
        if self.speed_profile_active == profile or self.pending_speed_profile == profile:
            return
        self.pending_speed_profile = profile

    def _handle_pending_speed_profile(self) -> bool:
        if self.pending_speed_profile is None:
            return False
        if not self._executor_accepting():
            return False
        msg = String()
        msg.data = self.pending_speed_profile
        self.speed_pub.publish(msg)
        self.speed_profile_active = self.pending_speed_profile
        self.rejected_busy_count = 0
        self.get_logger().info(f'Published speed_profile: {msg.data}')
        self.pending_speed_profile = None
        return True

    def _publish_cart_target(self, xyz: list):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('base_frame').value
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        self.cart_pub.publish(msg)
        self.rejected_busy_count = 0

    def _publish_joint_target(self, joint_pos: list):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in joint_pos]
        self.joint_pub.publish(msg)
        self.rejected_busy_count = 0

    def _publish_gripper_command(self, command: str):
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        self.rejected_busy_count = 0

    def _publish_reset_executor(self, command: str):
        msg = String()
        msg.data = command
        self.reset_executor_pub.publish(msg)

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
        return f'({float(xyz[0]):.3f}, {float(xyz[1]):.3f}, {float(xyz[2]):.3f})'


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
