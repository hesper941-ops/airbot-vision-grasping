"""抓取任务配置，从 ROS 参数中读取。

保持与现有 grasp_task_open_loop.py 和 open_loop_grasp.yaml 的
参数名兼容。
"""

from dataclasses import dataclass, field
from typing import Dict, List


# ---------------------------------------------------------------------------
# Per-target-type configuration
# ---------------------------------------------------------------------------

@dataclass
class TargetTypeConfig:
    """单个可检测物目标的配置。"""
    name: str = "duck"
    input_topic: str = "/duck_position"
    frame_id: str = "camera_color_optical_frame"
    stable_required_count: int = 5
    stable_position_tolerance_m: float = 0.015
    target_timeout_sec: float = 1.0


# ---------------------------------------------------------------------------
# Search pose configuration
# ---------------------------------------------------------------------------

@dataclass
class SearchPoseConfig:
    """主动搜索时使用的单个安全观察姿态。"""
    name: str = "observe_center"
    joint_pos: List[float] = field(default_factory=lambda: [
        0.0, -0.65, 1.20, 0.0, 1.20, 0.0])
    speed_scale: float = 0.3
    settle_time_sec: float = 0.5
    detect_window_sec: float = 1.0


# ---------------------------------------------------------------------------
# Active search configuration
# ---------------------------------------------------------------------------

@dataclass
class ActiveSearchConfig:
    enabled: bool = False
    target_acquire_timeout_sec: float = 3.0
    unstable_target_timeout_sec: float = 5.0
    search_timeout_sec: float = 25.0
    search_max_cycles: int = 2
    return_to_default_pose_on_fail: bool = True
    default_search_pose_name: str = "observe_center"
    search_speed_scale: float = 0.3


# ---------------------------------------------------------------------------
# Top-level grasp task configuration
# ---------------------------------------------------------------------------

@dataclass
class GraspTaskConfig:
    """所有抓取任务参数，初始化时从 ROS2 Node 读取。"""

    # -- Loop ----------------------------------------------------------------
    loop_hz: float = 4.0

    # -- Frames --------------------------------------------------------------
    base_frame: str = "base_link"
    allow_empty_target_frame: bool = False

    # -- Target stability ----------------------------------------------------
    stable_frame_count: int = 5
    stable_frame_count_required: int = 5
    stable_position_threshold_m: float = 0.015
    stable_depth_threshold_m: float = 0.03
    target_stability_window: int = 8
    target_stability_min_samples: int = 5
    target_stability_max_range_x: float = 0.015
    target_stability_max_range_y: float = 0.012
    target_stability_max_range_z: float = 0.015
    target_outlier_reject_distance: float = 0.04
    visual_sanity_max_radius_m: float = 0.64
    confidence_threshold: float = 0.7
    target_timeout_sec: float = 1.0
    max_target_jump_m: float = 0.08
    max_target_z_jump_m: float = 0.08

    # -- Last-seen target fallback -------------------------------------------
    use_last_seen_target_on_loss: bool = True
    visual_lost_grace_sec: float = 0.5
    last_seen_target_max_age_sec: float = 8.0
    update_target_during_motion: bool = False
    freeze_target_before_close: bool = True
    continue_with_last_seen_during_motion: bool = True
    ignore_visual_during_motion: bool = True
    clear_target_window_on_cycle_start: bool = True
    clear_target_window_on_cycle_end: bool = True
    inter_cycle_cooldown_sec: float = 2.0
    require_idle_before_target_collection: bool = True
    require_end_pose_stable_before_target_collection: bool = True
    end_pose_stability_window: int = 5
    end_pose_stability_epsilon_m: float = 0.003

    # -- Approach planning ---------------------------------------------------
    approach_mode: str = "front"
    approach_priority: List[str] = field(default_factory=lambda: ["front", "top_down"])
    front_first_then_top_down: bool = True
    max_approach_mode_retries: int = 1
    blend_approach_enabled: bool = True
    blend_approach_fallback_to_sequential: bool = True
    blend_approach_retry_on_busy: bool = True
    blend_approach_busy_retry_delay_sec: float = 1.5
    blend_approach_busy_max_retries: int = 3
    enable_top_down_fallback: bool = True
    top_down_max_target_z: float = 0.49
    top_down_max_pre_grasp_radius_m: float = 0.64
    top_down_pre_grasp_z_offset: float = 0.035

    # Geometry (meters)
    pre_grasp_z_offset: float = 0.06
    grasp_z_offset: float = 0.02
    lift_z_offset: float = 0.04
    front_approach_x_offset: float = -0.10
    front_approach_z_offset: float = 0.05
    adaptive_front_pre_grasp: bool = True
    workspace_soft_margin_m: float = 0.04
    min_front_pre_grasp_distance_m: float = 0.04
    front_grasp_x_offset: float = 0.0
    front_grasp_x_offset_max: float = 0.075

    # -- Table / workspace safety (meters) -----------------------------------
    table_z: float = 0.0
    table_clearance: float = 0.04
    final_grasp_clearance: float = 0.015
    min_safe_motion_z: float = 0.08
    reject_target_below_table: bool = True
    safe_pose: List[float] = field(default_factory=lambda: [0.35, 0.0, 0.35])
    official_reach_radius_m: float = 0.68

    workspace_x_min: float = 0.10
    workspace_x_max: float = 0.68
    workspace_y_min: float = -0.38
    workspace_y_max: float = 0.38
    workspace_z_min: float = 0.02
    workspace_z_max: float = 0.75

    # -- J6 end-effector orientation -----------------------------------------
    joint6_compensation_deg: float = 90.0
    joint6_min_rad: float = -2.9671
    joint6_max_rad: float = 2.9671
    j6_home_deg: float = 0.0
    j6_allowed_delta_deg: float = 90.0
    j6_preferred_offsets_deg: List[float] = field(
        default_factory=lambda: [90.0, -90.0])
    forbid_camera_upside_down: bool = True
    return_j6_to_home_on_recover: bool = True

    # -- Gripper -------------------------------------------------------------
    open_gripper_before_grasp: bool = True
    pre_grasp_open_timeout_sec: float = 4.0
    pre_grasp_open_settle_sec: float = 1.0
    gripper_settle_sec: float = 1.0
    return_to_init_after_grasp: bool = True
    keep_gripper_closed_after_grasp: bool = True
    final_init_joint_pos_deg: List[float] = field(
        default_factory=lambda: [0.0, -45.0, 110.0, -90.0, 90.0, 0.0])
    return_init_timeout_sec: float = 10.0

    # -- Motion control ------------------------------------------------------
    max_cartesian_step: float = 0.06
    lift_cartesian_step_m: float = 0.03
    cart_waypoint_max_step_m: float = 0.10
    cart_waypoint_safe_limit_m: float = 0.12
    position_tolerance_m: float = 0.015
    settle_time_sec: float = 0.8
    joint_speed_safe_threshold: float = 0.1
    post_joint_rotate_settle_sec: float = 0.5
    end_pose_timeout_sec: float = 1.0

    # -- Timeouts (seconds) --------------------------------------------------
    wait_pre_target_warn_sec: float = 15.0
    motion_timeout_sec: float = 16.0
    set_orientation_timeout_sec: float = 8.0
    close_gripper_timeout_sec: float = 4.0
    recover_timeout_sec: float = 15.0
    rejected_busy_recover_threshold: int = 2

    # -- Recovery ------------------------------------------------------------
    recover_clear_error_interval_sec: float = 0.5
    recover_return_init_pose: bool = True
    auto_recover_joint_limit: bool = True
    post_motion_command_cooldown_sec: float = 1.5
    sequential_busy_max_retries: int = 3

    # -- Active search -------------------------------------------------------
    active_search: ActiveSearchConfig = field(default_factory=ActiveSearchConfig)

    # -- Target types --------------------------------------------------------
    target_types: Dict[str, TargetTypeConfig] = field(default_factory=dict)

    # -- Search poses --------------------------------------------------------
    search_poses: List[SearchPoseConfig] = field(default_factory=list)

    # -----------------------------------------------------------------------
    # Factory: build from a ROS2 Node
    # -----------------------------------------------------------------------

    @classmethod
    def from_ros_node(cls, node) -> "GraspTaskConfig":
        """从 ROS2 Node 读取所有参数，保持现有参数名兼容。"""

        def _p(name, default):
            return node.get_parameter(name).value if node.has_parameter(name) else default

        def _pf(name, default):
            return float(_p(name, default))

        def _pi(name, default):
            return int(_p(name, default))

        def _pb(name, default):
            return bool(_p(name, default))

        def _plf(name, default):
            return [float(v) for v in (_p(name, default) or default)]

        # -- Workspace sub-keys ------------------------------------------
        wx_min = _pf("workspace_limits.x_min", 0.10)
        wx_max = _pf("workspace_limits.x_max", 0.68)
        wy_min = _pf("workspace_limits.y_min", -0.38)
        wy_max = _pf("workspace_limits.y_max", 0.38)
        wz_min = _pf("workspace_limits.z_min", 0.02)
        wz_max = _pf("workspace_limits.z_max", 0.75)

        # -- Active search sub-struct ------------------------------------
        asc = ActiveSearchConfig(
            enabled=_pb("active_search.enabled", False),
            target_acquire_timeout_sec=_pf("active_search.target_acquire_timeout_sec", 3.0),
            unstable_target_timeout_sec=_pf("active_search.unstable_target_timeout_sec", 5.0),
            search_timeout_sec=_pf("active_search.search_timeout_sec", 25.0),
            search_max_cycles=_pi("active_search.search_max_cycles", 2),
            return_to_default_pose_on_fail=_pb("active_search.return_to_default_pose_on_fail", True),
            default_search_pose_name=_p("active_search.default_search_pose_name", "observe_center"),
            search_speed_scale=_pf("active_search.search_speed_scale", 0.3),
        )

        return cls(
            loop_hz=_pf("loop_hz", 4.0),
            base_frame=str(_p("base_frame", "base_link")),
            allow_empty_target_frame=_pb("allow_empty_target_frame", False),

            stable_frame_count=_pi("stable_frame_count", 5),
            stable_frame_count_required=_pi("stable_frame_count_required", 5),
            stable_position_threshold_m=_pf("stable_position_threshold_m", 0.015),
            stable_depth_threshold_m=_pf("stable_depth_threshold_m", 0.03),
            target_stability_window=_pi("target_stability_window", 8),
            target_stability_min_samples=_pi("target_stability_min_samples", 5),
            target_stability_max_range_x=_pf("target_stability_max_range_x", 0.015),
            target_stability_max_range_y=_pf("target_stability_max_range_y", 0.012),
            target_stability_max_range_z=_pf("target_stability_max_range_z", 0.015),
            target_outlier_reject_distance=_pf("target_outlier_reject_distance", 0.04),
            visual_sanity_max_radius_m=_pf("visual_sanity_max_radius_m", 0.64),
            confidence_threshold=_pf("confidence_threshold", 0.7),
            target_timeout_sec=_pf("target_timeout_sec", 1.0),
            max_target_jump_m=_pf("max_target_jump_m", 0.08),
            max_target_z_jump_m=_pf("max_target_z_jump_m", 0.08),

            use_last_seen_target_on_loss=_pb("use_last_seen_target_on_loss", True),
            visual_lost_grace_sec=_pf("visual_lost_grace_sec", 0.5),
            last_seen_target_max_age_sec=_pf("last_seen_target_max_age_sec", 8.0),
            update_target_during_motion=_pb("update_target_during_motion", False),
            freeze_target_before_close=_pb("freeze_target_before_close", True),
            continue_with_last_seen_during_motion=_pb("continue_with_last_seen_during_motion", True),
            ignore_visual_during_motion=_pb("ignore_visual_during_motion", True),
            clear_target_window_on_cycle_start=_pb("clear_target_window_on_cycle_start", True),
            clear_target_window_on_cycle_end=_pb("clear_target_window_on_cycle_end", True),
            inter_cycle_cooldown_sec=_pf("inter_cycle_cooldown_sec", 2.0),
            require_idle_before_target_collection=_pb("require_idle_before_target_collection", True),
            require_end_pose_stable_before_target_collection=_pb(
                "require_end_pose_stable_before_target_collection", True),
            end_pose_stability_window=_pi("end_pose_stability_window", 5),
            end_pose_stability_epsilon_m=_pf("end_pose_stability_epsilon_m", 0.003),

            approach_mode=str(_p("approach_mode", "front")),
            approach_priority=list(_p("approach_priority", ["front", "top_down"])),
            front_first_then_top_down=_pb("front_first_then_top_down", True),
            max_approach_mode_retries=_pi("max_approach_mode_retries", 1),
            blend_approach_enabled=_pb("blend_approach_enabled", True),
            blend_approach_fallback_to_sequential=_pb(
                "blend_approach_fallback_to_sequential", True),
            blend_approach_retry_on_busy=_pb("blend_approach_retry_on_busy", True),
            blend_approach_busy_retry_delay_sec=_pf(
                "blend_approach_busy_retry_delay_sec", 1.5),
            blend_approach_busy_max_retries=_pi(
                "blend_approach_busy_max_retries", 3),
            enable_top_down_fallback=_pb("enable_top_down_fallback", True),
            top_down_max_target_z=_pf("top_down_max_target_z", 0.49),
            top_down_max_pre_grasp_radius_m=_pf("top_down_max_pre_grasp_radius_m", 0.64),
            top_down_pre_grasp_z_offset=_pf("top_down_pre_grasp_z_offset", 0.035),

            pre_grasp_z_offset=_pf("pre_grasp_z_offset", 0.06),
            grasp_z_offset=_pf("grasp_z_offset", 0.02),
            lift_z_offset=_pf("lift_z_offset", 0.04),
            front_approach_x_offset=_pf("front_approach_x_offset", -0.10),
            front_approach_z_offset=_pf("front_approach_z_offset", 0.05),
            adaptive_front_pre_grasp=_pb("adaptive_front_pre_grasp", True),
            workspace_soft_margin_m=_pf("workspace_soft_margin_m", 0.04),
            min_front_pre_grasp_distance_m=_pf("min_front_pre_grasp_distance_m", 0.04),
            front_grasp_x_offset=_pf("front_grasp_x_offset", 0.0),
            front_grasp_x_offset_max=_pf("front_grasp_x_offset_max", 0.075),

            table_z=_pf("table_z", 0.0),
            table_clearance=_pf("table_clearance", 0.04),
            final_grasp_clearance=_pf("final_grasp_clearance", 0.015),
            min_safe_motion_z=_pf("min_safe_motion_z", 0.08),
            reject_target_below_table=_pb("reject_target_below_table", True),
            safe_pose=_plf("safe_pose", [0.35, 0.0, 0.35]),
            official_reach_radius_m=_pf("official_reach_radius_m", 0.68),

            workspace_x_min=wx_min, workspace_x_max=wx_max,
            workspace_y_min=wy_min, workspace_y_max=wy_max,
            workspace_z_min=wz_min, workspace_z_max=wz_max,

            joint6_compensation_deg=_pf("joint6_compensation_deg", 90.0),
            joint6_min_rad=_pf("joint6_min_rad", -2.9671),
            joint6_max_rad=_pf("joint6_max_rad", 2.9671),
            j6_home_deg=_pf("j6_home_deg", 0.0),
            j6_allowed_delta_deg=_pf("j6_allowed_delta_deg", 90.0),
            j6_preferred_offsets_deg=_plf("j6_preferred_offsets_deg", [90.0, -90.0]),
            forbid_camera_upside_down=_pb("forbid_camera_upside_down", True),
            return_j6_to_home_on_recover=_pb("return_j6_to_home_on_recover", True),

            open_gripper_before_grasp=_pb("open_gripper_before_grasp", True),
            pre_grasp_open_timeout_sec=_pf("pre_grasp_open_timeout_sec", 4.0),
            pre_grasp_open_settle_sec=_pf("pre_grasp_open_settle_sec", 1.0),
            gripper_settle_sec=_pf("gripper_settle_sec", 1.0),
            return_to_init_after_grasp=_pb("return_to_init_after_grasp", True),
            keep_gripper_closed_after_grasp=_pb("keep_gripper_closed_after_grasp", True),
            final_init_joint_pos_deg=_plf("final_init_joint_pos_deg", [0.0, -45.0, 110.0, -90.0, 90.0, 0.0]),
            return_init_timeout_sec=_pf("return_init_timeout_sec", 10.0),

            max_cartesian_step=_pf("max_cartesian_step", 0.06),
            lift_cartesian_step_m=_pf("lift_cartesian_step_m", 0.03),
            cart_waypoint_max_step_m=_pf("cart_waypoint_max_step_m", 0.10),
            cart_waypoint_safe_limit_m=_pf("cart_waypoint_safe_limit_m", 0.12),
            position_tolerance_m=_pf("position_tolerance_m", 0.015),
            settle_time_sec=_pf("settle_time_sec", 0.8),
            joint_speed_safe_threshold=_pf("joint_speed_safe_threshold", 0.1),
            post_joint_rotate_settle_sec=_pf("post_joint_rotate_settle_sec", 0.5),
            end_pose_timeout_sec=_pf("end_pose_timeout_sec", 1.0),

            wait_pre_target_warn_sec=_pf("wait_pre_target_warn_sec", 15.0),
            motion_timeout_sec=_pf("motion_timeout_sec", 16.0),
            set_orientation_timeout_sec=_pf("set_orientation_timeout_sec", 8.0),
            close_gripper_timeout_sec=_pf("close_gripper_timeout_sec", 4.0),
            recover_timeout_sec=_pf("recover_timeout_sec", 15.0),
            rejected_busy_recover_threshold=_pi("rejected_busy_recover_threshold", 2),

            recover_clear_error_interval_sec=_pf("recover_clear_error_interval_sec", 0.5),
            recover_return_init_pose=_pb("recover_return_init_pose", True),
            auto_recover_joint_limit=_pb("auto_recover_joint_limit", True),
            post_motion_command_cooldown_sec=_pf("post_motion_command_cooldown_sec", 1.5),
            sequential_busy_max_retries=_pi("sequential_busy_max_retries", 3),

            active_search=asc,
        )

    @property
    def workspace_limits_dict(self) -> dict:
        return {
            "x_min": self.workspace_x_min,
            "x_max": self.workspace_x_max,
            "y_min": self.workspace_y_min,
            "y_max": self.workspace_y_max,
            "z_min": self.workspace_z_min,
            "z_max": self.workspace_z_max,
        }
