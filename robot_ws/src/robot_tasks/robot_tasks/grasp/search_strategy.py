"""主动搜索策略：当 detector 检测不到目标时，通过改变观察姿态来寻找抓取物。

SEARCH_TARGET 不是 RECOVER。RECOVER 负责错误恢复；SEARCH_TARGET 负责目标不可见时
主动观察。搜索动作仍然通过 arm_executor_node 执行。

内部子状态：
  SEARCH_IDLE → SEND_SEARCH_POSE → WAIT_MOTION_DONE → WAIT_SETTLE
  → DETECT → (target_found | next_pose | search_failed)
"""

from typing import Optional

from .search_pose_manager import SearchPose, SearchPoseManager
from .status_model import SearchResult, SearchResultStatus


class SearchStrategy:
    """主动搜索目标的子状态机。

    通过一组 YAML 配置的安全观察姿态来寻找目标。
    第一版只做固定姿态序列搜索，不做复杂视觉伺服。
    """

    def __init__(self, config, command_port, target_manager,
                 search_pose_manager: SearchPoseManager):
        self._config = config
        self._command_port = command_port
        self._target_manager = target_manager
        self._pose_manager = search_pose_manager

        self.active: bool = False
        self.state: str = "SEARCH_IDLE"
        self._current_pose: Optional[SearchPose] = None
        self._state_start_sec: float = 0.0
        self._search_start_sec: float = 0.0

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------

    def start(self, context) -> SearchResult:
        """进入搜索。返回首次结果。"""
        if not self._config.active_search.enabled:
            return SearchResult.failed("active_search_disabled")

        self.active = True
        self.state = "SEND_SEARCH_POSE"
        self._pose_manager.reset()
        self._search_start_sec = context.now_sec
        self._state_start_sec = context.now_sec
        self._current_pose = None
        context.task_state = "SEARCH_TARGET"

        return SearchResult.running("search_started")

    def tick(self, context) -> SearchResult:
        """每个 timer tick 调用一次。"""
        if not self.active:
            return SearchResult.idle()

        # 整体搜索超时
        if self._is_search_timeout(context):
            self.active = False
            self.state = "SEARCH_FAILED"
            return SearchResult.failed("search_timeout")

        # executor error
        if context.executor_status in ("ERROR", "TIMEOUT"):
            self.active = False
            return SearchResult.executor_error()

        # ---- SEND_SEARCH_POSE ----
        if self.state == "SEND_SEARCH_POSE":
            return self._tick_send_pose(context)

        # ---- WAIT_MOTION_DONE ----
        if self.state == "WAIT_MOTION_DONE":
            return self._tick_wait_motion(context)

        # ---- WAIT_SETTLE ----
        if self.state == "WAIT_SETTLE":
            return self._tick_wait_settle(context)

        # ---- DETECT ----
        if self.state == "DETECT":
            return self._tick_detect(context)

        return SearchResult.running(self.state)

    def is_active(self) -> bool:
        return self.active

    @property
    def current_pose_name(self) -> str:
        if self._current_pose is None:
            return ""
        return self._current_pose.name

    # ------------------------------------------------------------------
    # 子状态
    # ------------------------------------------------------------------

    def _tick_send_pose(self, context) -> SearchResult:
        self._current_pose = self._pose_manager.get_next_pose()

        if self._current_pose is None:
            self.active = False
            self.state = "SEARCH_FAILED"
            return SearchResult.failed("all_search_poses_exhausted")

        # 先设速度
        self._command_port.publish_speed_profile(
            str(self._current_pose.speed_scale),
            reason=f"search:{self._current_pose.name}",
        )
        # 再发关节目标
        self._command_port.publish_joint_target(
            self._current_pose.joint_pos,
            reason=f"search_pose:{self._current_pose.name}",
        )
        context.current_search_pose_name = self._current_pose.name

        self.state = "WAIT_MOTION_DONE"
        self._state_start_sec = context.now_sec
        return SearchResult.running(f"moving_to_{self._current_pose.name}")

    def _tick_wait_motion(self, context) -> SearchResult:
        if context.executor_status in ("DONE", "IDLE"):
            self.state = "WAIT_SETTLE"
            self._state_start_sec = context.now_sec
            return SearchResult.running("search_pose_reached")
        return SearchResult.running("waiting_motion_done")

    def _tick_wait_settle(self, context) -> SearchResult:
        settle = self._current_pose.settle_time_sec if self._current_pose else 0.5
        if context.now_sec - self._state_start_sec >= settle:
            self.state = "DETECT"
            self._state_start_sec = context.now_sec
            self._target_manager.reset_stability()
            return SearchResult.running("detecting")
        return SearchResult.running("settling")

    def _tick_detect(self, context) -> SearchResult:
        # 检查是否有稳定目标
        if self._target_manager.is_stable():
            snapshot = self._target_manager.make_fixed_snapshot(context.now_sec)
            if snapshot is not None:
                context.fixed_target_snapshot = snapshot
                self.active = False
                self.state = "SEARCH_TARGET_FOUND"
                return SearchResult.target_found()

        # 检测窗口是否超时
        detect_window = self._current_pose.detect_window_sec if self._current_pose else 1.0
        if context.now_sec - self._state_start_sec >= detect_window:
            self.state = "SEND_SEARCH_POSE"
            self._state_start_sec = context.now_sec
            return SearchResult.running("next_search_pose")

        return SearchResult.running("detecting")

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _is_search_timeout(self, context) -> bool:
        return (
            context.now_sec - self._search_start_sec
        ) > self._config.active_search.search_timeout_sec
