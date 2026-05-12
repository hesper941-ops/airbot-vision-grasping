"""多目标类型管理器：支持 duck / green_box / apple 及后续新增目标。

后续新增目标类型只需添加 target_types 配置和对应 detector topic，不需要改抓取状态机。
"""

from typing import Optional

from .config import TargetTypeConfig


class TargetSourceManager:
    """管理 selected_target_type 和对应的输入配置。

    第一版保留 /visual_target_base 兼容逻辑，但代码结构为多目标扩展做好准备。
    """

    def __init__(self, config):
        self._config = config
        self._selected: str = "duck"

    # ------------------------------------------------------------------
    # 目标类型选择
    # ------------------------------------------------------------------

    @property
    def selected_target_type(self) -> str:
        return self._selected

    def set_selected_target_type(self, target_type: str):
        """切换到指定目标类型。必须在 config.target_types 中存在。"""
        if target_type not in self._config.target_types:
            raise ValueError(
                f"Unknown target type: {target_type!r}. "
                f"Known: {sorted(self._config.target_types.keys())}")
        self._selected = target_type

    def get_selected_config(self) -> Optional[TargetTypeConfig]:
        """返回当前选中目标类型的配置。"""
        return self._config.target_types.get(self._selected)

    def get_input_topic(self) -> str:
        """返回当前目标类型的输入 topic。"""
        cfg = self.get_selected_config()
        if cfg is not None:
            return cfg.input_topic
        # 兼容：如果 target_types 为空，回退到 /visual_target_base
        return "/visual_target_base"

    @property
    def known_target_types(self) -> list:
        return sorted(self._config.target_types.keys())
