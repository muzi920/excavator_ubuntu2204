"""
无 ROS 时的内存 Mock 适配器，用于本地调试 / UI 预演。

行为模拟 RosV14Adapter：
  - publish_pose_deg 时把目标角写入内部 "cmd_deg"
  - get_current_pose_deg 返回最近一次发布的值（模拟立刻到达，零延迟）
  - get_last_update_ts 返回最近一次发布的 time.time()

注意：发布的目标角会立即"立刻成为"当前角度，这和仿真器零延迟响应。
如果需要模拟插值延迟/抖动，可以子类化重写 _apply_cmd_to_feedback。
"""

from __future__ import annotations

import time
import threading
from typing import Dict, Optional

from .types import DEFAULT_FRAME_ID, default_pose_deg
from .adapter_base import ControlAdapter


class MockAdapter(ControlAdapter):
    """纯内存 Mock 后端，不依赖 ROS，适合 UI 或脚本调试。"""

    def __init__(self, initial_pose_deg: Optional[Dict[str, float]] = None):
        self._lock = threading.Lock()
        self._pose_deg: Dict[str, float] = dict(
            initial_pose_deg if initial_pose_deg else default_pose_deg()
        )
        self._last_ts: Optional[float] = None
        self._open = False

    # ── 生命周期 ─────────────────────────────────────────────

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    # ── ControlAdapter 接口 ─────────────────────────────────

    def publish_pose_deg(self, pose_deg: Dict[str, float], frame_id: str = DEFAULT_FRAME_ID) -> bool:
        if not self._open:
            return False
        self._apply_cmd_to_feedback(pose_deg)
        return True

    def _apply_cmd_to_feedback(self, cmd: Dict[str, float]) -> None:
        """子类可重写：把命令 -> 当前反馈。默认立刻同步（零延迟）。"""
        with self._lock:
            for k, v in cmd.items():
                if k in self._pose_deg and v is not None:
                    self._pose_deg[k] = float(v)
            self._last_ts = time.time()

    def get_current_pose_deg(self) -> Optional[Dict[str, float]]:
        with self._lock:
            return dict(self._pose_deg)

    def get_last_update_ts(self) -> Optional[float]:
        with self._lock:
            return self._last_ts
