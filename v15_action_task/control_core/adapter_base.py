"""
适配器抽象基类。

URDFController 不直接依赖 ROS，而是通过 "Adapter" 接口与后端交互：

  ┌────────────────────┐       ┌──────────────┐
  │  URDFController    │──────▶│   Adapter    │ ◀── ROS / Mock / 硬件 / ...
  │  (上层统一接口)    │◀──────│              │
  └────────────────────┘       └──────────────┘

后端只需要实现 ControlAdapter 即可，控制器可以随时换后端而不需要改业务代码。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, Optional


class ControlAdapter(ABC):
    """控制后端的抽象接口。"""

    @abstractmethod
    def open(self) -> None:
        """打开/初始化后端。由 URDFController.__enter__ 调用。"""

    @abstractmethod
    def close(self) -> None:
        """关闭/释放后端。由 URDFController.__exit__ 调用。"""

    @abstractmethod
    def publish_pose_deg(self, pose_deg: Dict[str, float], frame_id: str = "base_link") -> bool:
        """
        发布一整组语义关节角度到后端。

        Args:
            pose_deg: 语义关节名 -> 目标角度（度）。允许只传部分关节。
            frame_id: 消息使用的 frame id。

        Returns:
            True 表示发布成功。
        """

    @abstractmethod
    def get_current_pose_deg(self) -> Optional[Dict[str, float]]:
        """
        查询当前关节角度（从反馈话题/硬件读取）。

        返回 None 表示没有最新数据（例如话题从未收到过消息）。
        返回的 dict 必须包含所有 4 个语义关节名。
        """

    @abstractmethod
    def get_last_update_ts(self) -> Optional[float]:
        """最后一次收到反馈的时间戳（秒，time.time() 格式）。没收到过返回 None。"""
