"""
对上层暴露的统一控制器：URDFController。

设计原则：
  1. 完全不关心你到底用的是 ROS / Mock / 串口硬件 —— 都通过 Adapter 接口。
  2. 只提供"关节级控制，不涉及限位、不涉及动作脚本。限位和动作脚本交给 action_library 或 UI 层。
  3. 支持 with 上下文管理器，自动 open/close。
  4. 语义名输入 / 度单位 —— 和 action_library 对齐。

基本用法：
    from v15_action_task.control_core import URDFController, RosV14Adapter, MockAdapter

    # 接真实 ROS（v14 协议）
    with URDFController(RosV14Adapter()) as ctl:
        ctl.set_joint("swing_yaw", 10.5)     # 单关节
        ctl.set_pose({"boom_swing": 20, "arm_boom": 30})  # 多位姿

    # 无 ROS 环境调试
    with URDFController(MockAdapter()) as ctl:
        print(ctl.get_pose())
"""

from __future__ import annotations

import time
from typing import Any, Dict, Iterable, Optional, Tuple

from .adapter_base import ControlAdapter
from .mock_adapter import MockAdapter
from .types import SEMANTIC_JOINT_ORDER, default_pose_deg


class URDFController:
    """
    统一关节控制器。
    """

    def __init__(
        self,
        adapter: Optional[ControlAdapter] = None,
        *,
        joint_limits: Optional[Dict[str, Any]] = None,
        clamp: bool = True,
    ):
        """
        Args:
            adapter:      ControlAdapter 实例（None 时用 MockAdapter）
            joint_limits: 可选。dict(semantic_name → JointLimit(min_deg, max_deg))，
                          或任何等价的映射：{name: {min_deg:x, max_deg:y}} / {name:(min, max)}
                          提供后，set_joint / set_pose 会在发布前自动裁剪到限位内。
            clamp:        True=启用限位自动裁剪，False=忽略 joint_limits（仅校验）。
        """
        self._adapter = adapter if adapter is not None else MockAdapter()
        self._limits: Optional[Dict[str, Tuple[float, float]]] = None
        self._clamp = bool(clamp)
        if joint_limits is not None:
            self._limits = {}
            for name, entry in joint_limits.items():
                if hasattr(entry, "min_deg") and hasattr(entry, "max_deg"):
                    self._limits[str(name)] = (float(entry.min_deg), float(entry.max_deg))
                elif isinstance(entry, dict):
                    self._limits[str(name)] = (
                        float(entry.get("min_deg", entry.get("min", -1e9))),
                        float(entry.get("max_deg", entry.get("max", +1e9))),
                    )
                elif isinstance(entry, (list, tuple)) and len(entry) >= 2:
                    self._limits[str(name)] = (float(entry[0]), float(entry[1]))
                else:
                    raise TypeError(
                        f"URDFController: joint_limits[{name}] 无法识别的类型 {type(entry)}"
                    )

    @property
    def joint_limits(self) -> Optional[Dict[str, Tuple[float, float]]]:
        return None if self._limits is None else dict(self._limits)

    def _clamp_pose(self, pose_deg: Dict[str, float]) -> Dict[str, float]:
        if not self._clamp or self._limits is None:
            return pose_deg
        out: Dict[str, float] = {}
        for k, v in pose_deg.items():
            lm = self._limits.get(k)
            vf = float(v)
            if lm is not None:
                if vf < lm[0]:
                    vf = lm[0]
                elif vf > lm[1]:
                    vf = lm[1]
            out[k] = vf
        return out

    # ── 生命周期 ────────────────────────────────────────────────

    def open(self) -> None:
        self._adapter.open()

    def close(self) -> None:
        self._adapter.close()

    def __enter__(self) -> "URDFController":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def adapter(self) -> ControlAdapter:
        return self._adapter

    # ── 关节设置（单步写入）───────────────────────────────────────

    def set_joint(
        self,
        semantic_joint: str,
        value_deg: float,
        *,
        frame_id: str = "base_link",
        clamp: Optional[bool] = None,
    ) -> bool:
        """
        设置单个语义关节的目标角度（度）。其余关节保持最后一次发布的值。

        Args:
            clamp: None=用构造函数的默认，True/False=临时覆盖是否裁剪
        """
        if semantic_joint not in SEMANTIC_JOINT_ORDER:
            raise ValueError(
                f"未知语义关节 {semantic_joint!r}，允许: {list(SEMANTIC_JOINT_ORDER)}"
            )
        raw = {semantic_joint: float(value_deg)}
        pose = self._clamp_pose(raw) if clamp is None or clamp else raw
        return self._adapter.publish_pose_deg(pose, frame_id=frame_id)

    def set_pose(
        self,
        pose_deg: Dict[str, float],
        *,
        frame_id: str = "base_link",
        clamp: Optional[bool] = None,
    ) -> bool:
        """
        发布一组关节角度（度）。未在 dict 里的关节保持不变。

        Args:
            clamp: None=用构造函数的默认，True/False=临时覆盖是否裁剪
        """
        unknown = [k for k in pose_deg.keys() if k not in SEMANTIC_JOINT_ORDER]
        if unknown:
            raise ValueError(
                f"未知语义关节 {unknown!r}，允许: {list(SEMANTIC_JOINT_ORDER)}"
            )
        raw = {k: float(v) for k, v in pose_deg.items()}
        pose = self._clamp_pose(raw) if clamp is None or clamp else raw
        return self._adapter.publish_pose_deg(pose, frame_id=frame_id)

    # ── 关节查询 ────────────────────────────────────────────────

    def get_pose(self) -> Optional[Dict[str, float]]:
        """查询当前角度反馈。没收到过反馈返回 None。"""
        return self._adapter.get_current_pose_deg()

    def get_pose_blocking(self, timeout_s: float = 5.0) -> Optional[Dict[str, float]]:
        """阻塞等待直到收到第一帧反馈，或超时。"""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            p = self.get_pose()
            if p is not None:
                return p
            time.sleep(0.05)
        return None

    def get_pose_or_default(self) -> Dict[str, float]:
        """没收到过反馈则返回全 0。"""
        return self.get_pose() or default_pose_deg()

    # ── 到位检测（简易）──────────────────────────────────────

    def is_at_pose(
        self,
        target_deg: Dict[str, float],
        tolerance_deg: float = 1.0,
        joints: Optional[Iterable[str]] = None,
    ) -> bool:
        """
        判断当前反馈是否已经到达目标。
        joints=None 时只检查 target_deg 中出现过的关节。
        """
        current = self.get_pose()
        if current is None:
            return False
        keys = list(joints) if joints is not None else list(target_deg.keys())
        for k in keys:
            if k not in target_deg or k not in current:
                continue
            diff = abs(float(current[k]) - float(target_deg[k]))
            if diff > tolerance_deg:
                return False
        return True
