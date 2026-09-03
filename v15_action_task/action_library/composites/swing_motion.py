"""
回转组合动作：按目标点对齐 swing_yaw。
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional
import math

from ..utils import StepBuilder, clamp_angle
from ..primitives import move_joint_step


def _wrap_angle_diff(target: float, current: float) -> float:
    """返回 target - current，归一化到 (-180, 180]。"""
    diff = (target - current) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff


def align_swing(
    sb: StepBuilder,
    target_yaw_deg: float,
    *,
    current_yaw_deg: Optional[float] = None,
    description: str = "",
    speed_deg_s: Optional[float] = None,
) -> Dict[str, Any]:
    """
    回转到目标 yaw 角。如果提供 current_yaw_deg，会自动选短路径；
    否则直接把目标角裁剪到限位并直接发目标。
    """
    t = clamp_angle("swing_yaw", float(target_yaw_deg))
    if current_yaw_deg is not None:
        diff = _wrap_angle_diff(t, float(current_yaw_deg))
        t = clamp_angle("swing_yaw", float(current_yaw_deg) + diff)

    return move_joint_step(
        sb, "swing_yaw", t,
        description=description or f"回转对准 {t:.2f} deg",
        speed_deg_s=speed_deg_s,
    )


def align_swing_to_point(
    sb: StepBuilder,
    x: float, y: float,
    *,
    current_yaw_deg: Optional[float] = None,
    description: str = "",
    speed_deg_s: Optional[float] = None,
) -> Dict[str, Any]:
    """按目标点 (x,y) 计算 atan2(y,x) 得 yaw 角，对齐回转。"""
    yaw = math.degrees(math.atan2(y, x))
    return align_swing(
        sb, yaw,
        current_yaw_deg=current_yaw_deg,
        description=description or f"回转对点 ({x:.3f}, {y:.3f})",
        speed_deg_s=speed_deg_s,
    )
