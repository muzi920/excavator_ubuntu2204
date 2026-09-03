"""
铲斗控制原语：标准化的开斗/闭斗/半开斗动作。

v14 工程语义对齐：
  - bucket_arm = 0     → 铲斗接近闭合（收斗取料位）
  - bucket_arm = -45   → 挖掘切入时的半开斗
  - bucket_arm ≈ -90  → 卸料时完全开斗
（限位已在 joint_limits 中定义为 [-95, +45]）
"""

from __future__ import annotations

from typing import Any, Dict

from ..utils import StepBuilder
from .joint_motion import move_joint_step


# 常用铲斗目标位（度）
BUCKET_CLOSED_DEG = 0.0
"""铲斗接近闭合（收斗后姿态）。"""

BUCKET_HALF_OPEN_FOR_DIG_DEG = -45.0
"""下切入土前的半开斗（齿尖先接触地面）。"""

BUCKET_FULL_OPEN_FOR_DUMP_DEG = -90.0
"""卸料时的大开斗（接近 -95 下界）。"""


def close_bucket(sb: StepBuilder, description: str = "") -> Dict[str, Any]:
    """原语：闭斗到 BUCKET_CLOSED_DEG。"""
    return move_joint_step(
        sb,
        "bucket_arm",
        BUCKET_CLOSED_DEG,
        description or "收斗闭合",
    )


def half_open_bucket_for_dig(sb: StepBuilder, description: str = "") -> Dict[str, Any]:
    """原语：半开斗，用于下切入土。"""
    return move_joint_step(
        sb,
        "bucket_arm",
        BUCKET_HALF_OPEN_FOR_DIG_DEG,
        description or "半开斗准备切入",
    )


def full_open_bucket_for_dump(sb: StepBuilder, description: str = "") -> Dict[str, Any]:
    """原语：大开斗，用于卸料。"""
    return move_joint_step(
        sb,
        "bucket_arm",
        BUCKET_FULL_OPEN_FOR_DUMP_DEG,
        description or "大开斗卸料",
    )
