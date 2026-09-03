"""
标准姿态组合：初始化位 / 循环过渡位 / 归位。

这些 pose 的具体数值按 v4/v14 的工程习惯整定，保持与
README 中推荐 init/cycle_transit/home 一致。
"""

from __future__ import annotations

from typing import Any, Dict, List

from ..utils import StepBuilder, clamp_pose
from ..primitives import move_joint_steps_independent


# ── 标准姿态（度）────────────────────────────────────────────────────────

INIT_POSE: Dict[str, float] = clamp_pose({
    "swing_yaw": 0.0,
    "boom_swing": 10.0,
    "arm_boom": 20.0,
    "bucket_arm": -30.0,
})
"""开机后的标准准备位：回转居中，大臂微抬，小臂和铲斗处于待机。"""

CYCLE_TRANSIT_POSE: Dict[str, float] = clamp_pose({
    "boom_swing": 25.0,
    "arm_boom": 55.0,
    "bucket_arm": -10.0,
})
"""
挖掘循环内部的过渡位：回转不强制（保留当前方向），
只把 boom/arm/bucket 抬到一个安全的高空过渡姿态。
"""

HOME_POSE: Dict[str, float] = clamp_pose({
    "swing_yaw": 0.0,
    "boom_swing": 5.0,
    "arm_boom": 10.0,
    "bucket_arm": -80.0,
})
"""任务结束归位：回转归零，大小臂收拢，铲斗大开以避免剐蹭。"""


# ── 生成 Steps ──────────────────────────────────────────────────────────

def move_to_init_pose(sb: StepBuilder) -> List[Dict[str, Any]]:
    """生成 INIT_POSE 的四步（含 swing），并都标记 is_init_step=True。"""
    return move_joint_steps_independent(
        sb, INIT_POSE,
        joint_order=["swing_yaw", "boom_swing", "arm_boom", "bucket_arm"],
        description_prefix="INIT: ",
        is_init_step=True,
    )


def move_to_cycle_transit_pose(sb: StepBuilder) -> List[Dict[str, Any]]:
    """生成 CYCLE_TRANSIT_POSE（不动 swing，只动 boom/arm/bucket）。"""
    return move_joint_steps_independent(
        sb, CYCLE_TRANSIT_POSE,
        joint_order=["boom_swing", "arm_boom", "bucket_arm"],
        description_prefix="CYCLE: ",
        is_init_step=False,
    )


def move_to_home_pose(sb: StepBuilder) -> List[Dict[str, Any]]:
    """生成 HOME_POSE 四步归位。"""
    return move_joint_steps_independent(
        sb, HOME_POSE,
        joint_order=["swing_yaw", "boom_swing", "arm_boom", "bucket_arm"],
        description_prefix="HOME: ",
        is_init_step=False,
    )
