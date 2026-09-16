"""
多点挖掘循环任务：init_segment + N×(dig→dump) + home_segment。

每个循环内部都使用 cycle_transit_pose 作为 dig↔dump 之间以及轮次之间的
安全过渡姿态，避免每一轮都回到 INIT 做无意义动作。
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from ..utils import StepBuilder, IKSolver, clamp_pose, check_pose_limits
from ..composites import (
    move_to_init_pose,
    move_to_cycle_transit_pose,
    move_to_home_pose,
    align_swing_to_point,
    dig_entry_sequence,
    dump_release_sequence,
)
from .single_dig_dump import _solve_or_raise, _require_ik


def build_multi_dig_cycles(
    dig_points: List[Tuple[float, float, float]],
    dump_point: Tuple[float, float, float],
    *,
    task_name: str = "multi_dig_cycle",
    dig_bucket_range: Tuple[float, float] = (-60.0, -10.0),
    dump_bucket_range: Tuple[float, float] = (-80.0, -30.0),
    include_init_segment: bool = True,
    include_home_segment: bool = True,
    dwell_after_dump_s: float = 0.4,
) -> Dict[str, Any]:
    """
    多点循环任务生成器。

    结构:
      [INIT →]
        对每个 dig_point:
          对准 dig → 挖掘 → 过渡 → 对准 dump → 卸料 dwell → 过渡
      [→ HOME]
    """
    solver = _require_ik()

    dump_pose = _solve_or_raise(solver, dump_point, dump_bucket_range, "卸料点")
    dig_poses: List[Dict[str, float]] = []
    for i, dp in enumerate(dig_points):
        dig_poses.append(_solve_or_raise(solver, dp, dig_bucket_range, f"挖掘点#{i+1}"))

    sb = StepBuilder(start=1)
    script: List[Dict[str, Any]] = []

    if include_init_segment:
        script.extend(move_to_init_pose(sb))

    for i, (dp, dpose) in enumerate(zip(dig_points, dig_poses)):
        # 对准挖掘点
        script.append(align_swing_to_point(
            sb, dp[0], dp[1],
            description=f"MULTI #{i+1}: 回转对准挖掘点",
        ))

        # 挖掘
        script.extend(dig_entry_sequence(sb, dpose))

        # 过渡位
        script.extend(move_to_cycle_transit_pose(sb))

        # 对准卸料点
        script.append(align_swing_to_point(
            sb, dump_point[0], dump_point[1],
            description=f"MULTI #{i+1}: 回转对准卸料点",
        ))

        # 卸料 + dwell
        dump_steps, dwell_proto = dump_release_sequence(
            sb,
            {k: dump_pose[k] for k in ("boom_swing", "arm_boom", "bucket_arm") if k in dump_pose},
            dwell_after_dump_s=dwell_after_dump_s,
        )
        script.extend(dump_steps)
        # dwell 作为单独一步插入（extend 会自动重编号并推进计数器）
        script.extend(sb.extend([dict(dwell_proto)]))

        # 过渡位（为下一轮准备）
        script.extend(move_to_cycle_transit_pose(sb))

    if include_home_segment:
        script.extend(move_to_home_pose(sb))

    metadata = {
        "task_name": task_name,
        "dump_point": list(dump_point),
        "dump_pose_deg": clamp_pose(dump_pose),
        "dig_points": [list(p) for p in dig_points],
        "dig_poses_deg": [clamp_pose(p) for p in dig_poses],
        "num_cycles": len(dig_points),
        "generated_by": "v15_action_task.action_library.tasks.multi_dig_cycle",
    }
    return {"metadata": metadata, "script": script}


def build_multi_dig_task(*args: Any, **kwargs: Any) -> Dict[str, Any]:
    """build_multi_dig_cycles 的兼容别名。"""
    return build_multi_dig_cycles(*args, **kwargs)
