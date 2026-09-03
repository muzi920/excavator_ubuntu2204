"""
单点挖掘-卸料任务生成器：dig_point → 挖掘 → 回转 → dump_point → 卸料 → 循环过渡位。

返回标准 JSON 结构：
  {
    "metadata": { "task_name": "...", "dig_point": ..., "dump_point": ..., ... },
    "script":   [ step1, step2, ... ]
  }
其中每个 step 都与 StepBuilder / terminal_stepper / replay_json_script 兼容。
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from ..utils import (
    StepBuilder,
    IKSolver,
    clamp_pose,
    check_pose_limits,
)
from ..primitives import move_joint_step
from ..composites import (
    INIT_POSE,
    move_to_init_pose,
    move_to_cycle_transit_pose,
    align_swing_to_point,
    dig_entry_sequence,
    dump_release_sequence,
)


def _require_ik() -> IKSolver:
    solver = IKSolver.get_singleton()
    if not solver.available:
        raise ImportError(
            "单点任务生成依赖 shandong/v10_cailbration_arm/inverse_kinematics.py，"
            "当前环境 ExcavatorIK 不可用。"
        )
    return solver


def _solve_or_raise(
    solver: IKSolver,
    point: Tuple[float, float, float],
    bucket_range: Tuple[float, float],
    point_label: str,
) -> Dict[str, float]:
    sol = solver.search_pose(*point, bucket_angle_range_deg=bucket_range, num_candidates=17)
    if sol is None:
        raise ValueError(f"{point_label}不可达: {point}")
    ok, viol = check_pose_limits(sol.as_pose())
    if not ok:
        raise ValueError(
            f"{point_label} IK 解裁剪后仍超限 {viol}: raw pose={sol.as_pose()}"
        )
    return sol.as_pose()


def build_single_dig_dump_script(
    sb: StepBuilder,
    dig_pose: Dict[str, float],
    dump_pose: Dict[str, float],
    *,
    dig_point: Optional[Tuple[float, float, float]] = None,
    dump_point: Optional[Tuple[float, float, float]] = None,
    dwell_after_dump_s: float = 0.4,
) -> List[Dict[str, Any]]:
    """
    按已有 pose 直接拼 script（不需要 IK，纯组合层调用）。
    返回生成的 steps 列表（这些 step 同时已被 append 到 sb）。
    """
    out: List[Dict[str, Any]] = []

    # 1) 对准挖掘点 swing
    if dig_point is not None:
        out.append(align_swing_to_point(sb, dig_point[0], dig_point[1],
                                        description="SINGLE: 回转对准挖掘点"))

    # 2) dig_entry_sequence（boom/arm/bucket）
    out.extend(dig_entry_sequence(sb, dig_pose))

    # 3) 抬到循环过渡位
    out.extend(move_to_cycle_transit_pose(sb))

    # 4) 对准卸料点 swing
    if dump_point is not None:
        out.append(align_swing_to_point(sb, dump_point[0], dump_point[1],
                                        description="SINGLE: 回转对准卸料点"))

    # 5) dump_release_sequence
    dump_steps, dwell_proto = dump_release_sequence(
        sb,
        {k: dump_pose[k] for k in ("boom_swing", "arm_boom", "bucket_arm") if k in dump_pose},
        dwell_after_dump_s=dwell_after_dump_s,
    )
    out.extend(dump_steps)
    # dwell 作为单独一步插入（extend 会自动重编号并推进计数器）
    out.extend(sb.extend([dict(dwell_proto)]))

    # 6) 抬到循环过渡位（为下一次挖掘做好准备）
    out.extend(move_to_cycle_transit_pose(sb))

    return out


def build_single_dig_dump_task(
    dig_point: Tuple[float, float, float],
    dump_point: Tuple[float, float, float],
    *,
    task_name: str = "single_dig_dump",
    dig_bucket_range: Tuple[float, float] = (-60.0, -10.0),
    dump_bucket_range: Tuple[float, float] = (-80.0, -30.0),
    include_init_segment: bool = True,
    dwell_after_dump_s: float = 0.4,
) -> Dict[str, Any]:
    """
    从 dig_point 和 dump_point 生成完整 JSON 任务。

    流程:
      [INIT →] 对准挖掘点 → 挖掘 → 过渡 → 对准卸料点 → 卸料 dwell → 过渡
    """
    solver = _require_ik()

    dig_pose = _solve_or_raise(solver, dig_point, dig_bucket_range, "挖掘点")
    dump_pose = _solve_or_raise(solver, dump_point, dump_bucket_range, "卸料点")

    sb = StepBuilder(start=1)
    script: List[Dict[str, Any]] = []

    if include_init_segment:
        script.extend(move_to_init_pose(sb))

    script.extend(build_single_dig_dump_script(
        sb,
        dig_pose, dump_pose,
        dig_point=dig_point, dump_point=dump_point,
        dwell_after_dump_s=dwell_after_dump_s,
    ))

    metadata = {
        "task_name": task_name,
        "dig_point": list(dig_point),
        "dump_point": list(dump_point),
        "dig_pose_deg": clamp_pose(dig_pose),
        "dump_pose_deg": clamp_pose(dump_pose),
        "generated_by": "v15_action_task.action_library.tasks.single_dig_dump",
    }
    return {"metadata": metadata, "script": script}
