"""
大臂/小臂/铲斗 组合动作：挖掘切入序列、卸料释放序列。

这一层组合多个原语，把实际作业中“每次都是同样顺序”的动作固定下来，
避免 task 层重复手写同样的 Step 序列。
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from ..utils import StepBuilder, clamp_pose
from ..primitives import (
    move_joint_step,
    close_bucket,
    half_open_bucket_for_dig,
    full_open_bucket_for_dump,
)


# ── 挖掘切入：半开斗下探 → 收斗取料 ──────────────────────────────────

def dig_entry_sequence(
    sb: StepBuilder,
    dig_pose_deg: Dict[str, float],
    *,
    close_speed_deg_s: Optional[float] = None,
) -> List[Dict[str, Any]]:
    """
    挖掘切入序列（3 步）：
      1) boom/arm 下探到 dig_pose_deg（bucket 先不动，避免提前闭斗）
      2) bucket 半开（便于齿尖切土）——如果 dig_pose_deg 里 bucket_arm 已经
         给到具体值，则直接用目标值
      3) 收斗闭合，完成取料

    传入的 dig_pose_deg 一般来自 IK 求解后的 {boom_swing, arm_boom, bucket_arm}。
    """
    pose = clamp_pose(dig_pose_deg)
    out: List[Dict[str, Any]] = []

    # 1) boom / arm 先到位
    if "boom_swing" in pose:
        out.append(move_joint_step(
            sb, "boom_swing", pose["boom_swing"],
            description="DIG: 大臂下探到挖掘位",
        ))
    if "arm_boom" in pose:
        out.append(move_joint_step(
            sb, "arm_boom", pose["arm_boom"],
            description="DIG: 小臂推进到挖掘位",
        ))

    # 2) bucket 切土姿态（若 pose 中给了 bucket_arm 则用，否则默认半开）
    if "bucket_arm" in pose:
        out.append(move_joint_step(
            sb, "bucket_arm", pose["bucket_arm"],
            description="DIG: 铲斗到达切土姿态",
        ))
    else:
        out.append(half_open_bucket_for_dig(sb))

    # 3) 收斗取料
    out.append(close_bucket(sb, description="DIG: 收斗取料"))
    if close_speed_deg_s is not None and out:
        out[-1]["speed_deg_s"] = float(close_speed_deg_s)

    return out


# ── 卸料释放：到卸料姿态 → 开斗卸料 → 回到过渡位 ────────────────────

def dump_release_sequence(
    sb: StepBuilder,
    dump_arm_pose_deg: Dict[str, float],
    *,
    dwell_after_dump_s: float = 0.4,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """
    卸料释放序列：
      1) boom / arm 到卸料预备姿态
      2) 大开斗卸料
      3) (单独返回) 可选的“保持大开斗静置” step，便于终端步进器做 dwell

    返回 (steps_list, dwell_step_proto)。
    如果调用方想把 dwell 单独做成一步，把 dwell_step_proto 也 append 到 sb 即可；
    不想插入 dwell，直接忽略返回的第二个元素。
    """
    pose = clamp_pose(dump_arm_pose_deg)
    out: List[Dict[str, Any]] = []

    if "boom_swing" in pose:
        out.append(move_joint_step(
            sb, "boom_swing", pose["boom_swing"],
            description="DUMP: 大臂抬到卸料预备位",
        ))
    if "arm_boom" in pose:
        out.append(move_joint_step(
            sb, "arm_boom", pose["arm_boom"],
            description="DUMP: 小臂推到卸料预备位",
        ))
    if "bucket_arm" in pose:
        out.append(move_joint_step(
            sb, "bucket_arm", pose["bucket_arm"],
            description="DUMP: 铲斗到卸料姿态",
        ))

    out.append(full_open_bucket_for_dump(sb, description="DUMP: 开斗卸料"))

    dwell_proto: Dict[str, Any] = {
        "joint": "bucket_arm",
        "target_val": pose.get("bucket_arm", -90.0),
        "description": f"DUMP: 卸料后静置 {dwell_after_dump_s:.2f}s",
        "ramp_up_s": 0.0,
        "ramp_down_s": 0.0,
        "speed_deg_s": 6.0,
        "tolerance_deg": 1.0,
        "is_init_step": False,
        "_dwell_s": float(dwell_after_dump_s),
    }
    return out, dwell_proto
