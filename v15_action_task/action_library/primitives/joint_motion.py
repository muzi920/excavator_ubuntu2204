"""
关节运动原语：把单个关节动作封装为标准 JSON Step。

所有函数都接受外部 StepBuilder，以便不同原语、不同组合层可以共享同一条
step 编号序列，避免重复编号或中间插步时顺序错乱。
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from ..utils import (
    StepBuilder,
    clamp_angle,
)


# ── 单关节单步 ──────────────────────────────────────────────────────────

def move_joint_step(
    sb: StepBuilder,
    joint: str,
    target_val_deg: float,
    description: str = "",
    *,
    ramp_up_s: Optional[float] = None,
    ramp_down_s: Optional[float] = None,
    speed_deg_s: Optional[float] = None,
    tolerance_deg: Optional[float] = None,
    is_init_step: bool = False,
) -> Dict[str, Any]:
    """
    原语：单关节移动一步。目标角度会被自动裁剪到关节限位内。
    返回追加后的 Step dict（也会被 append 到 sb.steps）。
    """
    clamped = clamp_angle(joint, float(target_val_deg))
    desc = description or f"move {joint} -> {clamped:.2f} deg"
    return sb.build(
        joint,
        clamped,
        desc,
        ramp_up_s=ramp_up_s,
        ramp_down_s=ramp_down_s,
        speed_deg_s=speed_deg_s,
        tolerance_deg=tolerance_deg,
        is_init_step=is_init_step,
    )


# ── 多关节独立分步（不保证同步，分别到达各自目标） ────────────────────

def move_joint_steps_independent(
    sb: StepBuilder,
    pose_deg: Dict[str, float],
    *,
    joint_order: Optional[List[str]] = None,
    description_prefix: str = "",
    is_init_step: bool = False,
    speed_deg_s_map: Optional[Dict[str, float]] = None,
) -> List[Dict[str, Any]]:
    """
    把一整组 pose 拆成若干个独立单步。

    joint_order 默认按 ["swing_yaw", "boom_swing", "arm_boom", "bucket_arm"]
    顺序推进，这是挖掘机工作循环中比较自然的顺序：
      1) 先转台回转到位
      2) 再调整大臂
      3) 再调整小臂
      4) 最后收/开斗
    """
    order = joint_order or [
        "swing_yaw", "boom_swing", "arm_boom", "bucket_arm",
    ]
    speed_map = speed_deg_s_map or {}
    out: List[Dict[str, Any]] = []
    for j in order:
        if j not in pose_deg:
            continue
        step = move_joint_step(
            sb,
            j,
            float(pose_deg[j]),
            description=f"{description_prefix}{j} -> {pose_deg[j]:.2f} deg",
            is_init_step=is_init_step,
            speed_deg_s=speed_map.get(j),
        )
        out.append(step)
    return out
