"""
挖掘-卸料组合动作：转运到卸料区 + 卸料过程。

严格通过 semantic_moves 中的 4 个布尔函数（swing_move / boom_move / arm_move /
bucket_move）来表达方向语义，严禁在本层手算 Δpose 正负号，避免方向语义错误。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

try:
    from ..primitives.semantic_moves import (
        swing_move,
        boom_move,
        arm_move,
        bucket_move,
    )
    from ...motion.cartesian_mover import MoveResult
    from ...control_core import URDFController
except (ImportError, ValueError):
    from action_library.primitives.semantic_moves import (
        swing_move,
        boom_move,
        arm_move,
        bucket_move,
    )
    from motion.cartesian_mover import MoveResult
    from control_core import URDFController


# ───────────────────────────────────────────────────────────
# 聚合结果对象
# ───────────────────────────────────────────────────────────

@dataclass
class CompositeActionResult:
    """组合动作的聚合结果：包含每一步原始 MoveResult 和整体是否全部成功。"""

    success: bool
    step_results: List[Any] = field(default_factory=list)  # List[MoveResult]
    final_pose_deg: Optional[Dict[str, float]] = None
    reason: str = ""

    def __bool__(self) -> bool:
        return self.success

    def __len__(self) -> int:
        return len(self.step_results)

    def __iter__(self):
        return iter(self.step_results)


# ───────────────────────────────────────────────────────────
# 接口2：转运到卸料区上方（大臂提升 + 回转）
# ───────────────────────────────────────────────────────────

def move_to_dump_transit(
    ctl: URDFController,
    *,
    target_point: Optional[Tuple[float, float, float]] = None,
    up_m: float,
    swing_deg: float,
    boom_deg_per_meter: float = 30.0,
    blocking: bool = True,
    tolerance_deg: float = 1.0,
    timeout_s: float = 3.0,
    cfg: Any = None,
) -> CompositeActionResult:
    """将挖掘后的料转运到卸料区上方（接口2）。

    动作顺序（严格布尔封装，不手算符号）：
      1. 大臂抬升：boom_move(up=True, angle_deg=round(up_m * boom_deg_per_meter, 2))
      2. 回转到卸料区：swing_move(right_or_left=(swing_deg >= 0), angle_deg=abs(swing_deg))

    Args:
        ctl: 已进入上下文的 URDFController（需 with ctl: 或 ctl.__enter__()）。
        target_point: 预留的目标参考点（当前不参与计算，保持接口对齐）。
        up_m: 大臂向上抬升的米数（正数）。默认按 boom_deg_per_meter 换算为角度。
        swing_deg: 回转角度（度）。正=顺时针，负=逆时针（与 swing_move 语义一致）。
        boom_deg_per_meter: 每米抬升对应的大臂转动角度倍率，默认 30°/m。
        blocking / tolerance_deg / timeout_s / cfg: 透传给每个原语。

    Returns:
        CompositeActionResult，step_results 长度为 2（抬升 + 回转）。
    """
    steps: List[MoveResult] = []

    # Step 1: 大臂抬升
    up_angle_deg = round(float(up_m) * float(boom_deg_per_meter), 2)
    r_boom = boom_move(
        ctl, True, up_angle_deg,
        blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
    )
    steps.append(r_boom)

    # Step 2: 回转到卸料区
    swing_right = bool(float(swing_deg) >= 0.0)
    swing_abs = abs(float(swing_deg))
    if swing_abs > 1e-6:
        r_swing = swing_move(
            ctl, swing_right, swing_abs,
            blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
        )
        steps.append(r_swing)

    all_ok = all(bool(s) for s in steps)
    final_pose = ctl.get_pose_or_default() if hasattr(ctl, "get_pose_or_default") else None
    reason = "" if all_ok else "; ".join(
        f"step#{i}: {s.reason}" for i, s in enumerate(steps) if not bool(s)
    )
    return CompositeActionResult(
        success=all_ok,
        step_results=steps,
        final_pose_deg=final_pose,
        reason=reason,
    )


# ───────────────────────────────────────────────────────────
# 接口3：卸料过程
# ───────────────────────────────────────────────────────────

def dump_material(
    ctl: URDFController,
    *,
    shake_count: int = 1,
    shake_angle_deg: float = 30.0,
    half_open_deg: float = 45.0,
    arm_push_deg: float = 30.0,
    full_open_deg: float = 50.0,
    blocking: bool = True,
    tolerance_deg: float = 1.0,
    timeout_s: float = 3.0,
    cfg: Any = None,
) -> CompositeActionResult:
    """卸料过程（接口3）。

    用户强制动作序列（B+）：
      1. 铲斗半开 45°：bucket_move(up=False, half_open_deg)
      2. 小臂半推 30°：arm_move(up=False, arm_push_deg)
      3. 铲斗全开 50°：bucket_move(up=False, full_open_deg)
      4. 抖斗 N 次：bucket_move(up/down 交替, shake_angle_deg)

    Args:
        ctl: 已进入上下文的 URDFController。
        shake_count: 抖斗次数，默认 1（对应 2 个动作：卷 + 翻），总步数 4+2=6。
        shake_angle_deg: 每次抖斗的摆动角度（度），默认 30°。
        half_open_deg: 半开斗角度（度），默认 45°（向外翻）。
        arm_push_deg: 小臂半推角度（度），默认 30°（向外伸）。
        full_open_deg: 全开斗角度（度），默认 50°（向外翻）。
        blocking / tolerance_deg / timeout_s / cfg: 透传给每个原语。

    Returns:
        CompositeActionResult，总 step_results 长度 ∈ [4, 6]。
    """
    steps: List[MoveResult] = []
    shake_n = max(0, int(shake_count))

    # Step 1: 铲斗半开
    r1 = bucket_move(
        ctl, False, float(half_open_deg),
        blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
    )
    steps.append(r1)

    # Step 2: 小臂半推
    r2 = arm_move(
        ctl, False, float(arm_push_deg),
        blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
    )
    steps.append(r2)

    # Step 3: 铲斗全开
    r3 = bucket_move(
        ctl, False, float(full_open_deg),
        blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
    )
    steps.append(r3)

    # Step 4: 抖斗（交替卷/翻 shake_count 次，每次 2 个动作）
    for i in range(shake_n):
        # 卷斗一下
        s_up = bucket_move(
            ctl, True, float(shake_angle_deg),
            blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
        )
        steps.append(s_up)
        # 翻斗回位
        s_down = bucket_move(
            ctl, False, float(shake_angle_deg),
            blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
        )
        steps.append(s_down)

    all_ok = all(bool(s) for s in steps)
    final_pose = ctl.get_pose_or_default() if hasattr(ctl, "get_pose_or_default") else None
    reason = "" if all_ok else "; ".join(
        f"step#{i}: {s.reason}" for i, s in enumerate(steps) if not bool(s)
    )
    return CompositeActionResult(
        success=all_ok,
        step_results=steps,
        final_pose_deg=final_pose,
        reason=reason,
    )
