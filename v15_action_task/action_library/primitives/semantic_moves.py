"""
语义级增量动作原语 (v15 Task 2)。

提供 8 个函数：
  4 个基础增量布尔函数（方向语义 = FR-2.1 约定）：
    - swing_move(ctl, right_or_left, angle_deg)   → swing_yaw += angle_deg  (正 = 顺时针)
    - boom_move(ctl, up_or_down, angle_deg)       → boom_swing -= angle_deg (负 = 向上抬升)
    - arm_move(ctl, up_or_down, angle_deg)        → arm_boom += angle_deg   (正 = 向内收回)
    - bucket_move(ctl, up_or_down, angle_deg)     → bucket_arm += angle_deg (正 = 向内卷斗)

  4 个扩展便利函数：
    - swing_move_to(ctl, abs_yaw_deg, **)         → 回转到绝对 yaw 角
    - boom_lift(mover, delta_z_m, keep_xy=True)   → 大臂抬/落 delta_z (笛卡尔)
    - arm_extend(mover, delta_x_m, keep_yz=True)  → 小臂伸/收 delta_x (笛卡尔)
    - bucket_tilt_to(ctl, abs_bucket_tip_deg)     → TODO stub 铲斗倾角绝对值

AC-1 FR-1.2 限位夹紧分层：
  Layer 2 (发布前): URDFController._clamp_pose (内部自动)
  Layer 3 (发布前): cfg.limits.clamp_pose / clamp_joint (本模块显式调用)
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Dict, Optional

try:
    from ...motion.cartesian_mover import MoveResult
    from ...control_core import URDFController
except (ImportError, ValueError):
    try:
        from motion.cartesian_mover import MoveResult
        from control_core import URDFController
    except ImportError:
        raise

if TYPE_CHECKING:
    try:
        from ...motion.cartesian_mover import CartesianMover
    except (ImportError, ValueError):
        from motion.cartesian_mover import CartesianMover


# ───────────────────────────────────────────────────────────
# 工具：获取有效 cfg / clamp 单关节
# ───────────────────────────────────────────────────────────

def _resolve_cfg(ctl: URDFController, cfg: Any) -> Any:
    """
    cfg 解析链：
      1) 调用方显式传 cfg → 直接用
      2) 否则尝试 ctl.adapter.config 属性
      3) 否则用 load_default_config()
    """
    if cfg is not None:
        return cfg
    try:
        adapter_cfg = getattr(ctl.adapter, "config", None)
        if adapter_cfg is not None:
            return adapter_cfg
    except Exception:
        pass
    try:
        from ...config import load_default_config
    except (ImportError, ValueError):
        from config import load_default_config
    return load_default_config()


def _clamp_single(cfg: Any, joint: str, raw_target: float) -> float:
    """
    用 cfg.limits.clamp_pose 做单关节限位夹紧（Layer 3）。
    若 cfg.limits 不可用（兜底异常），直接返回 raw_target。
    """
    try:
        clamped_pose = cfg.limits.clamp_pose({joint: raw_target})
        return float(clamped_pose[joint])
    except Exception:
        return float(raw_target)


def _base_joint_move(
    ctl: URDFController,
    joint: str,
    delta_sign: int,
    angle_deg: float,
    *,
    blocking: bool = True,
    tolerance_deg: float = 1.0,
    timeout_s: float = 3.0,
    cfg: Any = None,
) -> MoveResult:
    """
    基础增量单关节运动通用内核。

    Args:
        delta_sign: +1 表示 raw_target = cur + angle_deg
                    -1 表示 raw_target = cur - angle_deg
    """
    cfg_eff = _resolve_cfg(ctl, cfg)
    cur = ctl.get_pose_or_default()
    cur_val = float(cur.get(joint, 0.0))
    raw_target = cur_val + (delta_sign * float(angle_deg))
    clamp = _clamp_single(cfg_eff, joint, raw_target)
    published = ctl.set_joint(joint, clamp)
    if not published:
        return MoveResult(
            success=False,
            reached_pose_deg=ctl.get_pose_or_default(),
            final_tip_xyz=None,
            ik_solution=None,
            target_xyz=(0.0, 0.0, 0.0),
            target_bucket_angle_deg=None,
            waited_s=0.0,
            reason="adapter publish failed",
        )
    if not blocking:
        return MoveResult(
            success=True,
            reached_pose_deg=ctl.get_pose_or_default(),
            final_tip_xyz=None,
            ik_solution=None,
            target_xyz=(0.0, 0.0, 0.0),
            target_bucket_angle_deg=None,
            waited_s=0.0,
            reason="nonblocking",
        )
    target_pose = {joint: clamp}
    joints = [joint]
    t0 = time.monotonic()
    deadline = t0 + timeout_s
    if ctl.is_at_pose(target_pose, tolerance_deg, joints):
        return MoveResult(
            success=True,
            reached_pose_deg=ctl.get_pose_or_default(),
            final_tip_xyz=None,
            ik_solution=None,
            target_xyz=(0.0, 0.0, 0.0),
            target_bucket_angle_deg=None,
            waited_s=0.0,
            reason="",
        )
    while time.monotonic() < deadline:
        if ctl.is_at_pose(target_pose, tolerance_deg, joints):
            elapsed = time.monotonic() - t0
            return MoveResult(
                success=True,
                reached_pose_deg=ctl.get_pose_or_default(),
                final_tip_xyz=None,
                ik_solution=None,
                target_xyz=(0.0, 0.0, 0.0),
                target_bucket_angle_deg=None,
                waited_s=elapsed,
                reason="",
            )
        time.sleep(0.02)
    ok = ctl.is_at_pose(target_pose, tolerance_deg, joints)
    elapsed = time.monotonic() - t0
    return MoveResult(
        success=ok,
        reached_pose_deg=ctl.get_pose_or_default(),
        final_tip_xyz=None,
        ik_solution=None,
        target_xyz=(0.0, 0.0, 0.0),
        target_bucket_angle_deg=None,
        waited_s=elapsed,
        reason="" if ok else "timeout",
    )


# ───────────────────────────────────────────────────────────
# 4 个基础增量布尔函数（FR-2.1 方向语义）
# ───────────────────────────────────────────────────────────

def swing_move(
    ctl: URDFController,
    right_or_left: bool,
    angle_deg: float,
    *,
    blocking: bool = True,
    tolerance_deg: float = 1.0,
    timeout_s: float = 3.0,
    cfg: Any = None,
) -> MoveResult:
    """
    回转摆动：swing_yaw += angle_deg（正 = 顺时针，从上往下看）。

    Args:
        right_or_left: True = 顺时针（yaw 加），False = 逆时针（yaw 减）。
    """
    delta_sign = +1 if bool(right_or_left) else -1
    return _base_joint_move(
        ctl, "swing_yaw", delta_sign, angle_deg,
        blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
    )


def boom_move(
    ctl: URDFController,
    up_or_down: bool,
    angle_deg: float,
    *,
    blocking: bool = True,
    tolerance_deg: float = 1.0,
    timeout_s: float = 3.0,
    cfg: Any = None,
) -> MoveResult:
    """
    大臂俯仰：boom_swing -= angle_deg（负 = 向上抬升，正 = 向下落）。

    Args:
        up_or_down: True = 向上抬升（boom_swing 减），False = 向下落（boom_swing 加）。
    """
    delta_sign = -1 if bool(up_or_down) else +1
    return _base_joint_move(
        ctl, "boom_swing", delta_sign, angle_deg,
        blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
    )


def arm_move(
    ctl: URDFController,
    up_or_down: bool,
    angle_deg: float,
    *,
    blocking: bool = True,
    tolerance_deg: float = 1.0,
    timeout_s: float = 3.0,
    cfg: Any = None,
) -> MoveResult:
    """
    小臂收放：arm_boom += angle_deg（正 = 向内收回，负 = 向外伸出）。

    Args:
        up_or_down: True = 向内收回（arm_boom 加），False = 向外伸出（arm_boom 减）。
    """
    delta_sign = +1 if bool(up_or_down) else -1
    return _base_joint_move(
        ctl, "arm_boom", delta_sign, angle_deg,
        blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
    )


def bucket_move(
    ctl: URDFController,
    up_or_down: bool,
    angle_deg: float,
    *,
    blocking: bool = True,
    tolerance_deg: float = 1.0,
    timeout_s: float = 3.0,
    cfg: Any = None,
) -> MoveResult:
    """
    铲斗卷翻：bucket_arm += angle_deg（正 = 向内卷斗，负 = 向外翻斗）。

    Args:
        up_or_down: True = 向内卷斗（bucket_arm 加），False = 向外翻斗（bucket_arm 减）。
    """
    delta_sign = +1 if bool(up_or_down) else -1
    return _base_joint_move(
        ctl, "bucket_arm", delta_sign, angle_deg,
        blocking=blocking, tolerance_deg=tolerance_deg, timeout_s=timeout_s, cfg=cfg,
    )


# ───────────────────────────────────────────────────────────
# 4 个扩展便利函数
# ───────────────────────────────────────────────────────────

def swing_move_to(
    ctl: URDFController,
    abs_yaw_deg: float,
    **kwargs: Any,
) -> MoveResult:
    """
    回转到绝对 yaw 角（增量 = abs_yaw_deg - 当前）。

    自动判断方向：diff >= 0 → 顺时针（True），否则逆时针（False）。
    """
    cur = ctl.get_pose_or_default()
    cur_yaw = float(cur.get("swing_yaw", 0.0))
    diff = float(abs_yaw_deg) - cur_yaw
    right_or_left = diff >= 0
    angle_deg = abs(diff)
    return swing_move(ctl, right_or_left, angle_deg, **kwargs)


def boom_lift(
    mover: "CartesianMover",
    delta_z_m: float,
    *,
    keep_xy: bool = True,
    **kwargs: Any,
) -> MoveResult:
    """
    大臂抬/落 delta_z 米（笛卡尔空间，保持铲斗姿搜索）。

    Args:
        mover: 已构造的 CartesianMover 实例。
        delta_z_m: 正 = 向上抬，负 = 向下落。
        keep_xy: True=保持当前 x/y 不变（默认），False=只保留 y（兼容占位）。
    """
    try:
        cur_pose = mover.current_pose()
    except Exception:
        cur_pose = mover.ctl.get_pose_or_default()
    try:
        cur_tip = mover.current_tip()
    except Exception:
        cur_tip = (0.0, 0.0, 0.0)
    cx, cy, cz = cur_tip
    if keep_xy:
        new_x, new_y = cx, cy
    else:
        new_x, new_y = cx, cy
    new_z = cz + float(delta_z_m)
    return mover.move(new_x, new_y, new_z, **kwargs)


def arm_extend(
    mover: "CartesianMover",
    delta_x_m: float,
    *,
    keep_yz: bool = True,
    **kwargs: Any,
) -> MoveResult:
    """
    小臂伸/收 delta_x 米（笛卡尔空间，x 正向前方）。

    Args:
        mover: 已构造的 CartesianMover 实例。
        delta_x_m: 正 = 向前伸出，负 = 向后收回。
        keep_yz: True=保持当前 y/z 不变（默认）。
    """
    try:
        cur_tip = mover.current_tip()
    except Exception:
        cur_tip = (0.0, 0.0, 0.0)
    cx, cy, cz = cur_tip
    if keep_yz:
        new_y, new_z = cy, cz
    else:
        new_y, new_z = cy, cz
    new_x = cx + float(delta_x_m)
    return mover.move(new_x, new_y, new_z, **kwargs)


def bucket_tilt_to(
    ctl: URDFController,
    abs_bucket_tip_deg: float,
    **kwargs: Any,
) -> MoveResult:
    """
    TODO stub: 铲斗倾角设定到绝对值。

    当前按语义等价：把 bucket_arm 设定为 abs_bucket_tip_deg 对应的增量目标
    （近似等价：绝对角 ≈ bucket_arm，待后续接入 FK 精确映射）。

    Args:
        abs_bucket_tip_deg: 目标铲斗绝对角（度）。
    """
    cfg_eff = _resolve_cfg(ctl, kwargs.pop("cfg", None))
    cur = ctl.get_pose_or_default()
    cur_val = float(cur.get("bucket_arm", 0.0))
    raw_target = float(abs_bucket_tip_deg)
    clamp = _clamp_single(cfg_eff, "bucket_arm", raw_target)
    blocking = bool(kwargs.get("blocking", True))
    tolerance_deg = float(kwargs.get("tolerance_deg", 1.0))
    timeout_s = float(kwargs.get("timeout_s", 3.0))
    published = ctl.set_joint("bucket_arm", clamp)
    if not published:
        return MoveResult(
            success=False,
            reached_pose_deg=ctl.get_pose_or_default(),
            final_tip_xyz=None,
            ik_solution=None,
            target_xyz=(0.0, 0.0, 0.0),
            target_bucket_angle_deg=float(abs_bucket_tip_deg),
            waited_s=0.0,
            reason="adapter publish failed",
        )
    if not blocking:
        return MoveResult(
            success=True,
            reached_pose_deg=ctl.get_pose_or_default(),
            final_tip_xyz=None,
            ik_solution=None,
            target_xyz=(0.0, 0.0, 0.0),
            target_bucket_angle_deg=float(abs_bucket_tip_deg),
            waited_s=0.0,
            reason="nonblocking",
        )
    target_pose = {"bucket_arm": clamp}
    joints = ["bucket_arm"]
    t0 = time.monotonic()
    deadline = t0 + timeout_s
    if ctl.is_at_pose(target_pose, tolerance_deg, joints):
        return MoveResult(
            success=True,
            reached_pose_deg=ctl.get_pose_or_default(),
            final_tip_xyz=None,
            ik_solution=None,
            target_xyz=(0.0, 0.0, 0.0),
            target_bucket_angle_deg=float(abs_bucket_tip_deg),
            waited_s=0.0,
            reason="",
        )
    while time.monotonic() < deadline:
        if ctl.is_at_pose(target_pose, tolerance_deg, joints):
            elapsed = time.monotonic() - t0
            return MoveResult(
                success=True,
                reached_pose_deg=ctl.get_pose_or_default(),
                final_tip_xyz=None,
                ik_solution=None,
                target_xyz=(0.0, 0.0, 0.0),
                target_bucket_angle_deg=float(abs_bucket_tip_deg),
                waited_s=elapsed,
                reason="",
            )
        time.sleep(0.02)
    ok = ctl.is_at_pose(target_pose, tolerance_deg, joints)
    elapsed = time.monotonic() - t0
    return MoveResult(
        success=ok,
        reached_pose_deg=ctl.get_pose_or_default(),
        final_tip_xyz=None,
        ik_solution=None,
        target_xyz=(0.0, 0.0, 0.0),
        target_bucket_angle_deg=float(abs_bucket_tip_deg),
        waited_s=elapsed,
        reason="" if ok else "timeout",
    )


__all__ = [
    "swing_move",
    "boom_move",
    "arm_move",
    "bucket_move",
    "swing_move_to",
    "boom_lift",
    "arm_extend",
    "bucket_tilt_to",
]
