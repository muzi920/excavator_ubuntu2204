"""
关节限位与裁剪工具。

按照 v4/v10/v14 工程语义对齐：
  - swing_yaw   : 回转角，±180°
  - boom_swing  : 大臂相对上车，-5°（抬起）~ +55°（下探）
  - arm_boom    : 小臂相对大臂，0°（伸出）~ +130°（收回）
  - bucket_arm  : 铲斗相对小臂，-95°（开斗卸料）~ +45°（收斗闭合）
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


@dataclass
class JointLimit:
    """单个关节的角度限位（单位：度）。"""
    min_deg: float
    max_deg: float

    def contains(self, value_deg: float) -> bool:
        return self.min_deg - 1e-9 <= value_deg <= self.max_deg + 1e-9

    def clamp(self, value_deg: float) -> float:
        if value_deg < self.min_deg:
            return self.min_deg
        if value_deg > self.max_deg:
            return self.max_deg
        return value_deg


JOINT_LIMITS: Dict[str, JointLimit] = {
    "swing_yaw":  JointLimit(-180.0, 180.0),
    "boom_swing": JointLimit(-5.0, 55.0),
    "arm_boom":   JointLimit(0.0, 130.0),
    "bucket_arm": JointLimit(-95.0, 45.0),
}

VALID_JOINT_NAMES = set(JOINT_LIMITS.keys())


def clamp_angle(joint_name: str, value_deg: float) -> float:
    """裁剪单个关节角到限位内。关节名不存在时原样返回。"""
    lim = JOINT_LIMITS.get(joint_name)
    if lim is None:
        return float(value_deg)
    return lim.clamp(float(value_deg))


def clamp_pose(pose_deg: Dict[str, float]) -> Dict[str, float]:
    """裁剪一整组 pose（dict）到关节限位内。缺失的关节名保持原样。"""
    out: Dict[str, float] = {}
    for k, v in pose_deg.items():
        out[k] = clamp_angle(k, float(v))
    return out


def check_pose_limits(pose_deg: Dict[str, float]) -> Tuple[bool, Dict[str, Tuple[float, float, float]]]:
    """
    检查一整组 pose 是否全部在限位内。

    返回:
      (ok: bool, violations: dict)
      violations 的 key 是超限关节名，value 是 (value, min, max)。
    """
    violations: Dict[str, Tuple[float, float, float]] = {}
    for k, v in pose_deg.items():
        lim = JOINT_LIMITS.get(k)
        if lim is None:
            continue
        vf = float(v)
        if not lim.contains(vf):
            violations[k] = (vf, lim.min_deg, lim.max_deg)
    return (len(violations) == 0, violations)


def default_pose_deg() -> Dict[str, float]:
    """返回四个关节都取 0 的默认 pose。"""
    return {
        "swing_yaw": 0.0,
        "boom_swing": 0.0,
        "arm_boom": 0.0,
        "bucket_arm": 0.0,
    }


# ==============================================================
# 配置文件集成（v15 YAML config 层）
# ==============================================================


def from_joint_limits_config(cfg: Any) -> Dict[str, JointLimit]:
    """
    从 config.JointLimitsConfig 构造 JOINT_LIMITS 字典（不会修改模块全局 JOINT_LIMITS）。

    用法：
        from v15_action_task.config import load_default_config
        cfg = load_default_config()
        limits = from_joint_limits_config(cfg.limits)
        # → 传入 URDFController(adapter, limits=limits)
    """
    if hasattr(cfg, "to_joint_limits_dict"):
        return cfg.to_joint_limits_dict()
    # 兼容传入 dict：{semantic: {min_deg, max_deg}}
    if isinstance(cfg, dict):
        out: Dict[str, JointLimit] = {}
        for name, entry in cfg.items():
            if isinstance(entry, dict):
                out[str(name)] = JointLimit(float(entry["min_deg"]), float(entry["max_deg"]))
            elif isinstance(entry, JointLimit):
                out[str(name)] = entry
            elif isinstance(entry, (list, tuple)) and len(entry) >= 2:
                out[str(name)] = JointLimit(float(entry[0]), float(entry[1]))
        return out
    raise TypeError(f"无法识别的 JointLimits 配置类型: {type(cfg)}")


def apply_limits_config(cfg: Any) -> Dict[str, JointLimit]:
    """
    直接覆盖本模块全局的 JOINT_LIMITS / VALID_JOINT_NAMES（注意会影响全局 clamp_pose / check_pose_limits）。

    推荐：优先用 from_joint_limits_config() 显式传入 URDFController(limits=...)，避免全局副作用。
    """
    new_limits = from_joint_limits_config(cfg)
    global JOINT_LIMITS, VALID_JOINT_NAMES
    JOINT_LIMITS.clear()
    JOINT_LIMITS.update(new_limits)
    VALID_JOINT_NAMES = set(JOINT_LIMITS.keys())
    return new_limits


__all__ = [
    "JointLimit", "JOINT_LIMITS", "VALID_JOINT_NAMES",
    "clamp_angle", "clamp_pose", "check_pose_limits", "default_pose_deg",
    "from_joint_limits_config", "apply_limits_config",
]

