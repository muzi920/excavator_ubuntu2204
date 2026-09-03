"""
本地化 IK 封装（不再依赖 shandong/v10_cailbration_arm/ 任何文件！v15 标准库专用）。

将 v15 本地化的 InverseKinematics 封装为动作库友好的接口：
  - 三维点 (x,y,z) → (radius, yaw_deg, z)
  - 姿态搜索评分函数
  - 解的限位检查 + clamp
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# 双导入模式：子包相对 / 顶层绝对 都能跑
try:
    from ...kinematics import InverseKinematics as _V15IK
except (ImportError, ValueError):
    try:
        from kinematics import InverseKinematics as _V15IK
    except ImportError:
        _V15IK = None  # type: ignore

from .joint_limits import JOINT_LIMITS, clamp_pose


@dataclass
class CylindricalPoint:
    """三维点的柱坐标表示（挖掘机平面逆解输入）。"""
    radius: float
    yaw_deg: float
    z: float

    @classmethod
    def from_cartesian(cls, x: float, y: float, z: float) -> "CylindricalPoint":
        radius = math.sqrt(x * x + y * y)
        yaw_deg = math.degrees(math.atan2(y, x))
        return cls(radius=radius, yaw_deg=yaw_deg, z=z)


@dataclass
class PoseSolution:
    """逆解结果。"""
    swing_yaw: float
    boom_swing: float
    arm_boom: float
    bucket_arm: float
    bucket_abs_angle_deg: float
    score: float

    def as_pose(self) -> Dict[str, float]:
        return {
            "swing_yaw": self.swing_yaw,
            "boom_swing": self.boom_swing,
            "arm_boom": self.arm_boom,
            "bucket_arm": self.bucket_arm,
        }


class IKSolver:
    """
    动作库统一 IK 接口（v15 标准库版：100% 本地化，零外部依赖）。

    内部懒加载 v15 InverseKinematics，和动作库其他模块解耦。
    """

    _CACHED_INSTANCE: Optional["IKSolver"] = None

    def __init__(self):
        self._ik = None
        if _V15IK is not None:
            try:
                self._ik = _V15IK()
            except Exception:
                self._ik = None

    @classmethod
    def get_singleton(cls) -> "IKSolver":
        if cls._CACHED_INSTANCE is None:
            cls._CACHED_INSTANCE = cls()
        return cls._CACHED_INSTANCE

    @property
    def available(self) -> bool:
        return self._ik is not None

    # ── 核心求解 ────────────────────────────────────────────────────────

    def solve_bucket_pose(
        self,
        x: float, y: float, z: float,
        bucket_abs_angle_deg: float = -20.0,
    ) -> Optional[PoseSolution]:
        """
        给定铲尖目标点 (x,y,z) 与铲斗绝对角（度，水平为 0，向下为负），
        返回唯一 PoseSolution，无解则返回 None。

        返回的 pose 已经过限位裁剪（clamp_pose），调用方再用 check_pose_limits
        判断是否有被裁剪（=原解不可达）。
        """
        if not self.available:
            raise ImportError(
                "v15 InverseKinematics 不可用（v15_action_task/kinematics 导入失败）"
            )
        sol = self._ik.solve_bucket_pose(
            x, y, z, bucket_abs_angle_deg=bucket_abs_angle_deg,
        )
        if sol is None:
            return None
        pose_raw = {
            "swing_yaw": float(sol.swing_yaw_deg),
            "boom_swing": float(sol.boom_swing_deg),
            "arm_boom": float(sol.arm_boom_deg),
            "bucket_arm": float(sol.bucket_arm_deg),
        }
        pose = clamp_pose(pose_raw)
        diff = 0.0
        for k in pose_raw:
            diff += abs(pose[k] - pose_raw[k])
        score = -diff
        return PoseSolution(
            swing_yaw=pose["swing_yaw"],
            boom_swing=pose["boom_swing"],
            arm_boom=pose["arm_boom"],
            bucket_arm=pose["bucket_arm"],
            bucket_abs_angle_deg=float(bucket_abs_angle_deg),
            score=score,
        )

    def search_pose(
        self,
        x: float, y: float, z: float,
        bucket_angle_range_deg: Tuple[float, float] = (-60.0, 20.0),
        num_candidates: int = 17,
    ) -> Optional[PoseSolution]:
        """
        在一段铲斗绝对角范围里搜索评分最高的解。
        """
        best: Optional[PoseSolution] = None
        lo, hi = bucket_angle_range_deg
        for i in range(num_candidates):
            t = i / max(1, num_candidates - 1)
            b = lo + t * (hi - lo)
            sol = self.solve_bucket_pose(x, y, z, bucket_abs_angle_deg=b)
            if sol is None:
                continue
            if best is None or sol.score > best.score:
                best = sol
        return best
