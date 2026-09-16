"""
逆向运动学（IK）：

  输入: 铲尖 3D 目标点 (x, y, z) 米，以及铲斗绝对几何角 bucket_angle_deg（度）
        - x=前方, y=左方, z=上方（ROS REP103 对齐）
        - bucket_angle_deg：水平向前为 0，向上为正，向下挖掘为负（-60~-20 常用）

  输出: V4 风格语义相对角 {swing_yaw, boom_swing, arm_boom, bucket_arm}（度）
        若不可达返回 None。

  内部实现：
    - 先用 x,y → atan2 → 求出 swing_yaw，将问题归约到 2D 平面
    - 再用 v10 的 2D 平面逆解 + 标定偏置 → boom_swing / arm_boom / bucket_arm
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .link_params import LinkParams, get_default_params


@dataclass
class IKSolution:
    swing_yaw_deg: float
    boom_swing_deg: float
    arm_boom_deg: float
    bucket_arm_deg: float
    bucket_abs_angle_deg: float

    def as_pose(self) -> Dict[str, float]:
        return {
            "swing_yaw": self.swing_yaw_deg,
            "boom_swing": self.boom_swing_deg,
            "arm_boom": self.arm_boom_deg,
            "bucket_arm": self.bucket_arm_deg,
        }


class InverseKinematics:
    """本地化逆向运动学求解器（不依赖 v10 任何文件）。"""

    def __init__(self, params: Optional[LinkParams] = None):
        self.p = params or get_default_params()

    # ── 核心 2D 平面逆解（与 v10 逻辑一致）──────────────────────

    def _solve_2d_deg(
        self, radius_m: float, z_m: float, bucket_abs_angle_deg: float
    ) -> Optional[Tuple[float, float, float]]:
        """
        2D 平面内求解 (boom_swing, arm_boom, bucket_arm) 度。
        坐标输入：radius 是铲尖到回转中心在 X-Y 平面的投影距离（相当于 v10 里的 target_x），
                  z_m 是铲尖 Z（与 v10 target_z 相同）。
        不可达返回 None。
        """
        p = self.p
        theta3 = math.radians(bucket_abs_angle_deg)

        # 1) 扣除铲斗长度 + 底座偏移 → 腕关节 (小臂顶端) 的目标点 (相对于大臂底座的纯连杆坐标系)
        x_wrist = (radius_m - p.offset_x) - p.L_bucket * math.cos(theta3)
        z_wrist = (z_m - p.offset_z) - p.L_bucket * math.sin(theta3)

        # 2) 两连杆 (L_boom, L_arm) 的可达性判断
        distance = math.sqrt(x_wrist**2 + z_wrist**2)
        if distance > (p.L_boom + p.L_arm) or distance < abs(p.L_boom - p.L_arm):
            return None
        if distance < 1e-9:
            return None

        # 3) Elbow-up 几何解
        alpha = math.atan2(z_wrist, x_wrist)
        cos_gamma = (p.L_boom**2 + distance**2 - p.L_arm**2) / (2 * p.L_boom * distance)
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma = math.acos(cos_gamma)
        theta1 = alpha + gamma  # boom 绝对几何角 (相对于等效直线 L_boom)

        # 小臂绝对几何角（从大臂顶端 -> 腕关节 的向量方向）
        x_elbow = p.L_boom * math.cos(theta1)
        z_elbow = p.L_boom * math.sin(theta1)
        theta2 = math.atan2(z_wrist - z_elbow, x_wrist - x_elbow)

        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        theta3_deg = math.degrees(theta3)

        # 4) 几何绝对角 → V4 相对角（修正后正确，与 FK 形成闭环）
        abs_boom_L2_deg = theta1_deg - p.beta_deg
        sensor_boom_deg = p.offset_sensor_boom - abs_boom_L2_deg
        sensor_arm_deg = p.offset_sensor_arm - theta2_deg
        # 正确反映射公式（与 FK 对称：FK 是 abs = offset - sensor → 反映射 sensor = offset - abs 几何_theta3）
        sensor_bucket_deg = p.offset_sensor_bucket - theta3_deg

        boom_swing = sensor_boom_deg
        arm_boom = sensor_arm_deg - sensor_boom_deg
        bucket_arm = sensor_bucket_deg - sensor_arm_deg

        return (boom_swing, arm_boom, bucket_arm)

    # ── 对外：3D 求解 ─────────────────────────────────────────

    def solve_bucket_pose(
        self,
        x_m: float, y_m: float, z_m: float,
        bucket_abs_angle_deg: float = -20.0,
    ) -> Optional[IKSolution]:
        """给铲尖 3D 目标点 + 铲斗绝对角 → V4 风格语义相对角。"""
        radius = math.sqrt(x_m * x_m + y_m * y_m)
        yaw_deg = math.degrees(math.atan2(y_m, x_m))

        sol2 = self._solve_2d_deg(radius, z_m, bucket_abs_angle_deg)
        if sol2 is None:
            return None
        boom, arm, bucket = sol2
        return IKSolution(
            swing_yaw_deg=yaw_deg,
            boom_swing_deg=boom,
            arm_boom_deg=arm,
            bucket_arm_deg=bucket,
            bucket_abs_angle_deg=bucket_abs_angle_deg,
        )

    def search_bucket_angle(
        self,
        x_m: float, y_m: float, z_m: float,
        bucket_range_deg: Tuple[float, float] = (-60.0, 20.0),
        num_candidates: int = 17,
    ) -> Optional[IKSolution]:
        """
        扫描一段铲斗绝对角，优先选最接近限位中心（限位评分最低）的解。
        如果有多个解，选择 boom_swing 最接近中间值 (25°) 的解（能量最小）。
        """
        best: Optional[IKSolution] = None
        best_score = float("-inf")
        lo, hi = bucket_range_deg
        boom_center = 25.0
        for i in range(num_candidates):
            t = 0.0 if num_candidates == 1 else i / (num_candidates - 1)
            b = lo + t * (hi - lo)
            sol = self.solve_bucket_pose(x_m, y_m, z_m, bucket_abs_angle_deg=b)
            if sol is None:
                continue
            # 评分：-|boom_swing - 中心| - |bucket_abs_angle - 范围中心|（角度越平稳越好）
            score = (
                -abs(sol.boom_swing_deg - boom_center) * 1.0
                -abs(sol.bucket_abs_angle_deg - 0.5 * (lo + hi)) * 0.3
            )
            if score > best_score:
                best_score = score
                best = sol
        return best
