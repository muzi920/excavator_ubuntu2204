"""
正向运动学（FK）：

  输入: V4 风格语义相对角
           boom_swing, arm_boom, bucket_arm  (度)
           加上可选的 swing_yaw              (度，绕 Z 轴，0→前方，+→逆时针看Z轴向下看)

  输出: 所有关键点 (X,Y,Z) 世界坐标 (米)，以及各关节的绝对几何角 (度)。

  所有角度对外 API 都用"度"；内部按需要转弧度。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from .link_params import LinkParams, get_default_params


@dataclass
class FKSolution:
    swing_yaw_deg: float

    boom_swing_deg: float
    arm_boom_deg: float
    bucket_arm_deg: float

    # 传感器绝对读数（按 V4 语义累加）
    sensor_boom_deg: float
    sensor_arm_deg: float
    sensor_bucket_deg: float

    # 标准几何绝对角（向上为正）
    abs_boom_L2_deg: float
    abs_arm_deg: float
    abs_bucket_deg: float

    # 各点坐标，(X, Z) 是相对于回转中心 +X 向前 +Z 向上的平面投影
    # swing_yaw 不为 0 时再加入 Y 分量
    boom_bend_xz: Tuple[float, float] = (0.0, 0.0)
    boom_tip_xz:  Tuple[float, float] = (0.0, 0.0)
    arm_tip_xz:   Tuple[float, float] = (0.0, 0.0)
    bucket_tip_xz: Tuple[float, float] = (0.0, 0.0)

    # 3D 坐标 (含 Y)
    boom_bend_3d: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    boom_tip_3d:  Tuple[float, float, float] = (0.0, 0.0, 0.0)
    arm_tip_3d:   Tuple[float, float, float] = (0.0, 0.0, 0.0)
    bucket_tip_3d: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def as_dict(self) -> Dict[str, object]:
        return {
            "joints_deg": {
                "swing_yaw": self.swing_yaw_deg,
                "boom_swing": self.boom_swing_deg,
                "arm_boom": self.arm_boom_deg,
                "bucket_arm": self.bucket_arm_deg,
            },
            "sensor_abs_deg": {
                "boom": self.sensor_boom_deg,
                "arm": self.sensor_arm_deg,
                "bucket": self.sensor_bucket_deg,
            },
            "geometry_abs_deg": {
                "boom_L2": self.abs_boom_L2_deg,
                "arm": self.abs_arm_deg,
                "bucket": self.abs_bucket_deg,
            },
            "points_xz": {
                "boom_bend": self.boom_bend_xz,
                "boom_tip": self.boom_tip_xz,
                "arm_tip": self.arm_tip_xz,
                "bucket_tip": self.bucket_tip_xz,
            },
            "points_3d": {
                "boom_bend": self.boom_bend_3d,
                "boom_tip": self.boom_tip_3d,
                "arm_tip": self.arm_tip_3d,
                "bucket_tip": self.bucket_tip_3d,
            },
        }


class ForwardKinematics:
    """本地化正向运动学求解器。"""

    def __init__(self, params: Optional[LinkParams] = None):
        self.p = params or get_default_params()

    def solve(
        self,
        boom_swing_deg: float,
        arm_boom_deg: float,
        bucket_arm_deg: float,
        swing_yaw_deg: float = 0.0,
    ) -> FKSolution:
        p = self.p

        # 1) V4 相对角 → 传感器绝对读数
        sensor_boom = boom_swing_deg
        sensor_arm = boom_swing_deg + arm_boom_deg
        sensor_bucket = boom_swing_deg + arm_boom_deg + bucket_arm_deg

        # 2) 传感器 → 标准几何绝对角（向上为正）
        abs_boom_L2 = p.offset_sensor_boom - sensor_boom
        abs_arm = p.offset_sensor_arm - sensor_arm
        abs_bucket = p.offset_sensor_bucket - sensor_bucket

        abs_boom_L1 = abs_boom_L2 + p.boom_bend_angle_deg

        # 3) 各段单位方向向量（X-Z 平面）
        cL1 = math.cos(math.radians(abs_boom_L1))
        sL1 = math.sin(math.radians(abs_boom_L1))
        cL2 = math.cos(math.radians(abs_boom_L2))
        sL2 = math.sin(math.radians(abs_boom_L2))
        ca = math.cos(math.radians(abs_arm))
        sa = math.sin(math.radians(abs_arm))
        cb = math.cos(math.radians(abs_bucket))
        sb = math.sin(math.radians(abs_bucket))

        # 4) 2D 平面逐级累加
        x0, z0 = p.offset_x, p.offset_z

        xb = x0 + p.L1 * cL1
        zb = z0 + p.L1 * sL1

        x1 = xb + p.L2 * cL2
        z1 = zb + p.L2 * sL2

        x2 = x1 + p.L_arm * ca
        z2 = z1 + p.L_arm * sa

        x3 = x2 + p.L_bucket * cb
        z3 = z2 + p.L_bucket * sb

        # 5) swing_yaw 投影到 3D：yaw=0 时 (x,z)=平面 (x,z)；
        #    绕 Z 轴右旋 yaw_deg：新 (x,y) = (x*cos - z*sin ...  不对)
        #    正确：2D 平面里的点是 (x_plane, z)；yaw 后 -> 3D (x_plane*cy, x_plane*sy, z)
        cy = math.cos(math.radians(swing_yaw_deg))
        sy = math.sin(math.radians(swing_yaw_deg))

        def to3d(xp: float, zp: float) -> Tuple[float, float, float]:
            return (xp * cy, xp * sy, zp)

        sol = FKSolution(
            swing_yaw_deg=swing_yaw_deg,
            boom_swing_deg=boom_swing_deg,
            arm_boom_deg=arm_boom_deg,
            bucket_arm_deg=bucket_arm_deg,
            sensor_boom_deg=sensor_boom,
            sensor_arm_deg=sensor_arm,
            sensor_bucket_deg=sensor_bucket,
            abs_boom_L2_deg=abs_boom_L2,
            abs_arm_deg=abs_arm,
            abs_bucket_deg=abs_bucket,
            boom_bend_xz=(xb, zb),
            boom_tip_xz=(x1, z1),
            arm_tip_xz=(x2, z2),
            bucket_tip_xz=(x3, z3),
            boom_bend_3d=to3d(xb, zb),
            boom_tip_3d=to3d(x1, z1),
            arm_tip_3d=to3d(x2, z2),
            bucket_tip_3d=to3d(x3, z3),
        )
        return sol
