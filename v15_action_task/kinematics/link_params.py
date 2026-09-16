"""
挖掘机连杆参数与结构常量。

这些参数与 v10 项目中 ExcavatorKinematics / ExcavatorIK 使用的完全一致，
本地化后不再引用 shandong/v10_cailbration_arm/ 下任何文件。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class LinkParams:
    """挖掘机 2D 平面（X-Z 平面）连杆几何参数。"""

    # 坐标系原点（回转中心地面投影）→ 大臂底座销轴的偏移
    offset_x: float
    offset_z: float

    # 大臂的两段（L1=底座→折弯点，L2=折弯点→小臂连接点）
    L1: float
    L2: float
    boom_bend_angle_deg: float

    # 小臂、铲斗长度
    L_arm: float
    L_bucket: float

    # 传感器零点偏置
    offset_sensor_boom: float      # abs_boom_L2   = offset_sensor_boom - sensor_boom
    offset_sensor_arm: float       # abs_arm       = offset_sensor_arm  - sensor_arm
    offset_sensor_bucket: float    # abs_bucket    = offset_sensor_bucket - sensor_bucket

    # 派生量：大臂等效直线长度 & 结构偏置角 beta（L_boom 与 L2 的夹角）
    L_boom: float
    beta_deg: float

    @staticmethod
    def compute_boom_equiv(
        L1: float, L2: float, boom_bend_angle_deg: float
    ) -> tuple[float, float]:
        """由 L1 / L2 / 折弯角 推出 (L_boom, beta_deg)。"""
        inner = math.radians(180.0 - boom_bend_angle_deg)
        L_boom = math.sqrt(L1**2 + L2**2 - 2 * L1 * L2 * math.cos(inner))
        sin_beta = (L1 * math.sin(inner)) / L_boom
        beta_deg = math.degrees(math.asin(sin_beta))
        return L_boom, beta_deg


# 挖掘机 v10 标定参数（后续换机型只需修改这里一个文件）
_L_BOOM, _BETA_DEG = LinkParams.compute_boom_equiv(0.35, 0.60, 46.0)
DEFAULT_PARAMS = LinkParams(
    offset_x=0.25,
    offset_z=0.40,
    L1=0.35,
    L2=0.60,
    boom_bend_angle_deg=46.0,
    L_arm=0.44,
    L_bucket=0.26,
    offset_sensor_boom=40.9,
    offset_sensor_arm=19.6,
    offset_sensor_bucket=-56.2,
    L_boom=_L_BOOM,
    beta_deg=_BETA_DEG,
)


def get_default_params() -> LinkParams:
    """返回默认的挖掘机 v10 标定参数（单例形式）。"""
    return DEFAULT_PARAMS


# ==============================================================
# 配置文件集成（v15 YAML config 层）
# ==============================================================


def from_link_geometry_config(cfg: Any) -> LinkParams:
    """
    从 config.LinkGeometryConfig 直接构造 LinkParams（推荐，避免重复计算）。

    用法：
        from v15_action_task.config import load_default_config
        cfg = load_default_config()
        lp  = from_link_geometry_config(cfg.link)  # 或者直接 cfg.link.to_link_params()
    """
    if hasattr(cfg, "to_link_params"):
        return cfg.to_link_params()
    # 兼容传入 dict（兜底）
    return _build_link_params_from_dict(cfg)


def _build_link_params_from_dict(d: dict) -> LinkParams:
    """从 dict（与 LinkGeometryConfig 字段同名）构造 LinkParams。"""
    offset_x = float(d["offset_x"]); offset_z = float(d["offset_z"])
    L1 = float(d["L1"]); L2 = float(d["L2"])
    bend = float(d["boom_bend_angle_deg"])
    L_arm = float(d["L_arm"]); L_bucket = float(d["L_bucket"])
    os_boom = float(d["offset_sensor_boom"]); os_arm = float(d["offset_sensor_arm"])
    os_bucket = float(d["offset_sensor_bucket"])
    L_boom, beta = LinkParams.compute_boom_equiv(L1, L2, bend)
    return LinkParams(
        offset_x=offset_x, offset_z=offset_z,
        L1=L1, L2=L2, boom_bend_angle_deg=bend,
        L_arm=L_arm, L_bucket=L_bucket,
        offset_sensor_boom=os_boom, offset_sensor_arm=os_arm, offset_sensor_bucket=os_bucket,
        L_boom=L_boom, beta_deg=beta,
    )


__all__ = [
    "LinkParams", "DEFAULT_PARAMS", "get_default_params",
    "from_link_geometry_config",
]

