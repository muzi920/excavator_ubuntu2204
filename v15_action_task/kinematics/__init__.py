"""
v15 本地运动学子包（完全独立，不再依赖 v10_cailbration_arm/ 外部文件）。

暴露:
  - LinkParams / DEFAULT_PARAMS / get_default_params
  - ForwardKinematics / FKSolution
  - InverseKinematics / IKSolution

快速使用（无 ROS 也能跑）：

    from v15_action_task.kinematics import ForwardKinematics, InverseKinematics

    ik = InverseKinematics()
    sol = ik.solve_bucket_pose(1.2, 0.0, -0.2, bucket_abs_angle_deg=-50.0)
    pose = sol.as_pose()  # {swing_yaw, boom_swing, arm_boom, bucket_arm}

    fk = ForwardKinematics()
    fk_sol = fk.solve(**pose)
    (x, y, z) = fk_sol.bucket_tip_3d  # FK 回推铲尖 3D 位置自洽
"""

from .link_params import (
    DEFAULT_PARAMS,
    LinkParams,
    get_default_params,
)
from .forward import (
    FKSolution,
    ForwardKinematics,
)
from .inverse import (
    IKSolution,
    InverseKinematics,
)

__all__ = [
    # link_params
    "DEFAULT_PARAMS",
    "LinkParams",
    "get_default_params",
    # forward
    "FKSolution",
    "ForwardKinematics",
    # inverse
    "IKSolution",
    "InverseKinematics",
]
