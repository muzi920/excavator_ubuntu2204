"""
v15 motion 子包 —— 末端笛卡尔空间运动接口（高层封装）。

把 Kinematics（IK） + ControlCore（URDFController → /joint_states）
组合成一行代码的高层 API：给末端 (x, y, z) 目标点 → 机器人自动到位。

典型用法（无需用户手动拼接 IK、发布、轮询：

    from v15_action_task import (
        URDFController, MockAdapter,
        ForwardKinematics, InverseKinematics,
        move_to_cartesian, CartesianMover,
    )

    ik = InverseKinematics()
    with URDFController(MockAdapter()) as ctl:
        # 一行到位（阻塞，默认 3s 超时，铲斗角度自动搜索）
        ok, reached_pose, final_tip = move_to_cartesian(
            ctl, ik, x=1.0, y=0.0, z=-0.2,
        )

或者：

    mover = CartesianMover(ctl, ik)
    mover.move(0.8, 0.2, 0.0, blocking=True)   # 自动搜索 bucket 角度并到位
    mover.move_with_bucket(1.0, 0.0, -0.25, bucket_angle_deg=-60.0)
"""

from .cartesian_mover import CartesianMover, MoveResult, move_to_cartesian

__all__ = [
    "CartesianMover",
    "MoveResult",
    "move_to_cartesian",
]
