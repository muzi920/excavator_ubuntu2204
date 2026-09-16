"""shandong_0.5.control_lib 两个独立功能库（不改动 v15_action_task 任何原始文件）。

两个库的硬约束（挖掘机硬件特性：只能单关节运动，不能复合动作）：
  * 任何时刻只允许 4 个关节中的 **1 个关节运动**，其它 3 个保持当前值不变。
  * 运动顺序默认：swing_yaw -> boom_swing -> arm_boom -> bucket_arm
    （先回转到位，再抬/降大臂，再收/伸小臂，最后开/收铲斗 — 避免扫碰 + 保证奇异边界安全）

导出：
  * JointAngleController  — 库 1：角度控制（单关节）
  * SingleJointPointMover — 库 2：目标点控制（先 IK 求 4 关节目标，再串行单关节）
"""
from __future__ import annotations

from .angle_control import JointAngleController
from .point_control import SingleJointPointMover

__all__ = ["JointAngleController", "SingleJointPointMover"]
