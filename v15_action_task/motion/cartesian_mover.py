"""
CartesianMover —— 末端 笛卡尔空间运动执行器。

核心能力：
1. move_to_cartesian(x, y, z)              → IK 自动搜索铲斗角 + 发布 + 等待到位
2. move_with_bucket(x,y,z, bucket_angle) → 指定铲斗绝对角 + 发布 + 等待到位
3. 支持阻塞 / 非阻塞，超时控制，容差设置

所有坐标约定（与 kinematics 一致）：
  - X 轴：前方（从底盘指向前方）
  - Y 轴：左方（驾驶员视角）
  - Z 轴：上方（地面之上）
  - 单位：米，米，米
  - bucket_angle_deg：铲斗绝对几何角，向上为正，水平为 0，挖掘常用 -60~-20

无 ROS 也能跑（MockAdapter），有 ROS 就直接驱动 RViz / 真机。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

try:
    from ..control_core import URDFController
    from ..kinematics import ForwardKinematics, InverseKinematics, IKSolution
except (ImportError, ValueError):
    from control_core import URDFController
    from kinematics import ForwardKinematics, InverseKinematics, IKSolution


@dataclass
class MoveResult:
    """一次 move 操作的结果。"""

    success: bool
    reached_pose_deg: Optional[Dict[str, float]]
    final_tip_xyz: Optional[Tuple[float, float, float]]
    ik_solution: Optional[IKSolution]
    target_xyz: Tuple[float, float, float]
    target_bucket_angle_deg: Optional[float]
    waited_s: float
    reason: str = ""

    def __bool__(self) -> bool:
        return self.success


def _run_wait_loop(
    ctl: URDFController,
    target: Dict[str, float],
    *,
    tolerance_deg: float,
    timeout_s: float,
    poll_interval_s: float,
) -> Tuple[bool, float]:
    """轮询直到到位或超时。返回 (是否到位, 实际等待秒数)。"""
    t0 = time.monotonic()
    deadline = t0 + timeout_s
    # 发布命令之前先立刻检查一次（Mock 是零延迟同步，立即到位）
    if ctl.is_at_pose(target, tolerance_deg):
        return True, 0.0
    while time.monotonic() < deadline:
        if ctl.is_at_pose(target, tolerance_deg):
            return True, time.monotonic() - t0
        time.sleep(poll_interval_s)
    # 最后再查一次
    ok = ctl.is_at_pose(target, tolerance_deg)
    return ok, time.monotonic() - t0


def move_to_cartesian(
    controller: URDFController,
    ik: InverseKinematics,
    x_m: float,
    y_m: float,
    z_m: float,
    *,
    bucket_angle_deg: Optional[float] = None,
    bucket_range_deg: Tuple[float, float] = (-70.0, 10.0),
    num_bucket_candidates: int = 17,
    blocking: bool = True,
    tolerance_deg: float = 1.0,
    timeout_s: float = 3.0,
    poll_interval_s: float = 0.05,
    fk: Optional[ForwardKinematics] = None,
) -> MoveResult:
    """
    一行到位：末端目标 (x,y,z) → IK → 发布 → 等待到位。

    Parameters
    ----------
    bucket_angle_deg:
        - None → 自动搜索 bucket_range_deg 范围内的铲斗角（推荐）。
        - 指定数值 → 使用该铲斗绝对几何角（向上为正，0，向下挖掘负）。
    blocking:
        True  阻塞直到 is_at_pose 或 timeout。
        False 仅发布命令立刻返回（不保证执行器真正到位）。
    tolerance_deg:
        各关节语义相对角的容差（度）。
    timeout_s:
        阻塞等待的最大秒数。
    poll_interval_s:
        轮询间隔秒数。
    fk:
        可选 ForwardKinematics 实例，未传则内部临时创建，用于 final_tip 回推。
    """
    target_xyz = (float(x_m), float(y_m), float(z_m))
    # 1) IK
    if bucket_angle_deg is None:
        sol = ik.search_bucket_angle(
            *target_xyz,
            bucket_range_deg=bucket_range_deg,
            num_candidates=num_bucket_candidates,
        )
        used_bucket = None if sol is None else sol.bucket_abs_angle_deg
    else:
        sol = ik.solve_bucket_pose(*target_xyz, bucket_abs_angle_deg=float(bucket_angle_deg))
        used_bucket = float(bucket_angle_deg)
    if sol is None:
        return MoveResult(
            success=False,
            reached_pose_deg=None,
            final_tip_xyz=None,
            ik_solution=None,
            target_xyz=target_xyz,
            target_bucket_angle_deg=used_bucket,
            waited_s=0.0,
            reason="IK 无解 (超出工作空间或铲斗角不合适)",
        )
    pose_cmd = sol.as_pose()
    # 2) 发布
    published = controller.set_pose(pose_cmd)
    if not published:
        return MoveResult(
            success=False,
            reached_pose_deg=None,
            final_tip_xyz=None,
            ik_solution=sol,
            target_xyz=target_xyz,
            target_bucket_angle_deg=used_bucket,
            waited_s=0.0,
            reason="Adapter 发布失败",
        )
    # 3) 等待 (或跳过）
    waited = 0.0
    ok = True
    if blocking:
        ok, waited = _run_wait_loop(
            controller, pose_cmd,
            tolerance_deg=tolerance_deg,
            timeout_s=timeout_s,
            poll_interval_s=poll_interval_s,
        )
    # 4) 收集当前实际到达位姿 + 实际末端
    reached = controller.get_pose_or_default()
    final_tip: Optional[Tuple[float, float, float]]
    if fk is None:
        fk = ForwardKinematics(params=ik.p)
    try:
        fk_sol = fk.solve(
            boom_swing_deg=reached.get("boom_swing", 0.0),
            arm_boom_deg=reached.get("arm_boom", 0.0),
            bucket_arm_deg=reached.get("bucket_arm", 0.0),
            swing_yaw_deg=reached.get("swing_yaw", 0.0),
        )
        final_tip = fk_sol.bucket_tip_3d
    except Exception:
        final_tip = None
    reason = "" if ok else f"超时未达容差 {tolerance_deg}°（{timeout_s}s)"
    return MoveResult(
        success=ok,
        reached_pose_deg=reached,
        final_tip_xyz=final_tip,
        ik_solution=sol,
        target_xyz=target_xyz,
        target_bucket_angle_deg=used_bucket,
        waited_s=waited,
        reason=reason,
    )


class CartesianMover:
    """面向对象封装，方便保存 ik/ctl/tolerance 等默认配置。"""

    def __init__(
        self,
        controller: URDFController,
        ik: Optional[InverseKinematics] = None,
        *,
        fk: Optional[ForwardKinematics] = None,
        default_tolerance_deg: float = 1.0,
        default_timeout_s: float = 3.0,
        default_poll_s: float = 0.05,
        default_bucket_range: Tuple[float, float] = (-70.0, 10.0),
        default_bucket_candidates: int = 17,
    ) -> None:
        self.ctl = controller
        self.ik = ik or InverseKinematics()
        if fk is not None:
            self.fk = fk
        else:
            self.fk = ForwardKinematics(params=self.ik.p)
        self.tol = default_tolerance_deg
        self.timeout = default_timeout_s
        self.poll_s = default_poll_s
        self.bucket_range = default_bucket_range
        self.bucket_candidates = default_bucket_candidates

    # ── 高层 API ──────────────────────────────────────────────

    def move(
        self,
        x_m: float, y_m: float, z_m: float,
        *,
        blocking: bool = True,
        tolerance_deg: Optional[float] = None,
        timeout_s: Optional[float] = None,
    ) -> MoveResult:
        """自动搜索铲斗角 + 到位。"""
        return move_to_cartesian(
            self.ctl, self.ik, x_m, y_m, z_m,
            bucket_angle_deg=None,
            bucket_range_deg=self.bucket_range,
            num_bucket_candidates=self.bucket_candidates,
            blocking=blocking,
            tolerance_deg=tolerance_deg if tolerance_deg is not None else self.tol,
            timeout_s=timeout_s if timeout_s is not None else self.timeout,
            poll_interval_s=self.poll_s,
            fk=self.fk,
        )

    def move_with_bucket(
        self,
        x_m: float, y_m: float, z_m: float,
        bucket_angle_deg: float,
        *,
        blocking: bool = True,
        tolerance_deg: Optional[float] = None,
        timeout_s: Optional[float] = None,
    ) -> MoveResult:
        """指定铲斗绝对角到位。"""
        return move_to_cartesian(
            self.ctl, self.ik, x_m, y_m, z_m,
            bucket_angle_deg=bucket_angle_deg,
            blocking=blocking,
            tolerance_deg=tolerance_deg if tolerance_deg is not None else self.tol,
            timeout_s=timeout_s if timeout_s is not None else self.timeout,
            poll_interval_s=self.poll_s,
            fk=self.fk,
        )

    # ── 便捷 getter ──────────────────────────────────────────

    def current_pose(self) -> Dict[str, float]:
        return self.ctl.get_pose_or_default()

    def current_tip(self) -> Tuple[float, float, float]:
        """当前反馈 → FK → 铲尖 3D。"""
        p = self.current_pose()
        sol = self.fk.solve(
            boom_swing_deg=p.get("boom_swing", 0.0),
            arm_boom_deg=p.get("arm_boom", 0.0),
            bucket_arm_deg=p.get("bucket_arm", 0.0),
            swing_yaw_deg=p.get("swing_yaw", 0.0),
        )
        return sol.bucket_tip_3d

    # ── FK 便捷方法（P4 新增：对外暴露 fk，避免用户手动 import ForwardKinematics）───────

    def recompute_tip(
        self,
        pose_deg: Dict[str, float],
    ) -> Tuple[float, float, float]:
        """
        给定任意一组语义关节角 → 通过内部 FK 计算铲尖 3D 坐标。

        【用途】
          - UI 层拖动 4 关节滑条时，**实时预览铲尖空间位置**（不用真正发布到硬件）
          - 剧本回放前预检查：某步动作是否会超限或目标点不可达
          - 调试 IK 时：先给定关节 → FK 拿 tip → 再 IK 反解看是否自洽

        【参数】
          pose_deg: 语义关节 dict，支持部分 key（缺省视为 0°）
                    {"swing_yaw": float, "boom_swing": float, "arm_boom": float, "bucket_arm": float}

        【返回】
          (x, y, z) 单位米，来自 fk.solve(...).bucket_tip_3d

        【示例】
        ```python
        from v15_action_task import from_config
        ctx = from_config()
        mover = ctx["mover"]

        # 预览铲尖（不发任何指令到硬件）
        tip = mover.recompute_tip({"boom_swing": 20, "arm_boom": 40, "bucket_arm": -70, "swing_yaw": 10})
        print(f"预览铲尖 = ({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}) m")
        # → 默认 60FED 输出约 (1.46, 0.25, 0.46) m
        ```
        """
        sol = self.fk.solve(
            boom_swing_deg=float(pose_deg.get("boom_swing", 0.0)),
            arm_boom_deg=float(pose_deg.get("arm_boom", 0.0)),
            bucket_arm_deg=float(pose_deg.get("bucket_arm", 0.0)),
            swing_yaw_deg=float(pose_deg.get("swing_yaw", 0.0)),
        )
        return sol.bucket_tip_3d

    def recompute_pose_solution(
        self,
        pose_deg: Dict[str, float],
    ):
        """
        给定任意语义关节角 → 返回完整 FKSolution（不仅铲尖，还包括所有关键点、角度）。

        【返回字段（FKSolution）】
          - boom_tip_3d / arm_tip_3d / bucket_tip_3d : 3 个关键点的 3D 坐标
          - boom_tip_xz / arm_tip_xz / bucket_tip_xz : XZ 平面 2D 投影
          - abs_boom_deg / abs_arm_deg / abs_bucket_deg : 各段的绝对几何角

        【适用场景】
          - 画 UI 实时骨架图：拿到 boom_tip、arm_tip 画两杆折线
          - 数学问题定位：某段的绝对角异常、连杆几何出问题时查中间量
        """
        return self.fk.solve(
            boom_swing_deg=float(pose_deg.get("boom_swing", 0.0)),
            arm_boom_deg=float(pose_deg.get("arm_boom", 0.0)),
            bucket_arm_deg=float(pose_deg.get("bucket_arm", 0.0)),
            swing_yaw_deg=float(pose_deg.get("swing_yaw", 0.0)),
        )
