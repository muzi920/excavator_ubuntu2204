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
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

try:
    from ..control_core import URDFController
    from ..kinematics import ForwardKinematics, InverseKinematics, IKSolution
except (ImportError, ValueError):
    from control_core import URDFController
    from kinematics import ForwardKinematics, InverseKinematics, IKSolution

try:
    from .trajectory import (
        TrajectoryPlanner,
        JointTrajectoryPoint,
        IKPlanningResult,
    )
    _HAS_TRAJECTORY = True
except Exception:
    _HAS_TRAJECTORY = False
    JointTrajectoryPoint = Any  # type: ignore
    IKPlanningResult = Any  # type: ignore
    TrajectoryPlanner = Any  # type: ignore

try:
    from .workspace import WorkspaceChecker, ReachabilityReport as _Rep
    ReachabilityReport = _Rep
    _HAS_WORKSPACE = True
except Exception:
    _HAS_WORKSPACE = False
    WorkspaceChecker = Any  # type: ignore
    ReachabilityReport = Any  # type: ignore


@dataclass
class MoveResult:
    """一次 move 操作的结果。前 8 字段（success → reason）保持顺序不变（旧 API 零 break）。
    9/10/11 字段（尾部追加）仅在 move_to_point / 后续新接口中填充，旧 move/move_with_bucket 默认为 None。
    """

    success: bool
    reached_pose_deg: Optional[Dict[str, float]]
    final_tip_xyz: Optional[Tuple[float, float, float]]
    ik_solution: Optional[IKSolution]
    target_xyz: Tuple[float, float, float]
    target_bucket_angle_deg: Optional[float]
    waited_s: float
    reason: str = ""
    trajectory: Optional[List[Any]] = None  # List[JointTrajectoryPoint]，9
    ik_result: Optional[Any] = None          # IKPlanningResult，10
    reachability_report: Optional[Any] = None  # ReachabilityReport，11

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
        trajectory_planner: Optional[Any] = None,
        workspace_checker: Optional[Any] = None,
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
        # phase3 新增：轨迹规划器 + 可达域检查器（可 None，move_to_point 中可懒构建）
        self.trajectory_planner = trajectory_planner
        self.workspace_checker = workspace_checker

    # ── phase3 新增：setter ───────────────────────────────────
    def set_trajectory_planner(self, planner: Optional[Any]) -> None:
        self.trajectory_planner = planner

    def set_workspace_checker(self, ws: Optional[Any]) -> None:
        self.workspace_checker = ws

    def _ensure_planner(self) -> Any:
        """懒构建 TrajectoryPlanner（当没有显式注入时）。"""
        if self.trajectory_planner is None and _HAS_TRAJECTORY:
            try:
                self.trajectory_planner = TrajectoryPlanner(
                    workspace_checker=self.workspace_checker,
                    ik=self.ik,
                )
            except Exception:
                self.trajectory_planner = None
        return self.trajectory_planner

    # ── 高层 API: move_to_point（接口1）──────────────────────
    def move_to_point(
        self,
        target_xyz: Tuple[float, float, float],
        *,
        mode: str = "auto",
        bucket_guess_deg: float = -45.0,
        strategy: Optional[str] = None,
        execute: bool = True,
        blocking: bool = True,
        tolerance_deg: Optional[float] = None,
        timeout_s: Optional[float] = None,
    ) -> MoveResult:
        """
        接口1：按一个空间点 (x,y,z) 移动末端到该位置。

        相对 move() 的升级：
          - ✅ 先过可达域 5 层过滤（workspace_checker 注入后自动生效）
          - ✅ 4 关节空间 LERP / 梯形速度 双策略轨迹规划（完整运动轨迹）
          - ✅ 每关节单独 waypoint（单关节运动过程轨迹）
          - ✅ 返回 trajectory / ik_result / reachability_report 三扩展字段

        Args:
            target_xyz:       (x, y, z) 目标绝对位置（米）
            mode:             "auto" | "dig" | "transit" | "wrist"
            bucket_guess_deg: 铲斗绝对角（度，挖掘态默认 -45）
            strategy:         None → 用 TrajectoryPlanner.default_strategy；"lerp" / "trapezoidal"
            execute:          True=按轨迹逐点 set_pose 发布；False=只规划不发布（轨迹仿真）
            blocking:         仅 execute=True 时生效，True=逐点 wait，False=跳点只发终点
            tolerance_deg / timeout_s: 复用 self 默认值，可 override

        Returns:
            MoveResult：尾部 trajectory / ik_result / reachability_report 三字段保证已填充（若有异常则为 None 并写 reason）。
        """
        xyz = (float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2]))
        tol = tolerance_deg if tolerance_deg is not None else self.tol
        tout = timeout_s if timeout_s is not None else self.timeout
        reachability_report: Optional[Any] = None
        ik_res: Optional[Any] = None
        traj: Optional[List[Any]] = None
        if not _HAS_TRAJECTORY:
            return MoveResult(
                success=False, reached_pose_deg=None, final_tip_xyz=None, ik_solution=None,
                target_xyz=xyz, target_bucket_angle_deg=None, waited_s=0.0,
                reason="trajectory module unavailable: motion.trajectory import 失败（请检查语法）",
                trajectory=None, ik_result=None, reachability_report=None,
            )
        # 1) 懒构建 planner
        planner = self._ensure_planner()
        if planner is None:
            return MoveResult(
                success=False, reached_pose_deg=None, final_tip_xyz=None, ik_solution=None,
                target_xyz=xyz, target_bucket_angle_deg=None, waited_s=0.0,
                reason="planner unavailable: TrajectoryPlanner 构建失败",
                trajectory=None, ik_result=None, reachability_report=None,
            )
        # 2) 先可达域 + IK（planning_ik_solve 已经把 reachability 做了）
        ik_res = planner.planning_ik_solve(xyz, mode=mode, bucket_guess_deg=bucket_guess_deg)
        if not ik_res.success:
            reachability_report = getattr(ik_res, "reachability_report", None)
            return MoveResult(
                success=False, reached_pose_deg=None, final_tip_xyz=None, ik_solution=None,
                target_xyz=xyz, target_bucket_angle_deg=getattr(ik_res, "bucket_angle_deg", None), waited_s=0.0,
                reason=getattr(ik_res, "fail_reason", "planning_ik_solve fail"),
                trajectory=None, ik_result=ik_res, reachability_report=reachability_report,
            )
        reachability_report = getattr(ik_res, "reachability_report", None)
        target_pose = dict(ik_res.ik_pose_deg)
        # 3.1 夹紧目标关节角（IK 可能给出超限位的解，controller.set_pose 会自动 clamp，但 wait 时 target 不一致 → 超时）
        clamp_fn = getattr(self.ctl, "_clamp_pose", None)
        if callable(clamp_fn):
            try:
                target_pose = dict(clamp_fn(target_pose))
            except Exception:
                pass
        # 3.2 规划当前 pose → target_pose 的 4 关节空间轨迹
        current_pose = self.ctl.get_pose_or_default()
        try:
            traj = planner.plan_segment(current_pose, target_pose, strategy=strategy)
        except Exception as exc:
            return MoveResult(
                success=False, reached_pose_deg=None, final_tip_xyz=None, ik_solution=None,
                target_xyz=xyz, target_bucket_angle_deg=getattr(ik_res, "bucket_angle_deg", None), waited_s=0.0,
                reason=f"plan_segment 失败: {exc}",
                trajectory=None, ik_result=ik_res, reachability_report=reachability_report,
            )
        if traj is None or len(traj) == 0:
            return MoveResult(
                success=False, reached_pose_deg=None, final_tip_xyz=None, ik_solution=None,
                target_xyz=xyz, target_bucket_angle_deg=getattr(ik_res, "bucket_angle_deg", None), waited_s=0.0,
                reason="plan_segment 返回空轨迹",
                trajectory=traj, ik_result=ik_res, reachability_report=reachability_report,
            )
        # 4) execute：按轨迹点 set_pose（逐点 sleep dt）
        t0 = time.monotonic()
        waited_plan_and_exec = 0.0
        if execute:
            n = len(traj)
            for i in range(n):
                tp = traj[i]
                pose = tp.pose_deg
                published = self.ctl.set_pose(pose)
                if not published:
                    # 某个点发布失败不直接 fail，看最终是否到达
                    pass
                if blocking:
                    # 计算 Δt = t_i - t_{i-1}，i=0 跳过
                    if i > 0:
                        dt = traj[i].time_from_start_s - traj[i-1].time_from_start_s
                        if dt > 0:
                            time.sleep(dt)
            # 终点再阻塞等待到位 + poll
            if blocking:
                ok, waited = _run_wait_loop(
                    self.ctl, target_pose,
                    tolerance_deg=tol,
                    timeout_s=max(tout, 1e-3),
                    poll_interval_s=self.poll_s,
                )
                waited_plan_and_exec = time.monotonic() - t0
                if not ok:
                    reached = self.ctl.get_pose_or_default()
                    try:
                        fk_sol = self.fk.solve(
                            boom_swing_deg=reached.get("boom_swing", 0.0),
                            arm_boom_deg=reached.get("arm_boom", 0.0),
                            bucket_arm_deg=reached.get("bucket_arm", 0.0),
                            swing_yaw_deg=reached.get("swing_yaw", 0.0),
                        )
                        final_tip = fk_sol.bucket_tip_3d
                    except Exception:
                        final_tip = None
                    return MoveResult(
                        success=False,
                        reached_pose_deg=reached,
                        final_tip_xyz=final_tip,
                        ik_solution=getattr(ik_res, "_sol", None),
                        target_xyz=xyz,
                        target_bucket_angle_deg=getattr(ik_res, "bucket_angle_deg", None),
                        waited_s=waited_plan_and_exec,
                        reason=f"终点未到位（容差 {tol}°，{tout}s 超时）",
                        trajectory=traj,
                        ik_result=ik_res,
                        reachability_report=reachability_report,
                    )
        waited_plan_and_exec = time.monotonic() - t0
        # 5) 收集最终信息
        reached = self.ctl.get_pose_or_default()
        try:
            fk_sol = self.fk.solve(
                boom_swing_deg=reached.get("boom_swing", 0.0),
                arm_boom_deg=reached.get("arm_boom", 0.0),
                bucket_arm_deg=reached.get("bucket_arm", 0.0),
                swing_yaw_deg=reached.get("swing_yaw", 0.0),
            )
            final_tip = fk_sol.bucket_tip_3d
        except Exception:
            final_tip = None
        # ik_solution：从 ik_res 拿原始 IKSolution（若有），兼容 MoveResult 老字段
        ik_sol_old = None
        if hasattr(ik_res, "__dict__"):
            for attr_name in ("_sol", "sol", "raw_ik"):
                v = getattr(ik_res, attr_name, None)
                if v is not None:
                    ik_sol_old = v
                    break
        return MoveResult(
            success=True,
            reached_pose_deg=reached,
            final_tip_xyz=final_tip,
            ik_solution=ik_sol_old,
            target_xyz=xyz,
            target_bucket_angle_deg=getattr(ik_res, "bucket_angle_deg", None),
            waited_s=waited_plan_and_exec,
            reason="",
            trajectory=traj,
            ik_result=ik_res,
            reachability_report=reachability_report,
        )

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
