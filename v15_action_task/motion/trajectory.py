"""
TrajectoryPlanner —— 关节空间双策略轨迹规划器（LERP / 梯形速度）+ planning_ik_solve 封装。

双策略：
- LERP（液压步长模式）：按最大关节步长等分 N 段，所有关节同步；适合仿真+现场液压步长驱动。
- Trapezoidal（梯形加速度限幅）：每关节独立 ramp-up / 匀速 / ramp-down，统一总时长；适合真实阀控速度限幅。

零第三方依赖：stdlib math + dataclass + typing。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


JOINT_4 = ("swing_yaw", "boom_swing", "arm_boom", "bucket_arm")

_STRATEGY_LERP = "lerp"
_STRATEGY_TRAPEZOIDAL = "trapezoidal"
_VALID_STRATEGIES = {_STRATEGY_LERP, _STRATEGY_TRAPEZOIDAL}


@dataclass
class JointTrajectoryPoint:
    """单关节轨迹点（语义与 ROS2 trajectory_msgs/JointTrajectoryPoint 对齐，但简化版）。"""

    time_from_start_s: float
    pose_deg: Dict[str, float]
    velocity_deg_s: Optional[Dict[str, float]] = None


@dataclass
class IKPlanningResult:
    """planning_ik_solve 的统一返回：可达域检查 + IK 逆解 + target 信息。"""

    success: bool
    ik_pose_deg: Optional[Dict[str, float]] = None
    bucket_angle_deg: Optional[float] = None
    target_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    reachability_report: Optional[Any] = None  # ReachabilityReport 或 None
    fail_reason: str = ""


@dataclass
class TrajectoryPlanner:
    """
    双策略轨迹规划器。

    典型用法：
        planner = TrajectoryPlanner.from_config(cfg, workspace_checker=ws, ik=ik)
        res = planner.planning_ik_solve((1.2, 0.0, -0.1), mode="dig")
        if res.success:
            traj = planner.plan_segment(ctl.get_pose(), res.ik_pose_deg, strategy="trapezoidal")
            for tp in traj: ctl.set_pose(tp.pose_deg); time.sleep(tp.dt if has else 0.01)
    """

    # 轨迹默认策略
    default_strategy: str = _STRATEGY_LERP
    # LERP 模式：最大单关节步长（度）
    lerp_step_deg: float = 5.0
    # 最大速度（度/秒）
    max_speed_deg_s: Dict[str, float] = field(default_factory=lambda: {
        "swing_yaw": 15.0, "boom_swing": 20.0, "arm_boom": 30.0, "bucket_arm": 30.0,
    })
    # 最大加速度（度/秒^2）
    accel_deg_s2: Dict[str, float] = field(default_factory=lambda: {
        "swing_yaw": 30.0, "boom_swing": 40.0, "arm_boom": 60.0, "bucket_arm": 60.0,
    })
    # 可选依赖：可达域检查器 + IK（可后续通过 set_* 注入）
    workspace_checker: Optional[Any] = None
    ik: Optional[Any] = None

    # ============================================================
    # 构造
    # ============================================================

    @classmethod
    def from_config(
        cls,
        cfg: Any,
        *,
        workspace_checker: Optional[Any] = None,
        ik: Optional[Any] = None,
    ) -> "TrajectoryPlanner":
        """
        从 V15Config 构造。反射读取 cfg.trajectory 各字段，类型不强绑定，方便扩展。

        读取字段：
            cfg.trajectory.default_strategy / cfg.trajectory.lerp_step_deg
            cfg.trajectory.max_speed_deg_s (dict 4 joints) / cfg.trajectory.accel_deg_s2 (dict 4 joints)
        """
        # 默认值
        strategy = _STRATEGY_LERP
        step = 5.0
        speed = {
            "swing_yaw": 15.0, "boom_swing": 20.0, "arm_boom": 30.0, "bucket_arm": 30.0,
        }
        accel = {
            "swing_yaw": 30.0, "boom_swing": 40.0, "arm_boom": 60.0, "bucket_arm": 60.0,
        }
        # 有 trajectory 配置则覆盖
        tr = getattr(cfg, "trajectory", None)
        if tr is not None:
            s = getattr(tr, "default_strategy", None)
            if isinstance(s, str) and s.lower() in _VALID_STRATEGIES:
                strategy = s.lower()
            step_v = getattr(tr, "lerp_step_deg", None)
            if step_v is not None:
                step = float(step_v)
            # speed
            raw_spd = getattr(tr, "max_speed_deg_s", None)
            if isinstance(raw_spd, dict):
                for j in speed:
                    if j in raw_spd:
                        speed[j] = float(raw_spd[j])
            raw_acc = getattr(tr, "accel_deg_s2", None)
            if isinstance(raw_acc, dict):
                for j in accel:
                    if j in raw_acc:
                        accel[j] = float(raw_acc[j])
        return cls(
            default_strategy=strategy,
            lerp_step_deg=step,
            max_speed_deg_s=speed,
            accel_deg_s2=accel,
            workspace_checker=workspace_checker,
            ik=ik,
        )

    def set_workspace_checker(self, ws: Optional[Any]) -> None:
        self.workspace_checker = ws

    def set_ik(self, ik: Optional[Any]) -> None:
        self.ik = ik

    # ============================================================
    # 核心 1：planning_ik_solve（可达域过滤 + IK 逆解）
    # ============================================================

    def planning_ik_solve(
        self,
        target_xyz: Tuple[float, float, float],
        *,
        mode: str = "auto",
        bucket_guess_deg: float = -45.0,
    ) -> IKPlanningResult:
        """
        顺序：
          1) WorkspaceChecker 存在 → 先过可达域 5 层过滤；失败直接 return fail_reason=reachability + report
          2) IK 存在：
               - 先用 bucket_guess_deg 试 ik.solve_bucket_pose(x,y,z, bucket_abs_angle_deg=bucket_guess_deg)
               - 再试 ik.search_bucket_angle(x,y,z) 自动扫描
             任取第一个有解的
          3) 都没有：return fail，明确说明缺哪个

        Args:
            target_xyz: (x, y, z) 绝对坐标
            mode: "auto" / "dig" / "transit" / "wrist"（透传给 WorkspaceChecker）
            bucket_guess_deg: 铲斗绝对角（度，负值表示铲尖向下）

        Returns:
            IKPlanningResult：success=True 时 ik_pose_deg 保证是 4 关节 dict
        """
        tx, ty, tz = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])
        xyz = (tx, ty, tz)
        report = None
        # 1) 可达域过滤
        if self.workspace_checker is not None:
            report = self.workspace_checker.check_point_reachable(xyz, mode=mode, bucket_guess_deg=bucket_guess_deg)
            if not report.success:
                return IKPlanningResult(
                    success=False,
                    target_xyz=xyz,
                    reachability_report=report,
                    fail_reason="reachability_failed: " + " | ".join(report.reasons),
                )
        # 2) IK 逆解
        ik = self.ik
        if ik is None:
            return IKPlanningResult(
                success=False,
                target_xyz=xyz,
                reachability_report=report,
                fail_reason="ik_unavailable: ik 解算器未注入，请 TrajectoryPlanner.set_ik(ik) 或 from_config(ik=...)",
            )
        sol = None
        used_bucket = None
        # 2a 先试显式 bucket_guess_deg
        solve_bp = getattr(ik, "solve_bucket_pose", None)
        if callable(solve_bp):
            try:
                tmp = solve_bp(tx, ty, tz, bucket_abs_angle_deg=float(bucket_guess_deg))
                if tmp is not None:
                    sol = tmp
                    used_bucket = float(bucket_guess_deg)
            except Exception:
                sol = None
        # 2b 再试 search_bucket_angle 自动扫描
        if sol is None:
            search = getattr(ik, "search_bucket_angle", None)
            if callable(search):
                try:
                    tmp = search(tx, ty, tz)
                    if tmp is not None:
                        sol = tmp
                        bucket_attr = getattr(tmp, "bucket_abs_angle_deg", None)
                        used_bucket = float(bucket_attr) if bucket_attr is not None else None
                except Exception:
                    sol = None
        if sol is None:
            return IKPlanningResult(
                success=False,
                target_xyz=xyz,
                reachability_report=report,
                fail_reason="ik_no_solution: solve_bucket_pose + search_bucket_angle 均无解",
            )
        # as_pose 提取 dict
        as_pose_fn = getattr(sol, "as_pose", None)
        if not callable(as_pose_fn):
            return IKPlanningResult(
                success=False,
                target_xyz=xyz,
                reachability_report=report,
                fail_reason="ik_bad_solution: IKSolution.as_pose 不存在",
            )
        pose = as_pose_fn()
        if not isinstance(pose, dict):
            return IKPlanningResult(
                success=False,
                target_xyz=xyz,
                reachability_report=report,
                fail_reason=f"ik_bad_solution: as_pose 返回 {type(pose).__name__}，预期 dict",
            )
        # 补齐 4 关节键（防止老版本 ik 少键）
        for j in JOINT_4:
            if j not in pose:
                pose[j] = 0.0
            else:
                pose[j] = float(pose[j])
        return IKPlanningResult(
            success=True,
            ik_pose_deg=pose,
            bucket_angle_deg=used_bucket,
            target_xyz=xyz,
            reachability_report=report,
            fail_reason="",
        )

    # ============================================================
    # 核心 2：plan_segment（LERP / Trapezoidal 双策略）
    # ============================================================

    def plan_segment(
        self,
        start_pose_deg: Dict[str, float],
        end_pose_deg: Dict[str, float],
        *,
        strategy: Optional[str] = None,
    ) -> List[JointTrajectoryPoint]:
        """
        在关节空间从 start_pose_deg 规划到 end_pose_deg，返回 waypoint 列表（含起点 t=0 和终点 t=T）。

        Args:
            start_pose_deg: 起始关节角（至少 4 关节键）
            end_pose_deg:   终点关节角（至少 4 关节键）
            strategy:       None → 用 self.default_strategy；"lerp" 或 "trapezoidal"

        Returns:
            list[JointTrajectoryPoint]，waypoints 0..N；至少含 2 个点（start + end）
        """
        if strategy is None:
            strategy = self.default_strategy
        if strategy not in _VALID_STRATEGIES:
            raise ValueError(f"Unknown strategy {strategy!r}; valid: {sorted(_VALID_STRATEGIES)}")
        # 规范化 4 关节
        s = self._normalize_pose(start_pose_deg)
        e = self._normalize_pose(end_pose_deg)
        if strategy == _STRATEGY_LERP:
            return self._plan_lerp(s, e)
        return self._plan_trapezoidal(s, e)

    # ------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------

    @staticmethod
    def _normalize_pose(p: Dict[str, float]) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for j in JOINT_4:
            out[j] = float(p[j]) if j in p and p[j] is not None else 0.0
        return out

    def _plan_lerp(
        self,
        s: Dict[str, float],
        e: Dict[str, float],
    ) -> List[JointTrajectoryPoint]:
        # 1. 计算 4 关节 Δ
        deltas: Dict[str, float] = {j: e[j] - s[j] for j in JOINT_4}
        abs_deltas = [abs(deltas[j]) for j in JOINT_4]
        max_delta = max(abs_deltas) if abs_deltas else 0.0
        step = float(self.lerp_step_deg)
        if step <= 1e-9:
            N = 1
        else:
            N = int(math.ceil(max_delta / step))
            if N < 1:
                N = 1
        # 2. 求每关节时间 t_j = |Δj| / max_speed[j]，T = max
        T = 0.0
        for j in JOINT_4:
            vmax = float(self.max_speed_deg_s.get(j, 30.0))
            if vmax <= 1e-9:
                vmax = 30.0
            tj = abs(deltas[j]) / vmax
            if tj > T:
                T = tj
        if T <= 0:
            T = 1e-3
        # 3. 构造 N+1 个 waypoint（k=0..N）
        result: List[JointTrajectoryPoint] = []
        for k in range(N + 1):
            alpha = 0.0 if N == 0 else (k / N)
            t = T * alpha
            pose_k: Dict[str, float] = {}
            vel_k: Dict[str, float] = {}
            for j in JOINT_4:
                pose_k[j] = s[j] + deltas[j] * alpha
                # LERP 近似速度 = Δj / T
                vel_k[j] = (deltas[j] / T) if T > 0 else 0.0
            result.append(JointTrajectoryPoint(
                time_from_start_s=float(t),
                pose_deg=pose_k,
                velocity_deg_s=vel_k,
            ))
        return result

    def _plan_trapezoidal(
        self,
        s: Dict[str, float],
        e: Dict[str, float],
    ) -> List[JointTrajectoryPoint]:
        deltas = {j: e[j] - s[j] for j in JOINT_4}
        per_joint: Dict[str, Tuple[float, float, float, float]] = {}  # j -> (Tj, t_acc, t_const, sign, vpeak)
        T = 0.0
        for j in JOINT_4:
            d = deltas[j]
            sign = 1.0 if d > 0 else (-1.0 if d < 0 else 0.0)
            abs_d = abs(d)
            vmax = float(self.max_speed_deg_s.get(j, 30.0))
            a = float(self.accel_deg_s2.get(j, 60.0))
            if vmax <= 1e-9:
                vmax = 30.0
            if a <= 1e-9:
                a = 60.0
            if abs_d <= 1e-9:
                per_joint[j] = (0.0, 0.0, 0.0, 0.0, 0.0)
                continue
            t_acc = vmax / a
            s_acc = 0.5 * a * t_acc * t_acc
            if 2 * s_acc >= abs_d:
                # 三角速度剖面
                t_acc_actual = math.sqrt(abs_d / a)
                vpeak = a * t_acc_actual
                Tj = 2 * t_acc_actual
                t_acc_save = t_acc_actual
                t_const_save = 0.0
            else:
                s_const = abs_d - 2 * s_acc
                t_const = s_const / vmax
                Tj = 2 * t_acc + t_const
                t_acc_save = t_acc
                t_const_save = t_const
                vpeak = vmax
            per_joint[j] = (Tj, t_acc_save, t_const_save, sign, vpeak)
            if Tj > T:
                T = Tj
        if T <= 0:
            T = 1e-3
        # 构造 waypoint 数：N = max(ceil(T*50Hz), 10)
        N = int(math.ceil(T * 50.0))
        if N < 10:
            N = 10
        result: List[JointTrajectoryPoint] = []
        for k in range(N + 1):
            t = T * (k / N)
            pose_k: Dict[str, float] = {}
            vel_k: Dict[str, float] = {}
            for j in JOINT_4:
                Tj, t_acc, t_const, sign, vpeak = per_joint[j]
                start = s[j]
                delta_total = e[j] - s[j]
                if sign == 0.0 or Tj <= 0:
                    pose_k[j] = e[j]
                    vel_k[j] = 0.0
                    continue
                # 夹紧 t 到 [0, Tj]
                tc = max(0.0, min(Tj, t))
                abs_d = abs(delta_total)
                # 计算 scaler 位移 s_abs ∈ [0, abs_d]，以及当前速度 v_abs
                if 2 * (0.5 * float(self.accel_deg_s2.get(j, 60.0)) * t_acc ** 2) >= abs_d:
                    # 三角剖面
                    t_half = Tj / 2.0
                    if tc <= t_half:
                        s_abs = 0.5 * float(self.accel_deg_s2.get(j, 60.0)) * tc * tc
                        v_abs = float(self.accel_deg_s2.get(j, 60.0)) * tc
                    else:
                        tf = Tj - tc
                        s_abs = abs_d - 0.5 * float(self.accel_deg_s2.get(j, 60.0)) * tf * tf
                        v_abs = float(self.accel_deg_s2.get(j, 60.0)) * tf
                else:
                    # 梯形
                    if tc <= t_acc:
                        s_abs = 0.5 * float(self.accel_deg_s2.get(j, 60.0)) * tc * tc
                        v_abs = float(self.accel_deg_s2.get(j, 60.0)) * tc
                    elif tc <= t_acc + t_const:
                        s_acc_val = 0.5 * float(self.accel_deg_s2.get(j, 60.0)) * t_acc * t_acc
                        s_abs = s_acc_val + vpeak * (tc - t_acc)
                        v_abs = vpeak
                    else:
                        tf = Tj - tc
                        s_abs = abs_d - 0.5 * float(self.accel_deg_s2.get(j, 60.0)) * tf * tf
                        v_abs = float(self.accel_deg_s2.get(j, 60.0)) * tf
                # 夹紧防浮点溢出
                if s_abs < 0:
                    s_abs = 0.0
                if s_abs > abs_d:
                    s_abs = abs_d
                pose_k[j] = start + sign * s_abs
                vel_k[j] = sign * v_abs
            result.append(JointTrajectoryPoint(
                time_from_start_s=float(t),
                pose_deg=pose_k,
                velocity_deg_s=vel_k,
            ))
        # 最后一个 waypoint 硬对齐到 end_pose（浮点积分误差）
        if result:
            last = result[-1]
            for j in JOINT_4:
                last.pose_deg[j] = e[j]
                if last.velocity_deg_s is not None:
                    last.velocity_deg_s[j] = 0.0
        return result
