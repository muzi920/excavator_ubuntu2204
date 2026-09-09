"""库 2：目标点控制（Point -> 4 段串行单关节）。

约束与 v15 接口一致（100% 只读 import v15 的 FK/IK/WS/Config，不修改 v15 任何原始文件）：

* 输入：铲尖 (x, y, z) base_link 坐标系
* 内部步骤：
  1. WorkspaceChecker 可达域预检（5 层）。不可达时给出 closest_reachable_point + reasons
  2. InverseKinematics（自动搜索铲斗角）求 4 关节目标 (swing_yaw, boom_swing, arm_boom, bucket_arm)（单位度）
  3. 调用库 1 JointAngleController.move_pose_serial(...) — 串行单关节：swing -> boom -> arm -> bucket
* 最终：永远 **不出现 2 个以上关节同时运动**（满足『单一关节运动，禁止复合动作』硬件特性）

导出类：SingleJointPointMover(from_config_default() 推荐构建)
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

_log = logging.getLogger("shandong_0_5.point_ctrl")

# 允许两种用法（避免相对导入时 __package__ 不一致）：
#   (A) 作为 shandong_0_5.control_lib.point_control 子包导入
#   (B) 从 scripts 里直接 importlib 加载（此时 sys.modules 已注册 shandong_0_5.control_lib.angle_control）
try:
    from shandong_0_5.control_lib.angle_control import (
        JOINT_4, DEFAULT_ORDER, JointAngleController,
    )
except Exception:  # pragma: no cover
    from .angle_control import (  # type: ignore
        JOINT_4, DEFAULT_ORDER, JointAngleController,
    )


@dataclass
class SingleJointMoveResult:
    """库 2 的返回结构（尽量贴近 v15 MoveResult 风格，便于日志查找）。"""
    success: bool
    target_xyz: Tuple[float, float, float]
    mode: str
    # 可达域
    reachable: Optional[bool] = None
    reachability_reasons: List[str] = field(default_factory=list)
    closest_reachable_point: Optional[Tuple[float, float, float]] = None
    # IK
    ik_success: Optional[bool] = None
    ik_reason: str = ""
    ik_target_pose_deg: Optional[Dict[str, float]] = None  # 作为串行运动的 4 关节目标
    bucket_angle_deg_used: Optional[float] = None
    # FK 闭环验证（运动最后 FK 铲尖 vs target_xyz）
    final_tip_xyz: Optional[Tuple[float, float, float]] = None
    final_pose_deg: Optional[Dict[str, float]] = None
    tip_error_3d_m: Optional[float] = None
    # 执行（单关节串行）
    per_joint: List[Dict[str, Any]] = field(default_factory=list)
    order: List[str] = field(default_factory=list)
    waited_total_s: float = 0.0
    reason: str = ""


def _import_v15_point_deps():
    """运行时 import v15_action_task 顶层入口 / config 构造器（与 v15 43TR 完全一致，只读不写）。"""
    from v15_action_task import (  # type: ignore
        ForwardKinematics,
        InverseKinematics,
    )
    from v15_action_task.config import (  # type: ignore
        load_default_config,
    )
    return ForwardKinematics, InverseKinematics, load_default_config


class SingleJointPointMover:
    """目标点 -> 4 段串行单关节的执行器。

    典型用法：
        from shandong_0_5.control_lib import JointAngleController, SingleJointPointMover
        mover = SingleJointPointMover.from_config_default()
        with URDFController(RosV14Adapter()) as ctl:
            jc = JointAngleController(ctl)
            mover.bind_controller(jc)
            r = mover.move_to_point_serial((1.2, 0.0, 0.8))
            print(r.success, r.ik_target_pose_deg, r.tip_error_3d_m)
    """

    def __init__(
        self,
        *,
        ik: Any,
        fk: Any,
        ws: Optional[Any] = None,
        bucket_range_deg: Tuple[float, float] = (-95.0, 45.0),
        bucket_search_N: int = 33,
        serial_order: Optional[List[str]] = None,
        pose_clamp_fn: Optional[Any] = None,
    ) -> None:
        self.ik = ik
        self.fk = fk
        self.ws = ws
        self.bucket_range_deg = (float(bucket_range_deg[0]), float(bucket_range_deg[1]))
        self.bucket_search_N = int(bucket_search_N)
        self.serial_order: List[str] = list(serial_order) if serial_order is not None else list(DEFAULT_ORDER)
        self.joint_ctrl: Optional[JointAngleController] = None
        # 裁剪函数：把 IK 解出的 4 关节角按 v15 joint_limits 夹紧（避免把超限位发去硬件）。
        # 可选来源：V15Config.clamp_pose / URDFController._clamp_pose / 自定义；None 时不做 clamp（不推荐，但保持兼容）
        self._pose_clamp_fn = pose_clamp_fn

    # ------------------------------------------------------------
    # 构造器（推荐）：默认 60FED calibrated 机型 + from_config 链一致构造
    # ------------------------------------------------------------
    @classmethod
    def from_config_default(
        cls,
        *,
        mode_default: str = "dig",
        serial_order: Optional[List[str]] = None,
    ) -> "SingleJointPointMover":
        """使用 v15 的 load_default_config() 构造 FK/IK/WS（和 v15_main 的参数 1:1 一致）。

        * 零改动 v15 代码；所有几何/限位/WS 配置沿用 default_config.yaml
        * mode_default 仅用于 WS 构造后的默认 mode，可被 move_to_point_serial(mode=...) 覆盖
        """
        ForwardKinematics, InverseKinematics, load_default_config = _import_v15_point_deps()
        cfg = load_default_config()
        link_params = cfg.link.to_link_params()  # (ForwardKinematics/InverseKinematics 构造参数要求的就是这个 LinkParams 对象，而不是 Tuple)
        ik = InverseKinematics(link_params)
        fk = ForwardKinematics(link_params)
        # ★ 夹紧用 config 层自带的 limits.clamp_pose（= URDFController._clamp_pose 同源，零改 v15 只调用）
        try:
            clamp_fn = getattr(cfg.limits, "clamp_pose", None)
            if not callable(clamp_fn):
                clamp_fn = None
        except Exception:
            clamp_fn = None
        ws = None
        try:
            # v15 V15Config 没有 build_workspace 方法，用 WorkspaceChecker.from_config(cfg) 反射构造（保持零改 v15）
            from v15_action_task.motion.workspace import WorkspaceChecker  # type: ignore
            ws = WorkspaceChecker.from_config(cfg)
            ws.mode = str(mode_default)
        except Exception as e:
            _log.warning("[point_ctrl] WorkspaceChecker.from_config 失败: %s (禁用可达域预检)", e)
            ws = None
        return cls(ik=ik, fk=fk, ws=ws, serial_order=serial_order, pose_clamp_fn=clamp_fn)

    # ------------------------------------------------------------
    # 绑定控制器
    # ------------------------------------------------------------
    def bind_controller(self, joint_ctrl: JointAngleController) -> None:
        self.joint_ctrl = joint_ctrl

    # ------------------------------------------------------------
    # 对外主接口：单点 (x,y,z) → 4 段串行单关节执行
    # ------------------------------------------------------------
    def move_to_point_serial(
        self,
        xyz: Tuple[float, float, float],
        *,
        mode: Optional[str] = None,
        bucket_guess_deg: Optional[float] = None,
        tolerance_deg: Optional[float] = None,
        per_joint_timeout_s: Optional[float] = None,
        stop_on_first_fail: bool = True,
        # 如果目标不可达：False=直接失败；True=自动用 closest_reachable_point 替换执行
        auto_use_closest_if_unreachable: bool = False,
    ) -> SingleJointMoveResult:
        xyz_used: Tuple[float, float, float] = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        mode_use = str(mode) if mode is not None else ("dig" if self.ws is None else getattr(self.ws, "mode", "dig"))

        r = SingleJointMoveResult(success=False, target_xyz=xyz_used, mode=mode_use)

        # Step 1：可达域预检
        if self.ws is not None:
            try:
                report = self.ws.check_point_reachable(xyz_used, mode=mode_use)
                r.reachable = bool(getattr(report, "success", False))
                r.reachability_reasons = list(getattr(report, "reasons", []) or [])
                cp = getattr(report, "closest_reachable_point", None)
                if cp is not None:
                    r.closest_reachable_point = (float(cp[0]), float(cp[1]), float(cp[2]))
            except Exception as e:
                r.reachable = False
                r.reachability_reasons = [f"workspace_exception:{type(e).__name__}:{e}"]

            if not r.reachable:
                # ★ 先主动调 ws.closest_reachable_point 方法（因为 ReachabilityReport 本身没有该字段）
                if auto_use_closest_if_unreachable and r.closest_reachable_point is None:
                    try:
                        cp_get = getattr(self.ws, "closest_reachable_point", None)
                        if callable(cp_get):
                            got_cp = cp_get(xyz_used)
                            if got_cp is not None:
                                r.closest_reachable_point = (float(got_cp[0]), float(got_cp[1]), float(got_cp[2]))
                    except Exception as e2:
                        _log.warning("[point_ctrl] closest_reachable_point 调用失败: %s", e2)
                if auto_use_closest_if_unreachable and r.closest_reachable_point is not None:
                    xyz_used = r.closest_reachable_point  # 替换成最近可达点
                    _log.info(
                        "[point_ctrl] target unreachable, auto_use_closest: %s -> %s (reasons=%s)",
                        xyz, xyz_used, r.reachability_reasons,
                    )
                    # 自动替换后重新走 WS 预检一遍（确保替换后的点可达；避免 margin 边界问题）
                    try:
                        report = self.ws.check_point_reachable(xyz_used, mode=mode_use)
                        r.reachable = bool(getattr(report, "success", False))
                        r.reachability_reasons = list(getattr(report, "reasons", []) or [])
                    except Exception as e3:
                        _log.warning("[point_ctrl] closest 点预检失败: %s", e3)
                if not r.reachable:
                    r.reason = "unreachable: " + ",".join(r.reachability_reasons or ["?"])
                    return r

        # Step 2：IK 求解（如果给了 bucket_guess_deg 就用指定角度；否则自动搜索）
        # ★ v15 IKSolution dataclass 本身没有 .success 字段（考古 inverse.py:L26-40），
        #   所以 ik_success 判定只能看是否最终拿到 len(pose) == 4 的语义关节字典。
        t0 = time.monotonic()
        ik_target_pose: Optional[Dict[str, float]] = None
        try:
            if bucket_guess_deg is None:
                ik_sol = self.ik.search_bucket_angle(
                    xyz_used[0], xyz_used[1], xyz_used[2],
                    bucket_range_deg=self.bucket_range_deg,
                    num_candidates=int(self.bucket_search_N),
                )
            else:
                ik_sol = self.ik.solve_bucket_pose(
                    xyz_used[0], xyz_used[1], xyz_used[2],
                    bucket_abs_angle_deg=float(bucket_guess_deg),
                )
            # IKSolution 没有 .success/.reason 字段，赋值默认值避免报错
            r.ik_reason = str(getattr(ik_sol, "reason", "") or "")
            pose = getattr(ik_sol, "as_pose", None)
            if callable(pose):
                try:
                    got = pose()
                    if isinstance(got, dict) and len(got) >= 4:
                        ik_target_pose = dict(got)
                        r.ik_reason = r.ik_reason or "as_pose()"
                except Exception:
                    pass
            if ik_target_pose is None:
                raw = getattr(ik_sol, "pose_deg", None) or getattr(ik_sol, "pose", None) or None
                if isinstance(raw, dict) and len(raw) >= 4:
                    ik_target_pose = dict(raw)
                    r.ik_reason = r.ik_reason or "inferred_from_pose_dict"
            # ★ ik_success = 真拿到了 4 关节键（IKSolution 无 success 字段，故只依此判定）
            r.ik_success = isinstance(ik_target_pose, dict) and len(ik_target_pose) >= 4
            r.bucket_angle_deg_used = float(getattr(ik_sol, "bucket_abs_angle_deg", 0.0)) if hasattr(ik_sol, "bucket_abs_angle_deg") else None
        except Exception as e:
            r.ik_success = False
            r.ik_reason = f"ik_exception:{type(e).__name__}:{e}"
            ik_target_pose = None

        # Step 2.5：夹紧 IK 得到的 4 关节角（对齐 joint_limits[-180,-5,0,-95 | 180,55,130,45]）
        # 这一步保证后续 move_pose_serial 不会二次 clamp 超范围而导致 FK 闭环"看起来差了20cm"。
        clamped_happened = False
        if isinstance(ik_target_pose, dict) and callable(self._pose_clamp_fn):
            try:
                clamped = self._pose_clamp_fn({k: float(v) for k, v in ik_target_pose.items()})
                if isinstance(clamped, dict):
                    for k in JOINT_4:
                        a = ik_target_pose.get(k)
                        b = clamped.get(k)
                        if a is None or b is None:
                            continue
                        if abs(float(a) - float(b)) > 1e-4:
                            clamped_happened = True
                            break
                    ik_target_pose = {k: float(clamped[k]) for k in JOINT_4 if k in clamped}
            except Exception as e:
                _log.warning("[point_ctrl] clamp_pose 失败: %s (跳过夹紧)", e)

        r.ik_target_pose_deg = ik_target_pose
        if not ik_target_pose:
            if not r.reason:
                r.reason = f"ik_failed: {r.ik_reason or 'no_pose'}"
            return r

        # Step 3：库 1 JointAngleController → 串行单关节（永远不会出现复合动作！）
        if self.joint_ctrl is None:
            r.reason = "no_joint_controller_bound: 先 SingleJointPointMover.bind_controller(JointAngleController(ctl))"
            return r
        t1 = time.monotonic()
        # 使用 delay_between_joints_s 增加各关节动作间的缓冲时间
        exec_r = self.joint_ctrl.move_pose_serial(
            ik_target_pose,
            order=self.serial_order,
            tolerance_deg=tolerance_deg,
            per_joint_timeout_s=per_joint_timeout_s,
            stop_on_first_fail=stop_on_first_fail,
            delay_between_joints_s=0.5,
        )
        r.order = list(exec_r.get("order", self.serial_order))
        r.per_joint = list(exec_r.get("per_joint", []))
        r.waited_total_s = float(exec_r.get("waited_total_s", 0.0))
        if not exec_r.get("success"):
            if not r.reason:
                r.reason = f"serial_exec_failed: {exec_r.get('reason','unknown')}"
            # 即使失败也记录当前最终 FK
        t2 = time.monotonic()
        # Step 4：FK 闭环（运动后铲尖位置 - target 误差）
        try:
            final_pose = self.joint_ctrl._get_current_pose()
            r.final_pose_deg = dict(final_pose)
            fk_sol = self.fk.solve(
                boom_swing_deg=float(final_pose.get("boom_swing", 0.0)),
                arm_boom_deg=float(final_pose.get("arm_boom", 0.0)),
                bucket_arm_deg=float(final_pose.get("bucket_arm", 0.0)),
                swing_yaw_deg=float(final_pose.get("swing_yaw", 0.0)),
            )
            tip3 = getattr(fk_sol, "bucket_tip_3d", None)
            if tip3 is not None:
                r.final_tip_xyz = (float(tip3[0]), float(tip3[1]), float(tip3[2]))
                r.tip_error_3d_m = float(
                    sum((r.final_tip_xyz[i] - xyz_used[i]) ** 2 for i in range(3)) ** 0.5
                )
        except Exception as e:
            _log.warning("[point_ctrl] post-fk 闭环失败: %s", e)

        # 最终 success：执行 OK 且 FK 误差 < 5cm（或未 FK 成功时仅依赖执行 ok）
        exec_ok = bool(exec_r.get("success"))
        if exec_ok:
            if r.tip_error_3d_m is None:
                r.success = True
                if not r.reason:
                    r.reason = "exec_ok_no_fk_closed_loop"
            else:
                # 5cm 容差（与 v15 TR6.6 一致）
                if r.tip_error_3d_m <= 0.05:
                    r.success = True
                    if not r.reason:
                        r.reason = f"all_reached closed_loop_tip_err={r.tip_error_3d_m*1000:.1f}mm"
                else:
                    if not r.reason:
                        tag = "clamped" if clamped_happened else "exec"
                        r.reason = f"{tag}_ok_but_closed_loop_tip_err={r.tip_error_3d_m*1000:.1f}mm>50mm"
        else:
            if clamped_happened and not r.reason:
                r.reason = "serial_exec_failed after ik_joint_limits_clamp_applied"

        _log.info(
            "[point_ctrl] move_to_point_serial done ik+prep=%.3fs exec=%.3fs ok=%s clamped=%s ik_pose=%s tip_err=%s",
            t1 - t0, t2 - t1, r.success, clamped_happened,
            {k: round(float(ik_target_pose[k]), 1) for k in JOINT_4 if k in ik_target_pose},
            (f"{r.tip_error_3d_m*1000:.1f}mm" if r.tip_error_3d_m is not None else "n/a"),
        )
        return r
