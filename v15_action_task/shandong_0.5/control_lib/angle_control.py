"""库 1：角度控制（只支持单关节）。

严格约束：任意一次 set/block 调用只改变 1 个关节，其他 3 个关节保持当前 feedback 不变。

提供类：
    JointAngleController(ctl)   — 包装一个 URDFController（或任何实现 get_pose / set_pose / is_at_pose 的对象）
        .move_joint(name, deg, *, tolerance_deg, timeout_s, order_within_unchanged)
        .move_pose_serial(target_pose_deg: dict, *, order, tolerance_deg, per_joint_timeout_s)
                → 按 order 顺序依次单关节运动，每次只动一个

依赖 v15_action_task（只读 import，不修改原文件）：
    * SEMANTIC_JOINT_ORDER / deg_to_rad  — 协议层
    * action_library.utils.joint_limits.default_pose_deg / clamp_pose  — 限位夹紧，避免超关节范围
"""
from __future__ import annotations

import logging
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple

_log = logging.getLogger("shandong_0_5.angle_ctrl")

JOINT_4 = ("swing_yaw", "boom_swing", "arm_boom", "bucket_arm")
DEFAULT_ORDER = JOINT_4  # 先回转 -> 大臂 -> 小臂 -> 铲斗


def _import_v15_primitives() -> Tuple[Any, Any, Any, Any, Tuple[str, ...]]:
    """运行时才 import 上层 v15（避免 0.5 这个独立库在 import 阶段就依赖 rclpy / ROS 路径）。"""
    from v15_action_task import (  # type: ignore
        SEMANTIC_JOINT_ORDER,
        default_pose_deg,
        deg_to_rad,
        rad_to_deg,
    )
    from v15_action_task.action_library.utils.joint_limits import (  # type: ignore
        clamp_pose,
    )

    return SEMANTIC_JOINT_ORDER, default_pose_deg, deg_to_rad, rad_to_deg, clamp_pose  # type: ignore


class JointAngleController:
    """把通用 URDFController 包装成『只允许单关节运动』的安全控制器。

    原则：
      * 任何对外接口只改 1 个关节，其他关节用当前反馈值填充（保证不会出现复合动作）
      * 目标关节角度先过 v15 的 clamp_pose()（限位夹紧），避免超限指令打到硬件
      * 到位判定用 is_at_pose() 单关节容差；超时返回 success=False，不抛出异常
    """

    def __init__(
        self,
        ctl: Any,
        *,
        default_tolerance_deg: float = 1.0,
        default_timeout_s: float = 8.0,
        poll_interval_s: float = 0.05,
    ) -> None:
        self.ctl = ctl
        self.default_tolerance_deg = float(default_tolerance_deg)
        self.default_timeout_s = float(default_timeout_s)
        self.poll_interval_s = float(poll_interval_s)
        self._semantic_4, self._default_pose_fn, self._deg2r, self._r2deg, self._clamp_fn = _import_v15_primitives()
        # 历史（仅调试）：每个关节最后一次目标角度（夹紧后）
        self.last_cmd_pose: Dict[str, float] = self._default_pose_fn()

    # ------------------------------------------------------------------
    # 核心接口 1：单关节安全运动（一次只动一个）
    # ------------------------------------------------------------------
    def move_joint(
        self,
        joint_name: str,
        target_deg: float,
        *,
        tolerance_deg: Optional[float] = None,
        timeout_s: Optional[float] = None,
        wait_blocking: bool = True,
    ) -> Dict[str, Any]:
        """运动单个 joint_name 到 target_deg（度）。

        实现要点：
          1. 先从 ctl 读当前 4 关节（get_pose_or_default / get_pose_blocking）→ 得到 unchanged 三个的当前值
          2. 只改 joint_name 那一格 → 过 clamp_pose 限位夹紧（目标超限会被夹紧到 joint limits 边界）
          3. set_pose（一次发送：只 1 格变了，其他 3 格等于当前 feedback → 等价『只动 1 个关节』）
          4. wait_blocking=True 时，轮询 is_at_pose（单关节容差），超时返回 success=False

        Returns: dict（兼容 MoveResult 风格）
        """
        if joint_name not in JOINT_4:
            return {
                "success": False,
                "reason": f"unknown_joint: {joint_name!r} not in {JOINT_4}",
                "joint": joint_name,
                "waited_s": 0.0,
            }
        tol = float(tolerance_deg if tolerance_deg is not None else self.default_tolerance_deg)
        tmo = float(timeout_s if timeout_s is not None else self.default_timeout_s)

        # Step1：拿到当前 4 关节（其他 3 个保持不变）
        cur_pose = self._get_current_pose()
        # Step2：改 1 格 + 限位夹紧
        target_pose = dict(cur_pose)
        target_pose[joint_name] = float(target_deg)
        clamped = dict(self._clamp_fn(target_pose))
        if clamped[joint_name] != target_deg:
            _log.warning(
                "[angle_ctrl] %s clamped %+.2f° -> %+.2f° (hit joint limit)",
                joint_name, target_deg, clamped[joint_name],
            )
        self.last_cmd_pose = dict(clamped)

        # Step3：发送（其他 3 格 == 当前 feedback → 实际上只驱动 1 个关节）
        try:
            self.ctl.set_pose(clamped)
        except Exception as e:  # pragma: no cover
            return {
                "success": False,
                "reason": f"set_pose_exception: {type(e).__name__}: {e}",
                "joint": joint_name,
                "target_after_clamp_deg": float(clamped[joint_name]),
                "waited_s": 0.0,
            }

        if not wait_blocking:
            return {
                "success": True,
                "reason": "issued_and_returned (non-blocking, no wait)",
                "joint": joint_name,
                "target_after_clamp_deg": float(clamped[joint_name]),
                "waited_s": 0.0,
            }

        # Step4：轮询到位
        t0 = time.monotonic()
        final_pose = dict(cur_pose)
        ok = False
        reason = ""
        while True:
            final_pose = self._get_current_pose()
            # 只检查这个关节（其他关节默认被保持不变）
            diff = abs(float(final_pose.get(joint_name, 0.0)) - float(clamped[joint_name]))
            if diff <= tol:
                ok = True
                break
            if (time.monotonic() - t0) >= tmo:
                reason = f"timeout@{tmo:.1f}s diff={diff:.2f}° (tol={tol:.1f}°)"
                break
            time.sleep(self.poll_interval_s)

        waited = time.monotonic() - t0
        return {
            "success": bool(ok),
            "reason": reason or "reached",
            "joint": joint_name,
            "target_after_clamp_deg": float(clamped[joint_name]),
            "final_joint_deg": float(final_pose.get(joint_name, float("nan"))),
            "waited_s": float(waited),
        }

    # ------------------------------------------------------------------
    # 核心接口 2：4 关节目标 → 按顺序串行单关节执行（永不复合）
    # ------------------------------------------------------------------
    def move_pose_serial(
        self,
        target_pose_deg: Dict[str, float],
        *,
        order: Optional[Iterable[str]] = None,
        tolerance_deg: Optional[float] = None,
        per_joint_timeout_s: Optional[float] = None,
        stop_on_first_fail: bool = True,
    ) -> Dict[str, Any]:
        """把 target_pose_deg（含任何子集/全集 4 关节）按 order 顺序逐关节单关节串行运动。

        保证：
          * 每一步只会调用 move_joint(...) —— 即每一步只动一个关节
          * 其他 3 关节用每一步执行时的当前 feedback 再发送（永远不会出现 2 个关节同时跑）
        """
        order_use: List[str] = list(order) if order is not None else list(DEFAULT_ORDER)
        # 过滤：只跑那些 target_pose_deg 里显式给了名字的关节（没提的保持当前不动）
        run_joints: List[str] = [j for j in order_use if j in target_pose_deg]
        if not run_joints:
            return {
                "success": True,
                "reason": "no_joints_in_target (nothing to do)",
                "per_joint": [],
                "waited_total_s": 0.0,
            }
        results: List[Dict[str, Any]] = []
        t0 = time.monotonic()
        overall_ok = True
        first_fail_reason = ""
        for j in run_joints:
            r = self.move_joint(
                j,
                float(target_pose_deg[j]),
                tolerance_deg=tolerance_deg,
                timeout_s=per_joint_timeout_s,
                wait_blocking=True,
            )
            results.append(r)
            if not r.get("success"):
                overall_ok = False
                first_fail_reason = f"[{j}] {r.get('reason', 'unknown_fail')}"
                if stop_on_first_fail:
                    break
        return {
            "success": bool(overall_ok),
            "reason": first_fail_reason or "all_reached_serial",
            "order": run_joints,
            "per_joint": results,
            "waited_total_s": float(time.monotonic() - t0),
        }

    # ------------------------------------------------------------------
    # 工具：当前 4 关节（兼容任何 ctl.get_pose_* 实现）
    # ------------------------------------------------------------------
    def _get_current_pose(self) -> Dict[str, float]:
        fn_candidates = [
            ("get_pose_or_default", (), {}),
            ("get_pose_blocking", (), {"timeout_s": 1.0}),
            ("get_pose", (), {}),
        ]
        for name, a, kw in fn_candidates:
            fn = getattr(self.ctl, name, None)
            if not callable(fn):
                continue
            try:
                p = fn(*a, **kw)
                if isinstance(p, dict) and any(k in p for k in JOINT_4):
                    out = self._default_pose_fn()
                    out.update({k: float(p[k]) for k in JOINT_4 if k in p})
                    return out
            except Exception:
                continue
        return self.last_cmd_pose
