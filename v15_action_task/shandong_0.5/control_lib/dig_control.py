import logging
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

_log = logging.getLogger("shandong_0_5.dig_ctrl")

try:
    from shandong_0_5.control_lib.angle_control import JointAngleController
except Exception:
    from .angle_control import JointAngleController


def _ensure_v15_path_injected() -> None:
    """
    Try injecting parent shandong_0_5/v15_action_task paths into sys.path.
    v15_action_task is at:  <workspace>/src/shandong/v15_action_task/
    dig_control is at:     <workspace>/src/shandong/v15_action_task/shandong_0.5/control_lib/
    """
    _THIS_DIR = os.path.dirname(os.path.abspath(__file__))       # control_lib/
    _SHANDONG_0_5_DIR = os.path.dirname(_THIS_DIR)              # shandong_0.5/
    _V15_ACTION_DIR = os.path.dirname(_SHANDONG_0_5_DIR)        # v15_action_task/
    _SRC_SHANDONG_DIR = os.path.dirname(_V15_ACTION_DIR)        # src/shandong/
    for _p in (_SRC_SHANDONG_DIR, _V15_ACTION_DIR, _SHANDONG_0_5_DIR):
        if _p not in sys.path:
            sys.path.insert(0, _p)


def _import_v15_deps():
    _ensure_v15_path_injected()
    from v15_action_task.config import load_default_config
    from v15_action_task import InverseKinematics
    return load_default_config, InverseKinematics

class DigPhase1Controller:
    def __init__(self, joint_ctrl: JointAngleController):
        self.joint_ctrl = joint_ctrl
        load_default_config, InverseKinematics = _import_v15_deps()
        self.cfg = load_default_config()
        self.ik = InverseKinematics(self.cfg.link.to_link_params())
        
        # 提取限位用于 Clamp
        self.limits = {}
        if hasattr(self.cfg.limits, 'joint_limits'):
            for joint, limit in self.cfg.limits.joint_limits.items():
                self.limits[joint] = (limit.min_deg, limit.max_deg)
        else:
            # Fallback based on default_config.yaml if attribute structure differs
            # ★ 用户标定：大臂 0°=最高点（完全抬起），48°=最低点（压到地面）
            self.limits = {
                "swing_yaw": (-180.0, 180.0),
                "boom_swing": (0.0, 48.0),
                "arm_boom": (0.0, 130.0),
                "bucket_arm": (-95.0, 45.0)
            }
            
    # ------------------------------------------------------------------
    # 基础辅助：小臂 / 铲斗 百分比换算（0%=完全打开，100%=完全关闭）
    # 限位参考：
    #   bucket_arm range: [-95, +20]
    #     开斗 0% = bucket = -95°（完全打开卸料）
    #     收斗 75% ≈ bucket = -20° （挖深时半收斗）
    #     收斗 100% = bucket = +20° （抱紧满斗）
    #   arm_boom range: [0, +130]（用户物理标定，严禁修改）
    #     外推 0% = arm = 0°（完全外推伸直）
    #     外推 45% ≈ arm = 0 + 45% * 130 ≈ 58.5°（中间半收姿态）
    #     外推 100% = arm = +130°（完全收回抱紧）
    # ------------------------------------------------------------------
    def _bucket_pct(self, pct: float) -> float:
        mn, mx = self.limits.get("bucket_arm", (-95.0, 20.0))
        pct = max(0.0, min(100.0, float(pct)))
        return mn + (mx - mn) * (pct / 100.0)

    def _arm_pct(self, pct: float) -> float:
        mn, mx = self.limits.get("arm_boom", (0.0, 130.0))
        pct = max(0.0, min(100.0, float(pct)))
        return mn + (mx - mn) * (pct / 100.0)

    # ------------------------------------------------------------------
    # 基础工具：按 self.limits 钳制单关节角度
    #   用法: self._clamp("arm_boom", 999.0) → 130.0
    # ------------------------------------------------------------------
    def _clamp(self, joint: str, value_deg: float) -> float:
        lim = self.limits.get(str(joint))
        if lim is None or len(lim) != 2:
            return float(value_deg)
        mn, mx = float(lim[0]), float(lim[1])
        if mn > mx:
            mn, mx = mx, mn
        v = float(value_deg)
        if v < mn:
            return mn
        if v > mx:
            return mx
        return v

    # ------------------------------------------------------------------
    # 新增基础功能：小臂垂直地面控制（物理闭环，基于 V4 倾角传感器）
    # 关键约束（用户明确要求）：
    #   · 控制限位不要改，严格用 arm_boom ∈ [0°, 130°] 用户物理标定
    #   · 「垂直」是物理意义上的对地垂直（小臂杆 abs pitch ≈ 90° 向下），
    #     不是硬编码控制端 arm_boom = 90°
    #
    # 语义回顾：
    #   current_sensor_data["大臂"]["pitch"] = 大臂绝对对地倾角（重力参考，向下为正）
    #   current_sensor_data["小臂"]["pitch"] = 小臂绝对对地倾角（重力参考，向下为正）
    #   控制接口 arm_boom = 小臂.pitch − 大臂.pitch（小臂相对大臂的夹角）
    #
    # 算法：闭环调整 arm_boom（步长 ±5°/步，收敛阈值 ±5° 内），
    #       直到 小臂.abs_pitch ∈ [85°, 95°]（对地接近垂直）。
    # ------------------------------------------------------------------
    def set_arm_perpendicular(self, timeout_s: float = 30.0) -> bool:
        """
        单独控制小臂到「对地物理垂直」姿态（小臂绝对 pitch ≈ 90°）。
        采用传感器闭环：每步进 ±5° 并读回小臂绝对倾角，直到进入 ±5° 容差。
        """
        TARGET_ABS = 90.0
        TOLERANCE_DEG = 5.0
        STEP_DEG = 5.0
        MAX_ITERS = 20
        POLL_INTERVAL_S = 1.2
        t0 = time.time()

        boom_abs_cur, arm_abs_cur, arm_ctrl_cur = 0.0, 0.0, 0.0

        # 1. 读一次当前传感器 + 控制端角度（可选，拿不到就走兜底步数）
        try:
            sd = self.joint_ctrl._get_current_sensor_data()
            boom_abs_cur = float(sd["大臂"]["pitch"])
            arm_abs_cur  = float(sd["小臂"]["pitch"])
        except Exception as _e1:
            _log.warning(f"[set_arm_perp] 读倾角传感器失败，降级为几何兜底: {_e1}")
            boom_abs_cur, arm_abs_cur = None, None
        try:
            pose = self.joint_ctrl._get_current_pose()
            arm_ctrl_cur = float(pose.get("arm_boom", 65.0))
            if boom_abs_cur is None:
                boom_abs_cur = float(pose.get("boom_swing", 0.0))
        except Exception as _e2:
            _log.warning(f"[set_arm_perp] 读当前 pose 失败: {_e2}")

        _log.info(
            f"[dig_ctrl] 执行「小臂垂直地面（物理闭环）」:\n"
            f"            当前 大臂绝对 pitch = {boom_abs_cur}°    小臂绝对 pitch = {arm_abs_cur}°\n"
            f"            当前 arm_boom（相对夹角控制量） = {arm_ctrl_cur:.1f}°\n"
            f"            目标：小臂.abs_pitch ∈ [{TARGET_ABS - TOLERANCE_DEG:.0f}, {TARGET_ABS + TOLERANCE_DEG:.0f}]° 容差内"
        )

        # 2. 如果拿到 abs_pitch，按符号决定方向；否则基于 65° 中位先向 90° 绝对推
        if arm_abs_cur is not None:
            err = arm_abs_cur - TARGET_ABS  # +=>小臂太向下(需要小臂外展/减小 arm_boom)，-=>太平(需要收小臂/增大 arm_boom)
        else:
            err = (arm_ctrl_cur - 65.0)  # 兜底假设 65° 附近，尽量外展先

        arm_ctrl_target = arm_ctrl_cur
        it = 0
        last_abs = arm_abs_cur

        while it < MAX_ITERS and (time.time() - t0) < timeout_s:
            it += 1

            # 2.1 根据误差决定步长方向（每步 5°）
            if arm_abs_cur is not None:
                err = arm_abs_cur - TARGET_ABS
                if abs(err) <= TOLERANCE_DEG:
                    _log.info(
                        f"[set_arm_perp] 第 {it} 步收敛：小臂绝对 pitch={arm_abs_cur:.1f}° "
                        f"(误差 {err:+.1f}° ≤ {TOLERANCE_DEG}° 容差)，垂直成功 ✅"
                    )
                    return True
                # 误差为正（太向下）→ 外展小臂，arm_boom 减 STEP；误差为负（太平）→ 收小臂，arm_boom 加 STEP
                delta = -STEP_DEG if err > 0 else +STEP_DEG
            else:
                # 兜底模式：向 90° 几何估计靠拢，每次走一步直到超时或触限位
                err_guess = (arm_ctrl_target - 90.0)
                if abs(err_guess) <= TOLERANCE_DEG:
                    _log.info(f"[set_arm_perp] 兜底模式达到 90° 附近 arm_boom={arm_ctrl_target:.1f}°")
                    return True
                delta = -STEP_DEG if err_guess > 0 else +STEP_DEG

            arm_ctrl_next_raw = arm_ctrl_target + delta
            arm_ctrl_next = self._clamp("arm_boom", arm_ctrl_next_raw)

            # 触到限位且误差仍同号 → 不可能再调，直接返回（并 warn）
            if arm_ctrl_next == arm_ctrl_target:
                if arm_abs_cur is not None and abs(arm_abs_cur - TARGET_ABS) > TOLERANCE_DEG:
                    _log.warning(
                        f"[set_arm_perp] 已触臂限位 arm_boom={arm_ctrl_next:.1f}°，"
                        f"但小臂绝对 pitch={arm_abs_cur:.1f}° 仍未达 90°±{TOLERANCE_DEG}°。\n"
                        f"            （可能是大臂过高/过低导致几何上垂直落在物理 [0,130]° 范围外，请调整 boom 后重试）"
                    )
                else:
                    _log.info(f"[set_arm_perp] 限位收敛到 arm_boom={arm_ctrl_next:.1f}°")
                return True

            _log.info(
                f"[set_arm_perp] 第 {it}/{MAX_ITERS} 步: err={err:+.1f}°, "
                f"arm_boom {arm_ctrl_target:.1f} → {arm_ctrl_next:.1f}° (Δ={delta:+.0f}°)"
            )
            arm_ctrl_target = arm_ctrl_next

            # 2.2 发单关节 move 指令
            res = self.joint_ctrl.move_joint(
                "arm_boom", arm_ctrl_target,
                tolerance_deg=5.0, timeout_s=min(15.0, timeout_s - (time.time() - t0)),
                wait_blocking=True
            )
            if not res.get("success"):
                _log.error(f"[set_arm_perp] 第 {it} 步 move 失败: {res.get('reason')}")
                return False

            # 2.3 等待姿态稳定 + 刷新传感器读数
            time.sleep(POLL_INTERVAL_S)
            last_abs = arm_abs_cur
            try:
                sd = self.joint_ctrl._get_current_sensor_data()
                arm_abs_cur = float(sd["小臂"]["pitch"])
            except Exception:
                pass
            try:
                pose = self.joint_ctrl._get_current_pose()
                arm_ctrl_cur = float(pose.get("arm_boom", arm_ctrl_target))
            except Exception:
                pass

        # 循环结束：看最后一次是否达标
        if arm_abs_cur is not None and abs(arm_abs_cur - TARGET_ABS) <= TOLERANCE_DEG:
            _log.info(f"[set_arm_perp] 最终收敛: arm_abs={arm_abs_cur:.1f}° arm_boom={arm_ctrl_target:.1f}° ✅")
            return True
        _log.warning(
            f"[set_arm_perp] 超时/步数耗尽: 末 arm_abs={arm_abs_cur}°, arm_boom={arm_ctrl_target:.1f}°, "
            f"仍未进入 90±{TOLERANCE_DEG}°，已尽力停止。"
        )
        return False

    # ------------------------------------------------------------------
    # 内部通用：按 (joint, target, desc) 序列严格单关节串行执行
    # ------------------------------------------------------------------
    def _run_sequence(self, sequence: List[Tuple[str, float, str]]) -> bool:
        for i, (joint, target_deg, desc) in enumerate(sequence):
            # 打印当前步骤信息
            _log.info(f"[dig_ctrl] >>> 开始执行步骤 {i+1}/{len(sequence)}: {desc}")
            _log.info(f"[dig_ctrl]     目标: [{joint}] = {target_deg:.1f}°")

            # 打印下一步动作预告（如果有）
            if i + 1 < len(sequence):
                next_joint, next_target, next_desc = sequence[i + 1]
                _log.info(f"[dig_ctrl]     下一步预告: [{next_joint}] = {next_target:.1f}° ({next_desc})")
            else:
                _log.info(f"[dig_ctrl]     下一步预告: 序列结束")

            # 执行动作
            timeout = 30.0 if joint == "swing_yaw" else 25.0
            res = self.joint_ctrl.move_joint(joint, target_deg, tolerance_deg=5.0, timeout_s=timeout, wait_blocking=True)

            if not res.get("success"):
                _log.error(f"[dig_ctrl] <<< 步骤 {i+1} 执行失败 [{joint}={target_deg:.1f}°]: {res.get('reason')}")
                return False
            else:
                _log.info(f"[dig_ctrl] <<< 步骤 {i+1} 执行成功 [{joint}={target_deg:.1f}°]")

            time.sleep(0.5)
        return True

    def plan_and_execute_dig(self, xyz: Tuple[float, float, float]) -> bool:
        """
        阶段 1：按用户指定的 7 步挖掘剧本执行
          步骤 1：上抬大臂到用户预设的 1.2m 安全高度
          步骤 2：铲斗末端运动到输入点 (x,y) 附近，高度 ≥ 1m（安全预位）
          步骤 3：落大臂 → 铲斗末端下探到用户指定 (x,y,z)
          步骤 4：收铲斗 75%（半收斗切入料堆）
          步骤 5：收小臂 → 垂直于地平面（保持料不撒）
          步骤 6：收铲斗 100%（满斗抱紧）
        """
        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])

        _log.info(
            f"[dig_ctrl] ===== 阶段 1（挖掘） 7 步剧本开始 =====\n"
            f"            用户挖掘点 (x,y,z) = ({x:.3f}, {y:.3f}, {z:.3f}) m\n"
            f"            越障安全高度 = 1.2m（步骤 1）\n"
            f"            预位高度 min = 1.0m（步骤 2）"
        )

        # ---------------- 1. 可达域预检 ----------------
        try:
            # from v15_action_task.workspace_check import WorkspaceChecker
            from v15_action_task.motion.workspace import WorkspaceChecker 
            _ws = WorkspaceChecker.from_config(self.cfg)
            ok_ws, reason = _ws.check_point_reachable(x, y, z, mode="dig")
            if not ok_ws:
                _log.error(f"[dig_ctrl] 阶段 1 挖掘点不可达: {reason}")
                return False
        except Exception as _e:
            _log.warning(f"[dig_ctrl] WorkspaceChecker 导入失败，跳过预检: {_e}")

        # ---------------- 2. 两个关键姿态 IK（步骤 2 预位 + 步骤 3 目标点） ----------------
        #   - 预位姿态：z_approach = max(z, 0.5)
        #   - 目标姿态：就是用户的 (x, y, z)
        z_approach = max(z, 0.5)

        sol_target = self.ik.search_bucket_angle(x, y, z,
                                                 bucket_range_deg=(-70.0, -20.0), num_candidates=17)
        if not sol_target or not hasattr(sol_target, "as_pose") or not sol_target.as_pose():
            _log.error(f"[dig_ctrl] 挖掘目标点 IK 无解: ({x:.3f},{y:.3f},{z:.3f})")
            return False
        pose_target = sol_target.as_pose()

        sol_approach = self.ik.search_bucket_angle(x, y, z_approach,
                                                   bucket_range_deg=(-70.0, -20.0), num_candidates=17)
        if sol_approach and hasattr(sol_approach, "as_pose") and sol_approach.as_pose():
            pose_approach = sol_approach.as_pose()
            # 强制步骤 2 大臂在高姿态（≤20°），避免下探撞地
            if float(pose_approach.get("boom_swing", 48.0)) > 20.0:
                pose_approach["boom_swing"] = self._clamp("boom_swing", 15.0)
        else:
            # 兜底：boom 抬到 15°（默认高姿态），arm 沿用目标的
            _log.warning(f"[dig_ctrl] 预位高度 {z_approach:.2f}m IK 失败，使用高姿态兜底")
            pose_approach = {
                "swing_yaw": self._clamp("swing_yaw", pose_target["swing_yaw"]),
                "boom_swing": self._clamp("boom_swing", 15.0),
                "arm_boom":   self._clamp("arm_boom",   pose_target["arm_boom"]),
                "bucket_arm": self._clamp("bucket_arm", -80.0),  # 半开斗切入
            }

        # 限位钳制
        for p in (pose_target, pose_approach):
            for k, v in list(p.items()):
                if k in ("swing_yaw", "boom_swing", "arm_boom", "bucket_arm"):
                    p[k] = self._clamp(k, float(v))

        # ---------------- 3. 收铲斗百分比角度（步骤 4 / 步骤 6） ----------------
        scoop_75 = self._clamp("bucket_arm", -25.0)   # 收铲斗 75% → 固定 -20°
        scoop_100 = self._clamp("bucket_arm", 5.0)    # 收铲斗 100% → 固定 5°
        dig_entry_open = self._clamp("bucket_arm", -80.0)  # 预位时半开斗切入

        # ---------------- 4. 组装 7 步序列（严格单关节串行） ----------------
        sequence = [
            # 步骤 1：上抬大臂到 1.2m 安全高度（对应 boom_swing≈用户预设高姿态，这里 boom_swing=12° ≈ z≈1.2m）
            ("boom_swing", self._clamp("boom_swing", 12.0),
             "步骤1/7 上抬大臂（预设越障高 1.2m → boom_swing≈12° 高位）"),

            # 步骤 2：铲斗到预位 (x,y) 平面，高度 z≥1m
            ("swing_yaw",  pose_approach["swing_yaw"],  "步骤2/7 预位(回转 → 目标 x,y 平面方向)"),
            ("arm_boom",   pose_approach["arm_boom"],   "步骤2/7 预位(小臂 → z≥1m 高度姿态)"),
            ("bucket_arm", dig_entry_open,              "步骤2/7 预位(铲斗 → 半开斗切入姿态)"),
            # ("boom_swing", pose_approach["boom_swing"], "步骤2/7 预位(大臂 → z≥1m 安全高度)"),

            # 步骤 3：落大臂到 (x,y,z) 目标点（用户最终挖掘深度）
            # ("arm_boom",   pose_target["arm_boom"],     "步骤3/7 下探(小臂调整到挖掘深度姿态)"),
            ("arm_boom",   self._clamp("arm_boom", pose_target["arm_boom"] - 10.0),  "步骤3/7 下探(小臂外推补偿)"),
            ("boom_swing", pose_target["boom_swing"],   "步骤3/7 下探(大臂落至目标 x,y,z 点)"),
            # ("bucket_arm", pose_target["bucket_arm"],   "步骤3/7 下探(铲斗切入料堆)"),

            # 步骤 4：收铲斗 75%
            ("bucket_arm", scoop_75,                    "步骤4/7 收铲斗 75%（半收斗切入料堆）"),
        ]

        # ---------------- 5. 先执行步骤 1-4 ----------------
        if not self._run_sequence(sequence):
            _log.error("[dig_ctrl] 阶段 1 挖掘剧本 步骤 1-4 执行失败，中止！")
            return False

        # ---------------- 6. 步骤 5：收小臂到 65° ----------------
        _log.info("[dig_ctrl] 步骤5/7 收小臂 → 65°")
        if not self._run_sequence([("arm_boom", self._clamp("arm_boom", 65.0), "步骤5/7 收小臂到65°")]):
            _log.error("[dig_ctrl] 步骤 5（收小臂到65°）失败，中止后续！")
            return False

        # ---------------- 7. 步骤 6：收铲斗 100% ----------------
        sequence_final = [
            ("bucket_arm", scoop_100, "步骤6/7 收铲斗 100%（满斗抱紧）"),
        ]
        if not self._run_sequence(sequence_final):
            _log.error("[dig_ctrl] 步骤 6-7（满斗抱紧）失败！")
            return False

        _log.info("[dig_ctrl] ===== 阶段 1 挖掘 7 步剧本 全部完成！满斗抱紧 ✅ =====")
        return True

    def plan_and_execute_transit(self, target_yaw_deg: float) -> bool:
        """
        阶段 2（运输回转）：用户剧本新要求
          步骤 1：抬大臂（boom→0° 最高点，用户标定 0=最高 48=最低）
          步骤 2：收小臂 → 垂直于地平面（新增，稳料不撒）
          步骤 3：回转 swing_yaw → 目标卸料角度
        """
        target_yaw = self._clamp("swing_yaw", float(target_yaw_deg))
        _log.info(
            f"[dig_ctrl] ===== 阶段 2（运输回转） 3 步剧本开始 =====\n"
            f"            目标回转 swing_yaw = {target_yaw:.1f}°\n"
            f"            新剧本：抬大臂 → 小臂垂直地面 → 回转"
        )

        lift_boom = self._clamp("boom_swing", 0.0)  # 0 = 最高点

        sequence = [
            # 步骤 1：抬大臂（用户剧本的第一步）
            ("boom_swing", lift_boom, "步骤1/3 抬大臂到最高点 0°（用户标定 0=最高）"),
        ]

        # 步骤 2 → 先执行完抬大臂，再调用垂直（依赖 boom=0° 来计算正确的 arm 目标）
        if not self._run_sequence(sequence):
            _log.error("[dig_ctrl] 阶段 2 步骤 1（抬大臂）失败！")
            return False

        _log.info("[dig_ctrl] 步骤2/3 收小臂 → 85°（稳料姿态）")
        if not self._run_sequence([("arm_boom", self._clamp("arm_boom", 85.0), "步骤2/3 收小臂到85°")]):
            _log.error("[dig_ctrl] 阶段 2 步骤 2（收小臂到85°）失败！")
            return False

        sequence_tail = [
            # 步骤 3：回转到卸料角度（整机旋转）
            ("swing_yaw", target_yaw, f"步骤3/3 回转 swing_yaw → {target_yaw:.1f}°（卸料方向）"),
        ]
        if not self._run_sequence(sequence_tail):
            _log.error("[dig_ctrl] 阶段 2 步骤 3（回转）失败！")
            return False

        _log.info(f"[dig_ctrl] ===== 阶段 2 运输回转 3 步完成！最终 swing_yaw = {target_yaw:.1f}° 小臂垂直 ✅ =====")
        return True

    def plan_and_execute_dump(self, dump_xyz: Tuple[float, float, float]) -> Tuple[bool, Optional[Dict]]:
        """
        阶段 3（卸料点）：用户新剧本
          前置：boom 保持高姿态（boom_swing 最小/最高可达解）
          步骤 A：外推小臂 45%（从垂直地面的内收姿态推出到卸料位置）
          步骤 B：外推铲斗 100%（完全打开卸料 → bucket=-95° 开斗）
          步骤 C：回正（下一目标点准备位）
        返回：（success, best_pose or None）—— 组合流程要用到 best_pose 判断
        """
        x, y, z_user = float(dump_xyz[0]), float(dump_xyz[1]), float(dump_xyz[2])
        _log.info(
            f"[dig_ctrl] ===== 阶段 3（卸料点） 新剧本开始 =====\n"
            f"            卸料点 (x,y,z) = ({x:.3f}, {y:.3f}, {z_user:.3f}) m\n"
            f"            剧本：boom 高位 → 外推小臂 45% → 外推铲斗 100%（完全开斗）"
        )

        # 1) 卸料点 z ≥ 0.3m 兜底
        z_use = max(z_user, 0.3)

        # 2) 可达域 + IK 选 boom 最小（最高）解
        try:
            # from v15_action_task.workspace_check import WorkspaceChecker
            from v15_action_task.motion.workspace import WorkspaceChecker 
            _ws = WorkspaceChecker.from_config(self.cfg)
            r, reason = _ws.check_point_reachable(x, y, z_use)
            if not r:
                _log.error(f"[dig_ctrl] 卸料点不可达: {reason}")
                return False, None
        except Exception as _e:
            _log.warning(f"[dig_ctrl] 预检跳过: {_e}")

        bucket_segments = [(-90, -60), (-65, -35), (-40, -10), (-15, 20)]
        candidates: List[Tuple[float, float, Dict[str, float]]] = []
        for b_min, b_max in bucket_segments:
            sol = self.ik.search_bucket_angle(x, y, z_use, bucket_range_deg=(b_min, b_max), num_candidates=10)
            if not sol or not hasattr(sol, "as_pose"):
                continue
            pose = sol.as_pose()
            if not pose:
                continue
            boom_c = self._clamp("boom_swing", float(pose.get("boom_swing", 999)))
            arm_c  = self._clamp("arm_boom",   float(pose.get("arm_boom",  0)))
            buck_c = self._clamp("bucket_arm", float(pose.get("bucket_arm", 0)))
            sw_c   = self._clamp("swing_yaw",  float(pose.get("swing_yaw",  0)))
            # FK 校验误差
            try:
                from v15_action_task.kinematics.forward import ForwardKinematics
                _fk = ForwardKinematics(self.cfg)
                tx, ty, tz = _fk.compute_tip_position(sw_c, boom_c, arm_c, buck_c)
                err = math.hypot(tx - x, ty - y, tz - z_use)
            except Exception:
                err = 0.0
            candidates.append((boom_c + err * 100.0, err,
                               {"swing_yaw": sw_c, "boom_swing": boom_c, "arm_boom": arm_c, "bucket_arm": buck_c}))

        if not candidates:
            _log.error("[dig_ctrl] 阶段 3 卸料点 IK 无解！")
            return False, None
        candidates.sort(key=lambda t: t[0])
        best_score, best_err, best_pose = candidates[0]
        _log.info(
            f"[dig_ctrl] 阶段 3 最佳卸料姿态（boom 最高 + 误差最小）：\n"
            f"            boom_swing = {best_pose['boom_swing']:.1f}°\n"
            f"            arm_boom   = {best_pose['arm_boom']:.1f}°\n"
            f"            bucket_arm = {best_pose['bucket_arm']:.1f}°\n"
            f"            swing_yaw  = {best_pose['swing_yaw']:.1f}°\n"
            f"            末端误差 ≈ {best_err*100:.1f} cm"
        )

        # 3) 百分比角度计算：外推小臂 45%（从垂直内收的负角→往 0 方向推出 = 增大百分比）、开斗 100%（0%开斗=-95°即 bucket_min）
        push_arm_45   = self._clamp("arm_boom", 45.0)
        bucket_open_100 = self._clamp("bucket_arm", -90.0)   # 0% = 完全开斗卸料

        _log.info(
            f"[dig_ctrl] 百分比目标：\n"
            f"            外推小臂 45% → arm_boom = {push_arm_45:.1f}°\n"
            f"            外推铲斗 100%（完全开斗）→ bucket_arm = {bucket_open_100:.1f}°"
        )

        # 4) 动作序列（用户剧本）：boom→swing→定位→小臂45%→开斗100%
        sequence = [
            ("boom_swing", best_pose["boom_swing"], "阶段3/卸料 抬大臂到最高可达姿态 boom_swing"),
            ("swing_yaw",  best_pose["swing_yaw"],  "阶段3/卸料 回转 swing_yaw 对齐卸料方向"),
            ("arm_boom",   best_pose["arm_boom"],   "阶段3/卸料 小臂伸到卸料点姿态"),
            ("bucket_arm", best_pose["bucket_arm"], "阶段3/卸料 铲斗预位（暂不开斗）"),
            # 新增剧本动作：
            ("arm_boom",   push_arm_45,             "阶段3/卸料 步骤A：外推小臂 45%（卸料姿态推出）"),
            ("bucket_arm", bucket_open_100,         "阶段3/卸料 步骤B：外推铲斗 100%（完全开斗 → 倒料）"),
        ]

        if not self._run_sequence(sequence):
            _log.error("[dig_ctrl] 阶段 3 卸料剧本 A/B 步骤失败！")
            return False, best_pose

        # 5) 步骤 C：回正（下一目标点准备位）—— 在 dump 末尾就做
        _log.info("[dig_ctrl] 步骤C：回正（下一目标点准备位）")
        reset_seq = [
            ("boom_swing", self._clamp("boom_swing", 0.0),   "回正 1/4 boom→0°（最高越障）"),
            ("bucket_arm", self._clamp("bucket_arm", -50.0),           "回正 2/4 bucket→30%（半开）"),
            ("arm_boom",   self._clamp("arm_boom",   80.0),  "回正 3/4 arm→80°（微收）"),
            ("swing_yaw",  self._clamp("swing_yaw",  0.0),   "回正 4/4 swing→0°（正前方）"),
        ]
        if not self._run_sequence(reset_seq):
            _log.error("[dig_ctrl] 阶段 3 末尾回正步骤失败（不影响卸料已完成）")
            # 回正失败视为 minor（料已经倒掉了），不返回 False
        else:
            _log.info("[dig_ctrl] ===== 阶段 3 卸料 → 倒料 → 回正 全部完成 ✅ =====")

        return True, best_pose

    # ------------------------------------------------------------------
    # 辅助：平面 2D (d, z) 正向运动学（boom_fixed_0°）
    #   d = 平面投影距离（base_link 原点 → 铲尖 的水平距离）
    #   z = 高度（base_link 原点 → 铲尖 的 z）
    # ------------------------------------------------------------------
    def _fk_boom_tip_planar(self, offset_x, offset_z, L1, L2, boom_bend_deg, boom_swing_deg):
        """只计算 boom tip（小臂连接点）在 (d, z) 平面的坐标"""
        theta_1 = math.radians(boom_swing_deg)  # boom_swing=0 → L1 水平向前
        bend_r  = math.radians(boom_bend_deg)
        # L1 段
        p1_d = L1 * math.cos(theta_1)
        p1_z = L1 * math.sin(theta_1)
        # L2 段：相对于 L1 的方向 = theta_1 - (π - bend_r) （bend 是内夹角）
        theta_2 = theta_1 - (math.pi - bend_r)
        tip_d = p1_d + L2 * math.cos(theta_2)
        tip_z = p1_z + L2 * math.sin(theta_2)
        # 加上 pivot 相对 base 的偏移
        return (tip_d + offset_x, tip_z + offset_z)

    def _fk_planar_boom_fixed_0(
        self, offset_x, offset_z, L1, L2, boom_bend_deg, L_arm, L_bucket,
        boom_swing_deg, arm_boom_deg, bucket_arm_deg
    ):
        """boom_tip → wrist(arm) → tip(bucket) 的 2D FK"""
        tip_d, tip_z = self._fk_boom_tip_planar(offset_x, offset_z, L1, L2, boom_bend_deg, boom_swing_deg)
        # arm_boom 语义：arm_boom=0=伸直（与 boom L2 方向一致），增大=小臂向内收
        # 近似：arm_boom_deg 增加 1° → 小臂方向相对 boom_L2 方向向下弯 1°
        bend_r = math.radians(boom_bend_deg)
        theta_1 = math.radians(boom_swing_deg)
        theta_boom_L2 = theta_1 - (math.pi - bend_r)  # L2 的绝对方向
        theta_arm = theta_boom_L2 - math.radians(arm_boom_deg)  # 小臂方向
        wrist_d = tip_d + L_arm * math.cos(theta_arm)
        wrist_z = tip_z + L_arm * math.sin(theta_arm)
        # bucket_arm：正值=向内卷；相对 arm 方向再偏
        theta_bucket = theta_arm - math.radians(bucket_arm_deg + 90.0)  # 零点偏移
        final_d = wrist_d + L_bucket * math.cos(theta_bucket)
        final_z = wrist_z + L_bucket * math.sin(theta_bucket)
        return (final_d, final_z)

    # ========================================================================
    # 组合全流程（高级 API）
    # ========================================================================
    def execute_full_cycle_dig_dump(
        self,
        dig_xyz: Tuple[float, float, float],
        dump_xyz: Tuple[float, float, float],
    ) -> bool:
        """
        全流程组合函数 1（类型 A）：
          阶段 1 挖掘 → 阶段 2 自动回转 → 阶段 3 卸料 → 回正（下一目标点准备位）

        【参数约束（用户指定）】
          dump_xyz[2]（卸料点 z 值）：
              - 若 z < 1.0m  → 内部强制 z_dump = 1.0m （下限）
              - 若 z ≥ 1.0m  → 取 z_dump = dump_xyz[2] （不封顶，取最大值）
          即 max(1.0, dump_xyz[2])

        【回正（下一目标点准备位）—— 阶段 4】
          卸料后自动回到：boom=0°（最高）、小臂微收、bucket 半开、swing_yaw=0°（正前方）
          方便下一挖掘/卸料直接开始，无需每次手动回零。
        """
        _log.info(
            "\n" + "=" * 72 +
            f"\n[FULL-CYCLE A] ===== 开始全流程：挖掘 → 回转 → 卸料 → 回正 =====\n"
            f"              挖掘点 dig   = ({dig_xyz[0]:.3f}, {dig_xyz[1]:.3f}, {dig_xyz[2]:.3f}) m\n"
            f"              卸料点 dump  = ({dump_xyz[0]:.3f}, {dump_xyz[1]:.3f}, {dump_xyz[2]:.3f}) m  (原始输入)\n"
            + "=" * 72
        )

        # ------------- 卸料点 z 高度强制下限 1.0m（用户指定规则）-------------
        dx, dy, dz_raw = float(dump_xyz[0]), float(dump_xyz[1]), float(dump_xyz[2])
        dz_use = max(0.5, dz_raw)  # ★ <0.5m 强制 0.5m；>=0.5m 取原值
        if dz_use != dz_raw:
            _log.warning(
                f"[FULL-CYCLE A] 卸料点 z={dz_raw:.3f}m < 0.5m 阈值，\n"
                f"              按用户规则强制抬升为 dz_use={dz_use:.3f}m")
        dump_xyz_clamped = (dx, dy, dz_use)

        # ------------- 阶段 1：挖掘 dig_xyz -------------
        _log.info("[FULL-CYCLE A] === [1/4] 阶段 1：挖掘 ===")
        if not self.plan_and_execute_dig(dig_xyz):
            _log.error("[FULL-CYCLE A] 阶段 1 挖掘失败，中止全流程")
            return False

        # ------------- 阶段 2：运输回转（自动计算 swing_yaw = atan2(dy, dx)）-------------
        target_yaw_deg = 0.0
        if abs(dx) < 1e-4 and abs(dy) < 1e-4:
            target_yaw_deg = 0.0
            _log.warning("[FULL-CYCLE A] 卸料点 (dx,dy) 接近原点，阶段 2 默认为 swing=0°")
        else:
            target_yaw_deg = math.degrees(math.atan2(dy, dx))
        target_yaw_deg = self._clamp("swing_yaw", target_yaw_deg)
        _log.info(
            f"[FULL-CYCLE A] === [2/4] 阶段 2：运输回转 → swing_yaw={target_yaw_deg:.1f}° ===")
        if not self.plan_and_execute_transit(target_yaw_deg):
            _log.error("[FULL-CYCLE A] 阶段 2 运输回转失败，中止全流程")
            return False

        # ------------- 阶段 3：卸料 dump_xyz_clamped（z 已处理）-------------
        _log.info(
            f"[FULL-CYCLE A] === [3/4] 阶段 3：卸料 ({dump_xyz_clamped[0]:.2f},"
            f"{dump_xyz_clamped[1]:.2f},{dump_xyz_clamped[2]:.2f}) ===")
        dump_ok, _dump_pose = self.plan_and_execute_dump(dump_xyz_clamped)
        if not dump_ok:
            _log.error("[FULL-CYCLE A] 阶段 3 卸料失败，中止全流程")
            return False

        # ------------- 阶段 4：回正（下一目标点准备位）-------------
        _log.info("[FULL-CYCLE A] === [4/4] 阶段 4：回正（下一目标点准备位） ===")
        reset_boom  = self._clamp("boom_swing", 0.0)
        reset_arm   = self._clamp("arm_boom", 80.0)
        reset_buck  = self._clamp("bucket_arm", -30.0)
        reset_swing = self._clamp("swing_yaw", 0.0)
        reset_seq = [
            ("boom_swing", reset_boom,  "回正 1/4  boom→0°（最高，越障安全）"),
            ("bucket_arm", reset_buck,  "回正 2/4  bucket→-30°（半开）"),
            ("arm_boom",   reset_arm,   "回正 3/4  arm→80°（微收准备位）"),
            ("swing_yaw",  reset_swing, "回正 4/4  swing→0°（正前方）"),
        ]
        for i, (j, t, d) in enumerate(reset_seq):
            _log.info(f"[FULL-CYCLE A] {d} -> [{j}]={t:.1f}°")
            timeout = 30.0 if j == "swing_yaw" else 20.0
            res = self.joint_ctrl.move_joint(j, t, tolerance_deg=2.0, timeout_s=timeout, wait_blocking=True)
            if not res.get("success"):
                _log.error(f"[FULL-CYCLE A] 回正步骤 {i+1} 失败: {res.get('reason')}")
                return False
            time.sleep(0.5)

        _log.info(
            "\n" + "=" * 72 +
            "\n[FULL-CYCLE A] ✅ 全流程挖掘→回转→卸料→回正 全部成功！\n" +
            "=" * 72
        )
        return True

    def execute_dig_and_transit(
        self,
        dig_xyz: Tuple[float, float, float],
        transit_yaw_deg: float,
        dump_distance_m: float = 1.2,
        dump_height_min: float = 0.5,
    ) -> bool:
        """
        全流程组合函数 2（类型 B，完整闭环）：
          阶段 1 挖掘 dig_xyz
            → 阶段 2 按给定角度回转 transit_yaw_deg
            → 阶段 3 在「当前回转方向正前方」自动推卸料点并卸料
            → 阶段 4 回正（下一目标点准备位）

        【卸料点自动推导（因为用户只给了回转角度没给卸料点 xyz）】
          设 : dump_distance_m = 1.2m（正前方臂展常见安全卸料距离，可传参覆写）
               dump_height_min = 1.0m（用户规则下限，可传参覆写；同时 dump_z 内部再走 max(1.0, z) 双保险）
          则 : dump_x = dump_distance_m * cos(transit_yaw)
               dump_y = dump_distance_m * sin(transit_yaw)
               dump_z = max(dump_height_min, 0.5)  # 强制 ≥ 0.5m
          即：回转到 target_yaw 后，在该方向正前方 1.2m / 高度 0.5m 上方开斗卸料。

        【参数】
          dig_xyz:           挖掘点 (x, y, z) m
          transit_yaw_deg:   挖掘完成后回转目标角度（度）。正值=从上往下看顺时针。
          dump_distance_m:   （可选，默认 1.2）回转后「正前方」卸料距离 m。
          dump_height_min:   （可选，默认 0.5）卸料点最小高度 m。
        """
        yaw = self._clamp("swing_yaw", float(transit_yaw_deg))
        _log.info(
            "\n" + "=" * 72 +
            f"\n[FULL-CYCLE B] ===== 开始完整闭环：挖掘 → 回转指定角度 → 自动推卸料点 → 回正 =====\n"
            f"              挖掘点 dig          = ({dig_xyz[0]:.3f}, {dig_xyz[1]:.3f}, {dig_xyz[2]:.3f}) m\n"
            f"              回转角度 swing      = {yaw:.1f}°\n"
            f"              卸料距离(正前自动)   = {dump_distance_m:.2f} m\n"
            f"              卸料高度下限         = {max(dump_height_min, 0.5):.2f} m\n"
            + "=" * 72
        )

        # 阶段 1：挖掘
        _log.info("[FULL-CYCLE B] === [1/4] 阶段 1：挖掘 ===")
        if not self.plan_and_execute_dig(dig_xyz):
            _log.error("[FULL-CYCLE B] 阶段 1 挖掘失败，中止全流程")
            return False

        # 阶段 2：运输回转（给定角度）
        _log.info(f"[FULL-CYCLE B] === [2/4] 阶段 2：运输回转 swing={yaw:.1f}° ===")
        if not self.plan_and_execute_transit(yaw):
            _log.error("[FULL-CYCLE B] 阶段 2 运输回转失败，中止全流程")
            return False

        # 阶段 3：自动推卸料点（当前 yaw 方向正前方 1.2m + 高度≥0.5m）然后卸料
        dump_z = max(float(dump_height_min), 0.5)  # 用户 z<0.5→0.5 规则（双保险）
        yaw_rad = math.radians(yaw)
        dump_x = dump_distance_m * math.cos(yaw_rad)
        dump_y = dump_distance_m * math.sin(yaw_rad)
        auto_dump_xyz = (dump_x, dump_y, dump_z)
        _log.info(
            f"[FULL-CYCLE B] === [3/4] 阶段 3：自动推卸料点卸料\n"
            f"            自动推导: swing={yaw:.1f}° → 正前 {dump_distance_m:.2f}m × 高度 {dump_z:.2f}m\n"
            f"            得到 dump_xyz = ({auto_dump_xyz[0]:.2f}, {auto_dump_xyz[1]:.2f}, {auto_dump_xyz[2]:.2f}) m  ===")
        dump_ok_b, _dump_pose_b = self.plan_and_execute_dump(auto_dump_xyz)
        if not dump_ok_b:
            _log.error("[FULL-CYCLE B] 阶段 3 卸料失败，中止全流程")
            return False

        # 阶段 4：回正（下一目标点准备位）
        _log.info("[FULL-CYCLE B] === [4/4] 阶段 4：回正（下一目标点准备位） ===")
        reset_boom  = self._clamp("boom_swing", 0.0)
        reset_arm   = self._clamp("arm_boom", 80.0)
        reset_buck  = self._clamp("bucket_arm", -30.0)
        reset_swing = self._clamp("swing_yaw", 0.0)
        reset_seq = [
            ("boom_swing", reset_boom,  "回正 1/4  boom→0°（最高，越障安全）"),
            ("bucket_arm", reset_buck,  "回正 2/4  bucket→-30°（半开）"),
            ("arm_boom",   reset_arm,   "回正 3/4  arm→80°（微收准备位）"),
            ("swing_yaw",  reset_swing, "回正 4/4  swing→0°（正前方）"),
        ]
        for i, (j, t, d) in enumerate(reset_seq):
            _log.info(f"[FULL-CYCLE B] {d} -> [{j}]={t:.1f}°")
            timeout = 30.0 if j == "swing_yaw" else 20.0
            res = self.joint_ctrl.move_joint(j, t, tolerance_deg=2.0, timeout_s=timeout, wait_blocking=True)
            if not res.get("success"):
                _log.error(f"[FULL-CYCLE B] 回正步骤 {i+1} 失败: {res.get('reason')}")
                return False
            time.sleep(0.5)

        _log.info(
            "\n" + "=" * 72 +
            "\n[FULL-CYCLE B] ✅ 完整闭环成功：挖掘→回转指定角度→卸料→回正 全部完成！\n" +
            "=" * 72
        )
        return True
