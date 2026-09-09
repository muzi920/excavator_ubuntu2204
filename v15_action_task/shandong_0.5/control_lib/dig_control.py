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
            
    def _clamp(self, joint_name: str, value: float) -> float:
        if joint_name in self.limits:
            min_deg, max_deg = self.limits[joint_name]
            return max(min_deg, min(max_deg, float(value)))
        return value

    def plan_and_execute_dig(self, xyz: Tuple[float, float, float]) -> bool:
        """
        阶段 1：输入空间点，规划并执行挖掘动作（初始化 -> 下探 -> 挖掘 -> 抬起 -> 稳料）
        由于底层安全要求，所有动作严格单关节串行。
        
        ★ 大臂标定语义：boom_swing = 0° 最高点（完全抬起），48° 最低点（压到地面）
          所以抬大臂 = 角度减小向 0° 靠拢，下探/挖掘 = 角度增大向 48° 靠拢。
        """
        x, y, z = xyz
        
        _log.info(f"[dig_ctrl] 开始挖掘点规划: {xyz}")
        
        # 1. 求解挖掘点最优逆解 (使用半开斗范围进行搜索，偏好 -70 到 -20)
        sol = self.ik.search_bucket_angle(x, y, z, bucket_range_deg=(-70.0, -20.0), num_candidates=15)
        if not sol or not hasattr(sol, 'as_pose'):
            _log.error("[dig_ctrl] 挖掘点不可达或无可行姿态解")
            return False
            
        dig_pose = sol.as_pose()
        if not dig_pose:
            _log.error("[dig_ctrl] 挖掘点逆解失败")
            return False
            
        _log.info(f"[dig_ctrl] 挖掘点逆解成功: {dig_pose}")
        
        # 1.5 求解提升至 1m 高度（安全高度）的预备逆解，确保在下探前不会刮蹭地面
        lift_z = 1.0
        sol_lift = self.ik.search_bucket_angle(x, y, lift_z, bucket_range_deg=(-70.0, -20.0), num_candidates=15)
        if not sol_lift or not hasattr(sol_lift, 'as_pose') or not sol_lift.as_pose():
            _log.warning(f"[dig_ctrl] 无法求得 {lift_z}m 高度的逆解，使用默认的大臂抬起姿态")
            # ★ 0° = 最高点，默认先抬到 5° 左右的高空安全位
            lift_boom = self._clamp("boom_swing", 5.0)
            lift_arm = self._clamp("arm_boom", dig_pose["arm_boom"])
        else:
            lift_pose = sol_lift.as_pose()
            # 确保预备姿态的大臂是抬起来的（≤ 15°），如果 IK 算出的是低位则强制修正
            lift_boom_raw = float(lift_pose["boom_swing"])
            if lift_boom_raw > 15.0:
                _log.warning(f"[dig_ctrl] IK 预备大臂角度 {lift_boom_raw:.1f}° 过低，强制修正为 10°（高空安全位）")
                lift_boom = self._clamp("boom_swing", 10.0)
            else:
                lift_boom = self._clamp("boom_swing", lift_boom_raw)
            lift_arm = self._clamp("arm_boom", lift_pose["arm_boom"])
        
        # 2. 生成动作序列
        # 参考 v14 挖掘逻辑计算关键姿态
        dig_entry_bucket = self._clamp("bucket_arm", -55.0)
        scoop_bucket = self._clamp("bucket_arm", 0.0)
        scoop_arm = self._clamp("arm_boom", max(dig_pose["arm_boom"], 72.0))
        
        # 动作序列，每个元素为一个要执行的单关节动作 (joint_name, target_deg, description)
        sequence = [
            # 阶段 0：提臂至 1m 以上安全高度
            ("boom_swing", lift_boom, "挖掘预备(提大臂至1m安全高度)"),
            ("arm_boom", lift_arm, "挖掘预备(小臂调整至1m高度姿态)"),
            
            # 阶段 1：初始化（对准，准备下探）
            ("swing_yaw", dig_pose["swing_yaw"], "对准挖掘点(回转)"),
            ("bucket_arm", dig_entry_bucket, "挖掘预备(半开斗)"),
            ("arm_boom", dig_pose["arm_boom"], "挖掘预备(小臂下探)"),
            ("boom_swing", dig_pose["boom_swing"], "挖掘预备(大臂下探)"),
            
            # 阶段 2：挖掘取料 (仅执行收斗，并略微回拉小臂辅助)
            ("bucket_arm", scoop_bucket, "收斗取料"),
            ("arm_boom", scoop_arm, "回拉小臂辅助收料")
        ]
        
        # 3. 严格单关节串行执行
        _log.info(f"[dig_ctrl] 规划完成，共 {len(sequence)} 步，开始串行执行...")
        for i, (joint, target_deg, desc) in enumerate(sequence):
            _log.info(f"[dig_ctrl] 执行步骤 {i+1}/{len(sequence)}: {desc} -> {joint}={target_deg:.1f}°")
            res = self.joint_ctrl.move_joint(joint, target_deg, tolerance_deg=2.0, timeout_s=20.0, wait_blocking=True)
            if not res.get("success"):
                _log.error(f"[dig_ctrl] 步骤 {i+1} 执行失败: {res.get('reason')}")
                return False
            # 动作间安全延时
            time.sleep(0.5)
            
        _log.info("[dig_ctrl] 阶段 1 挖掘动作全部完成！")
        return True

    def plan_and_execute_transit(self, target_yaw_deg: float) -> bool:
        """
        阶段 2：提起大臂，然后旋转到卸料区域角度（Swing Yaw）。
        严格按照用户指定的动作顺序：先抬大臂脱离坑位 → 再回转。
        稳料（收小臂、微开斗）作为可选的后置步骤，默认关闭，避免干扰单独测试。
        """
        _log.info(f"[dig_ctrl] ===== 开始阶段 2 (运输回转) 规划: 目标偏航角 swing_yaw = {target_yaw_deg}° =====")
        
        # 1. 关键姿态计算
        # ★ 用户标定语义：大臂 boom_swing = 0° 最高点（完全抬起），48° 最低点（压到地面）
        #   所以抬大臂 = 向 0° 靠拢，压大臂 = 向 48° 增大
        # 运输回转时抬到最高点 0°，保证足够的越障空间
        lift_boom = self._clamp("boom_swing", 0.0)
        # 回转目标角度（直接使用用户传入的 yaw）
        target_yaw = self._clamp("swing_yaw", target_yaw_deg)
        
        # 2. 严格按照用户说的顺序：先抬大臂 → 再回转
        sequence = [
            # 步骤 1：提起大臂（用户要求的第一步），抬到最高点 0°
            ("boom_swing", lift_boom, f"步骤1/2 抬大臂到最高点 0°（用户标定 0=最高, 48=最低）"),
            
            # 步骤 2：旋转至目标卸料角度（用户要求的第二步，yaw就是传入值）
            ("swing_yaw", target_yaw, f"步骤2/2 回转 swing_yaw -> {target_yaw:.1f}°（用户传入 = {target_yaw_deg}°）")
        ]
        
        # 3. 严格单关节串行执行
        _log.info(f"[dig_ctrl] 阶段 2 规划完成，共 {len(sequence)} 步（简化模式，只做提臂+回转），开始串行执行...")
        for i, (joint, target_deg, desc) in enumerate(sequence):
            _log.info(f"[dig_ctrl] 执行步骤 {i+1}/{len(sequence)}: {desc} -> [{joint}]={target_deg:.1f}°")
            
            # 回转动作给更长超时（整机旋转惯性大）
            timeout = 30.0 if joint == "swing_yaw" else 20.0
            
            res = self.joint_ctrl.move_joint(joint, target_deg, tolerance_deg=2.0, timeout_s=timeout, wait_blocking=True)
            if not res.get("success"):
                _log.error(f"[dig_ctrl] 阶段 2 步骤 {i+1} 执行失败: {res.get('reason')}")
                return False
            # 动作间安全延时
            time.sleep(0.5)
            
        _log.info(f"[dig_ctrl] ===== 阶段 2 运输回转全部完成！最终 swing_yaw = {target_yaw:.1f}° =====")
        return True

    def plan_and_execute_dump(self, dump_xyz: Tuple[float, float, float]) -> bool:
        """
        阶段 3：卸料点控制（★ 与阶段 1 挖掘点严格区分）

        用户约束：无论给定 dump_xyz[2]（z）是多少，内部默认
                 → 选择「大臂姿态最高（即 boom_swing 取到可达的最小值 ≈ 0°）」
                 的那组 IK 解去执行。
                 → 也就是说：我们不硬卡"boom必须=0°"（因为 FK 方向易出错），
                    而是取 boom_swing 最小的那组解 → 等价于"大臂最高姿态"。

        执行流程：
          1. 先按「boom最小→末端误差最小」双准则筛选 IK 最佳解
          2. 先把 boom 抬到目标角度（保证先起高，安全越障）
          3. 回转到 IK 解给出的 swing_yaw
          4. 小臂伸到位；铲斗先不开斗
          5. 最后大角度开斗 = 卸料
        """
        x, y, z_user = float(dump_xyz[0]), float(dump_xyz[1]), float(dump_xyz[2])
        _log.info(
            f"[dig_ctrl] ===== 开始阶段 3 (卸料 Dump) 规划 =====\n"
            f"            卸料点 XYZ = ({x:.3f}, {y:.3f}, {z_user:.3f}) m  (base_link 系)\n"
            f"            ★ 约束策略：取 boom_swing 最小（大臂最高姿态）的 IK 解，保证安全卸料"
        )

        MIN_SAFE_Z_FOR_DUMP = 0.3  # 安全卸料最小离地高度
        z_use = max(z_user, MIN_SAFE_Z_FOR_DUMP)
        if z_use > z_user:
            _log.warning(
                f"[dig_ctrl] 用户传入 z={z_user:.3f}m 低于安全高度，抬升 z_use={z_use:.3f}m"
            )

        # ---------------- 1. 可达域预检（利用 WorkspaceChecker 提前拦不可达） ----------------
        try:
            from v15_action_task.workspace_check import WorkspaceChecker
            _ws = WorkspaceChecker(self.cfg)
            reachable, reason = _ws.check_point_reachable(x, y, z_use)
            if not reachable:
                _log.error(
                    f"[dig_ctrl] 阶段 3 预检未通过：卸料点 ({x:.2f},{y:.2f},{z_use:.2f}) 超出运动包络！\n"
                    f"            原因: {reason}"
                )
                return False
        except Exception as _e:
            _log.warning(f"[dig_ctrl] WorkspaceChecker 导入失败，跳过预检（继续 IK 兜底）: {_e}")

        # ---------------- 2. 多路 IK 候选搜索（boom 最小 + 末端误差小 双准则） ----------------
        bucket_segments = [
            (-90.0, -60.0), (-65.0, -35.0), (-40.0, -10.0), (-15.0, 20.0)
        ]
        candidates: List[Tuple[float, float, Dict[str, float]]] = []
        for b_min, b_max in bucket_segments:
            sol = self.ik.search_bucket_angle(x, y, z_use,
                                              bucket_range_deg=(b_min, b_max),
                                              num_candidates=10)
            if not sol or not hasattr(sol, 'as_pose'):
                continue
            pose = sol.as_pose()
            if not pose:
                continue
            boom = float(pose.get("boom_swing", 999.0))
            # 限位钳制到合法区间（避免奇异点）
            boom_clamped = self._clamp("boom_swing", boom)
            arm = self._clamp("arm_boom", float(pose.get("arm_boom", 90.0)))
            buck = self._clamp("bucket_arm", float(pose.get("bucket_arm", 0.0)))
            sw = self._clamp("swing_yaw", float(pose.get("swing_yaw", 0.0)))
            # 末端位置反向校验（用 v15 IK 自带 FK，不用我手写的）
            try:
                from v15_action_task.kinematics.forward import ForwardKinematics
                _fk = ForwardKinematics(self.cfg)
                _tip_x, _tip_y, _tip_z = _fk.compute_tip_position(sw, boom_clamped, arm, buck)
                err = math.hypot(_tip_x - x, _tip_y - y, _tip_z - z_use)
            except Exception:
                err = 0.0  # FK 失败就信任 IK
            # 评分：boom 越小（越高）权重越大；末端误差越小越好
            score_boom = boom_clamped  # 0 最好
            score_err = err * 100.0    # 放大到度量级
            total = score_boom + score_err
            candidates.append((
                total,
                err,
                {"swing_yaw": sw, "boom_swing": boom_clamped, "arm_boom": arm, "bucket_arm": buck},
            ))

        if not candidates:
            _log.error(
                f"[dig_ctrl] 阶段3 IK 无解：卸料点 ({x:.2f},{y:.2f},{z_use:.2f}) 任何姿态都够不到。"
            )
            return False

        candidates.sort(key=lambda t: t[0])
        best_score, best_err, best_pose = candidates[0]
        _log.info(
            f"[dig_ctrl] 阶段 3 卸料最佳 IK 解（boom 最小优先）：\n"
            f"            boom_swing = {best_pose['boom_swing']:.1f}°（越小=越高，目标就是让它≈0°最高）\n"
            f"            swing_yaw  = {best_pose['swing_yaw']:.1f}°\n"
            f"            arm_boom   = {best_pose['arm_boom']:.1f}°\n"
            f"            bucket_arm = {best_pose['bucket_arm']:.1f}°\n"
            f"            末端位置误差 ≈ {best_err*100:.1f}cm"
        )

        # 最终卸料姿态：boom 保持不动，bucket 强制大角度开斗（-85°）
        dump_open_bucket = self._clamp("bucket_arm", -85.0)

        # ---------------- 3. 动作序列（严格单关节串行） ----------------
        sequence = [
            # 步骤 1：先抬大臂（用户要求的"大臂最高角度 0°"）
            ("boom_swing", best_pose["boom_swing"],
             f"步骤1/4 抬大臂到最高可达姿态 boom={best_pose['boom_swing']:.1f}°"),
            # 步骤 2：回转
            ("swing_yaw", best_pose["swing_yaw"],
             f"步骤2/4 回转 swing={best_pose['swing_yaw']:.1f}° 对齐卸料方向"),
            # 步骤 3：小臂 → 铲斗预位（还不开斗）
            ("arm_boom", best_pose["arm_boom"],
             f"步骤3/4 伸出小臂 arm={best_pose['arm_boom']:.1f}°"),
            ("bucket_arm", best_pose["bucket_arm"],
             f"步骤3/4 铲斗预位 bucket={best_pose['bucket_arm']:.1f}°（未卸料）"),
            # 步骤 4：开斗 → 卸料
            ("bucket_arm", dump_open_bucket,
             f"步骤4/4 开斗卸料 bucket={dump_open_bucket:.1f}° → 倒料"),
        ]

        _log.info(f"[dig_ctrl] 阶段 3 规划完成，共 {len(sequence)} 步，开始串行执行...")
        for i, (joint, target_deg, desc) in enumerate(sequence):
            _log.info(f"[dig_ctrl] 执行步骤 {i+1}/{len(sequence)}: {desc} -> [{joint}]={target_deg:.1f}°")
            timeout = 30.0 if joint == "swing_yaw" else 20.0
            res = self.joint_ctrl.move_joint(joint, target_deg, tolerance_deg=2.0, timeout_s=timeout, wait_blocking=True)
            if not res.get("success"):
                _log.error(f"[dig_ctrl] 阶段 3 步骤 {i+1} 执行失败: {res.get('reason')}")
                return False
            time.sleep(0.5)

        _log.info(f"[dig_ctrl] ===== 阶段 3 卸料全部完成！最终 boom={best_pose['boom_swing']:.1f}° bucket={dump_open_bucket:.1f}° =====")
        return True

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
        dz_use = max(1.0, dz_raw)  # ★ <1.0m 强制 1.0m；>=1.0m 取原值
        if dz_use != dz_raw:
            _log.warning(
                f"[FULL-CYCLE A] 卸料点 z={dz_raw:.3f}m < 1.0m 阈值，\n"
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
        if not self.plan_and_execute_dump(dump_xyz_clamped):
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
        dump_height_min: float = 1.0,
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
               dump_z = max(dump_height_min, 1.0)  # 强制 ≥ 1.0m
          即：回转到 target_yaw 后，在该方向正前方 1.2m / 高度 1.0m 上方开斗卸料。

        【参数】
          dig_xyz:           挖掘点 (x, y, z) m
          transit_yaw_deg:   挖掘完成后回转目标角度（度）。正值=从上往下看顺时针。
          dump_distance_m:   （可选，默认 1.2）回转后「正前方」卸料距离 m。
          dump_height_min:   （可选，默认 1.0）卸料点最小高度 m。
        """
        yaw = self._clamp("swing_yaw", float(transit_yaw_deg))
        _log.info(
            "\n" + "=" * 72 +
            f"\n[FULL-CYCLE B] ===== 开始完整闭环：挖掘 → 回转指定角度 → 自动推卸料点 → 回正 =====\n"
            f"              挖掘点 dig          = ({dig_xyz[0]:.3f}, {dig_xyz[1]:.3f}, {dig_xyz[2]:.3f}) m\n"
            f"              回转角度 swing      = {yaw:.1f}°\n"
            f"              卸料距离(正前自动)   = {dump_distance_m:.2f} m\n"
            f"              卸料高度下限         = {max(dump_height_min, 1.0):.2f} m\n"
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

        # 阶段 3：自动推卸料点（当前 yaw 方向正前方 1.2m + 高度≥1.0m）然后卸料
        dump_z = max(float(dump_height_min), 1.0)  # 用户 z<1→1 规则（双保险）
        yaw_rad = math.radians(yaw)
        dump_x = dump_distance_m * math.cos(yaw_rad)
        dump_y = dump_distance_m * math.sin(yaw_rad)
        auto_dump_xyz = (dump_x, dump_y, dump_z)
        _log.info(
            f"[FULL-CYCLE B] === [3/4] 阶段 3：自动推卸料点卸料\n"
            f"            自动推导: swing={yaw:.1f}° → 正前 {dump_distance_m:.2f}m × 高度 {dump_z:.2f}m\n"
            f"            得到 dump_xyz = ({auto_dump_xyz[0]:.2f}, {auto_dump_xyz[1]:.2f}, {auto_dump_xyz[2]:.2f}) m  ===")
        if not self.plan_and_execute_dump(auto_dump_xyz):
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
