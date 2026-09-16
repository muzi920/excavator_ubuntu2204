"""
WorkspaceChecker —— 挖掘机可达域初始化 + 5 层约束检查（任务1）。

核心能力：
1. from_config(cfg) 一次读取 link_geometry / joint_limits / workspace 配置，计算内/外半径；
2. check_point_reachable((x,y,z)) 5 层检查：几何腕点三角不等式 → swing_yaw 限位 →
   ground_penetration(z < min_ground_z) → transit_min_z(z < min_transit_z，仅 transit 模式)
   → 奇异位姿拒绝（距离内/外边界 < margin）；
3. sample_reachable_height(x, y) 二分搜索该水平投影下可达的 [z_min, z_max]；
4. closest_reachable_point(target) 等比例缩回或抬 z 得到最近的可达点。

所有检查均为纯数学 O(1)，单次耗时 < 2ms（NFR-3）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Optional, Tuple


MODE_AUTO = "auto"
MODE_DIG = "dig"
MODE_TRANSIT = "transit"
MODE_WRIST = "wrist"
VALID_MODES = {MODE_AUTO, MODE_DIG, MODE_TRANSIT, MODE_WRIST}


# ==============================================================
# 数据结构
# ==============================================================

@dataclass
class ReachabilityReport:
    """一次可达域检查的完整报告（任何一层 fail 都不短路，所有原因都在 reasons 里）。"""

    success: bool
    reasons: list[str] = field(default_factory=list)
    checked_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    mode_used: str = MODE_AUTO
    horizontal_radius_m: float = 0.0
    yaw_deg: float = 0.0
    wrist_radius_in_boom_frame_m: Optional[float] = None
    inner_radius_m: float = 0.0
    outer_radius_m: float = 0.0
    singularity_margin_m: float = 0.0


# ==============================================================
# 主类
# ==============================================================

@dataclass
class WorkspaceChecker:
    """按 link_geometry + workspace 配置构建可达域。"""

    # 连杆有效长度（和 ik/fk 用的完全一致，避免位级不一致）
    L_boom: float
    L_arm: float
    L_bucket: float
    boom_pivot_offset_x: float
    boom_pivot_offset_z: float

    # 回转 + 几何限位
    swing_yaw_min_deg: float
    swing_yaw_max_deg: float

    # workspace 配置
    min_ground_z: float        # 挖地面穿透阈值（默认 -0.1m）
    min_transit_z: float       # 转运最小离地（默认 0.30m）
    singularity_margin_min_m: float
    singularity_margin_ratio: float

    # 派生边界（惰性填充一次）
    inner_radius_m: Optional[float] = None
    outer_radius_m: Optional[float] = None
    _margin_m: Optional[float] = None

    def __post_init__(self) -> None:
        # 腕点三角不等式的内/外半径（单位：米）
        Lb = float(self.L_boom)
        La = float(self.L_arm)
        self.inner_radius_m = abs(Lb - La)
        self.outer_radius_m = Lb + La
        # margin = max(硬下限, (L1+L2)*比例)
        ratio = float(self.singularity_margin_ratio)
        mn = float(self.singularity_margin_min_m)
        self._margin_m = max(mn, (Lb + La) * ratio)

    @property
    def margin_m(self) -> float:
        return self._margin_m or 0.0

    # ============================================================
    # 构造
    # ============================================================

    @classmethod
    def from_config(cls, cfg: Any) -> "WorkspaceChecker":
        """
        从 V15Config 构造。

        Args:
            cfg: V15Config 实例；若类型不可获得，反射取以下字段：
                - cfg.link.to_link_params() -> .L_boom / .L_arm / .L_bucket
                - cfg.link.offset_x / offset_z
                - cfg.limits.limits["swing_yaw"].(min_deg/max_deg)
                - cfg.workspace.(min_ground_z / min_transit_z / singularity_margin_min_m / singularity_margin_ratio)
        """
        link_params = cfg.link.to_link_params()
        L_boom = float(link_params.L_boom)
        L_arm = float(link_params.L_arm)
        L_bucket = float(link_params.L_bucket)
        offset_x = float(cfg.link.offset_x)
        offset_z = float(cfg.link.offset_z)
        sw_lm = cfg.limits.limits["swing_yaw"]
        sw_min = float(sw_lm.min_deg)
        sw_max = float(sw_lm.max_deg)

        wsc = getattr(cfg, "workspace", None)
        if wsc is None:
            min_g = -0.1
            min_t = 0.3
            s_min = 0.02
            s_ratio = 0.01
        else:
            min_g = float(wsc.min_ground_z)
            min_t = float(wsc.min_transit_z)
            s_min = float(wsc.singularity_margin_min_m)
            s_ratio = float(wsc.singularity_margin_ratio)

        return cls(
            L_boom=L_boom,
            L_arm=L_arm,
            L_bucket=L_bucket,
            boom_pivot_offset_x=offset_x,
            boom_pivot_offset_z=offset_z,
            swing_yaw_min_deg=sw_min,
            swing_yaw_max_deg=sw_max,
            min_ground_z=min_g,
            min_transit_z=min_t,
            singularity_margin_min_m=s_min,
            singularity_margin_ratio=s_ratio,
        )

    # ============================================================
    # 派生量（单次计算缓存）
    # ============================================================

    # ============================================================
    # 几何辅助
    # ============================================================

    @staticmethod
    def _project_xz(xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """3D 点 → 水平半径 / yaw_deg / z。"""
        x, y, z = xyz
        r = math.sqrt(x * x + y * y)
        yaw = math.degrees(math.atan2(y, x))
        return r, yaw, z

    def _wrist_radius_in_boom_frame(
        self,
        horizontal_radius_abs: float,
        z_abs: float,
        bucket_guess_deg: float,
    ) -> float:
        """
        给定 tip 的绝对位置 + 假设铲斗绝对角，计算 bucket pivot（腕点）
        相对于 boom_pivot 的 2D 半径长度，用于三角不等式判断。
        """
        theta = math.radians(float(bucket_guess_deg))
        # tip 相对 bucket_pivot 向量（2D XZ 平面，yaw 已投影掉）：
        #   tip = pivot + (L_bucket*cos(theta), L_bucket*sin(theta))
        # => pivot = tip - (L_bucket cos, L_bucket sin)
        pivot_x_in_world = horizontal_radius_abs - self.L_bucket * math.cos(theta)
        pivot_z_in_world = z_abs - self.L_bucket * math.sin(theta)
        # 转到 boom_pivot 为原点的坐标系
        px = pivot_x_in_world - self.boom_pivot_offset_x
        pz = pivot_z_in_world - self.boom_pivot_offset_z
        return math.sqrt(px * px + pz * pz)

    # ============================================================
    # 核心方法 1：检查点是否可达
    # ============================================================

    def check_point_reachable(
        self,
        xyz: Tuple[float, float, float],
        *,
        mode: str = MODE_AUTO,
        bucket_guess_deg: float = -45.0,
    ) -> ReachabilityReport:
        """
        5 层约束检查。

        Args:
            xyz: 铲尖目标 (x,y,z)，单位米（base_link 坐标系，X前/Y左/Z上）。
            mode:
                - 'auto'   : z<0 -> dig，z>=0 -> transit（默认）
                - 'dig'    : 仅关闭 ground_penetration 检查（允许挖地）
                - 'transit': 启用 ground_penetration + min_transit_z
                - 'wrist'  : 只做几何腕点约束（调试用）
        """
        if mode not in VALID_MODES:
            raise ValueError(f"Unknown mode {mode!r}; valid: {sorted(VALID_MODES)}")

        r_h, yaw, z = self._project_xz(xyz)
        eff_mode = mode
        if eff_mode == MODE_AUTO:
            eff_mode = MODE_DIG if z < 0 else MODE_TRANSIT

        reasons: list[str] = []
        wrist_d: Optional[float] = None
        success = True

        # --- Layer 1 / 5: swing_yaw 限位（必须执行） ----------------
        if yaw < self.swing_yaw_min_deg - 1e-6 or yaw > self.swing_yaw_max_deg + 1e-6:
            success = False
            reasons.append(
                f"swing_yaw_out_of_range: yaw={yaw:.3f} not in "
                f"[{self.swing_yaw_min_deg:.3f}, {self.swing_yaw_max_deg:.3f}] deg"
            )

        # --- Layer 2 / 5: ground_penetration（非 dig 模式） -------
        if eff_mode != MODE_DIG:
            if z < self.min_ground_z - 1e-6:
                success = False
                reasons.append(
                    f"ground_penetration: z={z:.4f} < min_ground_z={self.min_ground_z:.4f} m"
                )

        # --- Layer 3 / 5: transit_min_z（仅 transit 模式） ---------
        if eff_mode == MODE_TRANSIT:
            if z < self.min_transit_z - 1e-6:
                success = False
                reasons.append(
                    f"transit_min_z_violation: z={z:.4f} < min_transit_z={self.min_transit_z:.4f} m"
                )

        # --- Layer 4 / 5: 几何腕点三角不等式（mode 非 wrist/dig/transit 也执行） -
        wrist_d = self._wrist_radius_in_boom_frame(r_h, z, bucket_guess_deg)
        assert self.inner_radius_m is not None
        assert self.outer_radius_m is not None
        margin = self.margin_m
        if wrist_d < self.inner_radius_m - margin - 1e-6:
            success = False
            reasons.append(
                f"inner_radius_violation: wrist_r={wrist_d:.4f} < inner={self.inner_radius_m:.4f} - margin={margin:.4f} m"
            )
        if wrist_d > self.outer_radius_m + margin + 1e-6:
            success = False
            reasons.append(
                f"outer_radius_exceeded: wrist_r={wrist_d:.4f} > outer={self.outer_radius_m:.4f} + margin={margin:.4f} m"
            )

        # --- Layer 5 / 5: 奇异位姿拒绝 -----------------------------
        d_inner = abs(wrist_d - self.inner_radius_m)
        d_outer = abs(wrist_d - self.outer_radius_m)
        if d_inner < margin or d_outer < margin:
            # 只有当几何本身是可行的（在[inner-margin, outer+margin]内）才把 singularity 算成 fail；
            # 否则前面已经因为 outer/inner 失败了，再追加 singularity 就重复了
            geometrically_feasible = (
                wrist_d >= self.inner_radius_m - margin - 1e-6
                and wrist_d <= self.outer_radius_m + margin + 1e-6
            )
            if geometrically_feasible:
                success = False
                reasons.append(
                    f"near_singularity: wrist_r={wrist_d:.4f} too close to "
                    f"boundary (d_inner={d_inner:.4f}, d_outer={d_outer:.4f}, margin={margin:.4f})"
                )

        return ReachabilityReport(
            success=success,
            reasons=reasons,
            checked_xyz=tuple(float(v) for v in xyz),  # type: ignore[arg-type]
            mode_used=eff_mode,
            horizontal_radius_m=float(r_h),
            yaw_deg=float(yaw),
            wrist_radius_in_boom_frame_m=float(wrist_d),
            inner_radius_m=float(self.inner_radius_m),
            outer_radius_m=float(self.outer_radius_m),
            singularity_margin_m=float(margin),
        )

    # ============================================================
    # 核心方法 2：水平半径 x,y → 可达 z 范围 [z_min, z_max]
    # ============================================================

    def sample_reachable_height(
        self,
        x: float,
        y: float,
        *,
        z_min_scan: float = -0.5,
        z_max_scan: float = 3.0,
        step: float = 0.02,
        bucket_guess_deg: float = -45.0,
    ) -> Optional[Tuple[float, float]]:
        """
        扫描 z，返回第一个可达的 z_min 与最后一个可达的 z_max（区间连续近似）。
        若无任何可达 z，返回 None。
        """
        reachable_zs: list[float] = []
        z = z_min_scan
        while z <= z_max_scan + 1e-9:
            r = self.check_point_reachable((x, y, z), mode=MODE_DIG, bucket_guess_deg=bucket_guess_deg)
            if r.success:
                reachable_zs.append(z)
            z += step
        if not reachable_zs:
            return None
        return (float(min(reachable_zs)), float(max(reachable_zs)))

    # ============================================================
    # 核心方法 3：最近可达点
    # ============================================================

    def closest_reachable_point(
        self,
        target_xyz: Tuple[float, float, float],
        *,
        shrink_factor: float = 0.95,
        max_iter: int = 10,
        bucket_guess_deg: float = -45.0,
    ) -> Optional[Tuple[float, float, float]]:
        """
        策略：
          1) 先夹 yaw 到 swing 限位；
          2) 若估计的腕半径 > outer，直接把水平半径缩到 (outer - 2*margin)；
             若腕半径 < inner，直接把水平半径扩到 (inner + 2*margin)；
          3) z < min_transit_z 且 z>=0（转运假设）→ 抬 z 到 min_transit_z；
          4) 逐轮微调 z，最多 max_iter 轮。
        最多 max_iter 轮，找不到返回 None。
        """
        assert self.inner_radius_m is not None
        assert self.outer_radius_m is not None
        margin = self.margin_m
        tx, ty, tz = target_xyz
        x, y, z = float(tx), float(ty), float(tz)

        # Step 1: 夹 yaw 到 swing 限位
        r_h, yaw, _ = self._project_xz((x, y, z))
        yaw_clamped = max(self.swing_yaw_min_deg, min(self.swing_yaw_max_deg, yaw))
        if abs(yaw_clamped - yaw) > 1e-6:
            yaw_r = math.radians(yaw_clamped)
            x = r_h * math.cos(yaw_r)
            y = r_h * math.sin(yaw_r)

        # Step 2: 估计腕半径并按外/内边界硬夹
        wrist_d_est = self._wrist_radius_in_boom_frame(r_h, z, bucket_guess_deg)
        safe_outer = self.outer_radius_m - 2 * margin - 1e-3
        safe_inner = self.inner_radius_m + 2 * margin + 1e-3
        if wrist_d_est > safe_outer and r_h > 1e-4:
            scale = max(safe_outer / max(wrist_d_est, 1e-6), 0.1)
            r_h2 = r_h * scale
            yaw_r2 = math.atan2(y, x) if (x or y) else 0.0
            x = r_h2 * math.cos(yaw_r2)
            y = r_h2 * math.sin(yaw_r2)
        elif wrist_d_est < safe_inner and r_h > 1e-4:
            scale = max(safe_inner / max(wrist_d_est, 1e-6), 1.01)
            r_h2 = min(r_h * scale, self.outer_radius_m - margin)
            yaw_r2 = math.atan2(y, x) if (x or y) else 0.0
            x = r_h2 * math.cos(yaw_r2)
            y = r_h2 * math.sin(yaw_r2)

        # Step 3: 转运抬 z
        if z >= 0 and z < self.min_transit_z:
            z = self.min_transit_z

        # Step 4: 逐轮微调
        last_candidate = (x, y, z)
        for _ in range(max_iter):
            rep = self.check_point_reachable(last_candidate, mode=MODE_AUTO, bucket_guess_deg=bucket_guess_deg)
            if rep.success:
                return last_candidate
            reasons_cat = " ".join(rep.reasons)
            lx, ly, lz = last_candidate
            if "transit_min_z_violation" in reasons_cat:
                lz = max(lz + 0.05, self.min_transit_z)
            elif "ground_penetration" in reasons_cat:
                lz = max(lz + 0.05, self.min_ground_z)
            elif "outer_radius_exceeded" in reasons_cat:
                rh, yw, _ = self._project_xz((lx, ly, lz))
                new_rh = rh * shrink_factor
                yw_r = math.atan2(ly, lx) if (lx or ly) else 0.0
                lx = new_rh * math.cos(yw_r)
                ly = new_rh * math.sin(yw_r)
            elif "inner_radius_violation" in reasons_cat:
                rh, yw, _ = self._project_xz((lx, ly, lz))
                new_rh = min(rh / shrink_factor if shrink_factor > 0 else rh * 1.2,
                             self.outer_radius_m - margin)
                yw_r = math.atan2(ly, lx) if (lx or ly) else 0.0
                lx = new_rh * math.cos(yw_r)
                ly = new_rh * math.sin(yw_r)
            elif "near_singularity" in reasons_cat:
                rh, yw, _ = self._project_xz((lx, ly, lz))
                new_rh = rh + (margin * 1.5 if rep.wrist_radius_in_boom_frame_m is not None
                               and rep.wrist_radius_in_boom_frame_m < (self.inner_radius_m + self.outer_radius_m) / 2
                               else -margin * 1.5)
                yw_r = math.atan2(ly, lx) if (lx or ly) else 0.0
                lx = new_rh * math.cos(yw_r)
                ly = new_rh * math.sin(yw_r)
            else:
                lz += 0.05
            last_candidate = (lx, ly, lz)

        return None
