import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

from v15_action_task import from_config, TrajectoryPlanner, JointTrajectoryPoint
import math


def main() -> int:
    ctx = from_config(adapter_backend="mock", init_trajectory_planner=True)
    tp: TrajectoryPlanner = ctx["trajectory_planner"]
    cfg = ctx["config"]
    print(f"[ex14] TrajectoryPlanner default_strategy={tp.default_strategy}, lerp_step={tp.lerp_step_deg}deg")
    print(f"[ex14]   max_speed boom={tp.max_speed_deg_s.get('boom_swing',0):.1f}deg/s, accel boom={cfg.trajectory.accel_deg_s2.get('boom_swing',0):.1f}deg/s2")

    # 起点/终点：boom 从 0° → 45°，其他关节 0
    start_pose = {"swing_yaw": 0.0, "boom_swing": 0.0, "arm_boom": 0.0, "bucket_arm": 0.0}
    end_pose = {"swing_yaw": 0.0, "boom_swing": 45.0, "arm_boom": 0.0, "bucket_arm": 0.0}

    # ── AC-4: LERP 45° boom，段数 ceil(45/lerp_step) = ceil(45/5)=10 段(即10个waypoint)，最后一个点严格对齐45°
    traj_lerp = tp.plan_segment(start_pose, end_pose, strategy="lerp")
    n_lerp = len(traj_lerp)
    print(f"[ex14] AC-4 LERP boom 45° -> waypoints={n_lerp} (need 10)")
    ac4_n = (n_lerp == 10)
    last_boom = traj_lerp[-1].pose_deg["boom_swing"]
    first_boom = traj_lerp[0].pose_deg["boom_swing"]
    print(f"[ex14]   first boom={first_boom:.3f}°, last boom={last_boom:.3f}° (need 0,45)")
    ac4_endpoint = abs(first_boom - 0.0) < 0.01 and abs(last_boom - 45.0) < 0.01
    # 线性插值误差：检查每个中间点 alpha 线性
    ac4_linear = True
    for k, wp in enumerate(traj_lerp):
        alpha = k / (n_lerp - 1) if n_lerp > 1 else 1.0
        expected_boom = 0.0 + (45.0 - 0.0) * alpha
        actual_boom = wp.pose_deg["boom_swing"]
        if abs(actual_boom - expected_boom) > 0.01:
            ac4_linear = False
            print(f"[ex14]   k={k} linear mismatch: expect={expected_boom:.4f}, actual={actual_boom:.4f}")
            break
    ac4 = ac4_n and ac4_endpoint and ac4_linear

    # ── AC-5: Trapezoidal 45° boom，速度单调递增→峰→递减，峰值 ≤ max_speed * 1.05
    traj_trap = tp.plan_segment(start_pose, end_pose, strategy="trapezoidal")
    n_trap = len(traj_trap)
    times = [tp_i.time_from_start_s for tp_i in traj_trap]
    t_monotonic = all(times[i] < times[i + 1] for i in range(len(times) - 1))
    print(f"[ex14] AC-5 Trapezoidal boom 45° -> waypoints={n_trap}, T_total={times[-1]:.3f}s, t_monotonic={t_monotonic}")

    vels_boom = []
    for tp_i in traj_trap:
        v = tp_i.velocity_deg_s or {}
        vels_boom.append(float(v.get("boom_swing", 0.0)))
    abs_vels = [abs(v) for v in vels_boom]
    max_abs_v = max(abs_vels) if abs_vels else 0.0
    vmax_boom_cfg = float(tp.max_speed_deg_s.get("boom_swing", 20.0))
    print(f"[ex14]   max |boom_vel| = {max_abs_v:.3f}deg/s, cfg vmax={vmax_boom_cfg:.3f}deg/s, ratio={max_abs_v / vmax_boom_cfg if vmax_boom_cfg else 0:.3f} (≤1.05)")
    ac5_vmax = max_abs_v <= vmax_boom_cfg * 1.05
    # 末速度 = 0
    end_v = abs(vels_boom[-1])
    print(f"[ex14]   final |vel| = {end_v:.3f}deg/s (need ~0)")
    ac5_end_v = end_v < 0.1
    ac5 = t_monotonic and ac5_vmax and ac5_end_v

    # ── 额外：planning_ik_solve 可达域 hook (ws 未注入时能正常 fallback，只做 IK)
    ik_res_ok = tp.planning_ik_solve is not None
    print(f"[ex14] planning_ik_solve 存在: {ik_res_ok}")

    print()
    print(f"[ex14] AC-4 LERP (段数=10 / 端点 / 线性度): {'PASS' if ac4 else 'FAIL'}")
    print(f"[ex14]   ├ 段数 10: {'PASS' if ac4_n else 'FAIL'}")
    print(f"[ex14]   ├ 端点对齐: {'PASS' if ac4_endpoint else 'FAIL'}")
    print(f"[ex14]   └ 线性误差<0.01: {'PASS' if ac4_linear else 'FAIL'}")
    print(f"[ex14] AC-5 Trapezoidal (t单调 / 速度≤1.05vmax / 末速≈0): {'PASS' if ac5 else 'FAIL'}")
    print(f"[ex14]   ├ t 单调: {'PASS' if t_monotonic else 'FAIL'}")
    print(f"[ex14]   ├ max_vel/vmax ≤1.05: {'PASS' if ac5_vmax else 'FAIL'}")
    print(f"[ex14]   └ 末速度≈0: {'PASS' if ac5_end_v else 'FAIL'}")

    all_pass = ac4 and ac5
    print("PASS" if all_pass else "FAIL", file=sys.stdout if all_pass else sys.stderr)
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
