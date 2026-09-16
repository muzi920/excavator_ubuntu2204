import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

from v15_action_task import (
    from_config, CartesianMover, MoveResult,
    move_to_dump_transit, dump_material,
)
import math, time


def _tip_dist_3d(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def main() -> int:
    ctx = from_config(adapter_backend="mock", init_workspace=True, init_trajectory_planner=True)
    ctl = ctx["controller"]
    fk = ctx["fk"]
    ik = ctx["ik"]
    ws = ctx["workspace_checker"]
    tp = ctx["trajectory_planner"]
    mover: CartesianMover = ctx["mover"]
    mover.set_workspace_checker(ws)
    mover.set_trajectory_planner(tp)

    ac6 = False
    ac7 = False
    ac8 = False

    # ── AC-6: 接口1 move_to_point((1.2,0,0.8)) FK 3D err ≤5cm
    print("=" * 60)
    print("[ex15] AC-6 接口1 CartesianMover.move_to_point(1.2, 0, 0.8)")
    with ctl:
        target_xyz = (1.2, 0.0, 0.8)
        res: MoveResult = mover.move_to_point(
            target_xyz, mode="transit", bucket_guess_deg=-45.0,
            strategy="lerp", execute=True, blocking=True,
        )
        print(f"[ex15]   success={res.success}, reason={res.reason!r}")
        print(f"[ex15]   trajectory 段数: {len(res.trajectory) if res.trajectory else 0}")
        print(f"[ex15]   ik_result 非空: {res.ik_result is not None}")
        print(f"[ex15]   reachability_report 非空: {res.reachability_report is not None}")
        reached_tip = res.final_tip_xyz
        if reached_tip is None:
            # fallback 手动 FK
            fp = res.reached_pose_deg or {}
            reached_tip = fk.solve(
                boom_swing_deg=fp.get("boom_swing", 0.0),
                arm_boom_deg=fp.get("arm_boom", 0.0),
                bucket_arm_deg=fp.get("bucket_arm", 0.0),
                swing_yaw_deg=fp.get("swing_yaw", 0.0),
            ).bucket_tip_3d
        err_cm = _tip_dist_3d(target_xyz, reached_tip) * 100.0
        print(f"[ex15]   target = ({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f})m")
        print(f"[ex15]   reached= ({reached_tip[0]:.3f}, {reached_tip[1]:.3f}, {reached_tip[2]:.3f})m")
        print(f"[ex15]   FK 3D err = {err_cm:.2f} cm (≤5cm)")
        ac6 = (err_cm <= 5.0) and bool(res.success) and (res.trajectory is not None) and (len(res.trajectory) >= 2)

    # ── AC-7: 接口2 move_to_dump_transit(up=0.3, swing=30)，最终 z∈[0.65,0.75] 且 swing_yaw∈[28,32]
    print()
    print("=" * 60)
    print("[ex15] AC-7 接口2 move_to_dump_transit(up=0.3m, swing=30°)")
    ctx7 = from_config(adapter_backend="mock")
    ctl7 = ctx7["controller"]
    fk7 = ctx7["fk"]
    with ctl7:
        dig_pose = {"swing_yaw": 0.0, "boom_swing": -10.0, "arm_boom": 60.0, "bucket_arm": -30.0}
        ctl7.set_pose(dig_pose)
        time.sleep(0.02)
        res7 = move_to_dump_transit(
            ctl7, target_point=(1.2, 0.0, 0.5),
            up_m=0.3, swing_deg=30.0,
        )
        fp7 = res7.final_pose_deg or {}
        tip7 = fk7.solve(
            boom_swing_deg=fp7.get("boom_swing", 0.0),
            arm_boom_deg=fp7.get("arm_boom", 0.0),
            bucket_arm_deg=fp7.get("bucket_arm", 0.0),
            swing_yaw_deg=fp7.get("swing_yaw", 0.0),
        ).bucket_tip_3d
        z7 = tip7[2]
        sw7 = fp7.get("swing_yaw", 9999.0)
        steps7 = len(res7.step_results)
        all_step_ok7 = all(s.success for s in res7.step_results)
        print(f"[ex15]   steps={steps7}, all_step_ok={all_step_ok7}")
        print(f"[ex15]   final z = {z7:.3f}m (∈[0.65, 0.75])")
        print(f"[ex15]   final swing_yaw = {sw7:.2f}° (∈[28, 32])")
        ac7 = bool(res7.success) and all_step_ok7 and (0.65 <= z7 <= 0.75) and (28.0 <= sw7 <= 32.0)

    # ── AC-8: 接口3 dump_material(shake_count=1)，steps∈[4,6], 每步success=True,
    #         bucket_arm∈[-99,-91], arm_boom∈[26,34]
    print()
    print("=" * 60)
    print("[ex15] AC-8 接口3 dump_material(shake_count=1, 半开45→小臂半推30→全开50 +抖斗)")
    ctx8 = from_config(adapter_backend="mock")
    ctl8 = ctx8["controller"]
    with ctl8:
        prep = {"swing_yaw": 30.0, "boom_swing": -25.0, "arm_boom": 60.0, "bucket_arm": 0.0}
        ctl8.set_pose(prep)
        time.sleep(0.02)
        res8 = dump_material(ctl8, shake_count=1, shake_angle_deg=30.0,
                             half_open_deg=45.0, arm_push_deg=30.0, full_open_deg=50.0)
        fp8 = res8.final_pose_deg or {}
        n8 = len(res8.step_results)
        all_ok8 = all(s.success for s in res8.step_results)
        bk8 = fp8.get("bucket_arm", 9999.0)
        ar8 = fp8.get("arm_boom", 9999.0)
        print(f"[ex15]   steps={n8} (∈[4,6]), all_step_success={all_ok8}")
        for i, s in enumerate(res8.step_results):
            print(f"[ex15]     step#{i}: success={s.success}")
        print(f"[ex15]   final bucket_arm = {bk8:.2f}° (∈[-99, -91])")
        print(f"[ex15]   final arm_boom = {ar8:.2f}° (∈[26, 34])")
        ac8 = (
            bool(res8.success)
            and (4 <= n8 <= 6)
            and all_ok8
            and (-99.0 <= bk8 <= -91.0)
            and (26.0 <= ar8 <= 34.0)
        )

    # 输出总表
    print()
    print("=" * 60)
    print("[ex15] === THREE-CONTROL-INTERFACES SUMMARY ===")
    print(f"[ex15] AC-6 接口1 move_to_point FK err≤5cm: {'PASS' if ac6 else 'FAIL'}")
    print(f"[ex15] AC-7 接口2 dump_transit z/swing 范围 : {'PASS' if ac7 else 'FAIL'}")
    print(f"[ex15] AC-8 接口3 dump_material 步骤/角度  : {'PASS' if ac8 else 'FAIL'}")

    all_pass = ac6 and ac7 and ac8
    print("PASS" if all_pass else "FAIL", file=sys.stdout if all_pass else sys.stderr)
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
