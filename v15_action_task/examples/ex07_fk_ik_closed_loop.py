import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

import math
from v15_action_task import from_config


def main() -> int:
    ctx = from_config(adapter_backend="mock")
    fk = ctx["fk"]
    ik = ctx["ik"]

    swing_deg = 10.0
    boom_deg = 30.0
    arm_deg = 50.0
    bucket_deg = -60.0

    print(f"[ex07] Input pose: swing={swing_deg} boom={boom_deg} arm={arm_deg} bucket={bucket_deg}")

    fk_sol = fk.solve(
        boom_swing_deg=boom_deg,
        arm_boom_deg=arm_deg,
        bucket_arm_deg=bucket_deg,
        swing_yaw_deg=swing_deg,
    )
    tip_xyz = fk_sol.bucket_tip_3d
    bucket_abs = fk_sol.abs_bucket_deg
    print(f"[ex07] FK -> tip xyz=({tip_xyz[0]:.4f}, {tip_xyz[1]:.4f}, {tip_xyz[2]:.4f}) m")
    print(f"[ex07] FK -> bucket_abs_angle={bucket_abs:.2f} deg")

    ik_sol = ik.solve_bucket_pose(
        x_m=tip_xyz[0],
        y_m=tip_xyz[1],
        z_m=tip_xyz[2],
        bucket_abs_angle_deg=bucket_abs,
    )

    if ik_sol is None:
        print("[ex07] FAIL: IK returned None (no solution)", file=sys.stderr)
        return 1

    print(f"[ex07] IK -> swing={ik_sol.swing_yaw_deg:.3f} boom={ik_sol.boom_swing_deg:.3f} arm={ik_sol.arm_boom_deg:.3f} bucket={ik_sol.bucket_arm_deg:.3f}")

    diff_swing = abs(ik_sol.swing_yaw_deg - swing_deg)
    diff_boom = abs(ik_sol.boom_swing_deg - boom_deg)
    diff_arm = abs(ik_sol.arm_boom_deg - arm_deg)
    diff_bucket = abs(ik_sol.bucket_arm_deg - bucket_deg)

    tol = 1.0
    print(f"[ex07] diffs (tol={tol} deg):")
    print(f"[ex07]   swing:  |{ik_sol.swing_yaw_deg:.3f} - {swing_deg:.3f}| = {diff_swing:.4f} deg")
    print(f"[ex07]   boom:   |{ik_sol.boom_swing_deg:.3f} - {boom_deg:.3f}| = {diff_boom:.4f} deg")
    print(f"[ex07]   arm:    |{ik_sol.arm_boom_deg:.3f} - {arm_deg:.3f}| = {diff_arm:.4f} deg")
    print(f"[ex07]   bucket: |{ik_sol.bucket_arm_deg:.3f} - {bucket_deg:.3f}| = {diff_bucket:.4f} deg")

    ok = (
        diff_swing < tol
        and diff_boom < tol
        and diff_arm < tol
        and diff_bucket < tol
    )

    if ok:
        print("OK: closed-loop FK->IK diff < 1.0 deg on all 4 joints")
        return 0
    else:
        print("FAIL: closed-loop error exceeds tolerance on at least one joint", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
