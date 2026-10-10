import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

from v15_action_task import (
    from_config,
    swing_move, boom_move, arm_move, bucket_move,
    swing_move_to, boom_lift, arm_extend, bucket_tilt_to,
    MoveResult,
)
from v15_action_task.action_library import INIT_POSE


def main() -> int:
    ctx = from_config(adapter_backend="mock")
    ctl = ctx["controller"]
    mover = ctx["mover"]

    results = []

    with ctl:
        ctl.set_pose(dict(INIT_POSE))

        r1 = swing_move(ctl, right_or_left=True, angle_deg=3.0, blocking=True, tolerance_deg=0.5, timeout_s=2.0)
        results.append(("swing_move(right=True,3)", r1))

        r2 = boom_move(ctl, up_or_down=True, angle_deg=3.0, blocking=True, tolerance_deg=0.5, timeout_s=2.0)
        results.append(("boom_move(up=True,3)", r2))

        r3 = arm_move(ctl, up_or_down=True, angle_deg=3.0, blocking=True, tolerance_deg=0.5, timeout_s=2.0)
        results.append(("arm_move(up=True,3)", r3))

        r4 = bucket_move(ctl, up_or_down=True, angle_deg=3.0, blocking=True, tolerance_deg=0.5, timeout_s=2.0)
        results.append(("bucket_move(up=True,3)", r4))

        r5 = swing_move_to(ctl, abs_yaw_deg=0.0, blocking=True, tolerance_deg=0.5, timeout_s=2.0)
        results.append(("swing_move_to(0.0)", r5))

        r6 = boom_lift(mover, delta_z_m=0.05, blocking=True)
        results.append(("boom_lift(+0.05m)", r6))

        r7 = arm_extend(mover, delta_x_m=0.05, blocking=True)
        results.append(("arm_extend(+0.05m)", r7))

        r8 = bucket_tilt_to(ctl, abs_bucket_tip_deg=-30.0, blocking=True, tolerance_deg=0.5, timeout_s=2.0)
        results.append(("bucket_tilt_to(-30.0)", r8))

    print(f"{'#':<2} {'Command':<32} {'MoveResult.success'}")
    print("-" * 60)
    all_ok = True
    for i, (name, res) in enumerate(results, 1):
        ok = bool(res.success)
        if not ok:
            all_ok = False
        print(f"{i:<2} {name:<32} {ok}")

    print()
    if all_ok:
        print("[ex03] All 8 semantic commands returned MoveResult.success=True")
        return 0
    else:
        print("[ex03] Some commands failed", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
