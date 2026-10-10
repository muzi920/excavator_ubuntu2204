import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

from v15_action_task import from_config, MoveResult


def main() -> int:
    ctx = from_config(adapter_backend="mock")
    controller = ctx["controller"]
    mover = ctx["mover"]

    target_x = 0.9
    target_y = 0.0
    target_z = -0.2
    bucket_angle = -60.0

    print(f"[ex08] CartesianMover demo:")
    print(f"[ex08]   move_with_bucket({target_x}, {target_y}, {target_z}, {bucket_angle})")

    with controller:
        result: MoveResult = mover.move_with_bucket(
            target_x, target_y, target_z, bucket_angle,
            blocking=True,
        )

    print(f"[ex08] MoveResult.success = {result.success}")
    print(f"[ex08] bool(MoveResult)    = {bool(result)}")
    if result.reached_pose_deg:
        p = result.reached_pose_deg
        print(f"[ex08] reached pose: swing={p.get('swing_yaw',0):.2f} boom={p.get('boom_swing',0):.2f} arm={p.get('arm_boom',0):.2f} bucket={p.get('bucket_arm',0):.2f}")
    if result.final_tip_xyz:
        print(f"[ex08] final tip: ({result.final_tip_xyz[0]:.4f}, {result.final_tip_xyz[1]:.4f}, {result.final_tip_xyz[2]:.4f}) m")
    if result.reason:
        print(f"[ex08] reason: {result.reason}")

    if bool(result):
        print("PASS")
        return 0
    else:
        print("FAIL", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
