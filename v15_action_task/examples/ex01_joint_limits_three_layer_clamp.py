import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

from v15_action_task import from_config, boom_move
import copy


def main() -> int:
    ctx = from_config(adapter_backend="mock", use_config_limits=True)
    cfg = ctx["config"]
    ctl = ctx["controller"]

    original_max = cfg.limits.limits["boom_swing"].max_deg
    cfg.limits.limits["boom_swing"].max_deg = 55.0
    print(f"[ex01] Set boom_swing max_deg -> 55.0 (was {original_max})")

    with ctl:
        cur_before = ctl.get_pose_or_default()
        boom_before = float(cur_before.get("boom_swing", 0.0))
        print(f"[ex01] Initial boom_swing = {boom_before} deg")

        result = boom_move(ctl, up=False, angle_deg=999.0, cfg=cfg)
        pose_after = ctl.get_pose_or_default()
        boom_after = float(pose_after.get("boom_swing", 0.0))

        clamped_expected = min(boom_before + 999.0, 55.0)
        print(f"[ex01] boom_move(up=False, 999) -> reached boom_swing = {boom_after} deg")
        print(f"[ex01] MoveResult.success = {result.success}")
        print(f"[ex01] Clamped expected (Layer 3) = {clamped_expected}")

        clamp_ok = abs(boom_after - clamped_expected) < 0.5
        success_ok = bool(result.success)

        print(f"[ex01] Check: clamp_result_ok={clamp_ok}, result_success={success_ok}")

        if clamp_ok and success_ok:
            print("PASS")
            return 0
        else:
            print("FAIL", file=sys.stderr)
            return 1


if __name__ == "__main__":
    sys.exit(main())
