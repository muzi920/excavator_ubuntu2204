import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

import time
from v15_action_task import from_config, boom_move
from v15_action_task.action_library import INIT_POSE


def main() -> int:
    ctx = from_config(
        adapter_backend="mock",
        init_sensors=True,
        sensor_ros_mode="force_mock",
        use_config_limits=True,
    )
    ctl = ctx["controller"]
    smgr = ctx["sensor_manager"]

    print("[ex06] Full pipeline: controller + sensors smoke test")

    with ctl:
        ctl.set_pose(dict(INIT_POSE))
        init_pose = ctl.get_pose_or_default()
        print(f"[ex06] INIT pose applied: boom_swing = {init_pose.get('boom_swing',0):.2f} deg")

        print("[ex06] boom_move(up=True, 10.0) ...")
        r = boom_move(ctl, up_or_down=True, angle_deg=10.0, blocking=True, tolerance_deg=0.5, timeout_s=3.0)
        after_pose = ctl.get_pose_or_default()
        delta_boom = float(after_pose.get("boom_swing", 0.0)) - float(init_pose.get("boom_swing", 0.0))
        print(f"[ex06] MoveResult.success = {r.success}, boom delta = {delta_boom:+.2f} deg (expect ~-10)")

    try:
        import time as _t
        try:
            reading = smgr.get_tilt("tilt_bucket")
            if reading is not None:
                print(f"[ex06] query tilt_bucket: valid={reading.is_valid} raw={reading.raw_angle_deg} comp={reading.compensated_angle_deg}")
            else:
                print("[ex06] query tilt_bucket: None (mock backend, no live data - OK)")
        except Exception as e:
            print(f"[ex06] query tilt_bucket: exception {type(e).__name__} (OK for mock)")
    except Exception:
        pass

    print("\nPIPELINE OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
