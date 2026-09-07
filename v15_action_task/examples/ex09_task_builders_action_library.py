import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

from v15_action_task.action_library import build_single_dig_dump_script, INIT_POSE, StepBuilder


def main() -> int:
    print("[ex09] action_library demo: build_single_dig_dump_script (no run)")
    print(f"[ex09] INIT_POSE = {dict(INIT_POSE)}")

    dig_pose = {
        "swing_yaw": 0.0,
        "boom_swing": 40.0,
        "arm_boom": 90.0,
        "bucket_arm": -70.0,
    }
    dump_pose = {
        "swing_yaw": 60.0,
        "boom_swing": 25.0,
        "arm_boom": 50.0,
        "bucket_arm": 10.0,
    }

    steps = []
    try:
        sb = StepBuilder()
        steps = build_single_dig_dump_script(
            sb,
            dig_pose,
            dump_pose,
            dig_point=(1.0, 0.0, -0.25),
            dump_point=(0.6, 0.9, 0.1),
            dwell_after_dump_s=0.4,
        )
    except ImportError as e:
        print(f"[ex09] NOTE: build_single_dig_dump_script raised ImportError (expected if IK solver unavailable): {e}")
        steps = []
    except Exception as e:
        print(f"[ex09] NOTE: build_single_dig_dump_script raised {type(e).__name__}: {e}")
        steps = []

    print(f"[ex09] len(steps) = {len(steps)}")
    for i, s in enumerate(steps[:5]):
        desc = s.get("description", s.get("name", "?"))
        print(f"[ex09]   step#{i}: {desc}")
    if len(steps) > 5:
        print(f"[ex09]   ... and {len(steps) - 5} more steps")

    return 0


if __name__ == "__main__":
    sys.exit(main())
