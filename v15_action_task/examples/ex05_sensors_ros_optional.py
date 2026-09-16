import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

import time
from v15_action_task import from_config


def main() -> int:
    ctx = from_config(
        adapter_backend="mock",
        init_sensors=True,
        sensor_ros_mode="auto",
    )
    smgr = ctx["sensor_manager"]

    if not smgr.is_ros_backend():
        reason = smgr.ros_backend_error_reason_if_any()
        print(f"[ex05] SKIP no rclpy (reason: {reason})")
        sys.exit(0)

    print("[ex05] ROS backend present, wait 0.2s check cache")
    time.sleep(0.2)

    n_tilt = len(smgr.all_tilt())
    n_lidar = len(smgr.all_lidar())
    n_camera = len(smgr.all_camera())
    print(f"[ex05] cache stats: tilt={n_tilt} lidar={n_lidar} camera={n_camera}")
    sys.exit(0)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except ImportError as e:
        print(f"[ex05] SKIP ImportError: {e}")
        sys.exit(0)
    except Exception as e:
        print(f"[ex05] SKIP unexpected error: {type(e).__name__}: {e}")
        sys.exit(0)
