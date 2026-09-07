import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

import time
from v15_action_task import from_config
from v15_action_task.sensors import TiltReading, LidarReading, CameraReading


def try_inject_tilt(smgr, sid, angle_deg):
    try:
        reading = TiltReading(
            id=sid,
            ts_s=float(time.time()),
            is_valid=True,
            raw_angle_deg=float(angle_deg),
            compensated_angle_deg=float(angle_deg),
            calibration_done=True,
            calibration_remaining=0,
        )
        with smgr._lock:
            smgr._tilt_cache[sid] = reading
        return True
    except Exception:
        return False


def try_inject_lidar(smgr, sid, points=1000):
    try:
        reading = LidarReading(
            id=sid,
            ts_s=float(time.time()),
            is_valid=True,
            points_count=int(points),
            width_or_none=100,
            height_or_none=10,
            raw_ref_or_none=None,
        )
        with smgr._lock:
            smgr._lidar_cache[sid] = reading
        return True
    except Exception:
        return False


def try_inject_camera(smgr, sid, w=640, h=480):
    try:
        reading = CameraReading(
            id=sid,
            ts_s=float(time.time()),
            is_valid=True,
            width_px=int(w),
            height_px=int(h),
            encoding="rgb8",
            seq_or_none=1,
            raw_ref_or_none=None,
        )
        with smgr._lock:
            smgr._camera_cache[sid] = reading
        return True
    except Exception:
        return False


def main() -> int:
    ctx = from_config(
        adapter_backend="mock",
        init_sensors=True,
        sensor_ros_mode="force_mock",
    )
    smgr = ctx["sensor_manager"]

    tilt_ids = smgr.list_tilt_ids()
    lidar_ids = smgr.list_lidar_ids()
    camera_ids = smgr.list_camera_ids()

    n_tilt = len(tilt_ids)
    n_lidar = len(lidar_ids)
    n_camera = len(camera_ids)
    total = n_tilt + n_lidar + n_camera

    print(f"[ex04] Tilt sensors  : {n_tilt} -> {tilt_ids}")
    print(f"[ex04] Lidar sensors : {n_lidar} -> {lidar_ids}")
    print(f"[ex04] Camera sensors: {n_camera} -> {camera_ids}")
    print(f"[ex04] Total channels: {total} (expect tilt+lidar+camera = 4+5+6=15)")

    expected_tilt = 4
    expected_lidar = 5
    expected_camera = 6
    expected_total = expected_tilt + expected_lidar + expected_camera

    channel_ok = (
        n_tilt >= expected_tilt
        and n_lidar >= expected_lidar
        and n_camera >= expected_camera
        and total >= expected_total
    )
    print(f"[ex04] Channel count check: {'OK' if channel_ok else 'FAIL'}")

    injected_count = 0
    for sid in tilt_ids:
        if try_inject_tilt(smgr, sid, 10.0 + float(ord(sid[-1]))):
            injected_count += 1
    for sid in lidar_ids:
        if try_inject_lidar(smgr, sid, 2000):
            injected_count += 1
    for sid in camera_ids:
        if try_inject_camera(smgr, sid, 640, 480):
            injected_count += 1

    print(f"[ex04] Synthetic readings injected: {injected_count}/{total} (via private API if available)")

    all_tilt_dict = smgr.all_tilt()
    all_tilt_count = len(all_tilt_dict)
    print(f"[ex04] all_tilt() returned {all_tilt_count} readings: {list(all_tilt_dict.keys())}")

    tilt_readings_ok = all_tilt_count >= expected_tilt

    for sid, rd in all_tilt_dict.items():
        print(f"[ex04]   {sid}: valid={rd.is_valid} raw={rd.raw_angle_deg:.1f} comp={rd.compensated_angle_deg}")

    print(f"\n[ex04] all_tilt list count: {'OK' if tilt_readings_ok else 'FAIL'} (have {all_tilt_count}, expect >= {expected_tilt})")

    if channel_ok and tilt_readings_ok:
        print("PASS")
        return 0
    else:
        print("FAIL", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
