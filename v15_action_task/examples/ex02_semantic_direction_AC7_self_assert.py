import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

from v15_action_task import from_config, swing_move, boom_move, arm_move, bucket_move, default_pose_deg
from v15_action_task.action_library import INIT_POSE


def main() -> int:
    ctx = from_config(adapter_backend="mock")
    ctl = ctx["controller"]

    tol = 0.5
    passed = 0
    total = 8

    with ctl:
        ctl.set_pose(dict(INIT_POSE))
        init_pose = ctl.get_pose_or_default()
        print(f"[ex02] INIT_POSE applied: swing={init_pose.get('swing_yaw',0):.2f} boom={init_pose.get('boom_swing',0):.2f} arm={init_pose.get('arm_boom',0):.2f} bucket={init_pose.get('bucket_arm',0):.2f}")

        # ====== 正向 4 测试 ======
        print("\n--- Positive direction tests (FR-2.1 semantics) ---")

        # T1: swing_move(right=True, 5) → swing_yaw += 5 (+5)
        ctl.set_pose(dict(INIT_POSE))
        r = swing_move(ctl, right_or_left=True, angle_deg=5.0, blocking=True, tolerance_deg=0.1, timeout_s=2.0)
        p = ctl.get_pose_or_default()
        delta = float(p.get("swing_yaw", 0.0)) - float(INIT_POSE.get("swing_yaw", 0.0))
        ok = r.success and abs(delta - 5.0) < tol
        print(f"T1 swing(right=True,5): delta={delta:+.2f} expected=+5.0 success={r.success} -> {'PASS' if ok else 'FAIL'}")
        if ok: passed += 1

        # T2: boom_move(up=True, 5) → boom_swing -= 5 (-5)
        ctl.set_pose(dict(INIT_POSE))
        r = boom_move(ctl, up_or_down=True, angle_deg=5.0, blocking=True, tolerance_deg=0.1, timeout_s=2.0)
        p = ctl.get_pose_or_default()
        delta = float(p.get("boom_swing", 0.0)) - float(INIT_POSE.get("boom_swing", 0.0))
        ok = r.success and abs(delta - (-5.0)) < tol
        print(f"T2 boom(up=True,5):    delta={delta:+.2f} expected=-5.0 success={r.success} -> {'PASS' if ok else 'FAIL'}")
        if ok: passed += 1

        # T3: arm_move(up=True, 5) → arm_boom += 5 (+5)
        ctl.set_pose(dict(INIT_POSE))
        r = arm_move(ctl, up_or_down=True, angle_deg=5.0, blocking=True, tolerance_deg=0.1, timeout_s=2.0)
        p = ctl.get_pose_or_default()
        delta = float(p.get("arm_boom", 0.0)) - float(INIT_POSE.get("arm_boom", 0.0))
        ok = r.success and abs(delta - 5.0) < tol
        print(f"T3 arm(up=True,5):     delta={delta:+.2f} expected=+5.0 success={r.success} -> {'PASS' if ok else 'FAIL'}")
        if ok: passed += 1

        # T4: bucket_move(up=True, 5) → bucket_arm += 5 (+5)
        ctl.set_pose(dict(INIT_POSE))
        r = bucket_move(ctl, up_or_down=True, angle_deg=5.0, blocking=True, tolerance_deg=0.1, timeout_s=2.0)
        p = ctl.get_pose_or_default()
        delta = float(p.get("bucket_arm", 0.0)) - float(INIT_POSE.get("bucket_arm", 0.0))
        ok = r.success and abs(delta - 5.0) < tol
        print(f"T4 bucket(up=True,5):  delta={delta:+.2f} expected=+5.0 success={r.success} -> {'PASS' if ok else 'FAIL'}")
        if ok: passed += 1

        # ====== 反向 4 测试 ======
        print("\n--- Opposite direction tests ---")

        # T5: swing_move(right=False, 5) → swing_yaw -= 5 (-5)
        ctl.set_pose(dict(INIT_POSE))
        r = swing_move(ctl, right_or_left=False, angle_deg=5.0, blocking=True, tolerance_deg=0.1, timeout_s=2.0)
        p = ctl.get_pose_or_default()
        delta = float(p.get("swing_yaw", 0.0)) - float(INIT_POSE.get("swing_yaw", 0.0))
        ok = r.success and abs(delta - (-5.0)) < tol
        print(f"T5 swing(right=False,5): delta={delta:+.2f} expected=-5.0 success={r.success} -> {'PASS' if ok else 'FAIL'}")
        if ok: passed += 1

        # T6: boom_move(up=False, 5) → boom_swing += 5 (+5)
        ctl.set_pose(dict(INIT_POSE))
        r = boom_move(ctl, up_or_down=False, angle_deg=5.0, blocking=True, tolerance_deg=0.1, timeout_s=2.0)
        p = ctl.get_pose_or_default()
        delta = float(p.get("boom_swing", 0.0)) - float(INIT_POSE.get("boom_swing", 0.0))
        ok = r.success and abs(delta - 5.0) < tol
        print(f"T6 boom(up=False,5):    delta={delta:+.2f} expected=+5.0 success={r.success} -> {'PASS' if ok else 'FAIL'}")
        if ok: passed += 1

        # T7: arm_move(up=False, 5) → arm_boom -= 5 (-5)
        ctl.set_pose(dict(INIT_POSE))
        r = arm_move(ctl, up_or_down=False, angle_deg=5.0, blocking=True, tolerance_deg=0.1, timeout_s=2.0)
        p = ctl.get_pose_or_default()
        delta = float(p.get("arm_boom", 0.0)) - float(INIT_POSE.get("arm_boom", 0.0))
        ok = r.success and abs(delta - (-5.0)) < tol
        print(f"T7 arm(up=False,5):     delta={delta:+.2f} expected=-5.0 success={r.success} -> {'PASS' if ok else 'FAIL'}")
        if ok: passed += 1

        # T8: bucket_move(up=False, 5) → bucket_arm -= 5 (-5)
        ctl.set_pose(dict(INIT_POSE))
        r = bucket_move(ctl, up_or_down=False, angle_deg=5.0, blocking=True, tolerance_deg=0.1, timeout_s=2.0)
        p = ctl.get_pose_or_default()
        delta = float(p.get("bucket_arm", 0.0)) - float(INIT_POSE.get("bucket_arm", 0.0))
        ok = r.success and abs(delta - (-5.0)) < tol
        print(f"T8 bucket(up=False,5):  delta={delta:+.2f} expected=-5.0 success={r.success} -> {'PASS' if ok else 'FAIL'}")
        if ok: passed += 1

    print(f"\n[ex02] ====================")
    print(f"[ex02] AC-7 RESULT: {passed}/{total} PASS")
    if passed == total:
        print("AC-7 OK")
        return 0
    else:
        print(f"FAIL: {total - passed} test(s) failed", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
