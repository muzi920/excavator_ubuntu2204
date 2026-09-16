import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

import math
from v15_action_task import load_default_config, ForwardKinematics
from v15_action_task.kinematics import LinkParams


def main() -> int:
    cfg = load_default_config()
    link_cfg = cfg.link

    lp_old = link_cfg.to_link_params()
    print(f"[ex10] Original L1={lp_old.L1*1000:.0f}mm L2={lp_old.L2*1000:.0f}mm -> L_boom={lp_old.L_boom*1000:.1f}mm")

    delta_L2 = 0.240
    new_L1 = lp_old.L1
    new_L2 = lp_old.L2 + delta_L2
    new_L_boom, new_beta = LinkParams.compute_boom_equiv(new_L1, new_L2, lp_old.boom_bend_angle_deg)

    lp_new = LinkParams(
        offset_x=lp_old.offset_x,
        offset_z=lp_old.offset_z,
        L1=new_L1,
        L2=new_L2,
        boom_bend_angle_deg=lp_old.boom_bend_angle_deg,
        L_arm=lp_old.L_arm,
        L_bucket=lp_old.L_bucket,
        offset_sensor_boom=lp_old.offset_sensor_boom,
        offset_sensor_arm=lp_old.offset_sensor_arm,
        offset_sensor_bucket=lp_old.offset_sensor_bucket,
        L_boom=new_L_boom,
        beta_deg=new_beta,
    )

    print(f"[ex10] Modified L1={lp_new.L1*1000:.0f}mm L2={lp_new.L2*1000:.0f}mm (+{delta_L2*1000:.0f}mm) -> L_boom={lp_new.L_boom*1000:.1f}mm (+{(lp_new.L_boom-lp_old.L_boom)*1000:.1f}mm)")

    fk_old = ForwardKinematics(params=lp_old)
    fk_new = ForwardKinematics(params=lp_new)

    test_pose = {
        "boom_swing_deg": 0.0,
        "arm_boom_deg": 0.0,
        "bucket_arm_deg": 0.0,
        "swing_yaw_deg": 0.0,
    }
    print(f"[ex10] Test pose (fully stretched): boom=0 arm=0 bucket=0 swing=0")

    sol_old = fk_old.solve(**test_pose)
    sol_new = fk_new.solve(**test_pose)

    tip_old = sol_old.bucket_tip_3d
    tip_new = sol_new.bucket_tip_3d

    print(f"[ex10] OLD tip: ({tip_old[0]:.4f}, {tip_old[1]:.4f}, {tip_old[2]:.4f}) m")
    print(f"[ex10] NEW tip: ({tip_new[0]:.4f}, {tip_new[1]:.4f}, {tip_new[2]:.4f}) m")

    dx = tip_new[0] - tip_old[0]
    dy = tip_new[1] - tip_old[1]
    dz = tip_new[2] - tip_old[2]
    dist_m = math.sqrt(dx * dx + dy * dy + dz * dz)
    dist_mm = dist_m * 1000.0

    print(f"[ex10] tip delta dX={dx*1000:.1f} dY={dy*1000:.1f} dZ={dz*1000:.1f} mm")
    print(f"[ex10] |tip delta L| = {dist_mm:.2f} mm")
    print(f"[ex10] TR5.5 accept range: [235.0, 245.0] mm")

    min_mm = 235.0
    max_mm = 245.0
    ok = min_mm <= dist_mm <= max_mm

    if ok:
        print(f"OK: deltaL={dist_mm:.2f} mm within TR5.5 range [{min_mm}, {max_mm}]")
        return 0
    else:
        print(f"FAIL: deltaL={dist_mm:.2f} mm outside TR5.5 range [{min_mm}, {max_mm}]", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
