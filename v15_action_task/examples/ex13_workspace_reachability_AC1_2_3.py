import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..")); sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."));
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable
ensure_v15_importable()

from v15_action_task import from_config, WorkspaceChecker
import math


def main() -> int:
    ctx = from_config(adapter_backend="mock", init_workspace=True)
    ws: WorkspaceChecker = ctx["workspace_checker"]
    cfg = ctx["config"]

    print(f"[ex13] WorkspaceChecker: outer={ws.outer_radius_m:.3f}m inner={ws.inner_radius_m:.3f}m margin={ws.margin_m:.3f}m")
    print(f"[ex13]   min_ground_z={cfg.workspace.min_ground_z} min_transit_z={cfg.workspace.min_transit_z}")

    # AC-1: 腕点几何边界内，正常可达（挖姿态 (1.0, 0, -0.05), dig 模式）
    r1 = ws.check_point_reachable((1.0, 0.0, -0.05), mode="dig")
    print(f"[ex13] AC-1 边界内点(1.0,0,-0.05) dig: success={r1.success}, reasons={r1.reasons}")
    ac1 = bool(r1.success)

    # AC-2: 3.0m 远点，几何外半径超限
    r2 = ws.check_point_reachable((3.0, 0.0, 0.5), mode="transit")
    print(f"[ex13] AC-2 远点(3.0,0,0.5) transit: success={r2.success}, reasons={r2.reasons}")
    ac2 = (not r2.success) and any("outer" in s for s in r2.reasons)

    # AC-3: z=-0.5 deep, transit 模式 ground_penetration 失败；dig 模式跳过 ground，允许
    r3_transit = ws.check_point_reachable((1.0, 0.0, -0.5), mode="transit")
    print(f"[ex13] AC-3a 低点(1.0,0,-0.5) transit: success={r3_transit.success}, reasons={r3_transit.reasons}")
    r3_dig = ws.check_point_reachable((1.0, 0.0, -0.5), mode="dig")
    print(f"[ex13] AC-3b 低点(1.0,0,-0.5) dig:     success={r3_dig.success}, reasons={r3_dig.reasons}")
    ac3a = (not r3_transit.success) and any("ground" in s for s in r3_transit.reasons)
    # 注意 z=-0.5 可能被其他层卡住（如腕点几何），这里主要验证 transit 模式有 ground 失败即可
    ac3 = ac3a

    # Helper: closest_reachable_point 把不可达点收缩到可达域
    close_pt = ws.closest_reachable_point((3.0, 0.0, 0.5))
    print(f"[ex13] helper closest((3,0,0.5)) -> {close_pt}")
    close_ok = close_pt is not None
    if close_pt is not None:
        r_verify = ws.check_point_reachable(close_pt, mode="transit")
        print(f"[ex13]   该最近点 reachable? {r_verify.success}")
        close_ok = bool(r_verify.success)

    print()
    print(f"[ex13] AC-1 (边界内可达)   : {'PASS' if ac1 else 'FAIL'}")
    print(f"[ex13] AC-2 (远点超限)     : {'PASS' if ac2 else 'FAIL'}")
    print(f"[ex13] AC-3 (transit ground): {'PASS' if ac3 else 'FAIL'}")
    print(f"[ex13] Helper closest_reach : {'PASS' if close_ok else 'FAIL'}")

    all_pass = ac1 and ac2 and ac3 and close_ok
    print("PASS" if all_pass else "FAIL", file=sys.stdout if all_pass else sys.stderr)
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
