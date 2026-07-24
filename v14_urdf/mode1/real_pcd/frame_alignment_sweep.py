import argparse
import json
from pathlib import Path

import numpy as np

from pcd_numpy_io import read_pcd_xyz, write_pcd_xyz, write_pcd_xyzrgb
from workspace_volume_and_fuse import _collect_limits, _compute_chain_fk, _linspace, _load_urdf_joints


def _make_workspace(urdf_path, swing_samples, boom_samples, arm_samples, bucket_samples, workspace_z_max, max_points):
    joints = _load_urdf_joints(urdf_path)
    lim = _collect_limits(joints, ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"])
    swing_vals = _linspace(lim["swing_joint"][0], lim["swing_joint"][1], int(swing_samples))
    boom_vals = _linspace(lim["boom_joint"][0], lim["boom_joint"][1], int(boom_samples))
    arm_vals = _linspace(lim["arm_joint"][0], lim["arm_joint"][1], int(arm_samples))
    bucket_vals = _linspace(lim["bucket_joint"][0], lim["bucket_joint"][1], int(bucket_samples))

    pts = []
    for swing in swing_vals:
        base_positions = {"swing_joint": swing}
        for boom in boom_vals:
            for arm in arm_vals:
                for bucket in bucket_vals:
                    pos = {
                        **base_positions,
                        "boom_joint": boom,
                        "arm_joint": arm,
                        "bucket_joint": bucket,
                    }
                    x, y, z = _compute_chain_fk(joints, pos)
                    if workspace_z_max is not None and z > float(workspace_z_max):
                        continue
                    pts.append((x, y, z))

    workspace = np.array(pts, dtype=np.float32)
    if workspace.size == 0:
        raise ValueError("workspace 为空，请检查采样参数或 workspace_z_max。")
    if max_points is not None and workspace.shape[0] > int(max_points):
        step = max(1, workspace.shape[0] // int(max_points))
        workspace = workspace[::step]
    return workspace


def _color_rgb(points_count, r, g, b):
    return np.tile(np.array([[r, g, b]], dtype=np.uint8), (int(points_count), 1))


def _downsample(points, max_points):
    if max_points is None or points.shape[0] <= int(max_points):
        return points
    step = max(1, points.shape[0] // int(max_points))
    return points[::step]


def _transform(points, mode):
    if mode == "identity":
        return points
    if mode == "flip_x":
        out = points.copy()
        out[:, 0] *= -1.0
        return out
    if mode == "flip_y":
        out = points.copy()
        out[:, 1] *= -1.0
        return out
    if mode == "flip_z":
        out = points.copy()
        out[:, 2] *= -1.0
        return out
    if mode == "flip_xy":
        out = points.copy()
        out[:, 0] *= -1.0
        out[:, 1] *= -1.0
        return out
    if mode == "flip_xz":
        out = points.copy()
        out[:, 0] *= -1.0
        out[:, 2] *= -1.0
        return out
    if mode == "flip_yz":
        out = points.copy()
        out[:, 1] *= -1.0
        out[:, 2] *= -1.0
        return out
    if mode == "swap_xy":
        out = points.copy()
        out[:, [0, 1]] = out[:, [1, 0]]
        return out
    if mode == "swap_xz":
        out = points.copy()
        out[:, [0, 2]] = out[:, [2, 0]]
        return out
    if mode == "swap_yz":
        out = points.copy()
        out[:, [1, 2]] = out[:, [2, 1]]
        return out
    raise ValueError(f"unknown mode: {mode}")


def main():
    parser = argparse.ArgumentParser(description="生成多种坐标轴翻转/交换后的点云，用于排查坐标系倒置问题。")
    parser.add_argument("--pcd", required=True, help="输入点云（建议用 original_minus_removed.pcd）。")
    parser.add_argument("--out-dir", required=True, help="输出目录。")
    parser.add_argument(
        "--urdf",
        default="src/shandong/v14_urdf/describe_60FED/urdf/describe_60FED_calibrated.urdf",
        help="用于生成 workspace 的 URDF。",
    )
    parser.add_argument("--workspace-z-max", type=float, default=0.5, help="只显示 z<=该值的可作业域。")
    parser.add_argument("--swing-samples", type=int, default=72)
    parser.add_argument("--boom-samples", type=int, default=20)
    parser.add_argument("--arm-samples", type=int, default=20)
    parser.add_argument("--bucket-samples", type=int, default=16)
    parser.add_argument("--max-workspace-points", type=int, default=180000)
    parser.add_argument("--max-original-points", type=int, default=60000)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    original = read_pcd_xyz(args.pcd)
    original = _downsample(original, args.max_original_points)

    workspace = _make_workspace(
        urdf_path=args.urdf,
        swing_samples=args.swing_samples,
        boom_samples=args.boom_samples,
        arm_samples=args.arm_samples,
        bucket_samples=args.bucket_samples,
        workspace_z_max=args.workspace_z_max,
        max_points=args.max_workspace_points,
    )
    workspace_rgb = _color_rgb(workspace.shape[0], 0, 255, 0)
    write_pcd_xyzrgb(out_dir / "workspace_zmax_rgb.pcd", workspace, workspace_rgb)

    modes = [
        "identity",
        "flip_y",
        "flip_z",
        "flip_yz",
        "swap_xy",
        "swap_yz",
        "swap_xz",
    ]

    gray = _color_rgb(original.shape[0], 180, 180, 180)
    results = []
    for mode in modes:
        transformed = _transform(original, mode)
        write_pcd_xyz(out_dir / f"original_{mode}.pcd", transformed)
        fused_xyz = np.concatenate([transformed, workspace], axis=0)
        fused_rgb = np.concatenate([gray, workspace_rgb], axis=0)
        write_pcd_xyzrgb(out_dir / f"fused_{mode}_rgb.pcd", fused_xyz, fused_rgb)
        mins = transformed.min(axis=0).tolist()
        maxs = transformed.max(axis=0).tolist()
        results.append({"mode": mode, "min": mins, "max": maxs})

    print(
        json.dumps(
            {
                "out_dir": str(out_dir.resolve()),
                "modes": results,
                "workspace_points": int(workspace.shape[0]),
                "original_points_used": int(original.shape[0]),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()

