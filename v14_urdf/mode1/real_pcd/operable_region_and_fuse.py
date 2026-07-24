import argparse
import json
from pathlib import Path

import numpy as np

from pcd_numpy_io import read_pcd_xyz, write_pcd_xyz, write_pcd_xyzrgb


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _estimate_ground_z(points):
    if points.size == 0:
        return 0.0
    z = points[:, 2].astype(np.float32, copy=False)
    z = z[np.isfinite(z)]
    if z.size == 0:
        return 0.0
    near = z[(z >= -0.5) & (z <= 0.5)]
    if near.size == 0:
        return float(np.quantile(z, 0.05))
    zr = np.round(near, 3)
    values, counts = np.unique(zr, return_counts=True)
    return float(values[int(np.argmax(counts))])


def _load_operable_points(constraints_path, z_value):
    constraints = _load_json(constraints_path)
    zslice = constraints.get("bucket_tip_zslice_bounds", {})
    pts = zslice.get("slice_bin_points", [])
    if not pts:
        raise ValueError("constraints 缺少 bucket_tip_zslice_bounds.slice_bin_points。")
    xy = np.array([[float(p["x"]), float(p["y"])] for p in pts], dtype=np.float32)
    z = np.full((xy.shape[0], 1), float(z_value), dtype=np.float32)
    return np.concatenate([xy, z], axis=1)


def main():
    parser = argparse.ArgumentParser(description="生成可作业区域 PCD，并与原始点云融合输出。")
    parser.add_argument(
        "--original-pcd",
        required=True,
        help="原始点云（建议用 original_minus_removed.pcd / kept_points.pcd）。",
    )
    parser.add_argument(
        "--constraints-json",
        default="src/shandong/v14_urdf/mode1/constraints/workspace_constraints_360_z0.json",
        help="包含 slice_bin_points 的 constraints JSON。",
    )
    parser.add_argument("--out-dir", required=True, help="输出目录。")
    parser.add_argument(
        "--operable-z-offset",
        type=float,
        default=0.02,
        help="可作业区域点云叠加时的高度偏移，避免与地面点完全重合。",
    )
    parser.add_argument("--max-original-points", type=int, default=None, help="可选：下采样原始点云用于融合预览。")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    original = read_pcd_xyz(args.original_pcd)
    ground_z = _estimate_ground_z(original)
    operable = _load_operable_points(args.constraints_json, ground_z + float(args.operable_z_offset))

    operable_green = out_dir / "operable_region_green.pcd"
    operable_rgb = np.tile(np.array([[0, 255, 0]], dtype=np.uint8), (operable.shape[0], 1))
    write_pcd_xyzrgb(operable_green, operable, operable_rgb)

    fused_out = out_dir / "fused_original_plus_operable_rgb.pcd"
    if args.max_original_points is not None and len(original) > int(args.max_original_points):
        step = max(1, len(original) // int(args.max_original_points))
        original_ds = original[::step]
    else:
        original_ds = original

    fused_xyz = np.concatenate([original_ds, operable], axis=0)
    gray = np.tile(np.array([[180, 180, 180]], dtype=np.uint8), (original_ds.shape[0], 1))
    fused_rgb = np.concatenate([gray, operable_rgb], axis=0)
    write_pcd_xyzrgb(fused_out, fused_xyz, fused_rgb)

    operable_xyz_only = out_dir / "operable_region_xyz.pcd"
    write_pcd_xyz(operable_xyz_only, operable)

    print(
        json.dumps(
            {
                "original_pcd": str(Path(args.original_pcd).resolve()),
                "constraints": str(Path(args.constraints_json).resolve()),
                "ground_z": float(ground_z),
                "operable_points": int(len(operable)),
                "original_points_used": int(len(original_ds)),
                "operable_region_green": str(operable_green.resolve()),
                "fused_rgb": str(fused_out.resolve()),
                "operable_xyz_only": str(operable_xyz_only.resolve()),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()

