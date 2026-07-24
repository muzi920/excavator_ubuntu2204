import argparse
import json
from pathlib import Path

import numpy as np

from pcd_numpy_io import read_pcd_xyz, write_pcd_xyz


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


def _build_xy_mask(constraints):
    zslice = constraints.get("bucket_tip_zslice_bounds", {})
    xy_bin = float(zslice["xy_bin"])
    pts = zslice.get("slice_bin_points", [])
    if not pts:
        raise ValueError("constraints 缺少 bucket_tip_zslice_bounds.slice_bin_points，无法做精确 ROI。")
    valid = {
        (int(round(float(p["x"]) / xy_bin)), int(round(float(p["y"]) / xy_bin))) for p in pts
    }
    return {
        "xy_bin": xy_bin,
        "valid_bins": valid,
        "bounds": {
            "min_x": float(zslice["min_x"]),
            "max_x": float(zslice["max_x"]),
            "min_y": float(zslice["min_y"]),
            "max_y": float(zslice["max_y"]),
        },
    }


def main():
    parser = argparse.ArgumentParser(description="从点云中提取“工作区域 ROI + 离地面一定高度以内”的点并导出 PCD。")
    parser.add_argument("--pcd", required=True, help="输入 PCD 文件（建议是 original_minus_removed / kept_points）。")
    parser.add_argument(
        "--constraints-json",
        default="src/shandong/v14_urdf/mode1/constraints/workspace_constraints_360_z0.json",
        help="工作区域 constraints JSON（包含 slice_bin_points）。",
    )
    parser.add_argument("--out", required=True, help="输出 PCD 文件路径。")
    parser.add_argument("--height", type=float, default=0.2, help="离地面高度上限（m）。")
    parser.add_argument("--ground-z", type=float, default=None, help="可选：手动指定 ground_z。")
    parser.add_argument("--roi-front-x-min", type=float, default=None, help="可选：只保留 x>=该值（前方 ROI）。")
    parser.add_argument("--roi-y-abs-max", type=float, default=None, help="可选：只保留 |y|<=该值（左右 ROI）。")
    args = parser.parse_args()

    points = read_pcd_xyz(args.pcd)
    constraints = _load_json(args.constraints_json)
    roi = _build_xy_mask(constraints)

    ground_z = float(args.ground_z) if args.ground_z is not None else _estimate_ground_z(points)
    z_min = ground_z
    z_max = ground_z + float(args.height)

    b = roi["bounds"]
    mask = (points[:, 0] >= b["min_x"]) & (points[:, 0] <= b["max_x"])
    mask &= (points[:, 1] >= b["min_y"]) & (points[:, 1] <= b["max_y"])
    mask &= (points[:, 2] >= z_min) & (points[:, 2] <= z_max)

    if args.roi_front_x_min is not None:
        mask &= points[:, 0] >= float(args.roi_front_x_min)
    if args.roi_y_abs_max is not None:
        mask &= np.abs(points[:, 1]) <= float(args.roi_y_abs_max)

    pts = points[mask]
    if pts.size == 0:
        raise ValueError("ROI 后没有点。你可能需要放宽 height/ROI 或检查 constraints 坐标系是否匹配。")

    xy_bin = roi["xy_bin"]
    bx = np.round(pts[:, 0] / xy_bin).astype(np.int32)
    by = np.round(pts[:, 1] / xy_bin).astype(np.int32)
    keys = list(zip(bx.tolist(), by.tolist()))
    valid = roi["valid_bins"]
    keep_mask = np.array([k in valid for k in keys], dtype=bool)
    pts = pts[keep_mask]

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    write_pcd_xyz(out_path, pts)

    print(
        json.dumps(
            {
                "in_pcd": str(Path(args.pcd).resolve()),
                "constraints": str(Path(args.constraints_json).resolve()),
                "out": str(out_path.resolve()),
                "ground_z": float(ground_z),
                "z_min": float(z_min),
                "z_max": float(z_max),
                "height": float(args.height),
                "roi_front_x_min": args.roi_front_x_min,
                "roi_y_abs_max": args.roi_y_abs_max,
                "out_points": int(len(pts)),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()

