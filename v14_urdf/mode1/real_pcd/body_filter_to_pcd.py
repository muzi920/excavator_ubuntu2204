import argparse
from pathlib import Path

import numpy as np

from body_filter import filter_excavator_body
from pcd_numpy_io import read_pcd_xyz, write_pcd_xyz


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


def main():
    parser = argparse.ArgumentParser(description="对单帧 PCD 做本体屏蔽，并输出过滤后的 PCD。")
    parser.add_argument("--pcd", required=True, help="输入 PCD 文件路径。")
    parser.add_argument("--out-dir", required=True, help="输出目录。")
    parser.add_argument("--heap-z-min", type=float, default=0.05, help="土堆点带下界（输出 heap_band.pcd）。")
    parser.add_argument("--heap-z-max", type=float, default=0.5, help="土堆点带上界（输出 heap_band.pcd）。")
    parser.add_argument(
        "--heap-relative-ground",
        action="store_true",
        help="把土堆点带定义成 ground_z + [heap_z_min, heap_z_max]。",
    )
    parser.add_argument("--roi-front-x-min", type=float, default=None, help="可选：只保留 x >= 该值的土堆点。")
    parser.add_argument("--roi-y-abs-max", type=float, default=None, help="可选：只保留 |y| <= 该值的土堆点。")

    parser.add_argument("--center-radius", type=float, default=0.7)
    parser.add_argument("--z-min", type=float, default=0.0)
    parser.add_argument("--z-max", type=float, default=1.8)

    parser.add_argument("--box-x-min", type=float, default=-0.4)
    parser.add_argument("--box-x-max", type=float, default=0.7)
    parser.add_argument("--box-y-abs", type=float, default=0.55)
    parser.add_argument("--box-z-min", type=float, default=0.0)
    parser.add_argument("--box-z-max", type=float, default=0.9)

    parser.add_argument("--arm-enabled", action="store_true")
    parser.add_argument("--arm-x-min", type=float, default=0.2)
    parser.add_argument("--arm-x-max", type=float, default=1.3)
    parser.add_argument("--arm-y-abs", type=float, default=0.35)
    parser.add_argument("--arm-z-min", type=float, default=0.75)
    parser.add_argument("--arm-z-max", type=float, default=1.85)

    args = parser.parse_args()

    points = read_pcd_xyz(args.pcd)
    mask_cfg = {
        "center_radius": args.center_radius,
        "z_min": args.z_min,
        "z_max": args.z_max,
        "box_x_min": args.box_x_min,
        "box_x_max": args.box_x_max,
        "box_y_abs": args.box_y_abs,
        "box_z_min": args.box_z_min,
        "box_z_max": args.box_z_max,
        "arm_enabled": args.arm_enabled,
        "arm_x_min": args.arm_x_min,
        "arm_x_max": args.arm_x_max,
        "arm_y_abs": args.arm_y_abs,
        "arm_z_min": args.arm_z_min,
        "arm_z_max": args.arm_z_max,
    }

    kept, removed, stats = filter_excavator_body(points, mask_cfg)
    ground_z = _estimate_ground_z(kept)
    if args.heap_relative_ground:
        heap_z_min = ground_z + float(args.heap_z_min)
        heap_z_max = ground_z + float(args.heap_z_max)
    else:
        heap_z_min = float(args.heap_z_min)
        heap_z_max = float(args.heap_z_max)

    heap_band = kept[(kept[:, 2] >= heap_z_min) & (kept[:, 2] <= heap_z_max)]
    if args.roi_front_x_min is not None:
        heap_band = heap_band[heap_band[:, 0] >= float(args.roi_front_x_min)]
    if args.roi_y_abs_max is not None:
        heap_band = heap_band[np.abs(heap_band[:, 1]) <= float(args.roi_y_abs_max)]

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    kept_path = out_dir / "kept_points.pcd"
    removed_path = out_dir / "removed_points.pcd"
    heap_path = out_dir / "heap_points_z_band.pcd"

    write_pcd_xyz(kept_path, kept)
    write_pcd_xyz(removed_path, removed)
    write_pcd_xyz(heap_path, heap_band)

    print("written:")
    print(str(kept_path.resolve()))
    print(str(removed_path.resolve()))
    print(str(heap_path.resolve()))
    print("stats:")
    print(stats)
    print("heap_band:")
    print(
        {
            "heap_relative_ground": bool(args.heap_relative_ground),
            "ground_z": float(ground_z),
            "heap_z_min": float(heap_z_min),
            "heap_z_max": float(heap_z_max),
            "roi_front_x_min": args.roi_front_x_min,
            "roi_y_abs_max": args.roi_y_abs_max,
            "heap_points": int(len(heap_band)),
        }
    )


if __name__ == "__main__":
    main()
