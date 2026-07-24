import argparse
import json
from pathlib import Path

import numpy as np

from body_filter import filter_excavator_body
from pcd_numpy_io import read_pcd_xyz


def _save_points(path, points, meta):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "meta": meta,
        "points": [{"x": float(x), "y": float(y), "z": float(z)} for x, y, z in points],
    }
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)


def main():
    parser = argparse.ArgumentParser(description="对单帧 PCD 做挖掘机本体屏蔽示例，并输出 JSON。")
    parser.add_argument("--pcd", required=True, help="输入 PCD 文件路径。")
    parser.add_argument("--out-dir", required=True, help="输出目录。")
    parser.add_argument("--heap-z-min", type=float, default=0.05, help="土堆点带下界。")
    parser.add_argument("--heap-z-max", type=float, default=0.5, help="土堆点带上界。")
    parser.add_argument("--heap-relative-ground", action="store_true", help="把土堆点带定义成 ground_z + [heap_z_min, heap_z_max]。")

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
    z = kept[:, 2]
    z = z[np.isfinite(z)]
    near = z[(z >= -0.5) & (z <= 0.5)]
    if near.size:
        zr = np.round(near, 3)
        values, counts = np.unique(zr, return_counts=True)
        ground_z = float(values[int(np.argmax(counts))])
    elif z.size:
        ground_z = float(np.quantile(z, 0.05))
    else:
        ground_z = 0.0
    if args.heap_relative_ground:
        heap_z_min = ground_z + float(args.heap_z_min)
        heap_z_max = ground_z + float(args.heap_z_max)
    else:
        heap_z_min = float(args.heap_z_min)
        heap_z_max = float(args.heap_z_max)
    heap_band = kept[(kept[:, 2] >= heap_z_min) & (kept[:, 2] <= heap_z_max)]

    out_dir = Path(args.out_dir)
    meta = {
        "pcd": str(Path(args.pcd).resolve()),
        "mask_config": mask_cfg,
        "stats": stats,
    }
    _save_points(out_dir / "kept_points.json", kept, meta)
    _save_points(out_dir / "removed_points.json", removed, meta)
    _save_points(
        out_dir / "heap_points_z_band.json",
        heap_band,
        {
            **meta,
            "heap_relative_ground": bool(args.heap_relative_ground),
            "ground_z": float(ground_z),
            "heap_z_min": float(heap_z_min),
            "heap_z_max": float(heap_z_max),
        },
    )

    print(
        json.dumps(
            {
                "stats": stats,
                "heap_points": int(len(heap_band)),
                "heap_relative_ground": bool(args.heap_relative_ground),
                "ground_z": float(ground_z),
                "heap_z_min": float(heap_z_min),
                "heap_z_max": float(heap_z_max),
                "out_dir": str(out_dir.resolve()),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
