import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


CURRENT_DIR = Path(__file__).resolve().parent
MODE1_DIR = CURRENT_DIR.parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.append(str(CURRENT_DIR))
if str(MODE1_DIR) not in sys.path:
    sys.path.append(str(MODE1_DIR))

from pcd_numpy_io import read_pcd_xyz, write_pcd_xyz, write_pcd_xyzrgb
from workspace_volume_and_fuse import _collect_limits, _compute_chain_fk, _linspace, _load_urdf_joints


def _save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


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


def _make_workspace_points(urdf_path, swing_samples, boom_samples, arm_samples, bucket_samples, z_max):
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
                    if z_max is not None and float(z) > float(z_max):
                        continue
                    pts.append((x, y, z))
    if not pts:
        raise ValueError("workspace 采样后为空，请检查 z_max 或 URDF。")
    return np.array(pts, dtype=np.float32)


def _build_workspace_bins(workspace_points, xy_bin, expand_steps):
    base_bins = set()
    for x, y, _ in workspace_points:
        bx = int(round(float(x) / float(xy_bin)))
        by = int(round(float(y) / float(xy_bin)))
        base_bins.add((bx, by))

    if int(expand_steps) <= 0:
        return base_bins

    expanded = set()
    radius = int(expand_steps)
    for bx, by in base_bins:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                expanded.add((bx + dx, by + dy))
    return expanded


def _filter_heap_roi(points, z_min, z_max, front_x_min, y_abs_max):
    mask = np.isfinite(points).all(axis=1)
    mask &= points[:, 2] >= float(z_min)
    mask &= points[:, 2] <= float(z_max)
    if front_x_min is not None:
        mask &= points[:, 0] >= float(front_x_min)
    if y_abs_max is not None:
        mask &= np.abs(points[:, 1]) <= float(y_abs_max)
    return points[mask]


def _points_to_bins(points, xy_bin):
    bins = {}
    for x, y, z in points:
        bx = int(round(float(x) / float(xy_bin)))
        by = int(round(float(y) / float(xy_bin)))
        key = (bx, by)
        item = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "bin_x": float(bx) * float(xy_bin),
            "bin_y": float(by) * float(xy_bin),
            "bin_key": [int(bx), int(by)],
        }
        bins.setdefault(key, []).append(item)
    return bins


def _summarize_bin_points(bin_points):
    out = []
    for key, items in bin_points.items():
        z_sorted = sorted(float(item["z"]) for item in items)
        mid = len(z_sorted) // 2
        median_z = z_sorted[mid] if len(z_sorted) % 2 == 1 else 0.5 * (z_sorted[mid - 1] + z_sorted[mid])
        ref = items[0]
        out.append(
            {
                "x": float(ref["bin_x"]),
                "y": float(ref["bin_y"]),
                "z": float(median_z),
                "sample_count": int(len(items)),
                "bin_x": float(ref["bin_x"]),
                "bin_y": float(ref["bin_y"]),
                "bin_key": list(ref["bin_key"]),
            }
        )
    out.sort(key=lambda item: (item["bin_x"], item["bin_y"]))
    return out


def _color_block(rgb, count):
    return np.tile(np.array([rgb], dtype=np.uint8), (int(count), 1))


def main():
    parser = argparse.ArgumentParser(description="融合原始土堆点云与挖掘机作业区域，生成 ROI 待作业区域点云。")
    parser.add_argument("--pcd", required=True, help="pointcloud_base_link*.pcd")
    parser.add_argument(
        "--urdf",
        default="src/shandong/v14_urdf/describe_60FED/urdf/describe_60FED_calibrated.urdf",
        help="标定 URDF 路径。",
    )
    parser.add_argument("--out-dir", required=True, help="输出目录。")
    parser.add_argument("--heap-z-min", type=float, default=0.02)
    parser.add_argument("--heap-z-max", type=float, default=0.5)
    parser.add_argument("--roi-front-x-min", type=float, default=0.35)
    parser.add_argument("--roi-y-abs-max", type=float, default=1.2)
    parser.add_argument("--xy-bin", type=float, default=0.04)
    parser.add_argument("--workspace-expand-steps", type=int, default=1)
    parser.add_argument("--swing-samples", type=int, default=144)
    parser.add_argument("--boom-samples", type=int, default=28)
    parser.add_argument("--arm-samples", type=int, default=28)
    parser.add_argument("--bucket-samples", type=int, default=20)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    scene = read_pcd_xyz(args.pcd)
    ground_z = _estimate_ground_z(scene)
    heap_roi = _filter_heap_roi(
        scene,
        z_min=float(args.heap_z_min),
        z_max=float(args.heap_z_max),
        front_x_min=args.roi_front_x_min,
        y_abs_max=args.roi_y_abs_max,
    )
    if heap_roi.size == 0:
        raise ValueError("heap ROI 为空，请检查 z/前向/左右范围。")

    workspace_points = _make_workspace_points(
        urdf_path=args.urdf,
        swing_samples=args.swing_samples,
        boom_samples=args.boom_samples,
        arm_samples=args.arm_samples,
        bucket_samples=args.bucket_samples,
        z_max=float(args.heap_z_max),
    )
    workspace_bins = _build_workspace_bins(
        workspace_points,
        xy_bin=float(args.xy_bin),
        expand_steps=int(args.workspace_expand_steps),
    )

    heap_bins = _points_to_bins(heap_roi, float(args.xy_bin))
    operable_bins = {key: items for key, items in heap_bins.items() if key in workspace_bins}
    operable_points = np.array(
        [[item["x"], item["y"], item["z"]] for items in operable_bins.values() for item in items],
        dtype=np.float32,
    )
    if operable_points.size == 0:
        raise ValueError("土堆点云与作业区域 ROI 融合后为空。")

    heap_surface_bins = _summarize_bin_points(heap_bins)
    operable_surface_bins = _summarize_bin_points(operable_bins)

    heap_roi_pcd = out_dir / "heap_roi_points.pcd"
    operable_roi_pcd = out_dir / "operable_region_points.pcd"
    write_pcd_xyz(heap_roi_pcd, heap_roi)
    write_pcd_xyz(operable_roi_pcd, operable_points)

    heap_surface_pcd = out_dir / "heap_roi_surface_bins.pcd"
    operable_surface_pcd = out_dir / "operable_region_surface_bins.pcd"
    write_pcd_xyz(heap_surface_pcd, np.array([[p["x"], p["y"], p["z"]] for p in heap_surface_bins], dtype=np.float32))
    write_pcd_xyz(
        operable_surface_pcd,
        np.array([[p["x"], p["y"], p["z"]] for p in operable_surface_bins], dtype=np.float32),
    )

    scene_rgb = _color_block([180, 180, 180], len(scene))
    heap_rgb = _color_block([255, 80, 80], len(heap_roi))
    operable_rgb = _color_block([255, 230, 40], len(operable_points))
    fused_xyz = np.concatenate([scene, heap_roi, operable_points], axis=0)
    fused_rgb = np.concatenate([scene_rgb, heap_rgb, operable_rgb], axis=0)
    fused_pcd = out_dir / "fused_scene_heap_operable_rgb.pcd"
    write_pcd_xyzrgb(fused_pcd, fused_xyz, fused_rgb)

    heap_json = out_dir / "heap_roi_points.json"
    operable_json = out_dir / "operable_region_points.json"
    _save_json(
        heap_json,
        {
            "source_pcd": str(Path(args.pcd).resolve()),
            "ground_z": float(ground_z),
            "roi": {
                "heap_z_min": float(args.heap_z_min),
                "heap_z_max": float(args.heap_z_max),
                "roi_front_x_min": args.roi_front_x_min,
                "roi_y_abs_max": args.roi_y_abs_max,
                "xy_bin": float(args.xy_bin),
            },
            "points_count": int(heap_roi.shape[0]),
            "surface_bin_count": int(len(heap_surface_bins)),
            "surface_points": heap_surface_bins,
        },
    )
    _save_json(
        operable_json,
        {
            "source_pcd": str(Path(args.pcd).resolve()),
            "ground_z": float(ground_z),
            "roi": {
                "heap_z_min": float(args.heap_z_min),
                "heap_z_max": float(args.heap_z_max),
                "roi_front_x_min": args.roi_front_x_min,
                "roi_y_abs_max": args.roi_y_abs_max,
                "xy_bin": float(args.xy_bin),
                "workspace_expand_steps": int(args.workspace_expand_steps),
            },
            "workspace": {
                "points_count": int(workspace_points.shape[0]),
                "bin_count": int(len(workspace_bins)),
                "z_max": float(args.heap_z_max),
            },
            "points_count": int(operable_points.shape[0]),
            "surface_bin_count": int(len(operable_surface_bins)),
            "surface_points": operable_surface_bins,
        },
    )

    print(
        json.dumps(
            {
                "source_pcd": str(Path(args.pcd).resolve()),
                "ground_z": float(ground_z),
                "heap_roi_points": int(heap_roi.shape[0]),
                "operable_region_points": int(operable_points.shape[0]),
                "heap_surface_bins": int(len(heap_surface_bins)),
                "operable_surface_bins": int(len(operable_surface_bins)),
                "heap_roi_json": str(heap_json.resolve()),
                "operable_region_json": str(operable_json.resolve()),
                "heap_roi_pcd": str(heap_roi_pcd.resolve()),
                "operable_region_pcd": str(operable_roi_pcd.resolve()),
                "fused_pcd": str(fused_pcd.resolve()),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
