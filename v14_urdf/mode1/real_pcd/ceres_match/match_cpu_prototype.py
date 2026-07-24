import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


CURRENT_DIR = Path(__file__).resolve().parent
REAL_PCD_DIR = CURRENT_DIR.parent
MODE1_DIR = REAL_PCD_DIR.parent
if str(REAL_PCD_DIR) not in sys.path:
    sys.path.append(str(REAL_PCD_DIR))
if str(MODE1_DIR) not in sys.path:
    sys.path.append(str(MODE1_DIR))

from pcd_numpy_io import read_pcd_xyz, write_pcd_xyz, write_pcd_xyzrgb
from workspace_volume_and_fuse import _collect_limits, _compute_chain_fk, _linspace, _load_urdf_joints


def _save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def _filter_heap_roi(points, z_min, z_max, front_x_min, y_abs_max):
    mask = np.isfinite(points).all(axis=1)
    mask &= points[:, 2] >= float(z_min)
    mask &= points[:, 2] <= float(z_max)
    if front_x_min is not None:
        mask &= points[:, 0] >= float(front_x_min)
    if y_abs_max is not None:
        mask &= np.abs(points[:, 1]) <= float(y_abs_max)
    return points[mask]


def _make_workspace_template(urdf_path, swing_samples, boom_samples, arm_samples, bucket_samples, z_max):
    joints = _load_urdf_joints(urdf_path)
    lim = _collect_limits(joints, ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"])
    swing_vals = _linspace(lim["swing_joint"][0], lim["swing_joint"][1], int(swing_samples))
    boom_vals = _linspace(lim["boom_joint"][0], lim["boom_joint"][1], int(boom_samples))
    arm_vals = _linspace(lim["arm_joint"][0], lim["arm_joint"][1], int(arm_samples))
    bucket_vals = _linspace(lim["bucket_joint"][0], lim["bucket_joint"][1], int(bucket_samples))

    pts = []
    for swing in swing_vals:
        for boom in boom_vals:
            for arm in arm_vals:
                for bucket in bucket_vals:
                    x, y, z = _compute_chain_fk(
                        joints,
                        {
                            "swing_joint": swing,
                            "boom_joint": boom,
                            "arm_joint": arm,
                            "bucket_joint": bucket,
                        },
                    )
                    if z_max is not None and float(z) > float(z_max):
                        continue
                    pts.append((x, y, z))
    if not pts:
        raise ValueError("workspace template 为空。")
    return np.array(pts, dtype=np.float32)


def _voxel_downsample(points, voxel):
    if len(points) == 0:
        return points
    keys = np.floor(points / float(voxel)).astype(np.int32)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    unique_idx.sort()
    return points[unique_idx]


def _best_fit_transform(src, dst):
    src_cent = src.mean(axis=0)
    dst_cent = dst.mean(axis=0)
    src_c = src - src_cent
    dst_c = dst - dst_cent
    h = src_c.T @ dst_c
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = dst_cent - r @ src_cent
    return r.astype(np.float32), t.astype(np.float32)


def _apply_transform(points, r, t):
    return (points @ r.T) + t.reshape(1, 3)


def _run_icp(src_points, dst_points, max_corr_dist=0.18, iterations=20):
    src = src_points.copy()
    r_total = np.eye(3, dtype=np.float32)
    t_total = np.zeros(3, dtype=np.float32)
    tree = cKDTree(dst_points[:, :3])

    last_rmse = None
    for _ in range(int(iterations)):
        dists, idx = tree.query(src[:, :3], k=1, distance_upper_bound=float(max_corr_dist))
        valid = np.isfinite(dists) & (idx < len(dst_points))
        if valid.sum() < 12:
            break
        src_corr = src[valid]
        dst_corr = dst_points[idx[valid]]
        r_step, t_step = _best_fit_transform(src_corr, dst_corr)
        src = _apply_transform(src, r_step, t_step)
        r_total = r_step @ r_total
        t_total = r_step @ t_total + t_step
        rmse = float(np.sqrt(np.mean((src_corr - dst_corr) ** 2)))
        if last_rmse is not None and abs(last_rmse - rmse) < 1e-4:
            break
        last_rmse = rmse

    dists, idx = tree.query(src[:, :3], k=1, distance_upper_bound=float(max_corr_dist))
    valid = np.isfinite(dists) & (idx < len(dst_points))
    rmse = float(np.sqrt(np.mean(dists[valid] ** 2))) if valid.any() else None
    return {
        "aligned": src,
        "rotation": r_total,
        "translation": t_total,
        "rmse": rmse,
        "matched_count": int(valid.sum()),
        "matched_mask": valid,
        "matched_indices": idx[valid].tolist() if valid.any() else [],
    }


def _estimate_initial_translation(template_points, heap_points):
    src_cent = template_points.mean(axis=0)
    dst_cent = heap_points.mean(axis=0)
    return (dst_cent - src_cent).astype(np.float32)


def _unique_rows(points):
    if len(points) == 0:
        return points
    rounded = np.round(points.astype(np.float32), 4)
    _, idx = np.unique(rounded, axis=0, return_index=True)
    idx.sort()
    return points[idx]


def _extract_operable_points_xy(heap_points, matched_workspace, xy_max_dist):
    if len(heap_points) == 0 or len(matched_workspace) == 0:
        return np.zeros((0, 3), dtype=np.float32), 0
    tree_xy = cKDTree(matched_workspace[:, :2])
    dists, _ = tree_xy.query(heap_points[:, :2], k=1, distance_upper_bound=float(xy_max_dist))
    valid = np.isfinite(dists)
    points = heap_points[valid].astype(np.float32, copy=False)
    return _unique_rows(points), int(valid.sum())


def main():
    parser = argparse.ArgumentParser(description="CPU 原型：模板点云与原始 heap ROI 点云匹配。")
    parser.add_argument("--pcd", required=True)
    parser.add_argument(
        "--urdf",
        default="src/shandong/v14_urdf/describe_60FED/urdf/describe_60FED_calibrated.urdf",
    )
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--heap-z-min", type=float, default=0.02)
    parser.add_argument("--heap-z-max", type=float, default=0.5)
    parser.add_argument("--roi-front-x-min", type=float, default=0.0)
    parser.add_argument("--roi-y-abs-max", type=float, default=1.2)
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument(
        "--icp-max-corr-dist",
        type=float,
        default=0.20,
        help="ICP 配准时的最近邻距离阈值。",
    )
    parser.add_argument(
        "--operable-max-dist",
        type=float,
        default=0.28,
        help="从匹配后模板提取可挖区域时的最近邻距离阈值，稀疏点云可适当放宽。",
    )
    parser.add_argument(
        "--operable-xy-max-dist",
        type=float,
        default=0.12,
        help="只按 x,y 提取可挖区域时的最近邻距离阈值。",
    )
    parser.add_argument("--swing-samples", type=int, default=40)
    parser.add_argument("--boom-samples", type=int, default=12)
    parser.add_argument("--arm-samples", type=int, default=12)
    parser.add_argument("--bucket-samples", type=int, default=10)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    scene = read_pcd_xyz(args.pcd)
    heap_roi = _filter_heap_roi(
        scene,
        z_min=float(args.heap_z_min),
        z_max=float(args.heap_z_max),
        front_x_min=args.roi_front_x_min,
        y_abs_max=args.roi_y_abs_max,
    )
    if heap_roi.size == 0:
        raise ValueError("heap ROI 为空。")

    template = _make_workspace_template(
        urdf_path=args.urdf,
        swing_samples=args.swing_samples,
        boom_samples=args.boom_samples,
        arm_samples=args.arm_samples,
        bucket_samples=args.bucket_samples,
        z_max=float(args.heap_z_max),
    )

    heap_ds = _voxel_downsample(heap_roi, float(args.voxel_size))
    template_ds = _voxel_downsample(template, float(args.voxel_size))

    init_t = _estimate_initial_translation(template_ds, heap_ds)
    template_init = template_ds + init_t.reshape(1, 3)
    icp = _run_icp(template_init, heap_ds, max_corr_dist=float(args.icp_max_corr_dist), iterations=20)

    matched_workspace = icp["aligned"]
    tree = cKDTree(heap_roi[:, :3])
    dists, idx = tree.query(
        matched_workspace[:, :3],
        k=1,
        distance_upper_bound=float(args.operable_max_dist),
    )
    valid = np.isfinite(dists) & (idx < len(heap_roi))
    operable_points_3d = heap_roi[idx[valid]] if valid.any() else np.zeros((0, 3), dtype=np.float32)
    operable_points_3d = _unique_rows(operable_points_3d.astype(np.float32, copy=False))
    operable_points_xy, operable_xy_match_count = _extract_operable_points_xy(
        heap_roi,
        matched_workspace,
        xy_max_dist=float(args.operable_xy_max_dist),
    )
    operable_points = operable_points_xy

    heap_pcd = out_dir / "heap_roi_points.pcd"
    template_pcd = out_dir / "workspace_template_points.pcd"
    matched_pcd = out_dir / "matched_workspace_points.pcd"
    operable_pcd = out_dir / "operable_region_points.pcd"
    fused_pcd = out_dir / "fused_scene_matched_workspace_rgb.pcd"
    result_json = out_dir / "match_result.json"

    write_pcd_xyz(heap_pcd, heap_roi)
    write_pcd_xyz(template_pcd, template_ds)
    write_pcd_xyz(matched_pcd, matched_workspace)
    write_pcd_xyz(operable_pcd, operable_points)

    scene_rgb = np.tile(np.array([[180, 180, 180]], dtype=np.uint8), (len(scene), 1))
    matched_rgb = np.tile(np.array([[80, 255, 120]], dtype=np.uint8), (len(matched_workspace), 1))
    operable_rgb = np.tile(np.array([[255, 230, 40]], dtype=np.uint8), (len(operable_points), 1))
    fused_xyz = np.concatenate([scene, matched_workspace, operable_points], axis=0)
    fused_rgb = np.concatenate([scene_rgb, matched_rgb, operable_rgb], axis=0)
    write_pcd_xyzrgb(fused_pcd, fused_xyz, fused_rgb)

    result = {
        "source_pcd": str(Path(args.pcd).resolve()),
        "heap_roi_points": int(len(heap_roi)),
        "heap_roi_downsampled": int(len(heap_ds)),
        "workspace_template_points": int(len(template)),
        "workspace_template_downsampled": int(len(template_ds)),
        "matched_workspace_points": int(len(matched_workspace)),
        "operable_region_points": int(len(operable_points)),
        "operable_region_points_3d": int(len(operable_points_3d)),
        "voxel_size": float(args.voxel_size),
        "icp_max_corr_dist": float(args.icp_max_corr_dist),
        "operable_max_dist": float(args.operable_max_dist),
        "operable_xy_max_dist": float(args.operable_xy_max_dist),
        "translation_init": init_t.tolist(),
        "rotation": icp["rotation"].tolist(),
        "translation": icp["translation"].tolist(),
        "rmse": icp["rmse"],
        "matched_count": int(icp["matched_count"]),
        "operable_xy_match_count": int(operable_xy_match_count),
        "heap_roi_pcd": str(heap_pcd.resolve()),
        "workspace_template_pcd": str(template_pcd.resolve()),
        "matched_workspace_pcd": str(matched_pcd.resolve()),
        "operable_region_pcd": str(operable_pcd.resolve()),
        "fused_pcd": str(fused_pcd.resolve()),
    }
    _save_json(result_json, result)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
