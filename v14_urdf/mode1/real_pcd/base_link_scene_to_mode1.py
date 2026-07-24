import argparse
import copy
import json
import math
import os
import sys
from pathlib import Path

import numpy as np


CURRENT_DIR = Path(__file__).resolve().parent
MODE1_DIR = CURRENT_DIR.parent
V14_DIR = MODE1_DIR.parent
if str(MODE1_DIR) not in sys.path:
    sys.path.append(str(MODE1_DIR))
if str(V14_DIR) not in sys.path:
    sys.path.append(str(V14_DIR))

from mode1_task_planner import DEFAULT_POSES, build_task
from pointcloud_to_dig_points import _candidate_points, _surface_points
from pcd_numpy_io import read_pcd_xyz
from workspace_volume_and_fuse import _collect_limits, _compute_chain_fk, _linspace, _load_urdf_joints


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def _merge_poses(task_config):
    poses = copy.deepcopy(DEFAULT_POSES)
    for name, values in task_config.get("poses", {}).items():
        if name in poses and isinstance(values, dict):
            poses[name].update(values)
    return poses


def _estimate_ground_z(points):
    if points.size == 0:
        return 0.0
    z = points[:, 2].astype(np.float32, copy=False)
    z = z[np.isfinite(z)]
    near = z[(z >= -0.5) & (z <= 0.5)]
    if near.size == 0:
        return float(np.quantile(z, 0.05))
    zr = np.round(near, 3)
    values, counts = np.unique(zr, return_counts=True)
    return float(values[int(np.argmax(counts))])


def _make_workspace(urdf_path, swing_samples, boom_samples, arm_samples, bucket_samples, workspace_z_max):
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
    return np.array(pts, dtype=np.float32)


def _build_workspace_grid(workspace_points, xy_bin):
    bins = {}
    for x, y, z in workspace_points:
        bx = int(round(float(x) / xy_bin))
        by = int(round(float(y) / xy_bin))
        key = (bx, by)
        pt = (float(bx) * xy_bin, float(by) * xy_bin)
        if key in bins:
            bins[key]["z_max"] = max(float(z), bins[key]["z_max"])
            bins[key]["z_min"] = min(float(z), bins[key]["z_min"])
        else:
            bins[key] = {"x": pt[0], "y": pt[1], "z_min": float(z), "z_max": float(z)}
    return bins


def _build_surface_points(scene_roi, workspace_grid, xy_bin, surface_z_min=None, surface_z_max=None, top_k=5):
    per_bin = {}
    for x, y, z in scene_roi:
        bx = int(round(float(x) / float(xy_bin)))
        by = int(round(float(y) / float(xy_bin)))
        key = (bx, by)
        if key not in workspace_grid:
            continue
        if surface_z_min is not None and float(z) < float(surface_z_min):
            continue
        if surface_z_max is not None and float(z) > float(surface_z_max):
            continue
        entry = per_bin.setdefault(
            key,
            {
                "x": float(bx) * float(xy_bin),
                "y": float(by) * float(xy_bin),
                "z_samples": [],
            },
        )
        entry["z_samples"].append(float(z))

    surface_points = []
    for (bx, by), entry in per_bin.items():
        z_samples = np.array(entry["z_samples"], dtype=np.float32)
        z_sorted = np.sort(z_samples)
        z_top = z_sorted[-min(int(top_k), len(z_sorted)) :]
        surface_points.append(
            {
                "x": entry["x"],
                "y": entry["y"],
                "z": float(np.median(z_top)),
                "z_raw_max": float(z_sorted[-1]),
                "z_raw_min": float(z_sorted[0]),
                "sample_count": int(len(z_sorted)),
                "bin_x": entry["x"],
                "bin_y": entry["y"],
                "bin_key": [int(bx), int(by)],
            }
        )
    return surface_points


def _prune_isolated_surface_points(surface_points, support_gap=0.12, min_support_neighbors=1):
    grid = {tuple(point["bin_key"]): point for point in surface_points}
    kept = []
    rejected = 0
    for point in surface_points:
        if int(point.get("sample_count", 1)) >= 2:
            kept.append(point)
            continue
        bx, by = point["bin_key"]
        support = 0
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neighbor = grid.get((bx + dx, by + dy))
                if neighbor is None:
                    continue
                if abs(float(neighbor["z"]) - float(point["z"])) <= float(support_gap):
                    support += 1
        if support >= int(min_support_neighbors):
            kept.append(point)
        else:
            rejected += 1
    return kept, rejected


def _build_auto_rect(
    surface_points,
    workspace,
    padding_x=0.12,
    padding_y=0.12,
    min_length=0.30,
    min_width=0.30,
):
    selected = []
    for point in surface_points:
        x = float(point["x"])
        y = float(point["y"])
        z = float(point["z"])
        radius = math.sqrt(x * x + y * y)
        yaw = math.degrees(math.atan2(y, x))
        if (
            float(workspace["r_min"]) <= radius <= float(workspace["r_max"])
            and float(workspace["yaw_min_deg"]) <= yaw <= float(workspace["yaw_max_deg"])
        ):
            selected.append(point)
    if not selected:
        raise ValueError("ROI surface_points 为空，无法自动生成 dig_area_rect。")
    xs = np.array([float(point["x"]) for point in selected], dtype=np.float32)
    ys = np.array([float(point["y"]) for point in selected], dtype=np.float32)
    zs = np.array([float(point["z"]) for point in selected], dtype=np.float32)
    return {
        "center": {
            "x": float((xs.min() + xs.max()) * 0.5),
            "y": float((ys.min() + ys.max()) * 0.5),
            "z": float(np.median(zs)),
        },
        "length": float(max(min_length, (xs.max() - xs.min()) + padding_x)),
        "width": float(max(min_width, (ys.max() - ys.min()) + padding_y)),
        "yaw_deg": 0.0,
    }


def main():
    parser = argparse.ArgumentParser(description="把 base_link 场景点云转换为 mode1 候选点与任务 JSON。")
    parser.add_argument("--pcd", required=True, help="pointcloud_base_link*.pcd")
    parser.add_argument(
        "--urdf",
        default="src/shandong/v14_urdf/describe_60FED/urdf/describe_60FED_calibrated.urdf",
        help="标定 URDF 路径。",
    )
    parser.add_argument("--out-dir", required=True, help="输出目录。")
    parser.add_argument("--workspace-z-max", type=float, default=0.5)
    parser.add_argument("--xy-bin", type=float, default=0.06)
    parser.add_argument(
        "--surface-z-min",
        type=float,
        default=None,
        help="可选：只保留 z >= 该值的场景点作为局部表面候选。",
    )
    parser.add_argument(
        "--surface-z-max",
        type=float,
        default=None,
        help="可选：只保留 z <= 该值的场景点作为局部表面候选。",
    )
    parser.add_argument("--roi-front-x-min", type=float, default=0.35)
    parser.add_argument("--roi-y-abs-max", type=float, default=1.2)
    parser.add_argument("--swing-samples", type=int, default=72)
    parser.add_argument("--boom-samples", type=int, default=20)
    parser.add_argument("--arm-samples", type=int, default=20)
    parser.add_argument("--bucket-samples", type=int, default=16)
    parser.add_argument("--max-candidates", type=int, default=6)
    parser.add_argument("--surface-top-k", type=int, default=5)
    parser.add_argument("--surface-support-gap", type=float, default=0.12)
    parser.add_argument("--surface-min-support-neighbors", type=int, default=1)
    parser.add_argument(
        "--task-config",
        default="src/shandong/v14_urdf/mode1/real_pcd/config/real_pcd_mode1_demo.json",
        help="用于 dump_strategy/poses 的基础配置。",
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    scene = read_pcd_xyz(args.pcd)
    ground_z = _estimate_ground_z(scene)
    scene_roi = scene.copy()
    if args.roi_front_x_min is not None:
        scene_roi = scene_roi[scene_roi[:, 0] >= float(args.roi_front_x_min)]
    if args.roi_y_abs_max is not None:
        scene_roi = scene_roi[np.abs(scene_roi[:, 1]) <= float(args.roi_y_abs_max)]
    if scene_roi.size == 0:
        raise ValueError("基础场景 ROI 后没有点。")

    workspace = _make_workspace(
        urdf_path=args.urdf,
        swing_samples=args.swing_samples,
        boom_samples=args.boom_samples,
        arm_samples=args.arm_samples,
        bucket_samples=args.bucket_samples,
        workspace_z_max=args.workspace_z_max,
    )
    workspace_grid = _build_workspace_grid(workspace, float(args.xy_bin))

    surface_points_raw = _build_surface_points(
        scene_roi,
        workspace_grid,
        float(args.xy_bin),
        surface_z_min=args.surface_z_min,
        surface_z_max=args.surface_z_max,
        top_k=int(args.surface_top_k),
    )
    surface_points, rejected_surface_isolated = _prune_isolated_surface_points(
        surface_points_raw,
        support_gap=float(args.surface_support_gap),
        min_support_neighbors=int(args.surface_min_support_neighbors),
    )
    if not surface_points:
        raise ValueError("场景点与 z<=0.5 的作业区域没有 ROI 交集。")

    base_config = _load_json(args.task_config)
    workspace_cfg = copy.deepcopy(
        base_config.get(
            "workspace",
            {
                "r_min": 0.85,
                "r_max": 1.75,
                "yaw_min_deg": -60.0,
                "yaw_max_deg": 60.0,
                "z_min": ground_z,
                "z_max": float(args.workspace_z_max),
            },
        )
    )
    workspace_cfg["z_min"] = max(float(workspace_cfg.get("z_min", ground_z)), float(ground_z))
    workspace_cfg["z_max"] = float(args.workspace_z_max)

    auto_rect_cfg = base_config.get("auto_rect", {})
    auto_rect = _build_auto_rect(
        surface_points,
        workspace_cfg,
        padding_x=float(auto_rect_cfg.get("padding_x", 0.12)),
        padding_y=float(auto_rect_cfg.get("padding_y", 0.12)),
        min_length=float(auto_rect_cfg.get("min_length", 0.30)),
        min_width=float(auto_rect_cfg.get("min_width", 0.30)),
    )
    candidates, candidate_stats = _candidate_points(
        surface_points,
        auto_rect,
        workspace_cfg,
        float(args.xy_bin),
        "boustrophedon",
    )
    if not candidates:
        raise ValueError("ROI surface_points 存在，但 candidate_dig_points 为空。")

    candidate_path = out_dir / "scene_roi_candidate_dig_points.json"
    candidate_payload = {
        "source": "base_link_scene_roi",
        "scene_pcd": str(Path(args.pcd).resolve()),
        "ground_z": float(ground_z),
        "roi": {
            "ground_z": float(ground_z),
            "surface_z_min": args.surface_z_min,
            "surface_z_max": args.surface_z_max,
            "roi_front_x_min": args.roi_front_x_min,
            "roi_y_abs_max": args.roi_y_abs_max,
            "workspace_z_max": float(args.workspace_z_max),
        },
        "dig_area_rect": auto_rect,
        "filter_stats": {
            "scene_roi_points": int(scene_roi.shape[0]),
            "surface_bins_raw": int(len(surface_points_raw)),
            "rejected_surface_isolated": int(rejected_surface_isolated),
            "surface_bins": int(len(surface_points)),
            **candidate_stats,
        },
        "surface_points": surface_points,
        "candidate_dig_points": candidates,
    }
    _save_json(candidate_path, candidate_payload)

    task_cfg = {
        "task_name": "base_link_scene_mode1_demo",
        "workspace": workspace_cfg,
        "dig_area_rect": auto_rect,
        "dump_strategy": base_config["dump_strategy"],
        "poses": _merge_poses(base_config),
        "sampling": {"pattern": "boustrophedon"},
        "_path": str((out_dir / "scene_mode1_task_config.json").resolve()),
    }
    _save_json(task_cfg["_path"], task_cfg)

    task_plan_path = out_dir / "scene_mode1_task_plan.json"
    task_result = build_task(
        task_config=task_cfg,
        output_path=str(task_plan_path),
        candidate_path=str(candidate_path),
        max_candidates=int(args.max_candidates),
    )
    _save_json(task_plan_path, task_result)

    print(
        json.dumps(
            {
                "scene_pcd": str(Path(args.pcd).resolve()),
                "ground_z": float(ground_z),
                "scene_roi_points": int(scene_roi.shape[0]),
                "surface_bins": int(len(surface_points)),
                "candidate_bins": int(len(candidates)),
                "cycle_count": int(task_result["metadata"]["cycle_count"]),
                "candidate_json": str(candidate_path.resolve()),
                "task_plan": str(task_plan_path.resolve()),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
