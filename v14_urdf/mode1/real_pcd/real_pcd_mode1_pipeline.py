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
if str(MODE1_DIR) not in sys.path:
    sys.path.append(str(MODE1_DIR))

from mode1_task_planner import DEFAULT_POSES, build_task
from pointcloud_to_dig_points import _candidate_points, _surface_points
from pcd_numpy_io import list_pcd_files, read_pcd_xyz
from body_filter import filter_excavator_body


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def _merge_poses(base_config):
    poses = copy.deepcopy(DEFAULT_POSES)
    user_poses = base_config.get("poses", {})
    for name, values in user_poses.items():
        if name in poses and isinstance(values, dict):
            poses[name].update(values)
    return poses


def _select_input_files(dataset_dir, glob_pattern, limit):
    files = list_pcd_files(dataset_dir, glob_pattern)
    if limit is not None:
        files = files[: max(0, int(limit))]
    if not files:
        raise ValueError(f"没有匹配到 PCD 文件: dir={dataset_dir}, pattern={glob_pattern}")
    return files


def _extract_heap_points(points, extraction_cfg):
    mask_radius = float(extraction_cfg.get("mask_center_radius", 0.7))
    heap_z_min = float(extraction_cfg.get("heap_z_min", 0.05))
    heap_z_max = float(extraction_cfg.get("heap_z_max", 0.5))
    front_x_min = float(extraction_cfg.get("front_x_min", 0.35))
    y_abs_max = extraction_cfg.get("y_abs_max")
    if y_abs_max is not None:
        y_abs_max = float(y_abs_max)

    body_cfg = {
        "center_radius": mask_radius,
        "z_min": float(extraction_cfg.get("body_z_min", 0.0)),
        "z_max": float(extraction_cfg.get("body_z_max", 2.0)),
        "box_x_min": float(extraction_cfg.get("body_box_x_min", -0.4)),
        "box_x_max": float(extraction_cfg.get("body_box_x_max", 0.7)),
        "box_y_abs": float(extraction_cfg.get("body_box_y_abs", 0.55)),
        "box_z_min": float(extraction_cfg.get("body_box_z_min", 0.0)),
        "box_z_max": float(extraction_cfg.get("body_box_z_max", 0.9)),
        "arm_enabled": False,
    }
    kept, removed, body_stats = filter_excavator_body(points, body_cfg)

    mask = (kept[:, 2] >= heap_z_min) & (kept[:, 2] <= heap_z_max)
    mask &= kept[:, 0] > front_x_min
    if y_abs_max is not None:
        mask &= np.abs(kept[:, 1]) <= y_abs_max
    filtered = kept[mask]
    if len(filtered) == 0:
        raise ValueError("经过中点屏蔽和土堆高度过滤后没有剩余点。")

    stats = {
        "mask_center_radius": mask_radius,
        "heap_z_min": heap_z_min,
        "heap_z_max": heap_z_max,
        "front_x_min": front_x_min,
        "y_abs_max": y_abs_max,
        "input_points": int(len(points)),
        "body_removed_points": int(body_stats["removed_points"]),
        "heap_points": int(len(filtered)),
    }
    return filtered, stats


def _radius_yaw_deg(x, y):
    radius = math.sqrt(float(x) * float(x) + float(y) * float(y))
    yaw_deg = math.degrees(math.atan2(float(y), float(x)))
    return radius, yaw_deg


def _within_workspace(point, workspace):
    x = float(point["x"])
    y = float(point["y"])
    z = float(point["z"])
    radius, yaw_deg = _radius_yaw_deg(x, y)
    return (
        float(workspace["r_min"]) <= radius <= float(workspace["r_max"])
        and float(workspace["yaw_min_deg"]) <= yaw_deg <= float(workspace["yaw_max_deg"])
        and float(workspace["z_min"]) <= z <= float(workspace["z_max"])
    )


def _auto_rect_from_surface(surface_points, workspace, auto_rect_cfg):
    selected = [point for point in surface_points if _within_workspace(point, workspace)]
    if not selected:
        raise ValueError("真实点云在当前 workspace 内没有可用 surface_points，无法自动生成 dig_area_rect。")

    xs = np.array([float(point["x"]) for point in selected], dtype=np.float32)
    ys = np.array([float(point["y"]) for point in selected], dtype=np.float32)

    padding_x = float(auto_rect_cfg.get("padding_x", 0.12))
    padding_y = float(auto_rect_cfg.get("padding_y", 0.12))
    min_length = float(auto_rect_cfg.get("min_length", 0.30))
    min_width = float(auto_rect_cfg.get("min_width", 0.30))
    target_z = float(auto_rect_cfg.get("target_z", 0.17))

    rect = {
        "center": {
            "x": float((xs.min() + xs.max()) * 0.5),
            "y": float((ys.min() + ys.max()) * 0.5),
            "z": target_z,
        },
        "length": float(max(min_length, (xs.max() - xs.min()) + padding_x)),
        "width": float(max(min_width, (ys.max() - ys.min()) + padding_y)),
        "yaw_deg": float(auto_rect_cfg.get("yaw_deg", 0.0)),
    }
    return rect, selected


def _points_to_json(points):
    return [{"x": float(x), "y": float(y), "z": float(z)} for x, y, z in points]


def _build_candidate_json(
    heap_points,
    constraints_path,
    task_config,
    surface_points,
    candidate_points,
    surface_stats,
    candidate_stats,
    source_files,
    output_path,
):
    xy_bin = float(_load_json(constraints_path)["bucket_tip_zslice_bounds"]["xy_bin"])
    payload = {
        "source": "real_pcd_pipeline",
        "constraints_path": str(Path(constraints_path).resolve()),
        "points_path": None,
        "task_config_path": None,
        "xy_bin": xy_bin,
        "pattern": task_config.get("sampling", {}).get("pattern", "boustrophedon"),
        "source_files": [str(Path(path).resolve()) for path in source_files],
        "filter_stats": {
            **surface_stats,
            **candidate_stats,
        },
        "surface_points": surface_points,
        "candidate_dig_points": candidate_points,
    }
    _save_json(output_path, payload)
    return payload


def run_pipeline(config_path, out_dir):
    config = _load_json(config_path)
    dataset_cfg = config.get("pcd", {})
    dataset_dir = dataset_cfg.get("dataset_dir", str(MODE1_DIR / "pcd"))
    glob_pattern = dataset_cfg.get("glob", "fused3_pointcloud_base_link_*.pcd")
    limit = dataset_cfg.get("limit", 1)
    source_files = _select_input_files(dataset_dir, glob_pattern, limit)

    merged = []
    file_stats = []
    for path in source_files:
        points = read_pcd_xyz(path)
        merged.append(points)
        file_stats.append({"file": str(path), "points": int(len(points))})
    merged_points = np.vstack(merged)

    heap_points, extraction_stats = _extract_heap_points(merged_points, dataset_cfg)

    constraints_path = config.get(
        "constraints_path",
        str(MODE1_DIR / "constraints" / "workspace_constraints_360_z0.json"),
    )
    constraints = _load_json(constraints_path)
    zslice_bounds = constraints["bucket_tip_zslice_bounds"]
    surface_points, surface_stats = _surface_points(
        [tuple(map(float, point)) for point in heap_points],
        zslice_bounds,
    )

    workspace = config["workspace"]
    auto_rect_cfg = config.get("auto_rect", {})
    dig_area_rect, workspace_surface_points = _auto_rect_from_surface(
        surface_points,
        workspace,
        auto_rect_cfg,
    )

    task_config = {
        "task_name": config.get("task_name", "mode1_real_pcd_demo"),
        "workspace": workspace,
        "dig_area_rect": dig_area_rect,
        "dump_strategy": config["dump_strategy"],
        "poses": _merge_poses(config),
        "sampling": {
            "pattern": config.get("sampling", {}).get("pattern", "boustrophedon"),
        },
    }

    xy_bin = float(zslice_bounds["xy_bin"])
    candidate_points, candidate_stats = _candidate_points(
        surface_points,
        dig_area_rect,
        workspace,
        xy_bin,
        task_config["sampling"]["pattern"],
    )
    if not candidate_points:
        raise ValueError("真实点云已经提取出土堆，但没有候选挖掘点通过 workspace/rect 过滤。")

    out_dir = Path(out_dir)
    heap_points_path = out_dir / "real_pcd_heap_points.json"
    auto_task_config_path = out_dir / "real_pcd_auto_task_config.json"
    candidate_path = out_dir / "real_pcd_candidate_dig_points.json"
    task_plan_path = out_dir / "real_pcd_task_plan.json"

    heap_payload = {
        "source_files": [str(Path(path).resolve()) for path in source_files],
        "file_stats": file_stats,
        "extraction_stats": extraction_stats,
        "merged_points": int(len(merged_points)),
        "heap_bounds": {
            "min_x": float(heap_points[:, 0].min()),
            "max_x": float(heap_points[:, 0].max()),
            "min_y": float(heap_points[:, 1].min()),
            "max_y": float(heap_points[:, 1].max()),
            "min_z": float(heap_points[:, 2].min()),
            "max_z": float(heap_points[:, 2].max()),
        },
        "points": _points_to_json(heap_points),
    }
    _save_json(heap_points_path, heap_payload)

    auto_task_payload = copy.deepcopy(task_config)
    auto_task_payload["_path"] = str(auto_task_config_path.resolve())
    auto_task_payload["real_pcd_source_files"] = [str(Path(path).resolve()) for path in source_files]
    auto_task_payload["auto_rect_workspace_surface_count"] = len(workspace_surface_points)
    _save_json(auto_task_config_path, auto_task_payload)

    _build_candidate_json(
        heap_points=heap_points,
        constraints_path=constraints_path,
        task_config=task_config,
        surface_points=surface_points,
        candidate_points=candidate_points,
        surface_stats=surface_stats,
        candidate_stats=candidate_stats,
        source_files=source_files,
        output_path=candidate_path,
    )

    task_result = build_task(
        task_config=auto_task_payload,
        output_path=str(task_plan_path),
        candidate_path=str(candidate_path),
        max_candidates=config.get("max_candidates"),
    )
    _save_json(task_plan_path, task_result)

    summary = {
        "source_files": [str(Path(path).resolve()) for path in source_files],
        "heap_points_path": str(heap_points_path.resolve()),
        "auto_task_config_path": str(auto_task_config_path.resolve()),
        "candidate_path": str(candidate_path.resolve()),
        "task_plan_path": str(task_plan_path.resolve()),
        "merged_points": int(len(merged_points)),
        "heap_points": int(len(heap_points)),
        "surface_bins": int(candidate_stats["surface_bins"]),
        "candidate_bins": int(candidate_stats["candidate_bins"]),
        "cycle_count": int(task_result["metadata"]["cycle_count"]),
        "requested_cycle_count": int(task_result["metadata"]["requested_cycle_count"]),
        "auto_rect": dig_area_rect,
    }
    return summary


def main():
    parser = argparse.ArgumentParser(description="把真实 PCD 数据离线转换为 mode1 挖掘任务。")
    parser.add_argument(
        "--config",
        default=str(CURRENT_DIR / "config" / "real_pcd_mode1_demo.json"),
        help="真实点云离线规划配置 JSON。",
    )
    parser.add_argument(
        "--out-dir",
        default=str(CURRENT_DIR / "output"),
        help="输出目录。",
    )
    args = parser.parse_args()

    summary = run_pipeline(args.config, args.out_dir)
    print(f"source_files={len(summary['source_files'])}")
    print(f"merged_points={summary['merged_points']}")
    print(f"heap_points={summary['heap_points']}")
    print(f"surface_bins={summary['surface_bins']}")
    print(f"candidate_bins={summary['candidate_bins']}")
    print(f"cycle_count={summary['cycle_count']}")
    print(f"task_plan={summary['task_plan_path']}")


if __name__ == "__main__":
    main()
