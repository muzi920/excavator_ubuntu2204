import argparse
import json
import math
import os


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _save_json(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def _normalize_point(item):
    if isinstance(item, dict):
        if {"x", "y", "z"}.issubset(item.keys()):
            return float(item["x"]), float(item["y"]), float(item["z"])
        if "point" in item and isinstance(item["point"], dict):
            point = item["point"]
            if {"x", "y", "z"}.issubset(point.keys()):
                return float(point["x"]), float(point["y"]), float(point["z"])
    if isinstance(item, (list, tuple)) and len(item) >= 3:
        return float(item[0]), float(item[1]), float(item[2])
    raise ValueError(f"Unsupported point format: {item!r}")


def _extract_points(data):
    if isinstance(data, list):
        raw_points = data
    elif isinstance(data, dict):
        for key in ("points", "points_xyz", "point_cloud", "pointcloud"):
            if key in data and isinstance(data[key], list):
                raw_points = data[key]
                break
        else:
            raise ValueError("Input JSON must be a list or contain one of: points, points_xyz, point_cloud, pointcloud.")
    else:
        raise ValueError("Input JSON must be a list or object.")
    return [_normalize_point(item) for item in raw_points]


def _radius_yaw(x, y):
    radius = math.sqrt(x * x + y * y)
    yaw_deg = math.degrees(math.atan2(y, x))
    return radius, yaw_deg


def _is_in_bounds(x, y, bounds):
    return (
        bounds["min_x"] <= x <= bounds["max_x"]
        and bounds["min_y"] <= y <= bounds["max_y"]
    )


def _point_in_rect(x, y, rect):
    if not rect:
        return True, 0.0, 0.0
    cx = float(rect["center"]["x"])
    cy = float(rect["center"]["y"])
    length = float(rect["length"])
    width = float(rect["width"])
    yaw_deg = float(rect.get("yaw_deg", 0.0))

    dx = x - cx
    dy = y - cy
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    local_x = dx * cos_yaw + dy * sin_yaw
    local_y = -dx * sin_yaw + dy * cos_yaw
    inside = abs(local_x) <= (length * 0.5) and abs(local_y) <= (width * 0.5)
    return inside, local_x, local_y


def _within_workspace(x, y, z, workspace):
    if not workspace:
        return True, None

    radius, yaw_deg = _radius_yaw(x, y)
    if radius < float(workspace["r_min"]) or radius > float(workspace["r_max"]):
        return False, "radius"
    if yaw_deg < float(workspace["yaw_min_deg"]) or yaw_deg > float(workspace["yaw_max_deg"]):
        return False, "yaw"
    if z < float(workspace["z_min"]) or z > float(workspace["z_max"]):
        return False, "z"
    return True, None


def _build_valid_bin_set(slice_bin_points, xy_bin):
    return {
        (int(round(float(item["x"]) / xy_bin)), int(round(float(item["y"]) / xy_bin)))
        for item in slice_bin_points
    }


def _surface_points(points, zslice_bounds):
    xy_bin = float(zslice_bounds["xy_bin"])
    bounds = {
        "min_x": float(zslice_bounds["min_x"]),
        "max_x": float(zslice_bounds["max_x"]),
        "min_y": float(zslice_bounds["min_y"]),
        "max_y": float(zslice_bounds["max_y"]),
    }
    valid_bins = _build_valid_bin_set(zslice_bounds["slice_bin_points"], xy_bin)

    grid_max = {}
    stats = {
        "input_points": 0,
        "rejected_outside_bounds": 0,
        "rejected_outside_mask": 0,
        "accepted_points": 0,
    }

    for x, y, z in points:
        stats["input_points"] += 1
        if not _is_in_bounds(x, y, bounds):
            stats["rejected_outside_bounds"] += 1
            continue

        bx = int(round(x / xy_bin))
        by = int(round(y / xy_bin))
        key = (bx, by)
        if key not in valid_bins:
            stats["rejected_outside_mask"] += 1
            continue

        bin_x = float(bx) * xy_bin
        bin_y = float(by) * xy_bin
        current = grid_max.get(key)
        if current is None or z > current["z"]:
            grid_max[key] = {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "bin_x": bin_x,
                "bin_y": bin_y,
                "bin_key": [bx, by],
            }
        stats["accepted_points"] += 1

    return list(grid_max.values()), stats


def _candidate_points(surface_points, rect, workspace, xy_bin, pattern):
    accepted = []
    stats = {
        "surface_bins": len(surface_points),
        "rejected_outside_rect": 0,
        "rejected_workspace_radius": 0,
        "rejected_workspace_yaw": 0,
        "rejected_workspace_z": 0,
        "candidate_bins": 0,
    }

    for point in surface_points:
        x = float(point["x"])
        y = float(point["y"])
        z = float(point["z"])

        inside_rect, local_x, local_y = _point_in_rect(x, y, rect)
        if not inside_rect:
            stats["rejected_outside_rect"] += 1
            continue

        ok, reason = _within_workspace(x, y, z, workspace)
        if not ok:
            if reason == "radius":
                stats["rejected_workspace_radius"] += 1
            elif reason == "yaw":
                stats["rejected_workspace_yaw"] += 1
            elif reason == "z":
                stats["rejected_workspace_z"] += 1
            continue

        radius, yaw_deg = _radius_yaw(x, y)
        accepted.append(
            {
                "x": x,
                "y": y,
                "z": z,
                "radius": radius,
                "yaw_deg": yaw_deg,
                "bin_x": float(point["bin_x"]),
                "bin_y": float(point["bin_y"]),
                "bin_key": point["bin_key"],
                "local_x": local_x,
                "local_y": local_y,
            }
        )

    if pattern == "boustrophedon":
        rows = {}
        for point in accepted:
            row = int(round(point["local_y"] / xy_bin))
            rows.setdefault(row, []).append(point)

        ordered = []
        for idx, row_key in enumerate(sorted(rows.keys())):
            row_points = rows[row_key]
            row_points.sort(key=lambda item: item["local_x"], reverse=bool(idx % 2))
            ordered.extend(row_points)
    else:
        ordered = sorted(accepted, key=lambda item: (item["local_y"], item["local_x"]))

    for idx, point in enumerate(ordered, start=1):
        point["candidate_index"] = idx

    stats["candidate_bins"] = len(ordered)
    return ordered, stats


def main():
    parser = argparse.ArgumentParser(description="把点云 JSON 过滤为模式1候选挖掘点。")
    parser.add_argument("--points", required=True, help="输入点云 JSON。支持 list 或 {points:[...]} 结构。")
    parser.add_argument(
        "--constraints",
        default="/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v14_urdf/mode1/constraints/workspace_constraints_360_z0.json",
        help="约束 JSON，必须包含 bucket_tip_zslice_bounds.slice_bin_points。",
    )
    parser.add_argument("--task-config", help="可选的模式1任务配置 JSON，读取 dig_area_rect/workspace/sampling.pattern。")
    parser.add_argument(
        "--out",
        default="/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v14_urdf/mode1/output/candidate_dig_points.json",
        help="输出候选挖掘点 JSON。",
    )
    args = parser.parse_args()

    constraints = _load_json(args.constraints)
    zslice_bounds = constraints["bucket_tip_zslice_bounds"]
    if "slice_bin_points" not in zslice_bounds:
        raise ValueError("constraints JSON 缺少 bucket_tip_zslice_bounds.slice_bin_points。")

    task_config = _load_json(args.task_config) if args.task_config else {}
    rect = task_config.get("dig_area_rect")
    workspace = task_config.get("workspace")
    sampling = task_config.get("sampling", {})
    pattern = sampling.get("pattern", "boustrophedon")
    xy_bin = float(zslice_bounds["xy_bin"])

    point_data = _load_json(args.points)
    points = _extract_points(point_data)

    surface_points, surface_stats = _surface_points(points, zslice_bounds)
    candidates, candidate_stats = _candidate_points(surface_points, rect, workspace, xy_bin, pattern)

    output = {
        "constraints_path": os.path.abspath(args.constraints),
        "points_path": os.path.abspath(args.points),
        "task_config_path": os.path.abspath(args.task_config) if args.task_config else None,
        "xy_bin": xy_bin,
        "pattern": pattern,
        "filter_stats": {
            **surface_stats,
            **candidate_stats,
        },
        "surface_points": surface_points,
        "candidate_dig_points": candidates,
    }
    _save_json(args.out, output)

    print(f"input_points={surface_stats['input_points']}")
    print(f"surface_bins={candidate_stats['surface_bins']}")
    print(f"candidate_bins={candidate_stats['candidate_bins']}")
    print(f"output={args.out}")


if __name__ == "__main__":
    main()
