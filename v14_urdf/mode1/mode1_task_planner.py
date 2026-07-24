import argparse
import copy
import json
import os
import sys


CURRENT_DIR = os.path.dirname(__file__)
V14_DIR = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
if V14_DIR not in sys.path:
    sys.path.append(V14_DIR)

from area_sampling import order_local_points, point_in_rect, sample_rect_points
from dump_strategy import build_dump_point
from point_to_dig_dump_trajectory import DigDumpPlanner
from workspace import build_constraints_mask, point_in_constraints_mask, within_workspace


DEFAULT_POSES = {
    "init_pose": {
        "swing_yaw": 0.0,
        "boom_swing": 10.0,
        "arm_boom": 20.0,
        "bucket_arm": -30.0,
    },
    "cycle_transit_pose": {
        "boom_swing": 25.0,
        "arm_boom": 55.0,
        "bucket_arm": -10.0,
    },
    "home_pose": {
        "swing_yaw": 0.0,
        "boom_swing": 5.0,
        "arm_boom": 10.0,
        "bucket_arm": -80.0,
    },
}

JOINT_ORDER = ["swing_yaw", "boom_swing", "arm_boom", "bucket_arm"]


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _save_json(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def _merge_poses(task_config):
    poses = copy.deepcopy(DEFAULT_POSES)
    user_poses = task_config.get("poses", {})
    for pose_name, pose_values in user_poses.items():
        if pose_name not in poses or not isinstance(pose_values, dict):
            continue
        poses[pose_name].update(pose_values)
    return poses


def _load_candidates(candidate_path, max_candidates=None):
    data = _load_json(candidate_path)
    candidates = data.get("candidate_dig_points", [])
    if not candidates:
        raise ValueError("candidate JSON 中没有 candidate_dig_points。")

    if max_candidates is not None:
        candidates = candidates[: max(0, int(max_candidates))]
    if not candidates:
        raise ValueError("max-candidates 过滤后没有可用候选点。")

    dig_points = []
    for item in candidates:
        dig_points.append(
            {
                "candidate_index": int(item["candidate_index"]),
                "x": float(item["x"]),
                "y": float(item["y"]),
                "z": float(item["z"]),
                "radius": float(item["radius"]),
                "yaw_deg": float(item["yaw_deg"]),
                "bin_x": float(item["bin_x"]),
                "bin_y": float(item["bin_y"]),
                "bin_key": list(item["bin_key"]),
            }
        )
    return data, dig_points


def _sample_dig_points_from_area(task_config, constraints_path=None, max_candidates=None):
    rect = task_config.get("dig_area_rect")
    if not isinstance(rect, dict):
        raise ValueError("未提供 candidate-json 时，task-config 必须包含 dig_area_rect。")

    sampling = task_config.get("sampling", {})
    workspace = task_config.get("workspace")
    constraints_data = _load_json(constraints_path) if constraints_path else None
    constraints_mask = build_constraints_mask(constraints_data)
    pattern = str(sampling.get("pattern", "boustrophedon")).lower()

    slice_bin_points = None
    if constraints_data:
        zslice_bounds = constraints_data.get("bucket_tip_zslice_bounds", {})
        if isinstance(zslice_bounds.get("slice_bin_points"), list) and zslice_bounds.get("slice_bin_points"):
            slice_bin_points = zslice_bounds["slice_bin_points"]

    if slice_bin_points:
        rect_points = []
        for idx, item in enumerate(slice_bin_points, start=1):
            x = float(item["x"])
            y = float(item["y"])
            inside, local_x, local_y = point_in_rect(rect, x, y)
            if not inside:
                continue
            rect_points.append(
                {
                    "sample_index": idx,
                    "local_x": local_x,
                    "local_y": local_y,
                    "x": x,
                    "y": y,
                    "z": float(rect["center"].get("z", 0.0)),
                }
            )
        raw_points = order_local_points(
            rect_points,
            pattern=pattern,
            row_step=float(constraints_mask["xy_bin"]) if constraints_mask else float(sampling.get("step_along_width", 0.15)),
        )
    else:
        raw_points = sample_rect_points(rect, sampling)

    filter_stats = {
        "generated_points": len(raw_points),
        "rejected_workspace_radius": 0,
        "rejected_workspace_yaw": 0,
        "rejected_workspace_z": 0,
        "rejected_constraints_bounds": 0,
        "rejected_constraints_mask": 0,
        "candidate_bins": 0,
    }

    dig_points = []
    candidate_index = 1
    for item in raw_points:
        x = float(item["x"])
        y = float(item["y"])
        z = float(item["z"])

        ok, reason, radius, yaw_deg = within_workspace(x, y, z, workspace)
        if not ok:
            if reason == "radius":
                filter_stats["rejected_workspace_radius"] += 1
            elif reason == "yaw":
                filter_stats["rejected_workspace_yaw"] += 1
            elif reason == "z":
                filter_stats["rejected_workspace_z"] += 1
            continue

        in_mask, mask_reason, bin_x, bin_y = point_in_constraints_mask(x, y, constraints_mask)
        if not in_mask:
            if mask_reason == "bounds":
                filter_stats["rejected_constraints_bounds"] += 1
            elif mask_reason == "mask":
                filter_stats["rejected_constraints_mask"] += 1
            continue

        if bin_x is None or bin_y is None:
            bin_x, bin_y = x, y
        if constraints_mask:
            xy_bin = float(constraints_mask["xy_bin"])
            bin_key = [int(round(float(bin_x) / xy_bin)), int(round(float(bin_y) / xy_bin))]
        else:
            bin_key = None

        dig_points.append(
            {
                "candidate_index": candidate_index,
                "sample_index": int(item["sample_index"]),
                "row_index": int(item["row_index"]),
                "col_index": int(item["col_index"]),
                "x": x,
                "y": y,
                "z": z,
                "radius": radius,
                "yaw_deg": yaw_deg,
                "bin_x": float(bin_x),
                "bin_y": float(bin_y),
                "bin_key": bin_key,
                "local_x": float(item["local_x"]),
                "local_y": float(item["local_y"]),
                "source": "task_config.dig_area_rect",
            }
        )
        candidate_index += 1

    if max_candidates is not None:
        dig_points = dig_points[: max(0, int(max_candidates))]

    filter_stats["candidate_bins"] = len(dig_points)
    if not dig_points:
        raise ValueError("预设区域采样后没有可用挖掘点，请检查 workspace、dig_area_rect 或 constraints。")

    return {
        "source": "area_sampling",
        "constraints_path": os.path.abspath(constraints_path) if constraints_path else None,
        "grid_source": "constraints_bins" if slice_bin_points else "rect_sampling",
        "filter_stats": filter_stats,
        "candidate_dig_points": dig_points,
    }, dig_points


def _make_step(step_id, joint, description, target_val, is_init_step=False):
    step = {
        "step": step_id,
        "joint": joint,
        "description": description,
        "ch1_mv": 0,
        "ch2_mv": 0,
        "ch3_mv": 3500,
        "ramp_up_s": 0.2,
        "ramp_down_s": 0.2,
        "target_val": round(float(target_val), 2),
    }
    if is_init_step:
        step["is_init_step"] = True
    return step


def _pose_to_steps(start_step, pose_name, pose, description_prefix, is_init_step=False):
    steps = []
    step_id = start_step
    label_map = {
        "swing_yaw": "回转",
        "boom_swing": "大臂",
        "arm_boom": "小臂",
        "bucket_arm": "铲斗",
    }
    for joint in JOINT_ORDER:
        if joint not in pose:
            continue
        desc = f"{description_prefix}-{label_map[joint]}"
        steps.append(_make_step(step_id, joint, desc, pose[joint], is_init_step=is_init_step))
        steps[-1]["segment_name"] = pose_name
        step_id += 1
    return steps, step_id


def _retag_cycle_steps(raw_steps, start_step, cycle_index):
    steps = []
    step_id = start_step
    for item in raw_steps:
        step = dict(item)
        step["step"] = step_id
        step["description"] = f"[循环{cycle_index:02d}] {item.get('description', '')}"
        step["cycle_index"] = cycle_index
        step.pop("is_init_step", None)
        steps.append(step)
        step_id += 1
    return steps, step_id


def _planner_cycle_metadata(cycle_index, dig_point, cycle_result, start_step, end_step):
    dig_meta = cycle_result["metadata"]["dig_point"]
    return {
        "cycle_index": cycle_index,
        "candidate_index": dig_point["candidate_index"],
        "step_range": [start_step, end_step],
        "dig_point": {
            "x": dig_point["x"],
            "y": dig_point["y"],
            "z": dig_point["z"],
            "radius": dig_meta["radius"],
            "swing_yaw": dig_meta["swing_yaw"],
            "bin_x": dig_point["bin_x"],
            "bin_y": dig_point["bin_y"],
            "bin_key": dig_point["bin_key"],
            "source": dig_point.get("source"),
        },
    }


def build_task(task_config, output_path, candidate_path=None, constraints_path=None, max_candidates=None):
    if candidate_path:
        candidate_data, dig_points = _load_candidates(candidate_path, max_candidates=max_candidates)
    else:
        candidate_data, dig_points = _sample_dig_points_from_area(
            task_config,
            constraints_path=constraints_path,
            max_candidates=max_candidates,
        )

    dump_point = build_dump_point(task_config)
    poses = _merge_poses(task_config)
    planner = DigDumpPlanner()

    script = []
    segments = []
    cycle_plans = []
    skipped_candidates = []
    step_id = 1

    init_steps, step_id = _pose_to_steps(
        step_id,
        "init_segment",
        poses["init_pose"],
        "初始化",
        is_init_step=True,
    )
    if init_steps:
        script.extend(init_steps)
        segments.append(
            {
                "name": "init_segment",
                "type": "init",
                "step_range": [init_steps[0]["step"], init_steps[-1]["step"]],
            }
        )

    executed_cycle_index = 0
    for requested_cycle_index, dig_point in enumerate(dig_points, start=1):
        try:
            cycle_result = planner.plan(
                dig_point=(dig_point["x"], dig_point["y"], dig_point["z"]),
                dump_point=(dump_point["x"], dump_point["y"], dump_point["z"]),
            )
        except ValueError as exc:
            skipped_candidates.append(
                {
                    "requested_cycle_index": requested_cycle_index,
                    "candidate_index": dig_point["candidate_index"],
                    "dig_point": dig_point,
                    "reason": str(exc),
                }
            )
            continue

        executed_cycle_index += 1
        cycle_index = executed_cycle_index

        cycle_steps, step_id = _retag_cycle_steps(cycle_result["script"], step_id, cycle_index)
        if cycle_steps:
            script.extend(cycle_steps)
            segments.append(
                {
                    "name": f"cycle_{cycle_index:02d}",
                    "type": "cycle",
                    "cycle_index": cycle_index,
                    "step_range": [cycle_steps[0]["step"], cycle_steps[-1]["step"]],
                }
            )
            cycle_plans.append(
                _planner_cycle_metadata(
                    cycle_index=cycle_index,
                    dig_point=dig_point,
                    cycle_result=cycle_result,
                    start_step=cycle_steps[0]["step"],
                    end_step=cycle_steps[-1]["step"],
                )
            )
            cycle_plans[-1]["requested_cycle_index"] = requested_cycle_index

        transit_steps, step_id = _pose_to_steps(
            step_id,
            f"cycle_{cycle_index:02d}_transit",
            poses["cycle_transit_pose"],
            f"循环{cycle_index:02d}结束-过渡位",
        )
        if transit_steps:
            script.extend(transit_steps)
            segments.append(
                {
                    "name": f"cycle_{cycle_index:02d}_transit",
                    "type": "transit",
                    "cycle_index": cycle_index,
                    "step_range": [transit_steps[0]["step"], transit_steps[-1]["step"]],
                }
            )

    home_steps, step_id = _pose_to_steps(
        step_id,
        "home_segment",
        poses["home_pose"],
        "归位",
    )
    if home_steps:
        script.extend(home_steps)
        segments.append(
            {
                "name": "home_segment",
                "type": "home",
                "step_range": [home_steps[0]["step"], home_steps[-1]["step"]],
            }
        )

    if not cycle_plans:
        raise ValueError("没有可执行的循环段：全部候选点都未通过单点轨迹规划。")

    metadata = {
        "task_name": task_config.get("task_name", "mode1_multi_dig_task"),
        "task_type": "mode1_multi_dig",
        "candidate_path": os.path.abspath(candidate_path) if candidate_path else None,
        "constraints_path": os.path.abspath(constraints_path) if constraints_path else candidate_data.get("constraints_path"),
        "task_config_path": os.path.abspath(task_config.get("_path", "")) if task_config.get("_path") else None,
        "output_path": os.path.abspath(output_path),
        "requested_cycle_count": len(dig_points),
        "cycle_count": len(cycle_plans),
        "dig_point_source": candidate_data.get("source", "candidate_json"),
        "grid_source": candidate_data.get("grid_source"),
        "init_pose": poses["init_pose"],
        "cycle_transit_pose": poses["cycle_transit_pose"],
        "home_pose": poses["home_pose"],
        "dump_point": dump_point,
        "dig_points": dig_points,
        "candidate_filter_stats": candidate_data.get("filter_stats", {}),
        "segments": segments,
        "cycles": cycle_plans,
        "skipped_candidates": skipped_candidates,
    }
    return {"metadata": metadata, "script": script}


def main():
    parser = argparse.ArgumentParser(description="根据候选点或预设区域配置生成 mode1 多点任务 JSON。")
    parser.add_argument(
        "--candidate-json",
        help="pointcloud_to_dig_points.py 输出的 candidate JSON。若不提供，则直接从 task-config 的 dig_area_rect 采样。",
    )
    parser.add_argument(
        "--constraints-json",
        help="可选：预设区域采样时叠加的 constraints JSON，用于进一步按可作业 mask 过滤。",
    )
    parser.add_argument(
        "--task-config",
        required=True,
        help="模式1任务配置 JSON。直接区域规划时至少需要 dig_area_rect、sampling、dump_strategy/dump_point。",
    )
    parser.add_argument(
        "--max-candidates",
        type=int,
        help="可选：只取前 N 个候选挖掘点，便于先做小规模验证。",
    )
    parser.add_argument(
        "--out",
        default=os.path.join(CURRENT_DIR, "output", "mode1_task_plan.json"),
        help="输出任务 JSON。",
    )
    args = parser.parse_args()

    task_config = _load_json(args.task_config)
    task_config["_path"] = args.task_config
    result = build_task(
        task_config=task_config,
        output_path=args.out,
        candidate_path=args.candidate_json,
        constraints_path=args.constraints_json,
        max_candidates=args.max_candidates,
    )

    _save_json(args.out, result)
    print(f"task_name={result['metadata']['task_name']}")
    print(f"cycle_count={result['metadata']['cycle_count']}")
    print(f"script_steps={len(result['script'])}")
    print(f"output={args.out}")


if __name__ == "__main__":
    main()
