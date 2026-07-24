import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


CURRENT_DIR = Path(__file__).resolve().parent
REAL_PCD_DIR = CURRENT_DIR.parent
if str(REAL_PCD_DIR) not in sys.path:
    sys.path.append(str(REAL_PCD_DIR))

from pcd_numpy_io import read_pcd_xyz


def _save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def _bin_points(points, xy_bin):
    bins = {}
    for x, y, z in points:
        bx = int(round(float(x) / float(xy_bin)))
        by = int(round(float(y) / float(xy_bin)))
        bins.setdefault((bx, by), []).append((float(x), float(y), float(z)))
    return bins


def _to_candidate(vals, bx, by, xy_bin):
    arr = np.array(vals, dtype=np.float32)
    x = float(np.mean(arr[:, 0]))
    y = float(np.mean(arr[:, 1]))
    # 取稍靠上的表层，但不过分取最高点
    z = float(np.quantile(arr[:, 2], 0.7))
    radius = math.sqrt(x * x + y * y)
    yaw_deg = math.degrees(math.atan2(y, x))
    return {
        "x": x,
        "y": y,
        "z": z,
        "radius": radius,
        "yaw_deg": yaw_deg,
        "bin_x": float(bx) * float(xy_bin),
        "bin_y": float(by) * float(xy_bin),
        "bin_key": [int(bx), int(by)],
        "sample_count": int(len(vals)),
    }


def main():
    parser = argparse.ArgumentParser(description="把 operable_region_points.pcd 转成 mode1 候选挖掘点 JSON。")
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--xy-bin", type=float, default=0.08)
    parser.add_argument("--max-candidates", type=int, default=3)
    parser.add_argument("--min-x", type=float, default=0.5)
    parser.add_argument("--min-radius", type=float, default=0.85)
    parser.add_argument("--max-radius", type=float, default=1.85)
    parser.add_argument("--yaw-min-deg", type=float, default=-35.0)
    parser.add_argument("--yaw-max-deg", type=float, default=35.0)
    args = parser.parse_args()

    points = read_pcd_xyz(args.pcd)
    mask = np.isfinite(points).all(axis=1)
    mask &= points[:, 0] > float(args.min_x)
    points = points[mask]
    if len(points) == 0:
        raise ValueError("筛选后没有 operable points。")

    bins = _bin_points(points, float(args.xy_bin))
    candidates = []
    rejected = {
        "outside_radius": 0,
        "outside_yaw": 0,
    }
    for (bx, by), vals in bins.items():
        item = _to_candidate(vals, bx, by, float(args.xy_bin))
        if not (float(args.min_radius) <= float(item["radius"]) <= float(args.max_radius)):
            rejected["outside_radius"] += 1
            continue
        if not (float(args.yaw_min_deg) <= float(item["yaw_deg"]) <= float(args.yaw_max_deg)):
            rejected["outside_yaw"] += 1
            continue
        candidates.append(item)

    # 优先样本数多、z 适中、x 更靠前的点
    candidates.sort(
        key=lambda item: (
            item["sample_count"],
            -abs(item["yaw_deg"]),
            item["x"],
            item["z"],
        ),
        reverse=True,
    )
    candidates = candidates[: int(args.max_candidates)]
    for idx, item in enumerate(candidates, start=1):
        item["candidate_index"] = idx

    payload = {
        "source": "ceres_match_operable_points",
        "points_path": str(Path(args.pcd).resolve()),
        "xy_bin": float(args.xy_bin),
        "filter_stats": {
            "input_points": int(len(points)),
            "surface_bins": int(len(bins)),
            "rejected_workspace_radius": int(rejected["outside_radius"]),
            "rejected_workspace_yaw": int(rejected["outside_yaw"]),
            "candidate_bins": int(len(candidates)),
        },
        "candidate_dig_points": candidates,
    }
    _save_json(args.out, payload)
    print(json.dumps(payload["filter_stats"], ensure_ascii=False))


if __name__ == "__main__":
    main()
