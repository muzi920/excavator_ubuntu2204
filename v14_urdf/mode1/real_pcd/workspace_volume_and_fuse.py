import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from pcd_numpy_io import read_pcd_xyz, write_pcd_xyzrgb


def _mat_identity():
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _mat_trans(x, y, z):
    m = _mat_identity()
    m[0, 3] = float(x)
    m[1, 3] = float(y)
    m[2, 3] = float(z)
    return m


def _mat_rot_x(r):
    cr = math.cos(r)
    sr = math.sin(r)
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, cr, -sr, 0.0],
            [0.0, sr, cr, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _mat_rot_y(p):
    cp = math.cos(p)
    sp = math.sin(p)
    return np.array(
        [
            [cp, 0.0, sp, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-sp, 0.0, cp, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _mat_rot_z(y):
    cy = math.cos(y)
    sy = math.sin(y)
    return np.array(
        [
            [cy, -sy, 0.0, 0.0],
            [sy, cy, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _mat_rot_rpy(roll, pitch, yaw):
    return _mat_rot_z(yaw) @ _mat_rot_y(pitch) @ _mat_rot_x(roll)


def _mat_rot_axis_angle(ax, ay, az, angle):
    ax = float(ax)
    ay = float(ay)
    az = float(az)
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm == 0.0:
        return _mat_identity()
    ax /= norm
    ay /= norm
    az /= norm
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return np.array(
        [
            [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay, 0.0],
            [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax, 0.0],
            [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _parse_xyz(text):
    vals = [float(x) for x in (text or "0 0 0").split()]
    while len(vals) < 3:
        vals.append(0.0)
    return vals[0], vals[1], vals[2]


def _parse_joint(joint_elem):
    origin_elem = joint_elem.find("origin")
    if origin_elem is None:
        ox, oy, oz = 0.0, 0.0, 0.0
        rr, rp, ry = 0.0, 0.0, 0.0
    else:
        ox, oy, oz = _parse_xyz(origin_elem.attrib.get("xyz"))
        rr, rp, ry = _parse_xyz(origin_elem.attrib.get("rpy"))

    axis_elem = joint_elem.find("axis")
    if axis_elem is None:
        ax, ay, az = 0.0, 0.0, 1.0
    else:
        ax, ay, az = _parse_xyz(axis_elem.attrib.get("xyz"))

    limit_elem = joint_elem.find("limit")
    if limit_elem is None:
        lower = None
        upper = None
    else:
        lower = float(limit_elem.attrib.get("lower", "nan"))
        upper = float(limit_elem.attrib.get("upper", "nan"))
        if math.isnan(lower) or math.isnan(upper):
            lower, upper = None, None

    return {
        "name": joint_elem.attrib["name"],
        "type": joint_elem.attrib.get("type", ""),
        "origin_xyz": (ox, oy, oz),
        "origin_rpy": (rr, rp, ry),
        "axis": (ax, ay, az),
        "limit": (lower, upper),
    }


def _load_urdf_joints(urdf_path):
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for joint_elem in root.findall("joint"):
        joint = _parse_joint(joint_elem)
        joints[joint["name"]] = joint
    return joints


def _linspace(a, b, n):
    if n <= 1:
        return [a]
    step = (b - a) / float(n - 1)
    return [a + step * i for i in range(n)]


def _compute_chain_fk(joints, positions):
    chain = [
        ("swing_joint", True),
        ("boom_joint", True),
        ("arm_joint", True),
        ("bucket_joint", True),
        ("bucket_tip_fixed_joint", False),
    ]
    t = _mat_identity()
    for name, has_motion in chain:
        j = joints.get(name)
        if j is None:
            raise ValueError(f"Joint not found: {name}")
        ox, oy, oz = j["origin_xyz"]
        rr, rp, ry = j["origin_rpy"]
        ax, ay, az = j["axis"]
        jt = _mat_trans(ox, oy, oz) @ _mat_rot_rpy(rr, rp, ry)
        if has_motion and j["type"] in ("revolute", "continuous"):
            angle = float(positions.get(name, 0.0))
            jt = jt @ _mat_rot_axis_angle(ax, ay, az, angle)
        t = t @ jt
    return float(t[0, 3]), float(t[1, 3]), float(t[2, 3])


def _collect_limits(joints, names):
    out = {}
    for name in names:
        joint = joints.get(name)
        if not joint:
            continue
        lower, upper = joint["limit"]
        if lower is None or upper is None:
            continue
        out[name] = (float(lower), float(upper))
    return out


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


def _colorize_by_height(z, z_min, z_max):
    z_span = max(float(z_max - z_min), 1e-6)
    t = (float(z) - float(z_min)) / z_span
    t = max(0.0, min(1.0, t))
    r = int(round(255.0 * (1.0 - t)))
    g = int(round(255.0 * t))
    b = 40
    return r, g, b


def main():
    parser = argparse.ArgumentParser(description="生成末端 bucket tip 的 3D 可达域点云，并与原始点云融合输出。")
    parser.add_argument(
        "--urdf",
        default="src/shandong/v14_urdf/describe_60FED/urdf/describe_60FED_calibrated.urdf",
        help="标定 URDF 路径。",
    )
    parser.add_argument(
        "--original-pcd",
        required=True,
        help="原始点云（建议用 original_minus_removed.pcd）。",
    )
    parser.add_argument(
        "--out-dir",
        required=True,
        help="输出目录。",
    )
    parser.add_argument("--swing-samples", type=int, default=72)
    parser.add_argument("--boom-samples", type=int, default=24)
    parser.add_argument("--arm-samples", type=int, default=24)
    parser.add_argument("--bucket-samples", type=int, default=18)
    parser.add_argument(
        "--workspace-z-max",
        type=float,
        default=None,
        help="可选：仅保留 z <= workspace_z_max 的可达域点，用于定义“可作业区域”。",
    )
    parser.add_argument("--max-workspace-points", type=int, default=200000)
    parser.add_argument("--max-original-points", type=int, default=60000)
    args = parser.parse_args()

    joints = _load_urdf_joints(args.urdf)
    lim = _collect_limits(joints, ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"])
    swing_vals = _linspace(lim["swing_joint"][0], lim["swing_joint"][1], int(args.swing_samples))
    boom_vals = _linspace(lim["boom_joint"][0], lim["boom_joint"][1], int(args.boom_samples))
    arm_vals = _linspace(lim["arm_joint"][0], lim["arm_joint"][1], int(args.arm_samples))
    bucket_vals = _linspace(lim["bucket_joint"][0], lim["bucket_joint"][1], int(args.bucket_samples))

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
                    pts.append((x, y, z))

    workspace = np.array(pts, dtype=np.float32)
    if args.workspace_z_max is not None:
        workspace = workspace[workspace[:, 2] <= float(args.workspace_z_max)]
        if workspace.size == 0:
            raise ValueError("workspace_z_max 过滤后没有可达域点。")
    if args.max_workspace_points is not None and workspace.shape[0] > int(args.max_workspace_points):
        step = max(1, workspace.shape[0] // int(args.max_workspace_points))
        workspace = workspace[::step]

    z_min = float(workspace[:, 2].min())
    z_max = float(workspace[:, 2].max())
    workspace_rgb = np.array([_colorize_by_height(z, z_min, z_max) for z in workspace[:, 2]], dtype=np.uint8)

    original = read_pcd_xyz(args.original_pcd)
    if args.max_original_points is not None and original.shape[0] > int(args.max_original_points):
        step = max(1, original.shape[0] // int(args.max_original_points))
        original = original[::step]

    original_rgb = np.tile(np.array([[180, 180, 180]], dtype=np.uint8), (original.shape[0], 1))

    fused_xyz = np.concatenate([original, workspace], axis=0)
    fused_rgb = np.concatenate([original_rgb, workspace_rgb], axis=0)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    workspace_pcd = out_dir / "workspace_volume_rgb.pcd"
    fused_pcd = out_dir / "fused_original_plus_workspace_rgb.pcd"

    write_pcd_xyzrgb(workspace_pcd, workspace, workspace_rgb)
    write_pcd_xyzrgb(fused_pcd, fused_xyz, fused_rgb)

    ground_z = _estimate_ground_z(read_pcd_xyz(args.original_pcd))
    print(
        json.dumps(
            {
                "workspace_pcd": str(workspace_pcd.resolve()),
                "fused_pcd": str(fused_pcd.resolve()),
                "workspace_points": int(workspace.shape[0]),
                "original_points_used": int(original.shape[0]),
                "workspace_z_min": z_min,
                "workspace_z_max": z_max,
                "workspace_z_max_filter": args.workspace_z_max,
                "estimated_ground_z": float(ground_z),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
