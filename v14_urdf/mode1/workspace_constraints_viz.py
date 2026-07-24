import argparse
import json
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _mat_identity():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat_mul(a, b):
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = (
                a[i][0] * b[0][j]
                + a[i][1] * b[1][j]
                + a[i][2] * b[2][j]
                + a[i][3] * b[3][j]
            )
    return out


def _mat_trans(x, y, z):
    m = _mat_identity()
    m[0][3] = x
    m[1][3] = y
    m[2][3] = z
    return m


def _mat_rot_x(r):
    cr = math.cos(r)
    sr = math.sin(r)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, cr, -sr, 0.0],
        [0.0, sr, cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat_rot_y(p):
    cp = math.cos(p)
    sp = math.sin(p)
    return [
        [cp, 0.0, sp, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-sp, 0.0, cp, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat_rot_z(y):
    cy = math.cos(y)
    sy = math.sin(y)
    return [
        [cy, -sy, 0.0, 0.0],
        [sy, cy, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat_rot_rpy(roll, pitch, yaw):
    return _mat_mul(_mat_mul(_mat_rot_z(yaw), _mat_rot_y(pitch)), _mat_rot_x(roll))


def _mat_rot_axis_angle(ax, ay, az, angle):
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm == 0.0:
        return _mat_identity()
    ax /= norm
    ay /= norm
    az /= norm
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return [
        [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay, 0.0],
        [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax, 0.0],
        [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _parse_xyz(text):
    vals = [float(x) for x in (text or "0 0 0").split()]
    while len(vals) < 3:
        vals.append(0.0)
    return vals[0], vals[1], vals[2]


def _find_child(elem, name):
    found = elem.find(name)
    return found


def _parse_joint(joint_elem):
    joint_name = joint_elem.attrib["name"]
    joint_type = joint_elem.attrib.get("type", "")

    origin_elem = _find_child(joint_elem, "origin")
    if origin_elem is None:
        ox, oy, oz = 0.0, 0.0, 0.0
        rr, rp, ry = 0.0, 0.0, 0.0
    else:
        ox, oy, oz = _parse_xyz(origin_elem.attrib.get("xyz"))
        rr, rp, ry = _parse_xyz(origin_elem.attrib.get("rpy"))

    axis_elem = _find_child(joint_elem, "axis")
    if axis_elem is None:
        ax, ay, az = 0.0, 0.0, 1.0
    else:
        ax, ay, az = _parse_xyz(axis_elem.attrib.get("xyz"))

    limit_elem = _find_child(joint_elem, "limit")
    if limit_elem is None:
        lower = None
        upper = None
    else:
        lower = float(limit_elem.attrib.get("lower", "nan"))
        upper = float(limit_elem.attrib.get("upper", "nan"))
        if math.isnan(lower) or math.isnan(upper):
            lower, upper = None, None

    parent = _find_child(joint_elem, "parent").attrib["link"]
    child = _find_child(joint_elem, "child").attrib["link"]

    return {
        "name": joint_name,
        "type": joint_type,
        "parent": parent,
        "child": child,
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


def _rad_to_deg(rad):
    return rad * 180.0 / math.pi


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
            raise RuntimeError(f"Joint not found in URDF: {name}")
        ox, oy, oz = j["origin_xyz"]
        rr, rp, ry = j["origin_rpy"]
        ax, ay, az = j["axis"]
        jt = _mat_mul(_mat_trans(ox, oy, oz), _mat_rot_rpy(rr, rp, ry))
        if has_motion and j["type"] in ("revolute", "continuous"):
            angle = float(positions.get(name, 0.0))
            jt = _mat_mul(jt, _mat_rot_axis_angle(ax, ay, az, angle))
        t = _mat_mul(t, jt)
    return t[0][3], t[1][3], t[2][3]


def _collect_limits(joints, names):
    out = {}
    for name in names:
        joint = joints.get(name)
        if joint is None:
            continue
        lower, upper = joint["limit"]
        if lower is None or upper is None:
            continue
        out[name] = {
            "lower_rad": lower,
            "upper_rad": upper,
            "lower_deg": _rad_to_deg(lower),
            "upper_deg": _rad_to_deg(upper),
        }
    return out


class WorkspaceConstraintsViz(Node):
    def __init__(
        self,
        urdf_path,
        frame_id,
        boom_samples,
        arm_samples,
        bucket_samples,
        swing_rad,
        swing_samples,
        z_slice,
        z_tol,
        xy_bin,
        out_path,
    ):
        super().__init__("v14_workspace_constraints_viz")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/v14_urdf/workspace_constraints", qos)
        self.urdf_path = urdf_path
        self.frame_id = frame_id
        self.boom_samples = boom_samples
        self.arm_samples = arm_samples
        self.bucket_samples = bucket_samples
        self.swing_rad = swing_rad
        self.swing_samples = swing_samples
        self.z_slice = z_slice
        self.z_tol = z_tol
        self.xy_bin = xy_bin
        self.out_path = out_path
        self._publish_once()
        self.timer = self.create_timer(1.0, self._publish_once)

    def _publish_once(self):
        joints = _load_urdf_joints(self.urdf_path)

        limits = _collect_limits(joints, ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"])
        swing_lim = limits["swing_joint"]
        boom_lim = limits["boom_joint"]
        arm_lim = limits["arm_joint"]
        bucket_lim = limits["bucket_joint"]

        swing_vals = _linspace(swing_lim["lower_rad"], swing_lim["upper_rad"], self.swing_samples)
        boom_vals = _linspace(boom_lim["lower_rad"], boom_lim["upper_rad"], self.boom_samples)
        arm_vals = _linspace(arm_lim["lower_rad"], arm_lim["upper_rad"], self.arm_samples)
        bucket_vals = _linspace(bucket_lim["lower_rad"], bucket_lim["upper_rad"], self.bucket_samples)

        pts_xz = []
        zs_xz = []
        xs_xz = []
        rs_xz = []

        base_positions = {"swing_joint": self.swing_rad}
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
                    pts_xz.append((x, y, z))
                    xs_xz.append(x)
                    zs_xz.append(z)
                    rs_xz.append(math.sqrt(x * x + y * y))

        if not pts_xz:
            return

        min_x, max_x = min(xs_xz), max(xs_xz)
        min_z, max_z = min(zs_xz), max(zs_xz)
        min_r, max_r = min(rs_xz), max(rs_xz)

        slice_bins = {}
        slice_xs = []
        slice_ys = []
        slice_rs = []

        z_slice = float(self.z_slice)
        z_tol = float(self.z_tol)
        xy_bin = max(float(self.xy_bin), 1e-6)

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
                        if abs(z - z_slice) > z_tol:
                            continue
                        bx = int(round(x / xy_bin))
                        by = int(round(y / xy_bin))
                        key = (bx, by)
                        if key in slice_bins:
                            continue
                        slice_bins[key] = (float(bx) * xy_bin, float(by) * xy_bin)
                        slice_xs.append(float(bx) * xy_bin)
                        slice_ys.append(float(by) * xy_bin)
                        slice_rs.append(math.sqrt((float(bx) * xy_bin) ** 2 + (float(by) * xy_bin) ** 2))

        if slice_xs:
            slice_min_x, slice_max_x = min(slice_xs), max(slice_xs)
            slice_min_y, slice_max_y = min(slice_ys), max(slice_ys)
            slice_min_r, slice_max_r = min(slice_rs), max(slice_rs)
        else:
            slice_min_x = slice_max_x = 0.0
            slice_min_y = slice_max_y = 0.0
            slice_min_r = slice_max_r = 0.0

        # Export the de-duplicated XY bins as float coordinates so downstream
        # planners can use them as an explicit workspace mask instead of only
        # relying on min/max bounds.
        slice_bin_points = [
            {"x": x, "y": y}
            for _, (x, y) in sorted(slice_bins.items(), key=lambda item: item[0])
        ]

        out = {
            "urdf_path": self.urdf_path,
            "frame_id": self.frame_id,
            "sample_config": {
                "boom_samples": self.boom_samples,
                "arm_samples": self.arm_samples,
                "bucket_samples": self.bucket_samples,
                "swing_rad": self.swing_rad,
                "swing_deg": _rad_to_deg(self.swing_rad),
                "swing_samples": self.swing_samples,
                "z_slice": z_slice,
                "z_tol": z_tol,
                "xy_bin": xy_bin,
            },
            "joint_limits": limits,
            "bucket_tip_bounds": {
                "min_x": min_x,
                "max_x": max_x,
                "min_z": min_z,
                "max_z": max_z,
                "min_r": min_r,
                "max_r": max_r,
            },
            "bucket_tip_zslice_bounds": {
                "z_slice": z_slice,
                "z_tol": z_tol,
                "xy_bin": xy_bin,
                "min_x": slice_min_x,
                "max_x": slice_max_x,
                "min_y": slice_min_y,
                "max_y": slice_max_y,
                "min_r": slice_min_r,
                "max_r": slice_max_r,
                "bins": len(slice_bins),
                "slice_bin_points": slice_bin_points,
            },
        }
        if self.out_path:
            os.makedirs(os.path.dirname(self.out_path), exist_ok=True)
            with open(self.out_path, "w", encoding="utf-8") as f:
                json.dump(out, f, ensure_ascii=False, indent=2)

        now = self.get_clock().now().to_msg()

        marker_points = Marker()
        marker_points.header.frame_id = self.frame_id
        marker_points.header.stamp = now
        marker_points.ns = "workspace_points_xz"
        marker_points.id = 1
        marker_points.type = Marker.POINTS
        marker_points.action = Marker.ADD
        marker_points.pose.orientation.w = 1.0
        marker_points.scale.x = 0.01
        marker_points.scale.y = 0.01
        marker_points.color = ColorRGBA(r=0.10, g=0.65, b=0.95, a=0.75)

        z_span = max(max_z - min_z, 1e-6)
        for x, y, z in pts_xz:
            marker_points.points.append(Point(x=float(x), y=float(y), z=float(z)))
            t = (z - min_z) / z_span
            marker_points.colors.append(ColorRGBA(r=float(1.0 - t), g=float(t), b=0.15, a=0.85))

        marker_slice = Marker()
        marker_slice.header.frame_id = self.frame_id
        marker_slice.header.stamp = now
        marker_slice.ns = "workspace_points_zslice"
        marker_slice.id = 3
        marker_slice.type = Marker.POINTS
        marker_slice.action = Marker.ADD
        marker_slice.pose.orientation.w = 1.0
        marker_slice.scale.x = xy_bin
        marker_slice.scale.y = xy_bin
        marker_slice.color = ColorRGBA(r=0.20, g=0.95, b=0.35, a=0.70)
        for x, y in slice_bins.values():
            marker_slice.points.append(Point(x=float(x), y=float(y), z=float(z_slice)))

        marker_text = Marker()
        marker_text.header.frame_id = self.frame_id
        marker_text.header.stamp = now
        marker_text.ns = "workspace_text"
        marker_text.id = 2
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.pose.position.x = 0.1
        marker_text.pose.position.y = 0.0
        marker_text.pose.position.z = max_z + 0.2
        marker_text.pose.orientation.w = 1.0
        marker_text.scale.z = 0.08
        marker_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        marker_text.text = (
            f"swing: [{swing_lim['lower_deg']:.1f}, {swing_lim['upper_deg']:.1f}] deg\\n"
            f"bucket_tip bounds xz (swing={_rad_to_deg(self.swing_rad):.1f}deg)\\n"
            f"r: [{min_r:.2f}, {max_r:.2f}] m\\n"
            f"x: [{min_x:.2f}, {max_x:.2f}] m\\n"
            f"z: [{min_z:.2f}, {max_z:.2f}] m\\n"
            f"z-slice={z_slice:.2f} tol={z_tol:.2f} bins={len(slice_bins)}\\n"
            f"xy@z-slice x: [{slice_min_x:.2f}, {slice_max_x:.2f}] y: [{slice_min_y:.2f}, {slice_max_y:.2f}]\\n"
            f"boom: [{boom_lim['lower_deg']:.1f}, {boom_lim['upper_deg']:.1f}] deg\\n"
            f"arm: [{arm_lim['lower_deg']:.1f}, {arm_lim['upper_deg']:.1f}] deg\\n"
            f"bucket: [{bucket_lim['lower_deg']:.1f}, {bucket_lim['upper_deg']:.1f}] deg"
        )

        marker_array = MarkerArray()
        marker_array.markers.append(marker_points)
        marker_array.markers.append(marker_text)
        marker_array.markers.append(marker_slice)
        self.pub.publish(marker_array)


def main():
    default_urdf = (
        "/media/libo/libo_sn7100/ubuntu2204/shandong_ws/"
        "src/shandong/v14_urdf/describe_60FED/urdf/describe_60FED_calibrated.urdf"
    )
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=default_urdf)
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--boom-samples", type=int, default=35)
    parser.add_argument("--arm-samples", type=int, default=35)
    parser.add_argument("--bucket-samples", type=int, default=25)
    parser.add_argument("--swing-rad", type=float, default=0.0)
    parser.add_argument("--swing-samples", type=int, default=72)
    parser.add_argument("--z-slice", type=float, default=0.0)
    parser.add_argument("--z-tol", type=float, default=0.02)
    parser.add_argument("--xy-bin", type=float, default=0.05)
    parser.add_argument(
        "--out",
        default="/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v14_urdf/mode1/constraints/workspace_constraints.json",
    )
    args = parser.parse_args()

    rclpy.init()
    node = WorkspaceConstraintsViz(
        urdf_path=args.urdf,
        frame_id=args.frame,
        boom_samples=args.boom_samples,
        arm_samples=args.arm_samples,
        bucket_samples=args.bucket_samples,
        swing_rad=args.swing_rad,
        swing_samples=args.swing_samples,
        z_slice=args.z_slice,
        z_tol=args.z_tol,
        xy_bin=args.xy_bin,
        out_path=args.out,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
