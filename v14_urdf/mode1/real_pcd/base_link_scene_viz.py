import argparse
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from pcd_numpy_io import read_pcd_xyz
from workspace_volume_and_fuse import _colorize_by_height
from operable_region_and_fuse import _load_json


def _point_in_rect(x, y, rect):
    if not rect:
        return True
    cx = float(rect["center"]["x"])
    cy = float(rect["center"]["y"])
    length = float(rect["length"])
    width = float(rect["width"])
    yaw_deg = float(rect.get("yaw_deg", 0.0))

    dx = float(x) - cx
    dy = float(y) - cy
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    local_x = dx * cos_yaw + dy * sin_yaw
    local_y = -dx * sin_yaw + dy * cos_yaw
    return abs(local_x) <= (length * 0.5) and abs(local_y) <= (width * 0.5)


def _within_workspace(x, y, z, workspace):
    if not workspace:
        return True
    radius = math.sqrt(float(x) * float(x) + float(y) * float(y))
    yaw_deg = math.degrees(math.atan2(float(y), float(x)))
    return (
        float(workspace["r_min"]) <= radius <= float(workspace["r_max"])
        and float(workspace["yaw_min_deg"]) <= yaw_deg <= float(workspace["yaw_max_deg"])
        and float(workspace["z_min"]) <= float(z) <= float(workspace["z_max"])
    )


class BaseLinkSceneViz(Node):
    def __init__(
        self,
        scene_pcd,
        workspace_pcd,
        candidate_json,
        frame_id,
        max_scene_points,
        hide_workspace,
        workspace_alpha,
    ):
        super().__init__("mode1_base_link_scene_viz")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/v14_urdf/base_link_scene", qos)
        self.scene_pcd = scene_pcd
        self.workspace_pcd = workspace_pcd
        self.candidate_json = candidate_json
        self.frame_id = frame_id
        self.max_scene_points = max_scene_points
        self.hide_workspace = bool(hide_workspace)
        self.workspace_alpha = float(workspace_alpha)
        self.timer = self.create_timer(1.0, self._publish_once)
        self._publish_once()

    def _publish_once(self):
        scene = read_pcd_xyz(self.scene_pcd)
        candidate = _load_json(self.candidate_json) if self.candidate_json else {}
        surface_points = candidate.get("surface_points", [])
        candidates = candidate.get("candidate_dig_points", [])
        rect = candidate.get("dig_area_rect")
        roi_cfg = candidate.get("roi", {})
        workspace_cfg = {
            "r_min": 0.85,
            "r_max": 1.75,
            "yaw_min_deg": -50.0,
            "yaw_max_deg": 50.0,
            "z_min": roi_cfg.get("ground_z", -0.5),
            "z_max": roi_cfg.get("workspace_z_max", 0.5),
        }
        operable_surface_points = [
            point
            for point in surface_points
            if float(point["z"]) <= float(roi_cfg.get("workspace_z_max", 0.5))
            and _point_in_rect(point["x"], point["y"], rect)
            and _within_workspace(point["x"], point["y"], point["z"], workspace_cfg)
        ]

        if self.max_scene_points and len(scene) > self.max_scene_points:
            step = max(1, len(scene) // self.max_scene_points)
            scene = scene[::step]

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        m_scene = Marker()
        m_scene.header.frame_id = self.frame_id
        m_scene.header.stamp = now
        m_scene.ns = "scene"
        m_scene.id = 1
        m_scene.type = Marker.POINTS
        m_scene.action = Marker.ADD
        m_scene.pose.orientation.w = 1.0
        m_scene.scale = Vector3(x=0.03, y=0.03, z=0.03)
        m_scene.color = ColorRGBA(r=0.75, g=0.75, b=0.75, a=0.60)
        for x, y, z in scene:
            m_scene.points.append(Point(x=float(x), y=float(y), z=float(z)))
        markers.markers.append(m_scene)

        if not self.hide_workspace:
            workspace = self._read_workspace_xyz(self.workspace_pcd)
            m_workspace = Marker()
            m_workspace.header.frame_id = self.frame_id
            m_workspace.header.stamp = now
            m_workspace.ns = "workspace"
            m_workspace.id = 2
            m_workspace.type = Marker.POINTS
            m_workspace.action = Marker.ADD
            m_workspace.pose.orientation.w = 1.0
            m_workspace.scale = Vector3(x=0.02, y=0.02, z=0.02)
            z_min = float(workspace[:, 2].min()) if len(workspace) else 0.0
            z_max = float(workspace[:, 2].max()) if len(workspace) else 1.0
            for x, y, z in workspace:
                m_workspace.points.append(Point(x=float(x), y=float(y), z=float(z)))
                r, g, b = _colorize_by_height(z, z_min, z_max)
                m_workspace.colors.append(
                    ColorRGBA(r=r / 255.0, g=g / 255.0, b=b / 255.0, a=self.workspace_alpha)
                )
            markers.markers.append(m_workspace)
        else:
            m_workspace_del = Marker()
            m_workspace_del.header.frame_id = self.frame_id
            m_workspace_del.header.stamp = now
            m_workspace_del.ns = "workspace"
            m_workspace_del.id = 2
            m_workspace_del.action = Marker.DELETE
            markers.markers.append(m_workspace_del)

        m_surface = Marker()
        m_surface.header.frame_id = self.frame_id
        m_surface.header.stamp = now
        m_surface.ns = "surface_points"
        m_surface.id = 3
        m_surface.type = Marker.POINTS
        m_surface.action = Marker.ADD
        m_surface.pose.orientation.w = 1.0
        m_surface.scale = Vector3(x=0.045, y=0.045, z=0.045)
        m_surface.color = ColorRGBA(r=0.98, g=0.92, b=0.10, a=0.88)
        for point in operable_surface_points:
            m_surface.points.append(
                Point(x=float(point["x"]), y=float(point["y"]), z=float(point["z"]) + 0.01)
            )
        markers.markers.append(m_surface)

        m_candidates = Marker()
        m_candidates.header.frame_id = self.frame_id
        m_candidates.header.stamp = now
        m_candidates.ns = "candidates"
        m_candidates.id = 4
        m_candidates.type = Marker.SPHERE_LIST
        m_candidates.action = Marker.ADD
        m_candidates.pose.orientation.w = 1.0
        m_candidates.scale = Vector3(x=0.08, y=0.08, z=0.08)
        m_candidates.color = ColorRGBA(r=1.0, g=0.55, b=0.05, a=0.98)
        for point in candidates:
            m_candidates.points.append(
                Point(x=float(point["x"]), y=float(point["y"]), z=float(point["z"]) + 0.03)
            )
        markers.markers.append(m_candidates)

        self.pub.publish(markers)

    def _read_workspace_xyz(self, path):
        path = Path(path)
        with path.open("rb") as f:
            points_count = None
            fields = None
            sizes = None
            types = None

            while True:
                line = f.readline()
                if not line:
                    raise ValueError(f"{path} 在 DATA 字段前提前结束。")
                text = line.decode("ascii", "replace").strip()
                if text.startswith("FIELDS "):
                    fields = text.split()[1:]
                elif text.startswith("SIZE "):
                    sizes = [int(x) for x in text.split()[1:]]
                elif text.startswith("TYPE "):
                    types = text.split()[1:]
                elif text.startswith("POINTS "):
                    points_count = int(text.split()[1])
                elif text.startswith("DATA "):
                    mode = text.split()[1].lower()
                    if mode != "binary":
                        raise ValueError(f"{path} 仅支持 binary PCD，当前是 {mode}。")
                    break

            if fields == ["x", "y", "z"]:
                raw = f.read(points_count * 12)
                arr = np.frombuffer(raw, dtype=np.float32).reshape(points_count, 3).copy()
                return arr[np.isfinite(arr).all(axis=1)]

            if fields == ["x", "y", "z", "rgb"] and sizes == [4, 4, 4, 4] and types == ["F", "F", "F", "F"]:
                raw = f.read(points_count * 16)
                arr = np.frombuffer(raw, dtype=np.float32).reshape(points_count, 4).copy()
                xyz = arr[:, :3]
                return xyz[np.isfinite(xyz).all(axis=1)]

        raise ValueError(f"不支持的 workspace PCD FIELDS: {fields}")


def main():
    parser = argparse.ArgumentParser(description="把 base_link 场景、作业域和候选点一起发布到 RViz。")
    parser.add_argument("--scene-pcd", required=True)
    parser.add_argument("--workspace-pcd", required=True)
    parser.add_argument("--candidate-json", required=True)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--max-scene-points", type=int, default=15000)
    parser.add_argument("--hide-workspace", action="store_true")
    parser.add_argument("--workspace-alpha", type=float, default=0.10)
    args = parser.parse_args()

    rclpy.init()
    node = BaseLinkSceneViz(
        scene_pcd=args.scene_pcd,
        workspace_pcd=args.workspace_pcd,
        candidate_json=args.candidate_json,
        frame_id=args.frame_id,
        max_scene_points=args.max_scene_points,
        hide_workspace=args.hide_workspace,
        workspace_alpha=args.workspace_alpha,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
