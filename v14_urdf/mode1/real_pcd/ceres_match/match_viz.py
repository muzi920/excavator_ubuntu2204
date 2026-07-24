import argparse
import json
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


CURRENT_DIR = Path(__file__).resolve().parent
REAL_PCD_DIR = CURRENT_DIR.parent
if str(REAL_PCD_DIR) not in sys.path:
    sys.path.append(str(REAL_PCD_DIR))

from pcd_numpy_io import read_pcd_xyz


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


class MatchViz(Node):
    def __init__(self, scene_pcd, match_json, frame_id, max_scene_points):
        super().__init__("mode1_ceres_match_viz")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/v14_urdf/ceres_match", qos)
        self.scene_pcd = scene_pcd
        self.match_json = match_json
        self.frame_id = frame_id
        self.max_scene_points = max_scene_points
        self.timer = self.create_timer(1.0, self._publish_once)
        self._publish_once()

    def _publish_once(self):
        scene = read_pcd_xyz(self.scene_pcd)
        if self.max_scene_points and len(scene) > self.max_scene_points:
            step = max(1, len(scene) // self.max_scene_points)
            scene = scene[::step]

        match = _load_json(self.match_json)
        matched_workspace = read_pcd_xyz(match["matched_workspace_pcd"])
        operable_region = read_pcd_xyz(match["operable_region_pcd"])

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        m_scene = Marker()
        m_scene.header.frame_id = self.frame_id
        m_scene.header.stamp = now
        m_scene.ns = "scene"
        m_scene.id = 1
        m_scene.type = Marker.POINTS
        m_scene.action = Marker.ADD
        m_scene.pose.orientation.w = 1.0
        m_scene.scale = Vector3(x=0.03, y=0.03, z=0.03)
        m_scene.color = ColorRGBA(r=0.75, g=0.75, b=0.75, a=0.55)
        for x, y, z in scene:
            m_scene.points.append(Point(x=float(x), y=float(y), z=float(z)))
        markers.markers.append(m_scene)

        m_match = Marker()
        m_match.header.frame_id = self.frame_id
        m_match.header.stamp = now
        m_match.ns = "matched_workspace"
        m_match.id = 2
        m_match.type = Marker.POINTS
        m_match.action = Marker.ADD
        m_match.pose.orientation.w = 1.0
        m_match.scale = Vector3(x=0.05, y=0.05, z=0.05)
        m_match.color = ColorRGBA(r=0.30, g=0.95, b=0.35, a=0.80)
        for x, y, z in matched_workspace:
            m_match.points.append(Point(x=float(x), y=float(y), z=float(z)))
        markers.markers.append(m_match)

        m_operable = Marker()
        m_operable.header.frame_id = self.frame_id
        m_operable.header.stamp = now
        m_operable.ns = "operable_region"
        m_operable.id = 3
        m_operable.type = Marker.SPHERE_LIST
        m_operable.action = Marker.ADD
        m_operable.pose.orientation.w = 1.0
        m_operable.scale = Vector3(x=0.08, y=0.08, z=0.08)
        m_operable.color = ColorRGBA(r=0.98, g=0.92, b=0.10, a=0.95)
        for x, y, z in operable_region:
            m_operable.points.append(Point(x=float(x), y=float(y), z=float(z) + 0.02))
        markers.markers.append(m_operable)

        self.pub.publish(markers)


def main():
    parser = argparse.ArgumentParser(description="RViz 显示点云匹配结果。")
    parser.add_argument("--scene-pcd", required=True)
    parser.add_argument("--match-json", required=True)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--max-scene-points", type=int, default=15000)
    args = parser.parse_args()

    rclpy.init()
    node = MatchViz(args.scene_pcd, args.match_json, args.frame_id, args.max_scene_points)
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
