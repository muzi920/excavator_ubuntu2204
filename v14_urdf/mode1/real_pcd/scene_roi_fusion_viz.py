import argparse
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from pcd_numpy_io import read_pcd_xyz


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


class SceneRoiFusionViz(Node):
    def __init__(self, scene_pcd, heap_json, operable_json, frame_id, max_scene_points):
        super().__init__("mode1_scene_roi_fusion_viz")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/v14_urdf/scene_roi_fusion", qos)
        self.scene_pcd = scene_pcd
        self.heap_json = heap_json
        self.operable_json = operable_json
        self.frame_id = frame_id
        self.max_scene_points = max_scene_points
        self.timer = self.create_timer(1.0, self._publish_once)
        self._publish_once()

    def _publish_once(self):
        scene = read_pcd_xyz(self.scene_pcd)
        if self.max_scene_points and len(scene) > self.max_scene_points:
            step = max(1, len(scene) // self.max_scene_points)
            scene = scene[::step]

        heap = _load_json(self.heap_json)
        operable = _load_json(self.operable_json)
        heap_points = heap.get("surface_points", [])
        operable_points = operable.get("surface_points", [])

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
        m_scene.color = ColorRGBA(r=0.75, g=0.75, b=0.75, a=0.55)
        for x, y, z in scene:
            m_scene.points.append(Point(x=float(x), y=float(y), z=float(z)))
        markers.markers.append(m_scene)

        m_heap = Marker()
        m_heap.header.frame_id = self.frame_id
        m_heap.header.stamp = now
        m_heap.ns = "heap_roi"
        m_heap.id = 2
        m_heap.type = Marker.POINTS
        m_heap.action = Marker.ADD
        m_heap.pose.orientation.w = 1.0
        m_heap.scale = Vector3(x=0.05, y=0.05, z=0.05)
        m_heap.color = ColorRGBA(r=1.0, g=0.30, b=0.25, a=0.92)
        for point in heap_points:
            m_heap.points.append(
                Point(x=float(point["x"]), y=float(point["y"]), z=float(point["z"]) + 0.01)
            )
        markers.markers.append(m_heap)

        m_operable = Marker()
        m_operable.header.frame_id = self.frame_id
        m_operable.header.stamp = now
        m_operable.ns = "operable_region"
        m_operable.id = 3
        m_operable.type = Marker.SPHERE_LIST
        m_operable.action = Marker.ADD
        m_operable.pose.orientation.w = 1.0
        m_operable.scale = Vector3(x=0.08, y=0.08, z=0.08)
        m_operable.color = ColorRGBA(r=0.98, g=0.92, b=0.10, a=0.96)
        for point in operable_points:
            m_operable.points.append(
                Point(x=float(point["x"]), y=float(point["y"]), z=float(point["z"]) + 0.03)
            )
        markers.markers.append(m_operable)

        self.pub.publish(markers)


def main():
    parser = argparse.ArgumentParser(description="在 RViz 中显示场景点云、土堆 ROI 和待作业区域。")
    parser.add_argument("--scene-pcd", required=True)
    parser.add_argument("--heap-json", required=True)
    parser.add_argument("--operable-json", required=True)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--max-scene-points", type=int, default=15000)
    args = parser.parse_args()

    rclpy.init()
    node = SceneRoiFusionViz(
        scene_pcd=args.scene_pcd,
        heap_json=args.heap_json,
        operable_json=args.operable_json,
        frame_id=args.frame_id,
        max_scene_points=args.max_scene_points,
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
