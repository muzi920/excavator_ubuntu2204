import argparse
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _color_for_height(z, z_min, z_max):
    if z_max <= z_min:
        return ColorRGBA(r=0.2, g=0.9, b=0.3, a=0.75)
    t = max(0.0, min(1.0, (float(z) - z_min) / (z_max - z_min)))
    return ColorRGBA(r=float(1.0 - t), g=float(0.25 + 0.75 * t), b=0.2, a=0.85)


class RealPcdHeapViz(Node):
    def __init__(self, heap_json, candidate_json, frame_id, max_heap_points):
        super().__init__("mode1_real_pcd_heap_viz")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/v14_urdf/real_pcd_heap", qos)
        self.heap_json = heap_json
        self.candidate_json = candidate_json
        self.frame_id = frame_id
        self.max_heap_points = max_heap_points
        self.timer = self.create_timer(1.0, self._publish_once)
        self._publish_once()

    def _publish_once(self):
        heap = _load_json(self.heap_json)
        candidate = _load_json(self.candidate_json) if self.candidate_json else {}
        heap_points = heap.get("points", [])
        candidates = candidate.get("candidate_dig_points", [])

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        marker_heap = Marker()
        marker_heap.header.frame_id = self.frame_id
        marker_heap.header.stamp = now
        marker_heap.ns = "real_pcd_heap"
        marker_heap.id = 1
        marker_heap.type = Marker.POINTS
        marker_heap.action = Marker.ADD
        marker_heap.pose.orientation.w = 1.0
        marker_heap.scale = Vector3(x=0.03, y=0.03, z=0.03)

        z_values = [float(point["z"]) for point in heap_points] or [0.0]
        z_min = min(z_values)
        z_max = max(z_values)
        if self.max_heap_points and len(heap_points) > self.max_heap_points:
            step = max(1, len(heap_points) // self.max_heap_points)
            draw_points = heap_points[::step]
        else:
            draw_points = heap_points

        for point in draw_points:
            marker_heap.points.append(
                Point(x=float(point["x"]), y=float(point["y"]), z=float(point["z"]))
            )
            marker_heap.colors.append(_color_for_height(point["z"], z_min, z_max))
        marker_array.markers.append(marker_heap)

        marker_candidates = Marker()
        marker_candidates.header.frame_id = self.frame_id
        marker_candidates.header.stamp = now
        marker_candidates.ns = "real_pcd_candidates"
        marker_candidates.id = 2
        marker_candidates.type = Marker.SPHERE_LIST
        marker_candidates.action = Marker.ADD
        marker_candidates.pose.orientation.w = 1.0
        marker_candidates.scale = Vector3(x=0.07, y=0.07, z=0.07)
        marker_candidates.color = ColorRGBA(r=0.95, g=0.9, b=0.15, a=0.95)
        for point in candidates:
            marker_candidates.points.append(
                Point(x=float(point["x"]), y=float(point["y"]), z=float(point["z"]) + 0.04)
            )
        marker_array.markers.append(marker_candidates)

        marker_text = Marker()
        marker_text.header.frame_id = self.frame_id
        marker_text.header.stamp = now
        marker_text.ns = "real_pcd_summary"
        marker_text.id = 3
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.pose.position.x = 0.2
        marker_text.pose.position.y = 0.0
        marker_text.pose.position.z = z_max + 0.25
        marker_text.pose.orientation.w = 1.0
        marker_text.scale.z = 0.08
        marker_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        marker_text.text = (
            f"real_pcd heap={len(heap_points)} "
            f"candidates={len(candidates)} "
            f"files={len(heap.get('source_files', []))}"
        )
        marker_array.markers.append(marker_text)

        self.pub.publish(marker_array)


def main():
    parser = argparse.ArgumentParser(description="把真实 PCD 提取出的土堆和候选点叠加到 URDF/RViz。")
    parser.add_argument("--clear", action="store_true")
    parser.add_argument(
        "--heap-json",
        default=str(Path(__file__).resolve().parent / "output" / "real_pcd_heap_points.json"),
        help="real_pcd_mode1_pipeline.py 输出的 heap points JSON。",
    )
    parser.add_argument(
        "--candidate-json",
        default=str(Path(__file__).resolve().parent / "output" / "real_pcd_candidate_dig_points.json"),
        help="可选：候选挖掘点 JSON。",
    )
    parser.add_argument("--frame-id", default="base_link", help="RViz 中的坐标系。")
    parser.add_argument("--max-heap-points", type=int, default=8000, help="最多发布多少个土堆点。")
    args = parser.parse_args()

    rclpy.init()
    node = RealPcdHeapViz(
        heap_json=args.heap_json,
        candidate_json=args.candidate_json,
        frame_id=args.frame_id,
        max_heap_points=args.max_heap_points,
    )
    if args.clear:
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        node.pub.publish(marker_array)
        time.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
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
