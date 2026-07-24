import argparse
import json
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


class BodyFilterViz(Node):
    def __init__(self, kept_json, removed_json, frame_id, max_points):
        super().__init__("mode1_body_filter_viz")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/v14_urdf/body_filter", qos)
        self.kept_json = kept_json
        self.removed_json = removed_json
        self.frame_id = frame_id
        self.max_points = max_points
        self.timer = self.create_timer(1.0, self._publish_once)
        self._publish_once()

    def _downsample(self, points):
        if not self.max_points or len(points) <= self.max_points:
            return points
        step = max(1, len(points) // int(self.max_points))
        return points[::step]

    def _publish_once(self):
        kept = _load_json(self.kept_json)
        removed = _load_json(self.removed_json)
        kept_points = self._downsample(kept.get("points", []))
        removed_points = self._downsample(removed.get("points", []))

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        m_removed = Marker()
        m_removed.header.frame_id = self.frame_id
        m_removed.header.stamp = now
        m_removed.ns = "removed"
        m_removed.id = 1
        m_removed.type = Marker.POINTS
        m_removed.action = Marker.ADD
        m_removed.pose.orientation.w = 1.0
        m_removed.scale = Vector3(x=0.03, y=0.03, z=0.03)
        m_removed.color = ColorRGBA(r=0.95, g=0.20, b=0.20, a=0.55)
        for p in removed_points:
            m_removed.points.append(Point(x=float(p["x"]), y=float(p["y"]), z=float(p["z"])))
        markers.markers.append(m_removed)

        m_kept = Marker()
        m_kept.header.frame_id = self.frame_id
        m_kept.header.stamp = now
        m_kept.ns = "kept"
        m_kept.id = 2
        m_kept.type = Marker.POINTS
        m_kept.action = Marker.ADD
        m_kept.pose.orientation.w = 1.0
        m_kept.scale = Vector3(x=0.03, y=0.03, z=0.03)
        m_kept.color = ColorRGBA(r=0.20, g=0.90, b=0.35, a=0.55)
        for p in kept_points:
            m_kept.points.append(Point(x=float(p["x"]), y=float(p["y"]), z=float(p["z"])))
        markers.markers.append(m_kept)

        m_text = Marker()
        m_text.header.frame_id = self.frame_id
        m_text.header.stamp = now
        m_text.ns = "summary"
        m_text.id = 3
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position.x = 0.2
        m_text.pose.position.y = 0.0
        m_text.pose.position.z = 1.8
        m_text.pose.orientation.w = 1.0
        m_text.scale.z = 0.08
        m_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        stats = kept.get("meta", {}).get("stats", {})
        m_text.text = (
            f"kept={stats.get('kept_points')} removed={stats.get('removed_points')} "
            f"ratio={stats.get('removed_ratio'):.3f}"
        )
        markers.markers.append(m_text)

        self.pub.publish(markers)


def main():
    parser = argparse.ArgumentParser(description="把本体过滤的 kept/removed 点云结果叠加到 RViz。")
    parser.add_argument("--kept-json", required=True, help="kept_points.json")
    parser.add_argument("--removed-json", required=True, help="removed_points.json")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--max-points", type=int, default=9000)
    args = parser.parse_args()

    rclpy.init()
    node = BodyFilterViz(
        kept_json=args.kept_json,
        removed_json=args.removed_json,
        frame_id=args.frame_id,
        max_points=args.max_points,
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

