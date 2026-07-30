#!/usr/bin/env python3
"""
检测结果 RViz2 可视化桥接节点。

订阅 Detection3DArray → 转换为 MarkerArray 在 RViz2 中渲染 3D 包围盒。

此节点在 inference_node 已直接发布 MarkerArray 时不需要。仅在以下场景有用：
- 需要使用不同的可视化参数重绘检测结果 (颜色, 透明度, 字体等)
- 检测结果来自其他节点而不是 dsvt_inference
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray


class DetectionVisualizerNode(Node):
    """将 Detection3DArray 转为 RViz2 MarkerArray 进行可视化。"""

    def __init__(self):
        super().__init__('detection_visualizer')

        # ---- 参数 ----
        self.declare_parameter('score_thresh', 0.1)
        self.declare_parameter('alpha', 0.6)

        self.score_thresh = self.get_parameter('score_thresh').get_parameter_value().double_value
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value

        # ---- QoS ----
        sensor_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.det_sub = self.create_subscription(
            Detection3DArray, '/perception/detections', self.det_callback, sensor_qos)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/perception/markers_viz', 10)

        self.get_logger().info('Detection Visualizer Node ready.')

    def det_callback(self, msg: Detection3DArray):
        """收到 Detection3DArray → 提取 per-class 数据 → 发布 MarkerArray"""
        from dsvt_ros2.utils import to_marker_array
        from geometry_msgs.msg import Pose, Vector3
        import numpy as np

        boxes = []
        scores = []
        labels = []

        # 构建类别名称列表 (从消息中推断)
        class_name_set = {}
        for det in msg.detections:
            for res in det.results:
                cid = res.hypothesis.class_id
                if cid not in class_name_set:
                    class_name_set[cid] = len(class_name_set)
                label_id = class_name_set[cid]

                # 提取 bbox
                pose = res.pose.pose
                x = pose.position.x
                y = pose.position.y
                z = pose.position.z
                # 从 bbox 取尺寸
                dx = det.bbox.size.x
                dy = det.bbox.size.y
                dz = det.bbox.size.z
                # heading 从 quaternion 还原
                heading = 2.0 * np.arctan2(pose.orientation.z, pose.orientation.w)

                boxes.append([x, y, z, dx, dy, dz, heading])
                scores.append(res.hypothesis.score)
                labels.append(label_id)

        if not boxes:
            return

        boxes = np.array(boxes, dtype=np.float32)
        scores = np.array(scores, dtype=np.float32)
        labels = np.array(labels, dtype=np.int32)
        class_names = list(class_name_set.keys())

        marker_msg = to_marker_array(
            boxes, scores, labels, class_names, msg.header,
            score_thresh=self.score_thresh, alpha=self.alpha,
        )
        self.marker_pub.publish(marker_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
