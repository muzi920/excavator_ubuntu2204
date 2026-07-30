#!/usr/bin/env python3
"""
DSVT/PointPillars 推理 ROS2 节点。

订阅 sensor_msgs/PointCloud2 → GPU 推理 → 发布 MarkerArray (RViz2)

用法 (ROS2 Humble + Python 3.10):
    source /opt/ros/humble/setup.bash
    # 终端 1: 发布点云
    ros2 run dsvt_ros2 pc_publisher --ros-args -p data_path:=pcd_npy/
    # 终端 2: 推理
    ros2 run dsvt_ros2 inference_node \
        --ros-args -p ckpt_path:=/path/to/checkpoint.pth -p class_names:=Soil
    # 终端 3: RViz2, 添加 /perception/markers (MarkerArray) 显示
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from dsvt_ros2.inference_engine import create_engine
from dsvt_ros2.utils import (
    pointcloud2_to_numpy,
    boxes_to_marker_array,
)


# ---- 颜色方案 ----
CLASS_COLORS = {
    0: (0.0, 0.0, 1.0),   # blue
    1: (0.0, 1.0, 0.0),   # green
    2: (0.0, 1.0, 1.0),   # cyan
    3: (1.0, 1.0, 0.0),   # yellow
    4: (1.0, 0.0, 1.0),   # magenta
    5: (1.0, 0.5, 0.0),   # orange
    6: (1.0, 0.0, 0.0),   # red
    7: (0.5, 0.5, 0.5),   # gray
}


class DSVTInferenceNode(Node):
    """DSVT/PointPillars 推理 ROS2 节点。

    订阅:
        /lidar/points (sensor_msgs/PointCloud2) - LiDAR 点云

    发布:
        /perception/markers (visualization_msgs/MarkerArray) - RViz2 3D 包围盒
    """

    def __init__(self):
        super().__init__('dsvt_inference')

        # ---- 声明参数 ----
        self.declare_parameter('ckpt_path', '')
        self.declare_parameter('cfg_file', '')
        self.declare_parameter('engine_type', 'auto')
        self.declare_parameter('class_names', 'Soil')
        self.declare_parameter('point_cloud_range', '-75.2,-75.2,-2,75.2,75.2,4')
        self.declare_parameter('score_thresh', 0.1)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('input_topic', '/lidar/points')
        self.declare_parameter('output_topic', '/perception/markers')

        # 读取参数
        cfg_file = self.get_parameter('cfg_file').get_parameter_value().string_value
        ckpt_path = self.get_parameter('ckpt_path').get_parameter_value().string_value
        engine_type = self.get_parameter('engine_type').get_parameter_value().string_value
        class_names_str = self.get_parameter('class_names').get_parameter_value().string_value
        pcr_str = self.get_parameter('point_cloud_range').get_parameter_value().string_value
        score_thresh = self.get_parameter('score_thresh').get_parameter_value().double_value
        device = self.get_parameter('device').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        if engine_type != 'cluster' and not ckpt_path:
            self.get_logger().fatal('非 cluster 模式需要 ckpt_path 参数!')
            self.get_logger().fatal(
                '用法: ros2 run dsvt_ros2 inference_node '
                '--ros-args -p engine_type:=cluster'
            )
            raise RuntimeError('Missing ckpt_path')

        class_names = [c.strip() for c in class_names_str.split(',')]

        # ---- 加载模型 ----
        self.get_logger().info(f'engine_type: {engine_type}')
        self.get_logger().info(f'classes: {class_names}')

        self.engine = create_engine(
            cfg_file=cfg_file if cfg_file else None,
            ckpt_path=ckpt_path if ckpt_path else None,
            engine_type=engine_type,
            class_names=class_names,
            point_cloud_range=[float(x) for x in pcr_str.split(',')],
            device=device,
            score_thresh=score_thresh,
        )
        self.class_names = class_names
        self.get_logger().info(f'Model loaded. Classes: {self.class_names}')

        # ---- ROS2 接口 ----
        # 使用默认 RELIABLE QoS 确保与任意 publisher 兼容
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, qos)

        self.marker_pub = self.create_publisher(
            MarkerArray, output_topic, 10)

        # 同时转发点云给 RViz2 显示
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/perception/cloud', 10)

        # ---- 统计 ----
        self.frame_count = 0
        self.total_latency = 0.0

        self.get_logger().info(
            f'Listening on {input_topic}, publishing to {output_topic}'
        )
        self.get_logger().info('✅ DSVT Inference Node ready. 等待点云...')

    def cloud_callback(self, msg: PointCloud2):
        """收到点云 → 推理 → 发布 MarkerArray"""
        t0 = time.perf_counter()

        try:
            # 1. 转发原始点云给 RViz2
            self.cloud_pub.publish(msg)

            # 2. PointCloud2 → numpy (N, 4)
            points = pointcloud2_to_numpy(msg)
            if points.shape[0] == 0:
                self.get_logger().warn('收到空点云, 跳过.', throttle_duration_sec=3.0)
                return

            # 2. 过滤 NaN 和无效点
            valid = np.all(np.isfinite(points), axis=1)
            points = points[valid]
            if len(points) < 100:
                self.get_logger().warn(f'Too few valid points ({len(points)}), skipping.', throttle_duration_sec=3.0)
                return

            # 3. 推理 (GPU 模型 或 CPU 聚类)
            result = self.engine.infer(points)
            if len(result) == 3:
                boxes, scores, labels = result
            else:
                boxes, labels = result
                scores = np.ones(len(boxes)) if len(boxes) > 0 else np.array([])

            # 3. 发布 MarkerArray → RViz2
            markers = boxes_to_marker_array(
                boxes, scores, labels,
                class_names=self.class_names,
                header=msg.header,
                color_map=CLASS_COLORS,
                alpha=0.5,
                ns='dsvt_detections',
            )
            self.marker_pub.publish(markers)

        except Exception as e:
            self.get_logger().error(
                f'Inference error: {e}', throttle_duration_sec=1.0)
            import traceback
            self.get_logger().error(traceback.format_exc())
            return

        elapsed = (time.perf_counter() - t0) * 1000.0
        self.frame_count += 1
        self.total_latency += elapsed

        self.get_logger().info(
            f'Frame {self.frame_count}: {len(boxes)} detections | '
            f'pts={points.shape[0]} | '
            f'latency={elapsed:.0f}ms | '
            f'avg={self.total_latency / self.frame_count:.0f}ms',
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DSVTInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
