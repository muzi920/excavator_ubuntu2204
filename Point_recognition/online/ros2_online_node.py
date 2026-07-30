import sys
from pathlib import Path
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2

from online_detector import OnlineDetector

class PointCloudOnlineNode(Node):
    def __init__(self):
        super().__init__('pointcloud_online_node')
        
        # 1. 声明参数
        self.declare_parameter('cfg_file', '../DSVT/tools/cfgs/custom_models/second.yaml')
        self.declare_parameter('ckpt', '../DSVT/output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth')
        self.declare_parameter('topic', '/pointcloud_base_link')
        
        cfg_file = self.get_parameter('cfg_file').value
        ckpt_file = self.get_parameter('ckpt').value
        topic = self.get_parameter('topic').value
        
        # 2. 初始化检测器 (加载模型到 GPU)
        self.get_logger().info("Initializing DSVT Online Detector...")
        self.detector = OnlineDetector(cfg_file=cfg_file, ckpt_file=ckpt_file)
        self.get_logger().info("DSVT Online Detector Initialized Successfully.")
        
        # 3. 创建点云订阅者
        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self.pointcloud_callback,
            10 # QoS profile 队列长度
        )
        self.get_logger().info(f"Subscribed to PointCloud2 topic: {topic}")
        
        # 4. 创建发布者 (用于在 RViz 中可视化 3D 检测框)
        self.marker_pub = self.create_publisher(MarkerArray, '/dsvt_detections', 10)
        
        # 记录上一帧的 Marker 数量，用于清除画面上的旧 Marker 残影
        self.last_marker_count = 0

    def pointcloud_callback(self, msg):
        start_time = time.time()
        
        # 1. 解析 ROS PointCloud2 消息为 Numpy Array
        field_names = [field.name for field in msg.fields]
        target_fields = ['x', 'y', 'z']
        if 'intensity' in field_names:
            target_fields.append('intensity')
        elif 'i' in field_names:
            target_fields.append('i')
            
        # 读取点云数据
        gen = pc2.read_points(msg, field_names=target_fields, skip_nans=True)
        points = np.array(list(gen), dtype=np.float32)
        
        if points.shape[0] == 0:
            self.get_logger().warn("Received empty point cloud!")
            return
            
        # 补齐维度至 N x 4 (x, y, z, intensity)
        if points.shape[1] == 3:
            padding = np.zeros((points.shape[0], 1), dtype=np.float32)
            points = np.hstack([points, padding])
        elif points.shape[1] > 4:
            points = points[:, :4]
            
        parse_time = time.time() - start_time
        
        # 2. 执行在线实时推理
        infer_start = time.time()
        boxes, scores, labels = self.detector.inference(points)
        infer_time = time.time() - infer_start
        
        total_time = time.time() - start_time
        
        # 打印耗时与检测结果
        self.get_logger().info(
            f"Frame Processed | Points: {points.shape[0]} | "
            f"Parse: {parse_time*1000:.1f}ms | Infer: {infer_time*1000:.1f}ms | "
            f"Total: {total_time*1000:.1f}ms | Detected {len(boxes)} objects."
        )
        
        # 3. 发布检测框到 RViz
        self.publish_markers(msg.header, boxes, scores, labels)

    def publish_markers(self, header, boxes, scores, labels):
        marker_array = MarkerArray()
        
        # 清除上一帧多余的 Marker (避免画面上残留旧框)
        for i in range(len(boxes), self.last_marker_count):
            del_marker = Marker()
            del_marker.header = header
            del_marker.ns = "dsvt_boxes"
            del_marker.id = i
            del_marker.action = Marker.DELETE
            marker_array.markers.append(del_marker)
            
        self.last_marker_count = len(boxes)
        
        # 构造当前帧的 3D 框 Marker
        for i, box in enumerate(boxes):
            marker = Marker()
            marker.header = header
            marker.ns = "dsvt_boxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # 位置
            marker.pose.position.x = float(box[0])
            marker.pose.position.y = float(box[1])
            marker.pose.position.z = float(box[2])
            
            # 尺寸 (OpenPCDet 输出通常是 dx, dy, dz，对应 length, width, height)
            marker.scale.x = float(box[3])
            marker.scale.y = float(box[4])
            marker.scale.z = float(box[5])
            
            # 姿态 (绕 Z 轴的 Yaw 角转四元数)
            yaw = float(box[6])
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            
            # 颜色 (Score 越高透明度越低，绿色表示检测框)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = max(0.4, min(1.0, float(scores[i])))
            
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudOnlineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
