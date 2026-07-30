import sys
import time
import math
import io
import requests
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2

class ROS2FilterNode(Node):
    def __init__(self):
        super().__init__('dsvt_ros2_filter_node')
        
        # 参数声明
        self.declare_parameter('topic_in', '/pointcloud_base_link')
        self.declare_parameter('topic_out', '/dsvt_filtered_points')
        self.declare_parameter('api_url', 'http://127.0.0.1:8000/predict')
        
        self.topic_in = self.get_parameter('topic_in').value
        self.topic_out = self.get_parameter('topic_out').value
        self.api_url = self.get_parameter('api_url').value
        
        # 订阅原始点云
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic_in,
            self.pointcloud_callback,
            10
        )
        
        # 发布过滤后的点云
        self.filtered_pub = self.create_publisher(PointCloud2, self.topic_out, 10)
        # 发布检测框 Marker
        self.marker_pub = self.create_publisher(MarkerArray, '/dsvt_detections', 10)
        self.last_marker_count = 0
        
        self.get_logger().info("ROS 2 PointCloud Filter Node initialized.")
        self.get_logger().info(f"Listening on: {self.topic_in}")
        self.get_logger().info(f"Publishing filtered points to: {self.topic_out}")

    def filter_points_in_boxes(self, points, boxes):
        """
        过滤出位于所有 3D 框内部的点云，并且过滤掉 z < 0 的点
        points: (N, C) numpy array，前3列为 x, y, z
        boxes: (M, 7) numpy array，格式为 [cx, cy, cz, dx, dy, dz, yaw]
        """
        if len(boxes) == 0 or len(points) == 0:
            # 如果没有检测到框，返回空点云
            return np.empty((0, points.shape[1]), dtype=points.dtype)

        # 1. 过滤掉 z < 0 的点
        z_mask = points[:, 2] >= 0.0
        points = points[z_mask]
        
        if len(points) == 0:
            return np.empty((0, points.shape[1]), dtype=points.dtype)

        # 最终的 mask，记录哪些点被保留
        keep_mask = np.zeros(points.shape[0], dtype=bool)
        
        px = points[:, 0]
        py = points[:, 1]
        pz = points[:, 2]

        for box in boxes:
            cx, cy, cz, dx, dy, dz, yaw = box
            
            # 1. 平移到框中心
            tx = px - cx
            ty = py - cy
            tz = pz - cz
            
            # 2. 绕 Z 轴旋转 (-yaw)，将点对齐到框的局部坐标系
            cos_y = np.cos(-yaw)
            sin_y = np.sin(-yaw)
            rx = tx * cos_y - ty * sin_y
            ry = tx * sin_y + ty * cos_y
            rz = tz
            
            # 3. 判断是否在长方体边界内
            in_box = (
                (np.abs(rx) <= dx / 2.0) &
                (np.abs(ry) <= dy / 2.0) &
                (np.abs(rz) <= dz / 2.0)
            )
            
            # 取并集 (只要点在任意一个框内就保留)
            keep_mask = keep_mask | in_box

        return points[keep_mask]

    def pointcloud_callback(self, msg):
        start_time = time.time()
        
        # 1. 提取点云数据
        field_names = [field.name for field in msg.fields]
        target_fields = ['x', 'y', 'z']
        has_intensity = False
        if 'intensity' in field_names:
            target_fields.append('intensity')
            has_intensity = True
        elif 'i' in field_names:
            target_fields.append('i')
            has_intensity = True
            
        gen = pc2.read_points(msg, field_names=target_fields, skip_nans=True)
        points_raw = np.array(list(gen))
        
        if points_raw.shape[0] == 0:
            return
            
        # 转为普通的 2D float32 数组
        if points_raw.dtype.names:
            points = np.stack([points_raw[field] for field in target_fields], axis=-1).astype(np.float32)
        else:
            points = points_raw.astype(np.float32)
            
        # 2. 调用 API 获取检测框
        buffer = io.BytesIO()
        np.save(buffer, points)
        buffer.seek(0)
        
        try:
            response = requests.post(
                self.api_url, 
                files={"file": ("cloud.npy", buffer.getvalue(), "application/octet-stream")},
                timeout=0.5
            )
            response.raise_for_status()
            res_data = response.json()
            boxes = res_data.get("boxes", [])
            scores = res_data.get("scores", [])
            labels = res_data.get("labels", [])
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"API Error: {e}")
            return
            
        # 3. 过滤出框内的点云 (同时过滤了 z < 0 的点)
        filtered_points = self.filter_points_in_boxes(points, boxes)
        
        # 为了给点云上色为红色，我们需要添加 rgb 字段
        # 将 filtered_points 扩展，增加 rgb 列
        # 红色在 ROS PointCloud2 中通常被编码为一个 float32 (从 0x00FF0000 转换)
        import struct
        # Red: R=255, G=0, B=0
        red_rgb = struct.unpack('f', struct.pack('I', 0x00FF0000))[0]
        
        # 创建带有 RGB 信息的点云数组
        colored_points = np.zeros((filtered_points.shape[0], filtered_points.shape[1] + 1), dtype=np.float32)
        if filtered_points.shape[0] > 0:
            colored_points[:, :-1] = filtered_points
            colored_points[:, -1] = red_rgb
        
        # 4. 组装并发布新的 PointCloud2 消息
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        current_offset = 12
        if has_intensity:
            fields.append(PointField(name='intensity', offset=current_offset, datatype=PointField.FLOAT32, count=1))
            current_offset += 4
            
        # 添加 RGB 字段
        fields.append(PointField(name='rgb', offset=current_offset, datatype=PointField.FLOAT32, count=1))
            
        filtered_msg = pc2.create_cloud(msg.header, fields, colored_points)
        self.filtered_pub.publish(filtered_msg)
        
        # 5. 发布 3D 框 (可视化)
        self.publish_markers(msg.header, boxes, scores, labels)
        
        total_time = time.time() - start_time
        self.get_logger().info(
            f"Original: {points.shape[0]} pts | Filtered: {filtered_points.shape[0]} pts | "
            f"Objects: {len(boxes)} | Total Time: {total_time*1000:.1f}ms"
        )

    def publish_markers(self, header, boxes, scores, labels):
        marker_array = MarkerArray()
        
        for i in range(len(boxes), self.last_marker_count):
            del_marker = Marker()
            del_marker.header = header
            del_marker.ns = "dsvt_boxes"
            del_marker.id = i
            del_marker.action = Marker.DELETE
            marker_array.markers.append(del_marker)
            
        self.last_marker_count = len(boxes)
        
        for i, box in enumerate(boxes):
            marker = Marker()
            marker.header = header
            marker.ns = "dsvt_boxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(box[0])
            marker.pose.position.y = float(box[1])
            marker.pose.position.z = float(box[2])
            marker.scale.x = float(box[3])
            marker.scale.y = float(box[4])
            marker.scale.z = float(box[5])
            
            yaw = float(box[6])
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = max(0.4, min(1.0, float(scores[i])))
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ROS2FilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
