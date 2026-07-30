import sys
import time
import math
import io
import requests
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2

class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('dsvt_ros2_bridge_node')
        
        # 声明参数
        self.declare_parameter('topic', '/pointcloud_base_link')
        self.declare_parameter('api_url', 'http://127.0.0.1:8000/predict')
        
        self.topic = self.get_parameter('topic').value
        self.api_url = self.get_parameter('api_url').value
        
        # 创建点云订阅者
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic,
            self.pointcloud_callback,
            10
        )
        
        # 创建 Marker 发布者
        self.marker_pub = self.create_publisher(MarkerArray, '/dsvt_detections', 10)
        self.last_marker_count = 0
        
        self.get_logger().info(f"ROS 2 Bridge initialized.")
        self.get_logger().info(f"Listening to PointCloud2 on {self.topic}")
        self.get_logger().info(f"Forwarding to API at {self.api_url}")

    def pointcloud_callback(self, msg):
        start_time = time.time()
        
        # 1. 解析 ROS PointCloud2 消息为 Numpy Array
        field_names = [field.name for field in msg.fields]
        target_fields = ['x', 'y', 'z']
        if 'intensity' in field_names:
            target_fields.append('intensity')
        elif 'i' in field_names:
            target_fields.append('i')
            
        # 提取点云数据
        gen = pc2.read_points(msg, field_names=target_fields, skip_nans=True)
        # 1. 强制将 generator 提取为 numpy 数组 (此时可能是 structured array)
        points_raw = np.array(list(gen))
        
        if points_raw.shape[0] == 0:
            return
            
        # 2. 如果是 structured array，将其转为普通的 2D float32 数组
        if points_raw.dtype.names:
            points = np.stack([points_raw[field] for field in target_fields], axis=-1).astype(np.float32)
        else:
            points = points_raw.astype(np.float32)
            
        parse_time = time.time()
        
        # 2. 将 Numpy 数组序列化为内存二进制字节流 (npy 格式)
        buffer = io.BytesIO()
        np.save(buffer, points)
        buffer.seek(0)
        
        # 3. 通过 HTTP POST 发送给 Conda 环境中的 API 服务
        try:
            response = requests.post(
                self.api_url, 
                files={"file": ("cloud.npy", buffer.getvalue(), "application/octet-stream")},
                timeout=0.5 # 超时时间 500ms
            )
            response.raise_for_status()
            res_data = response.json()
            
            boxes = res_data.get("boxes", [])
            scores = res_data.get("scores", [])
            labels = res_data.get("labels", [])
            infer_time_ms = res_data.get("inference_time_ms", 0.0)
            
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"Failed to connect to DSVT API: {e}")
            return
            
        api_time = time.time()
        
        # 4. 发布检测框到 RViz
        self.publish_markers(msg.header, boxes, scores, labels)
        
        total_time = time.time() - start_time
        
        self.get_logger().info(
            f"Points: {points.shape[0]} | "
            f"Parse: {(parse_time-start_time)*1000:.1f}ms | "
            f"API(Network+Infer): {(api_time-parse_time)*1000:.1f}ms (GPU Infer: {infer_time_ms}ms) | "
            f"Total: {total_time*1000:.1f}ms | Detected: {len(boxes)}"
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
    node = ROS2BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
