#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class NetworkCamPublisher(Node):
    def __init__(self):
        super().__init__('network_cam_publisher')
        
        # 声明参数
        self.declare_parameter('rtsp_url', 'rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        
        rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.target_width = self.get_parameter('width').get_parameter_value().integer_value
        self.target_height = self.get_parameter('height').get_parameter_value().integer_value
        
        # 针对 RTSP 流，设置 FFmpeg 环境变量 (包含 udp 和超时设置)
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|stimeout;3000000"
        
        # 创建发布者
        self.publisher_ = self.create_publisher(Image, 'network_cam/image_raw', 10)
        
        # 初始化 cv_bridge
        self.br = CvBridge()
        
        # 打开网络摄像头
        self.get_logger().info(f'尝试连接网络摄像头 (RTSP): {rtsp_url}')
        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        
        if not self.cap.isOpened():
            self.get_logger().error('无法连接到 RTSP 流。请检查网络和地址。')
            return
            
        self.get_logger().info(f'成功连接到摄像头流，发布目标分辨率将被缩放至: {self.target_width}x{self.target_height}')
        
        # 创建定时器进行发布
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            return
            
        ret, frame = self.cap.read()
        
        if ret:
            # 对于 RTSP 网络流，直接修改底层属性(CAP_PROP_FRAME_WIDTH)通常无效，
            # 最稳定可靠的降低分辨率的方法是获取到帧后，使用 cv2.resize 强制缩放
            resized_frame = cv2.resize(frame, (self.target_width, self.target_height))
            
            # 将 OpenCV 图像转换为 ROS2 Image 消息 (bgr8 编码)
            msg = self.br.cv2_to_imgmsg(resized_frame, encoding="bgr8")
            
            # 添加时间戳和 frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "network_cam_frame"
            
            # 发布消息
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('读取图像帧失败或流已断开!')

    def destroy_node(self):
        # 节点销毁前释放摄像头资源
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('摄像头资源已释放')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = NetworkCamPublisher()
    
    if hasattr(node, 'cap') and node.cap.isOpened():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
            
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
