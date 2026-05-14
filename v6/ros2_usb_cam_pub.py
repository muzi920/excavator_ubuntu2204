#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class UsbCamPublisher(Node):
    def __init__(self):
        super().__init__('usb_cam_publisher')
        
        # 声明参数
        self.declare_parameter('device_path', '/dev/video2')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        
        device_path = self.get_parameter('device_path').get_parameter_value().string_value
        frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        
        # 创建发布者
        self.publisher_ = self.create_publisher(Image, 'usb_cam/image_raw', 10)
        
        # 初始化 cv_bridge
        self.br = CvBridge()
        
        # 打开摄像头
        self.get_logger().info(f'尝试打开 USB 摄像头: {device_path}')
        self.cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开摄像头 {device_path}!')
            return
            
        # 设置摄像头分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f'成功打开摄像头，分辨率设置为: {actual_width}x{actual_height}')
        
        # 创建定时器进行发布
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # 将 OpenCV 图像转换为 ROS2 Image 消息 (bgr8 编码)
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            # 添加时间戳
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "usb_cam_frame"
            
            # 发布消息
            self.publisher_.publish(msg)
            # self.get_logger().info('发布了一帧图像')
        else:
            self.get_logger().warn('读取图像帧失败!')

    def destroy_node(self):
        # 节点销毁前释放摄像头资源
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('摄像头已释放')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = UsbCamPublisher()
    
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
