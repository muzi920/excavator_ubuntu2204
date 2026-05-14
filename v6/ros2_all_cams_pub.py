#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading
import time

class AllCamsPublisher(Node):
    def __init__(self):
        super().__init__('all_cams_publisher')
        
        # ================= 1. 声明和获取参数 =================
        self.declare_parameter('usb_device_path', '/dev/video0')
        self.declare_parameter('net_rtsp_url', 'rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream')
        self.declare_parameter('hik_rtsp_url', 'rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        
        usb_path = self.get_parameter('usb_device_path').value
        net_url = self.get_parameter('net_rtsp_url').value
        hik_url = self.get_parameter('hik_rtsp_url').value
        self.target_width = self.get_parameter('width').value
        self.target_height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        
        self.br = CvBridge()
        self.running = True
        
        # ================= 2. 初始化发布者 =================
        self.pub_usb = self.create_publisher(Image, 'usb_cam/image_raw', 10)
        self.pub_net = self.create_publisher(Image, 'network_cam/image_raw', 10)
        self.pub_hik = self.create_publisher(Image, 'hikvision_cam/image_raw', 10)
        
        # ================= 3. 依次初始化摄像头 =================
        self.get_logger().info('正在初始化所有摄像头...')
        
        # 3.1 USB 摄像头
        self.cap_usb = cv2.VideoCapture(usb_path, cv2.CAP_V4L2)
        if self.cap_usb.isOpened():
            self.cap_usb.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
            self.cap_usb.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
            self.get_logger().info(f'[成功] USB 摄像头已连接: {usb_path}')
        else:
            self.get_logger().error(f'[失败] USB 摄像头连接失败: {usb_path}')
            
        # 3.2 普通网络摄像头 (UDP 传输，防卡死)
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|stimeout;3000000"
        self.cap_net = cv2.VideoCapture(net_url, cv2.CAP_FFMPEG)
        if self.cap_net.isOpened():
            self.get_logger().info(f'[成功] 网络摄像头已连接: {net_url}')
        else:
            self.get_logger().error(f'[失败] 网络摄像头连接失败: {net_url}')
            
        # 3.3 海康网络摄像头 (TCP 传输防花屏，防卡死)
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|stimeout;3000000"
        self.cap_hik = cv2.VideoCapture(hik_url, cv2.CAP_FFMPEG)
        if self.cap_hik.isOpened():
            self.get_logger().info(f'[成功] 海康摄像头已连接: {hik_url}')
        else:
            self.get_logger().error(f'[失败] 海康摄像头连接失败: {hik_url}')
            
        # ================= 4. 启动多线程独立读取 =================
        # 为了防止不同摄像头的阻塞互相影响，我们采用多线程独立读取和发布
        self.threads = []
        
        if self.cap_usb.isOpened():
            t1 = threading.Thread(target=self.capture_loop, args=(self.cap_usb, self.pub_usb, "usb_cam_frame", False))
            self.threads.append(t1)
            t1.start()
            
        if self.cap_net.isOpened():
            t2 = threading.Thread(target=self.capture_loop, args=(self.cap_net, self.pub_net, "network_cam_frame", True))
            self.threads.append(t2)
            t2.start()
            
        if self.cap_hik.isOpened():
            t3 = threading.Thread(target=self.capture_loop, args=(self.cap_hik, self.pub_hik, "hikvision_cam_frame", True))
            self.threads.append(t3)
            t3.start()

    def capture_loop(self, cap, publisher, frame_id, need_resize):
        """独立的线程函数，负责读取和发布指定的摄像头数据"""
        sleep_time = 1.0 / self.fps
        
        while self.running and rclpy.ok():
            start_time = time.time()
            ret, frame = cap.read()
            
            if ret:
                # 只有 RTSP 流需要软件强制缩放，USB 摄像头已经在硬件级配置了
                if need_resize:
                    frame = cv2.resize(frame, (self.target_width, self.target_height))
                    
                # 转换为 ROS2 消息
                msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = frame_id
                
                # 发布
                publisher.publish(msg)
            else:
                self.get_logger().warn(f'{frame_id} 抓取图像失败，尝试等待恢复...')
                time.sleep(1.0) # 失败后稍微等待，避免死循环疯狂刷屏
                
            # 帧率控制
            elapsed = time.time() - start_time
            if elapsed < sleep_time:
                time.sleep(sleep_time - elapsed)

    def destroy_node(self):
        # 优雅关闭：先通知线程退出，等待线程结束，再释放资源
        self.running = False
        for t in self.threads:
            t.join(timeout=1.0)
            
        if hasattr(self, 'cap_usb') and self.cap_usb.isOpened(): self.cap_usb.release()
        if hasattr(self, 'cap_net') and self.cap_net.isOpened(): self.cap_net.release()
        if hasattr(self, 'cap_hik') and self.cap_hik.isOpened(): self.cap_hik.release()
        
        self.get_logger().info('所有摄像头资源已安全释放')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AllCamsPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
