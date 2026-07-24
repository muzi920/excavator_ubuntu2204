#!/usr/bin/env python3
"""
海康威视摄像头 RTSP 流  ROS 2 Image 发布节点
==============================================
从海康摄像头拉取 RTSP 视频流，并以 10Hz 频率发布到 ROS 2 topic。

Topic: /camera_hik/image_raw  (sensor_msgs/Image, bgr8)
依赖:  opencv-python, cv_bridge, sensor_msgs, rclpy
用法:  ros2 run v11_multimodal_dataset_collection hikvision_cam_node
      或通过 launch 文件启动
"""

import os
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class HikvisionCamNode(Node):
    """海康摄像头 RTSP  ROS 2 Image publisher"""

    def __init__(self):
        super().__init__('hikvision_cam_node')

        # 声明参数（可在 launch 文件中覆盖）
        self.declare_parameter('rtsp_url', 'rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101')
        self.declare_parameter('transport', 'tcp')          # 海康建议 TCP
        self.declare_parameter('camera_name', 'cam_hik')
        self.declare_parameter('topic', '/camera_hik/image_raw')
        self.declare_parameter('pub_rate_hz', 10.0)         # 发布频率
        self.declare_parameter('timeout_sec', 3.0)          # RTSP 超时

        rtsp_url = self.get_parameter('rtsp_url').value
        transport = self.get_parameter('transport').value
        self.camera_name = self.get_parameter('camera_name').value
        topic = self.get_parameter('topic').value
        pub_rate_hz = self.get_parameter('pub_rate_hz').value
        timeout = self.get_parameter('timeout_sec').value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic, 10)

        # 设置 FFmpeg 低延迟选项
        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = (
            f'rtsp_transport;{transport}|'
            f'stimeout;{int(timeout * 1_000_000)}|'
            f'fflags;nobuffer|flags;low_delay'
        )

        self.get_logger().info(f'[海康摄像头] 正在连接: {rtsp_url}')
        self.get_logger().info(f'[海康摄像头] 传输协议: {transport}, 发布频率: {pub_rate_hz} Hz')

        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().fatal(f'无法打开 RTSP 流: {rtsp_url}')
            raise RuntimeError('海康摄像头连接失败')

        self.get_logger().info(f'[海康摄像头]  连接成功，开始发布到 {topic}')

        # 后台线程全速读取帧，只保留最新一帧（防止缓冲区积压）
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

        # 定时器按目标频率发布最新帧
        self.pub_interval = 1.0 / pub_rate_hz
        self.timer = self.create_timer(self.pub_interval, self._publish_loop)

    def _read_loop(self):
        """后台线程：全速读取帧，清空缓冲区，只保留最新一帧"""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('[海康摄像头] 读取帧失败，尝试重连...')
                self.cap.release()
                rtsp_url = self.get_parameter('rtsp_url').value
                self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                time.sleep(1.0)
                continue
            with self.frame_lock:
                self.latest_frame = frame

    def _publish_loop(self):
        """定时器回调：发布最新帧到 ROS2 topic"""
        with self.frame_lock:
            frame = self.latest_frame
            self.latest_frame = None   # 避免重复发布同一帧

        if frame is None:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_name
        self.publisher.publish(msg)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HikvisionCamNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'[FATAL] {e}')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
