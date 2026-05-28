import cv2
import os
import time
import threading
import json
import numpy as np
from datetime import datetime

class MultimodalRecorder:
    def __init__(self):
        self.is_recording = False
        self._lock = threading.Lock()
        self._sensor_file = None
        self._cmd_file = None
        self.session_dir = ""
        self.dirs = {}

    def start(self):
        """初始化目录并启动记录"""
        # 将统一的数据保存目录设置为 src/shandong/data
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "data"))
        os.makedirs(base_dir, exist_ok=True)
        
        # 创建本次录制的主目录
        self.session_id = f"v11_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.session_dir = os.path.join(base_dir, self.session_id)
        
        # 创建各个传感器的数据子目录
        self.dirs = {
            "cam1": os.path.join(self.session_dir, "cam_net_1"),
            "cam2": os.path.join(self.session_dir, "cam_net_2"),
            "cam_hik": os.path.join(self.session_dir, "cam_hikvision"),
            "lidar": os.path.join(self.session_dir, "pointclouds"),
        }
        self.sensors_log_path = os.path.join(self.session_dir, "sensor_states.csv")
        self.commands_log_path = os.path.join(self.session_dir, "control_commands.csv")
        
        for d in self.dirs.values():
            os.makedirs(d, exist_ok=True)
            
        self._sensor_file = open(self.sensors_log_path, 'w', encoding='utf-8')
        self._sensor_file.write("timestamp,boom_pitch,arm_pitch,bucket_pitch,swing_yaw,yaw_rate\n")
        
        self._cmd_file = open(self.commands_log_path, 'w', encoding='utf-8')
        self._cmd_file.write("timestamp,ch1,ch2,ch3\n")
        
        self.is_recording = True
        print(f"[录制启动] 数据集保存在: {self.session_dir}")

    def stop(self):
        """停止记录并保存"""
        self.is_recording = False
        time.sleep(0.5) # 等待队列排空
        if self._sensor_file:
            self._sensor_file.close()
        if self._cmd_file:
            self._cmd_file.close()
        print(f"[录制结束] 数据集已保存。")

    # ================= 数据写入接口 =================
    
    def log_sensor_state(self, ts, boom, arm, bucket, swing, yaw_rate):
        if self.is_recording and self._sensor_file:
            self._sensor_file.write(f"{ts:.3f},{boom:.2f},{arm:.2f},{bucket:.2f},{swing:.2f},{yaw_rate:.3f}\n")

    def log_control_cmd(self, ts, ch1, ch2, ch3):
        if self.is_recording and self._cmd_file:
            self._cmd_file.write(f"{ts:.3f},{ch1},{ch2},{ch3}\n")

    def save_image(self, cam_name, ts, frame):
        """在独立线程中调用，避免阻塞主循环"""
        if not self.is_recording or frame is None:
            return
        filename = os.path.join(self.dirs[cam_name], f"{ts:.3f}.jpg")
        # 使用低压缩率快速保存
        cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])

    def save_pointcloud(self, ts, points_array):
        """保存点云 (NX3 numpy array)"""
        if not self.is_recording or points_array is None or len(points_array) == 0:
            return
        filename = os.path.join(self.dirs["lidar"], f"{ts:.3f}.npy")
        np.save(filename, points_array)


# ================= 独立的视频流拉取线程类 =================
class VideoStreamThread(threading.Thread):
    def __init__(self, name, rtsp_url, recorder, transport="udp", hw_status_dict=None):
        super().__init__(daemon=True)
        self.name = name
        self.rtsp_url = rtsp_url
        self.recorder = recorder
        self.transport = transport
        self.hw_status_dict = hw_status_dict
        self.running = True
        
        # 针对 RTSP 流，为了降低延迟和防止花屏，设置 FFmpeg 环境变量
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = f"rtsp_transport;{self.transport}|stimeout;3000000"

    def run(self):
        print(f"[Camera {self.name}] 尝试连接 RTSP流: {self.rtsp_url}")
        cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        
        if not cap.isOpened():
            print(f"[Camera {self.name}] 错误: 无法连接流！")
            if self.hw_status_dict is not None:
                self.hw_status_dict[self.name] = "failed"
            return
            
        print(f"[Camera {self.name}] 连接成功，开始异步拉流。")
        if self.hw_status_dict is not None:
            self.hw_status_dict[self.name] = "connected"
        
        # 为了降低延迟，只保存被主循环（或者录制循环）抽样的帧，但需要不断清空缓冲区
        while self.running:
            ret, frame = cap.read()
            if not ret:
                print(f"[Camera {self.name}] 警告: 读取失败，尝试重连...")
                if self.hw_status_dict is not None:
                    self.hw_status_dict[self.name] = "failed"
                time.sleep(1)
                cap.release()
                cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                if cap.isOpened() and self.hw_status_dict is not None:
                    self.hw_status_dict[self.name] = "connected"
                continue
            
            # 如果处于录制状态，才触发磁盘保存
            if self.recorder.is_recording:
                ts = time.time()
                # 放在单独线程里保存，防止拉流变卡
                threading.Thread(target=self.recorder.save_image, args=(self.name, ts, frame), daemon=True).start()
                # 为了防止磁盘IO爆炸，我们将摄像头的保存帧率限制在约 10Hz
                time.sleep(0.1)

        cap.release()
        print(f"[Camera {self.name}] 线程退出。")

    def stop(self):
        self.running = False