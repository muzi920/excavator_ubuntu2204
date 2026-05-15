import cv2
import threading
import os
import time

class MultiCameraReader:
    def __init__(self, target_width=1280, target_height=720):
        self.target_width = target_width
        self.target_height = target_height
        
        # 定义相机拉流地址
        self.cam_urls = {
            "hikvision": "rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101",
            "net_102": "rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream",
            "net_103": "rtsp://admin:@192.168.158.103:554/stream"
        }
        
        # 存储最新的画面
        self.latest_frames = {
            "hikvision": None,
            "net_102": None,
            "net_103": None
        }
        
        self.caps = {}
        self.threads = []
        self.running = False
        self.lock = threading.Lock()

    def _capture_loop(self, cam_name, url, use_tcp=False):
        """独立的线程函数，负责无阻塞拉取指定摄像头的最新帧"""
        if use_tcp:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|stimeout;3000000"
        else:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|stimeout;3000000"
            
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            print(f"[CameraReader] 无法连接到相机: {cam_name} ({url})")
            return
            
        self.caps[cam_name] = cap
        print(f"[CameraReader] 相机 {cam_name} 连接成功.")

        while self.running:
            # grab() 只抓取不解码，配合 retrieve() 获取最新帧，能有效清空 OpenCV 内部堆积的 RTSP 缓存
            ret = cap.grab()
            if ret:
                ret, frame = cap.retrieve()
                if ret:
                    # 缩放并更新最新帧
                    if frame.shape[:2] != (self.target_height, self.target_width):
                        frame = cv2.resize(frame, (self.target_width, self.target_height))
                        
                    with self.lock:
                        self.latest_frames[cam_name] = frame.copy()
            else:
                print(f"[CameraReader] 相机 {cam_name} 抓取失败，尝试等待恢复...")
                time.sleep(1.0)
                
        cap.release()

    def start(self):
        self.running = True
        # 海康用 TCP，普通网络摄像头用 UDP
        t_hik = threading.Thread(target=self._capture_loop, args=("hikvision", self.cam_urls["hikvision"], True))
        t_102 = threading.Thread(target=self._capture_loop, args=("net_102", self.cam_urls["net_102"], False))
        t_103 = threading.Thread(target=self._capture_loop, args=("net_103", self.cam_urls["net_103"], False))
        
        self.threads.extend([t_hik, t_102, t_103])
        for t in self.threads:
            t.daemon = True
            t.start()

    def get_frames(self):
        """获取所有相机的最新画面"""
        with self.lock:
            return {
                "hikvision": self.latest_frames["hikvision"],
                "net_102": self.latest_frames["net_102"],
                "net_103": self.latest_frames["net_103"]
            }

    def stop(self):
        print("[CameraReader] 正在关闭相机拉流...")
        self.running = False
        for t in self.threads:
            t.join(timeout=2.0)
        print("[CameraReader] 相机流已完全关闭.")
