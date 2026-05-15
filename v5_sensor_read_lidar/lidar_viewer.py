import socket
import math
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from collections import deque

class BlueSeaLidar:
    """蓝海光电 M300 UDP 点云接收器"""

    HEADER_SIZE = 36
    POINT_SIZE = 8

    def __init__(
        self,
        lidar_ip,
        local_ip,
        lidar_port=6543,
        listen_port=6668,
        frame_package_num=180,
        socket_timeout=1.0,
    ):
        self.lidar_ip = lidar_ip
        self.local_ip = local_ip
        self.lidar_port = int(lidar_port)
        self.listen_port = int(listen_port)
        self.frame_package_num = int(frame_package_num)
        self.socket_timeout = socket_timeout
        self.sock = None
        self.is_running = False

        self._frame_points = []
        self._last_frame_cnt = None
        self._packet_counter = 0
        self._last_packet_time = 0.0

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.settimeout(self.socket_timeout)
            self.sock.bind((self.local_ip, self.listen_port))
            self.is_running = True
            self._frame_points = []
            self._last_frame_cnt = None
            self._packet_counter = 0
            self._last_packet_time = time.time()
            return True
        except Exception as e:
            print(f"无法连接激光雷达: {e}")
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None
            return False

    def disconnect(self):
        self.is_running = False
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    @staticmethod
    def _decode_point(raw_point):
        word0 = int.from_bytes(raw_point[0:4], "little", signed=False)
        word1 = int.from_bytes(raw_point[4:8], "little", signed=False)

        depth_raw = word0 & 0xFFFFFF
        theta_hi = (word0 >> 24) & 0xFF
        theta_lo = word1 & 0xFFF
        phi_raw = (word1 >> 12) & 0xFFFFF

        theta = (theta_hi << 12) | theta_lo
        vertical_angle = (90000 - theta) * math.pi / 180000.0
        depth_m = depth_raw / 1000.0
        radius_xy = depth_m * math.cos(vertical_angle)
        azimuth = phi_raw * math.pi / 180000.0

        return {
            "x": math.cos(azimuth) * radius_xy,
            "y": math.sin(azimuth) * radius_xy,
            "z": depth_m * math.sin(vertical_angle),
            "distance_mm": depth_raw,
            "reflectivity": raw_point[6],
            "tag": raw_point[7],
        }

    def _parse_packet(self, packet):
        if len(packet) < self.HEADER_SIZE:
            return None

        version = packet[0]
        if version not in (0, 1):
            return None

        length = int.from_bytes(packet[1:3], "little", signed=False)
        time_interval = int.from_bytes(packet[3:5], "little", signed=False)
        dot_num = int.from_bytes(packet[5:7], "little", signed=False)
        udp_cnt = int.from_bytes(packet[7:9], "little", signed=False)
        frame_cnt = packet[9]
        data_type = packet[10]
        timestamp = int.from_bytes(packet[28:36], "little", signed=False)

        expected_length = self.HEADER_SIZE + dot_num * self.POINT_SIZE
        if expected_length > len(packet):
            return None

        points = []
        payload = packet[self.HEADER_SIZE : expected_length]
        for offset in range(0, len(payload), self.POINT_SIZE):
            raw_point = payload[offset : offset + self.POINT_SIZE]
            if len(raw_point) < self.POINT_SIZE:
                continue
            point = self._decode_point(raw_point)
            if point["distance_mm"] <= 0:
                continue
            points.append(point)

        return {
            "version": version,
            "length": length,
            "time_interval": time_interval,
            "dot_num": dot_num,
            "udp_cnt": udp_cnt,
            "frame_cnt": frame_cnt,
            "data_type": data_type,
            "timestamp": timestamp,
            "points": points,
        }

    def read_frame(self):
        """按照官方 SDK 的 frame_package_num 聚合一帧点云。"""
        if not self.sock or not self.is_running:
            return None

        while self.is_running:
            try:
                packet, addr = self.sock.recvfrom(4096)
                if addr[0] != self.lidar_ip:
                    continue

                parsed = self._parse_packet(packet)
                if not parsed:
                    continue

                if parsed["data_type"] not in (0, 1):
                    continue

                self._last_packet_time = time.time()
                frame_cnt = parsed["frame_cnt"]

                if self._last_frame_cnt is None:
                    self._last_frame_cnt = frame_cnt

                if frame_cnt != self._last_frame_cnt and self._frame_points:
                    frame = {
                        "points": self._frame_points,
                        "frame_cnt": self._last_frame_cnt,
                        "packet_count": self._packet_counter,
                        "timestamp": parsed["timestamp"],
                    }
                    self._frame_points = list(parsed["points"])
                    self._packet_counter = 1
                    self._last_frame_cnt = frame_cnt
                    return frame

                self._frame_points.extend(parsed["points"])
                self._packet_counter += 1

                if self._packet_counter >= self.frame_package_num:
                    frame = {
                        "points": self._frame_points,
                        "frame_cnt": frame_cnt,
                        "packet_count": self._packet_counter,
                        "timestamp": parsed["timestamp"],
                    }
                    self._frame_points = []
                    self._packet_counter = 0
                    self._last_frame_cnt = frame_cnt
                    return frame

            except Exception as e:
                if not self.is_running:
                    return None
                if isinstance(e, socket.timeout):
                    continue
                print(f"雷达读取异常: {e}")
                time.sleep(0.1)

        return None

    def seconds_since_packet(self):
        if self._last_packet_time <= 0:
            return None
        return time.time() - self._last_packet_time

class LidarViewerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("蓝海光电 M300 激光雷达点云显示")
        self.root.geometry("980x860")

        self.lidar = None
        self.scan_thread = None
        self.latest_frame = {"points": [], "packet_count": 0, "frame_cnt": 0}
        self.latest_points = []
        self.max_distance = 15000
        self.data_lock = threading.Lock()
        self.frame_times = deque(maxlen=30)

        self._build_ui()

    def _build_ui(self):
        ctrl_frame = ttk.Frame(self.root, padding=10)
        ctrl_frame.pack(fill=tk.X)

        ttk.Label(ctrl_frame, text="雷达IP:").pack(side=tk.LEFT, padx=5)
        self.lidar_ip_var = tk.StringVar(value="192.168.158.98")
        ttk.Entry(ctrl_frame, textvariable=self.lidar_ip_var, width=16).pack(side=tk.LEFT, padx=5)

        ttk.Label(ctrl_frame, text="本机IP:").pack(side=tk.LEFT, padx=5)
        self.local_ip_var = tk.StringVar(value="192.168.158.15")
        ttk.Entry(ctrl_frame, textvariable=self.local_ip_var, width=16).pack(side=tk.LEFT, padx=5)

        ttk.Label(ctrl_frame, text="雷达端口:").pack(side=tk.LEFT, padx=5)
        self.lidar_port_var = tk.IntVar(value=6543)
        ttk.Entry(ctrl_frame, textvariable=self.lidar_port_var, width=8).pack(side=tk.LEFT, padx=5)

        ttk.Label(ctrl_frame, text="监听端口:").pack(side=tk.LEFT, padx=5)
        self.listen_port_var = tk.IntVar(value=6668)
        ttk.Entry(ctrl_frame, textvariable=self.listen_port_var, width=8).pack(side=tk.LEFT, padx=5)

        ctrl_frame_2 = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        ctrl_frame_2.pack(fill=tk.X)

        ttk.Label(ctrl_frame_2, text="每帧包数:").pack(side=tk.LEFT, padx=5)
        self.frame_package_var = tk.IntVar(value=180)
        ttk.Entry(ctrl_frame_2, textvariable=self.frame_package_var, width=8).pack(side=tk.LEFT, padx=5)

        ttk.Label(ctrl_frame_2, text="量程(mm):").pack(side=tk.LEFT, padx=5)
        self.range_var = tk.IntVar(value=15000)
        ttk.Entry(ctrl_frame_2, textvariable=self.range_var, width=8).pack(side=tk.LEFT, padx=5)

        self.btn_connect = ttk.Button(ctrl_frame_2, text="连接雷达", command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=20)

        self.btn_clear = ttk.Button(ctrl_frame_2, text="清空画面", command=self.clear_points)
        self.btn_clear.pack(side=tk.LEFT, padx=5)

        self.canvas_size = 760
        self.canvas = tk.Canvas(self.root, width=self.canvas_size, height=self.canvas_size, bg="black")
        self.canvas.pack(pady=10)

        self._draw_grid()

        info_frame = ttk.Frame(self.root, padding=(10, 0, 10, 5))
        info_frame.pack(fill=tk.X)

        self.info_var = tk.StringVar(value="点数: 0 | 包数: 0 | 帧号: 0 | FPS: 0.0")
        ttk.Label(info_frame, textvariable=self.info_var).pack(side=tk.LEFT)

        self.status_var = tk.StringVar(value="准备就绪")
        ttk.Label(self.root, textvariable=self.status_var).pack(side=tk.BOTTOM, pady=5)

    def _draw_grid(self):
        self.canvas.delete("all")
        cx = self.canvas_size / 2
        cy = self.canvas_size / 2

        try:
            self.max_distance = max(1000, int(self.range_var.get()))
        except (TypeError, ValueError, tk.TclError):
            self.max_distance = 15000

        rings = 5
        for i in range(1, rings + 1):
            r = (self.canvas_size / 2 - 20) * (i / rings)
            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#333333", dash=(2, 4))
            dist_label = int(self.max_distance * (i / rings))
            self.canvas.create_text(cx + r, cy, text=f"{dist_label}mm", fill="#555555", anchor="w")

        self.canvas.create_line(cx, 0, cx, self.canvas_size, fill="#333333", dash=(2, 4))
        self.canvas.create_line(0, cy, self.canvas_size, cy, fill="#333333", dash=(2, 4))

        self.canvas.create_text(cx, 15, text="Y+", fill="#666666")
        self.canvas.create_text(self.canvas_size - 15, cy, text="X+", fill="#666666")
        self.canvas.create_text(15, cy, text="X-", fill="#666666")
        self.canvas.create_text(cx, self.canvas_size - 15, text="Y-", fill="#666666")
        self.canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="red")

    def toggle_connection(self):
        if self.lidar and self.lidar.is_running:
            self.lidar.disconnect()
            self.btn_connect.config(text="连接雷达")
            self.status_var.set("已断开")
        else:
            try:
                self.lidar = BlueSeaLidar(
                    lidar_ip=self.lidar_ip_var.get().strip(),
                    local_ip=self.local_ip_var.get().strip(),
                    lidar_port=int(self.lidar_port_var.get()),
                    listen_port=int(self.listen_port_var.get()),
                    frame_package_num=int(self.frame_package_var.get()),
                )
            except (ValueError, tk.TclError):
                messagebox.showerror("错误", "请检查 IP、端口和每帧包数输入。")
                return

            if self.lidar.connect():
                self.btn_connect.config(text="断开雷达")
                self.status_var.set(
                    f"已监听 {self.local_ip_var.get()}:{self.listen_port_var.get()}，目标雷达 {self.lidar_ip_var.get()}:{self.lidar_port_var.get()}..."
                )
                self.clear_points()
                self._draw_grid()
                self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
                self.scan_thread.start()
                self._update_ui_loop()
            else:
                messagebox.showerror(
                    "错误",
                    f"无法绑定 {self.local_ip_var.get()}:{self.listen_port_var.get()}。\n请检查电脑网卡 IP 是否已设置为该地址，或端口是否被占用。",
                )

    def _scan_loop(self):
        while self.lidar and self.lidar.is_running:
            frame = self.lidar.read_frame()
            if frame and frame["points"]:
                with self.data_lock:
                    self.latest_frame = frame
                    self.latest_points = frame["points"]
                self.frame_times.append(time.time())

    def _update_ui_loop(self):
        if not self.lidar or not self.lidar.is_running:
            return

        self._draw_points()
        self.root.after(50, self._update_ui_loop)

    def clear_points(self):
        with self.data_lock:
            self.latest_frame = {"points": [], "packet_count": 0, "frame_cnt": 0}
            self.latest_points = []
        self.frame_times.clear()
        self.info_var.set("点数: 0 | 包数: 0 | 帧号: 0 | FPS: 0.0")
        self._draw_grid()

    def _calc_fps(self):
        if len(self.frame_times) < 2:
            return 0.0
        duration = self.frame_times[-1] - self.frame_times[0]
        if duration <= 0:
            return 0.0
        return (len(self.frame_times) - 1) / duration

    def _draw_points(self):
        self._draw_grid()

        with self.data_lock:
            frame = dict(self.latest_frame)
            points = list(self.latest_points)

        if not points:
            if self.lidar:
                idle = self.lidar.seconds_since_packet()
                if idle is not None and idle > 2.0:
                    self.status_var.set(
                        f"已连接但暂未收到数据，请确认雷达上传目标是否为 {self.local_ip_var.get()}:{self.listen_port_var.get()}"
                    )
            return

        cx = self.canvas_size / 2
        cy = self.canvas_size / 2
        scale = (self.canvas_size / 2 - 20) / self.max_distance

        point_count = len(points)
        fps = self._calc_fps()
        self.info_var.set(
            f"点数: {point_count} | 包数: {frame.get('packet_count', 0)} | 帧号: {frame.get('frame_cnt', 0)} | FPS: {fps:.1f}"
        )
        self.status_var.set(f"实时数据正常，最近一帧收到 {point_count} 个点")

        for point in points:
            distance = point["distance_mm"]
            if distance <= 0 or distance > self.max_distance:
                continue

            x = cx + point["x"] * 1000.0 * scale
            y = cy - point["y"] * 1000.0 * scale
            if x < 0 or y < 0 or x > self.canvas_size or y > self.canvas_size:
                continue

            point_color = "#00FF00"
            if point["z"] > 0.1:
                point_color = "#00B7FF"
            elif point["z"] < -0.1:
                point_color = "#FFD700"

            self.canvas.create_oval(x - 1, y - 1, x + 1, y + 1, fill=point_color, outline="")

    def on_closing(self):
        if self.lidar:
            self.lidar.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = LidarViewerGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
