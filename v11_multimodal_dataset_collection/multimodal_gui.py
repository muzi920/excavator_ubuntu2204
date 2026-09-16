import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import sys
import os
import time
import json
import threading
import math
import struct
import socket

# 引入底层库
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v1_control_base")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v3_sensor_read_wit", "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v5_sensor_read_lidar")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v4_control_closed")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v10_cailbration_arm")))

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from matplotlib.figure import Figure
from kinematics import ExcavatorKinematics

from zs_excavator_controller import build_controller
import device_model
from angle_controller import AngleController
from imu_direct_swing_estimator import DirectSwingAngleEstimator, LISTEN_PORT

# LIDAR 协议常量
LIDARPOINTCLOUD = 0x01
LIDAR_IP = "192.168.158.99"
LIDAR_PORT = 6543

from multimodal_recorder import MultimodalRecorder, VideoStreamThread

class V11MultimodalGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("V11 多模态挖掘机端到端数据集采集系统")
        self.root.geometry("800x700")
        
        # 初始化数据集记录器
        self.recorder = MultimodalRecorder()
        self.camera_threads = []
        
        # 硬件状态字典
        self.hw_status = {
            "controller": "waiting",
            "sensors": "waiting",
            "lidar": "waiting",
            "cam_hik": "waiting",
            "cam1": "waiting",
            "cam2": "waiting"
        }

        # 1. 初始化物理控制器
        try:
            self.base_controller = build_controller(port="/dev/ttyUSB_Controller", baudrate=115200)
            if not self.base_controller.connect():
                self.hw_status["controller"] = "failed"
                messagebox.showwarning("连接失败", "无法打开 CAN 串口(/dev/ttyUSB_Controller)，当前处于离线模式。")
            else:
                self.hw_status["controller"] = "connected"
        except Exception as e:
            self.hw_status["controller"] = "failed"
            messagebox.showerror("CAN 初始化失败", str(e))
            self.base_controller = None

        # 2. 包装成闭环角度控制器
        self.angle_ctrl = AngleController(self.base_controller)
        self.kin = ExcavatorKinematics()

        # 实时3D可视化相关变量
        self.live_3d_window = None
        self.live_3d_canvas = None
        self.live_frames = []
        self.live_traj_x = []
        self.live_traj_y = []
        self.live_traj_d = []
        self.live_traj_z = []

        # 传感器缓存与设备列表
        self.sensor_data = {
            "大臂": {"pitch": 0.0, "yaw": 0.0},
            "小臂": {"pitch": 0.0, "yaw": 0.0},
            "铲斗": {"pitch": 0.0, "yaw": 0.0},
            "回转": {"pitch": 0.0, "yaw": 0.0},
        }
        self.devices = []

        # UI 变量
        self.target_bucket_arm = tk.DoubleVar(value=90.0)
        self.target_arm_boom = tk.DoubleVar(value=90.0)
        self.target_boom_swing = tk.DoubleVar(value=90.0)
        self.target_swing_yaw = tk.DoubleVar(value=0.0) # 现为目标角度，正右负左
        
        self.ch1_var = tk.IntVar(value=0)
        self.ch2_var = tk.IntVar(value=0)
        self.ch3_var = tk.IntVar(value=2000)
        
        # 柔性控制加减速参数
        self.ramp_up_var = tk.DoubleVar(value=0.2)
        self.ramp_down_var = tk.DoubleVar(value=0.2)
        
        # 新增剧本录制相关变量
        self.is_recording = False
        self.recorded_script = []
        self.script_running = False
        
        # 保存当前实时计算的角度
        self.current_angles = {
            "bucket_arm": 0.0,
            "arm_boom": 0.0,
            "boom_swing": 0.0,
            "swing_yaw": 0.0
        }

        # 设置 JSON 统一保存目录
        self.json_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "json"))
        os.makedirs(self.json_dir, exist_ok=True)

        # 启动雷达 IMU 监听线程
        self.imu_running = True
        self.imu_thread = threading.Thread(target=self._imu_listener_loop, daemon=True)
        self.imu_thread.start()

        # 启动三个摄像头的视频流拉取线程
        self._start_camera_threads()

        # 初始化传感器
        self._init_sensors()
        self._build_ui()
        self._update_loop()

    def _start_camera_threads(self):
        # 海康摄像头 (建议 TCP)
        hik_url = "rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101"
        t_hik = VideoStreamThread("cam_hik", hik_url, self.recorder, transport="tcp", hw_status_dict=self.hw_status)
        
        # 网络摄像头 1 (UDP)
        net1_url = "rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream"
        t_net1 = VideoStreamThread("cam1", net1_url, self.recorder, transport="udp", hw_status_dict=self.hw_status)
        
        # 网络摄像头 2 (UDP)
        net2_url = "rtsp://admin:@192.168.158.103:554/stream"
        t_net2 = VideoStreamThread("cam2", net2_url, self.recorder, transport="udp", hw_status_dict=self.hw_status)
        
        self.camera_threads = [t_hik, t_net1, t_net2]
        self.cams = self.camera_threads
        for t in self.camera_threads:
            t.start()

    def _send_lidar_start_command(self, sock):
        """模拟 C++ 驱动发送启动雷达的点云/IMU 推送指令 (LSTARH)"""
        def crc32_stm32(data):
            crc = 0xFFFFFFFF
            for i in range(0, len(data), 4):
                word = struct.unpack_from('>I', data, i)[0] if i + 4 <= len(data) else 0
                crc ^= word
                for _ in range(32):
                    if crc & 0x80000000:
                        crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
                    else:
                        crc = (crc << 1) & 0xFFFFFFFF
            return crc

        import random
        sn = random.randint(0, 65535)
        sign = 0x484C
        cmd_type = 0x0043
        cmd_str = "LSTARH"
        length = len(cmd_str)
        len4 = ((length + 3) >> 2) * 4
        padded_payload = cmd_str.encode('ascii') + b'\x00' * (len4 - length)
        
        header = struct.pack('<H H H H', sign, cmd_type, sn, length)
        packet_without_crc = header + padded_payload
        crc = crc32_stm32(packet_without_crc)
        packet = packet_without_crc + struct.pack('<I', crc)
        
        for i in range(5):
            sock.sendto(packet, (LIDAR_IP, LIDAR_PORT))
            time.sleep(0.1)
        print("[Lidar] 已发送启动指令 (LSTARH)")

    def _imu_listener_loop(self):
        print("Starting UDP Lidar IMU & PointCloud listener...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Increase UDP receive buffer size to prevent dropping packets
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024 * 8)
        try:
            sock.bind(('0.0.0.0', LISTEN_PORT))
        except Exception as e:
            print(f"Failed to bind UDP socket for Lidar: {e}")
            self.hw_status["lidar"] = "failed"
            return
            
        # 发送启动雷达指令
        self._send_lidar_start_command(sock)

        estimator = DirectSwingAngleEstimator()
        pc_count = 0
        
        while self.imu_running:
            try:
                sock.settimeout(0.5)
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except Exception:
                break
                
            if not data:
                continue
            
            # 只要收到雷达任意有效包，就认为 Lidar 连接成功
            self.hw_status["lidar"] = "connected"
                
            # === 1. 解析 IMU 报文 ===
            if len(data) >= 27 and data[0] == 0xfa and data[1] == 0x88:
                imu_fmt = '<B h h h h h h b H Q'
                try:
                    imu_data = struct.unpack_from(imu_fmt, data, 8 + 1)
                    accel_x = imu_data[1] * 4.0 / 0x10000
                    accel_y = imu_data[2] * 4.0 / 0x10000
                    accel_z = imu_data[3] * 4.0 / 0x10000
                    gyro_x = imu_data[4] * 4000.0 / 0x10000 * math.pi / 180
                    gyro_y = imu_data[5] * 4000.0 / 0x10000 * math.pi / 180
                    gyro_z = imu_data[6] * 4000.0 / 0x10000 * math.pi / 180
                    timestamp = imu_data[9]
                    
                    res = estimator.process_imu((accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), timestamp)
                    if res is not None:
                        swing_deg, w_yaw = res
                        self.sensor_data["回转"]["yaw"] = swing_deg
                        self.sensor_data["回转"]["yaw_rate"] = w_yaw
                        self.sensor_data["回转"]["ts"] = time.time()
                except struct.error:
                    pass
                    
            # === 2. 解析 PointCloud 点云报文 ===
            elif len(data) >= 36 and (data[0] == 0x00 or data[0] == 0x01):
                header = struct.unpack_from('<B H H H H B B B 12s I Q', data, 0)
                dot_num = header[3]
                data_type = header[6]
                
                # 打印出雷达数据包类型，让我们知道是不是雷达发过来的数据不是 LIDARPOINTCLOUD
                # 不用 pc_count 限制打印，因为如果根本没进点云循环，pc_count 就不会增加
                # 为了防止刷屏，我们用时间控制
                if not hasattr(self, '_last_print_time'):
                    self._last_print_time = 0
                if time.time() - self._last_print_time > 2.0:
                    print(f"[雷达数据监听] UDP 接收中... 当前收到的报文类型: {data_type}, 数据点数量: {dot_num}")
                    self._last_print_time = time.time()
                
                # C++ 驱动实际上并不检查 data_type，只要包头是 0 或 1 均认为是点云
                if self.recorder.is_recording:
                    try:
                        # 优化：使用 numpy 高效解析点云并聚合保存，避免每秒创建几百个线程和文件导致卡死
                        if not hasattr(self, 'pc_buffer'):
                            self.pc_buffer = []
                            self.last_pc_save_time = time.time()
                            
                        if dot_num > 0 and len(data) >= 36 + dot_num * 10:
                            buf = data[36:36 + dot_num * 10]
                            dt = np.dtype([('word1', '<u4'), ('word2', '<u4'), ('ref', 'u1'), ('tag', 'u1')])
                            arr = np.frombuffer(buf, dtype=dt)
                            
                            depth = arr['word1'] & 0xFFFFFF
                            theta_hi = (arr['word1'] >> 24) & 0xFF
                            theta_lo = arr['word2'] & 0xFFF
                            phi = (arr['word2'] >> 12) & 0xFFFFF
                            
                            theta = (theta_hi << 12) | theta_lo
                            ang = (90000 - theta) * (math.pi / 180000.0)
                            depth_m = depth / 1000.0
                            
                            r = depth_m * np.cos(ang)
                            z = depth_m * np.sin(ang)
                            phi_ang = phi * (math.pi / 180000.0)
                            x = np.cos(phi_ang) * r
                            y = np.sin(phi_ang) * r
                            
                            pts = np.column_stack((x, y, z))
                            self.pc_buffer.extend(pts.tolist())
                            
                            # 每 0.1 秒 (10Hz) 落盘一次聚合的点云，极大降低 IO 压力
                            if time.time() - self.last_pc_save_time >= 0.1:
                                if self.pc_buffer:
                                    ts = time.time()
                                    pts_arr = np.array(self.pc_buffer, dtype=np.float32)
                                    threading.Thread(target=self.recorder.save_pointcloud, args=(ts, pts_arr), daemon=True).start()
                                    self.pc_buffer = []
                                self.last_pc_save_time = time.time()
                    except Exception as e:
                        print(f"[雷达数据监听] 解析点云异常: {e}")

        sock.close()

    def _init_sensors(self):
        addrLis = [0x50, 0x51, 0x52, 0x53]
        baud = 230400
        
        # 使用 Ubuntu 下 udev 规则绑定的软链接名称
        ports = [
            "/dev/ttyUSB_Sensor1",
            "/dev/ttyUSB_Sensor2",
            "/dev/ttyUSB_Sensor3",
            "/dev/ttyUSB_Sensor4",
        ]
        
        success_count = 0
        for port in ports:
            try:
                # 注意这里传入 port，因为我们要通过 id_to_name 在回调里判断具体是哪个传感器
                dev = device_model.DeviceModel(port, port, baud, addrLis, self._sensor_callback(port))
                dev.openDevice()
                dev.startLoopRead()
                self.devices.append(dev)
                print(f"[{port}] 传感器初始化成功")
                success_count += 1
            except Exception as e:
                print(f"[{port}] 初始化失败: {e}")
                
        if success_count == 4:
            self.hw_status["sensors"] = "connected"
        elif success_count > 0:
            self.hw_status["sensors"] = "partial"
        else:
            self.hw_status["sensors"] = "failed"

    def _sensor_callback(self, port_name):
        id_to_name = {
            0x50: "铲斗",
            0x51: "小臂",
            0x52: "大臂",
            0x53: "回转"
        }
        
        def update(dm):
            for addr, name in id_to_name.items():
                data = dm.deviceData.get(addr, {})
                # 我们这里获取 AngX(Roll) 代替之前的 AngY，保持和 v3 ROS2 一致
                if data and "AngX" in data:
                    self.sensor_data[name]["pitch"] = data.get("AngX", 0.0)
                    # 仅当不是回转传感器时才更新 yaw，因为回转 yaw 现由 IMU 专门接管提供
                    if name != "回转":
                        self.sensor_data[name]["yaw"] = data.get("AngZ", 0.0)
                    
                    self.sensor_data[name]["ts"] = time.time()
                    
                    # 取出数据后清除缓存
                    dm.deviceData[addr].clear()
        return update

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # --- 顶部：多模态录制总控区 ---
        dataset_frame = ttk.LabelFrame(main_frame, text="【多模态数据集录制 (图像/点云/本体状态/控制指令)】", padding=10)
        dataset_frame.pack(fill=tk.X, pady=5)
        
        self.btn_dataset_record = tk.Button(dataset_frame, text="🚀 启动端到端数据采集", command=self._toggle_dataset_recording, bg="#ff9999", font=("Arial", 12, "bold"))
        self.btn_dataset_record.pack(fill=tk.X, pady=5)
        
        # --- 硬件连接状态区 ---
        hw_frame = ttk.LabelFrame(main_frame, text="硬件连接状态", padding=10)
        hw_frame.pack(fill=tk.X, pady=5)
        
        self.lbl_hw_ctrl = tk.Label(hw_frame, text="CAN控制: ⏳", width=15, anchor="w")
        self.lbl_hw_ctrl.grid(row=0, column=0, padx=5)
        
        self.lbl_hw_sensors = tk.Label(hw_frame, text="倾角传感器: ⏳", width=18, anchor="w")
        self.lbl_hw_sensors.grid(row=0, column=1, padx=5)
        
        self.lbl_hw_lidar = tk.Label(hw_frame, text="激光雷达: ⏳", width=15, anchor="w")
        self.lbl_hw_lidar.grid(row=0, column=2, padx=5)
        
        self.lbl_hw_cam_hik = tk.Label(hw_frame, text="海康相机: ⏳", width=15, anchor="w")
        self.lbl_hw_cam_hik.grid(row=1, column=0, padx=5, pady=5)
        
        self.lbl_hw_cam1 = tk.Label(hw_frame, text="网络相机1: ⏳", width=15, anchor="w")
        self.lbl_hw_cam1.grid(row=1, column=1, padx=5, pady=5)
        
        self.lbl_hw_cam2 = tk.Label(hw_frame, text="网络相机2: ⏳", width=15, anchor="w")
        self.lbl_hw_cam2.grid(row=1, column=2, padx=5, pady=5)
        
        # --- 传感器实时数据 ---
        status_frame = ttk.LabelFrame(main_frame, text="传感器实时状态", padding=10)
        status_frame.pack(fill=tk.X, pady=5)
        self.lbl_bucket_arm = ttk.Label(status_frame, text="铲斗-小臂 夹角: --°")
        self.lbl_bucket_arm.grid(row=0, column=0, padx=20, pady=5, sticky="w")
        self.lbl_arm_boom = ttk.Label(status_frame, text="小臂-大臂 夹角: --°")
        self.lbl_arm_boom.grid(row=1, column=0, padx=20, pady=5, sticky="w")
        self.lbl_boom_swing = ttk.Label(status_frame, text="大臂-回转 夹角: --°")
        self.lbl_boom_swing.grid(row=0, column=1, padx=20, pady=5, sticky="w")
        self.lbl_swing_yaw = ttk.Label(status_frame, text="回转 偏航角: --°")
        self.lbl_swing_yaw.grid(row=1, column=1, padx=20, pady=5, sticky="w")

        # --- 中间：推力配置 ---
        analog_frame = ttk.LabelFrame(main_frame, text="模拟量与柔性参数配置", padding=10)
        analog_frame.pack(fill=tk.X, pady=5)
        ttk.Label(analog_frame, text="CH1(左):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch1_var, width=6, state="disabled").pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="CH2(右):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch2_var, width=6, state="disabled").pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="CH3(液压):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch3_var, width=6).pack(side=tk.LEFT, padx=5)
        
        ttk.Label(analog_frame, text="|  加速时间(s):").pack(side=tk.LEFT, padx=(15, 5))
        ttk.Entry(analog_frame, textvariable=self.ramp_up_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="减速时间(s):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ramp_down_var, width=5).pack(side=tk.LEFT, padx=5)

        # --- 下方：闭环目标控制 ---
        ctrl_frame = ttk.LabelFrame(main_frame, text="闭环角度目标控制", padding=10)
        ctrl_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self._create_ctrl_row(ctrl_frame, 0, "铲斗-小臂", "bucket_arm", self.target_bucket_arm, "目标角度(°):")
        self._create_ctrl_row(ctrl_frame, 1, "小臂-大臂", "arm_boom", self.target_arm_boom, "目标角度(°):")
        self._create_ctrl_row(ctrl_frame, 2, "大臂-回转", "boom_swing", self.target_boom_swing, "目标角度(°):")
        
        # 回转改为基于 IMU 角度控制
        self._create_ctrl_row(ctrl_frame, 3, "回转动作", "swing_yaw", self.target_swing_yaw, "目标角度(°): (正右负左)")

        # --- 底部：剧本录制与保存区 ---
        record_frame = ttk.Frame(main_frame)
        record_frame.pack(fill=tk.X, pady=10)
        
        self.btn_record = tk.Button(record_frame, text="🔴 开始录制剧本", command=self._toggle_recording, bg="#ffcccc", width=15)
        self.btn_record.pack(side=tk.LEFT, padx=10)
        
        ttk.Button(record_frame, text="💾 保存为 JSON 剧本", command=self._save_script, width=20).pack(side=tk.LEFT, padx=10)
        
        # --- 新增：剧本一键执行区 ---
        self.btn_load_script = tk.Button(record_frame, text="📂 选择并执行 JSON 剧本", command=self._load_and_run_script, bg="#ccccff", width=22)
        self.btn_load_script.pack(side=tk.LEFT, padx=10)
        
        self.lbl_exec_status = ttk.Label(record_frame, text="当前状态: 未执行", font=("Arial", 11))
        self.lbl_exec_status.pack(side=tk.LEFT, padx=10)
        
        ttk.Button(main_frame, text="【急停所有闭环动作】", command=self._emergency_stop).pack(pady=10, ipadx=20, ipady=10)

    def _emergency_stop(self):
        self.script_running = False
        self.angle_ctrl.stop_all()

    def _toggle_dataset_recording(self):
        """开启或关闭多模态数据集录制"""
        if not self.recorder.is_recording:
            self.recorder.start()
            self.btn_dataset_record.config(text="⏹ 停止端到端数据采集", bg="#99ff99")
            messagebox.showinfo("采集启动", f"正在高频同步记录 3路视觉+点云+本体状态！\n保存目录: {self.recorder.session_dir}")
        else:
            self.recorder.stop()
            self.btn_dataset_record.config(text="🚀 启动端到端数据采集", bg="#ff9999")
            messagebox.showinfo("采集停止", "多模态数据集采集已停止，文件已安全刷入磁盘。")

    def _toggle_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.recorded_script = []
            self.btn_record.config(text="⏹ 停止录制剧本", bg="#ccffcc")
            messagebox.showinfo("开始录制", "已开始录制剧本。现在您下发的每一次【开始移动】都会被记录下来。")
        else:
            self.is_recording = False
            self.btn_record.config(text="🔴 开始录制剧本", bg="#ffcccc")
            messagebox.showinfo("停止录制", f"录制已停止，当前共记录了 {len(self.recorded_script)} 个动作，请点击保存。")

    def _load_and_run_script(self):
        if self.script_running:
            messagebox.showwarning("警告", "当前已有剧本正在执行，请先急停！")
            return
            
        file_path = filedialog.askopenfilename(
            initialdir=self.json_dir,
            title="选择要执行的 JSON 剧本",
            filetypes=[("JSON files", "*.json")]
        )
        if not file_path:
            return
            
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                script_data = json.load(f)
        except Exception as e:
            messagebox.showerror("读取失败", f"无法解析 JSON 剧本:\n{e}")
            return
            
        self.script_running = True
        self.btn_load_script.config(state="disabled")
        
        self.live_frames = []
        self.live_traj_x.clear()
        self.live_traj_y.clear()
        self.live_traj_d.clear()
        self.live_traj_z.clear()
        self._open_live_3d_window()
        
        # 自动开启多模态录制（如果还没开）
        self.auto_started_recording = False
        if not self.recorder.is_recording:
            self.recorder.start()
            self.auto_started_recording = True
            self.btn_dataset_record.config(text="⏹ 停止端到端数据采集 (剧本自动)", bg="#99ff99")
        
        threading.Thread(target=self._execute_script_thread, args=(script_data, os.path.basename(file_path)), daemon=True).start()

    def _open_live_3d_window(self):
        if self.live_3d_window is not None:
            try:
                self.live_3d_window.destroy()
            except Exception:
                pass
            
        self.live_3d_window = tk.Toplevel(self.root)
        self.live_3d_window.title("实时 3D 挖掘机位姿可视化")
        self.live_3d_window.geometry("1000x500")
        
        self.fig = Figure(figsize=(10, 4.5))
        self.fig.suptitle('Real-time Excavator 3D Trajectory', fontsize=14)
        
        self.ax_top = self.fig.add_subplot(121)
        self.ax_side = self.fig.add_subplot(122)
        
        # --- 俯视图 ---
        self.ax_top.set_xlim(-2.0, 2.0)
        self.ax_top.set_ylim(-2.0, 2.0)
        self.ax_top.set_aspect('equal')
        self.ax_top.grid(True)
        self.ax_top.set_title('Top View (Swing X-Y)')
        self.ax_top.plot([0], [0], 'rX', markersize=10)
        
        self.line_top, = self.ax_top.plot([], [], 'o-', lw=4, markersize=6, color='blue')
        self.traj_top, = self.ax_top.plot([], [], 'r-', lw=1.5, alpha=0.6)
        
        # --- 侧视图 ---
        self.ax_side.set_xlim(-0.5, 2.0)
        self.ax_side.set_ylim(-1.0, 1.8)
        self.ax_side.set_aspect('equal')
        self.ax_side.grid(True)
        self.ax_side.set_title('Side View (Profile D-Z)')
        self.ax_side.axhline(0, color='brown', linestyle='--')
        self.ax_side.plot([0], [0], 'rX', markersize=10)
        
        self.line_side, = self.ax_side.plot([], [], 'o-', lw=4, markersize=6, color='green')
        self.traj_side, = self.ax_side.plot([], [], 'r-', lw=1.5, alpha=0.6)
        
        self.fig.tight_layout()
        
        self.live_3d_canvas = FigureCanvasTkAgg(self.fig, master=self.live_3d_window)
        self.live_3d_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _save_live_gif(self, filename, save_dir):
        if not self.live_frames:
            return
        print("[GIF] 开始生成并保存实时运动轨迹 GIF...")
        try:
            fig = Figure(figsize=(10, 4.5))
            fig.suptitle(f'Trajectory: {filename}', fontsize=14)
            
            ax_top = fig.add_subplot(121)
            ax_side = fig.add_subplot(122)
            
            ax_top.set_xlim(-2.0, 2.0)
            ax_top.set_ylim(-2.0, 2.0)
            ax_top.set_aspect('equal')
            ax_top.grid(True)
            ax_top.plot([0], [0], 'rX', markersize=10)
            line_top, = ax_top.plot([], [], 'o-', lw=4, color='blue')
            traj_top, = ax_top.plot([], [], 'r-', lw=1.5, alpha=0.6)
            
            ax_side.set_xlim(-0.5, 2.0)
            ax_side.set_ylim(-1.0, 1.8)
            ax_side.set_aspect('equal')
            ax_side.grid(True)
            ax_side.axhline(0, color='brown', linestyle='--')
            ax_side.plot([0], [0], 'rX', markersize=10)
            line_side, = ax_side.plot([], [], 'o-', lw=4, color='green')
            traj_side, = ax_side.plot([], [], 'r-', lw=1.5, alpha=0.6)
            
            traj_x, traj_y, traj_d, traj_z = [], [], [], []
            
            # 降采样，防止生成 GIF 过大或过慢
            frames_to_render = self.live_frames[::2] if len(self.live_frames) > 200 else self.live_frames
            
            def init():
                line_top.set_data([], [])
                traj_top.set_data([], [])
                line_side.set_data([], [])
                traj_side.set_data([], [])
                return line_top, traj_top, line_side, traj_side
                
            def update(frame_state):
                res = self.kin.forward_kinematics_v4(frame_state['boom_swing'], frame_state['arm_boom'], frame_state['bucket_arm'])
                pts_2d = [
                    (self.kin.offset_x, self.kin.offset_z),
                    res['boom_bend'],
                    res['boom_tip'],
                    res['arm_tip'],
                    res['bucket_tip']
                ]
                yaw_rad = math.radians(frame_state['swing_yaw_deg'])
                pts_3d = [(x * math.cos(yaw_rad), x * math.sin(yaw_rad), z) for x, z in pts_2d]
                
                xs = [p[0] for p in pts_3d]
                ys = [p[1] for p in pts_3d]
                zs = [p[2] for p in pts_3d]
                
                line_top.set_data(xs, ys)
                traj_x.append(xs[-1])
                traj_y.append(ys[-1])
                traj_top.set_data(traj_x, traj_y)
                
                ds_signed = [math.hypot(p[0], p[1]) * (1 if p[0] >= 0 else -1) for p in pts_3d]
                line_side.set_data(ds_signed, zs)
                traj_d.append(ds_signed[-1])
                traj_z.append(zs[-1])
                traj_side.set_data(traj_d, traj_z)
                
                return line_top, traj_top, line_side, traj_side
                
            ani = animation.FuncAnimation(fig, update, frames=frames_to_render, init_func=init, blit=True, interval=50)
            
            if save_dir and os.path.exists(save_dir):
                gif_path = os.path.join(save_dir, f"{os.path.splitext(filename)[0]}_realtime.gif")
            else:
                gif_path = os.path.join(self.json_dir, f"{os.path.splitext(filename)[0]}_realtime.gif")
                
            ani.save(gif_path, writer='pillow', fps=20)
            print(f"[GIF] 实时 3D GIF 已保存至: {gif_path}")
            self.root.after(0, lambda: messagebox.showinfo("保存成功", f"执行过程的实时 3D GIF 已保存至:\n{gif_path}"))
        except Exception as e:
            print(f"[GIF] 保存失败: {e}")

    def _execute_script_thread(self, script_data, filename):
        try:
            for idx, step in enumerate(script_data):
                if not self.script_running:
                    break
                if getattr(self.angle_ctrl, "fatal_stop", False):
                    fatal_reason = getattr(self.angle_ctrl, "fatal_reason", "已触发越限急停")
                    self.root.after(0, lambda r=fatal_reason: messagebox.showerror("安全急停", f"剧本执行已中断：\n{r}"))
                    break
                    
                step_num = step.get('step', idx + 1)
                joint = step.get('joint', '')
                desc = step.get('description', '')
                
                # 兼容时间回转
                if joint == "swing_yaw":
                    if 'duration_s' in step and 'target_val' not in step:
                        joint = "swing_time"
                        target_val = step.get('duration_s', 0.0)
                    else:
                        target_val = step.get('target_val', 0.0)
                else:
                    target_val = step.get('target_val', 0.0)
                    
                ch1 = step.get('ch1_mv', 0)
                ch2 = step.get('ch2_mv', 0)
                ch3 = step.get('ch3_mv', 2000)
                ramp_up = step.get('ramp_up_s', 0.0)
                ramp_down = step.get('ramp_down_s', 0.0)
                
                is_init = step.get('is_init_step', False)
                if not is_init and step_num <= 3 and ("初始" in desc or "归位" in desc):
                    is_init = True
                    
                # 更新 UI 状态
                status_text = f"正在执行 [{filename}]: 第{step_num}步 {desc} (目标: {target_val})"
                self.root.after(0, lambda t=status_text: self.lbl_exec_status.config(text=t))
                
                # 触发运动
                self.angle_ctrl.move_joint_to_angle(
                    joint, target_val, tolerance=2.0, 
                    ch1_mv=ch1, ch2_mv=ch2, ch3_mv=ch3,
                    ramp_up_s=ramp_up, ramp_down_s=ramp_down,
                    is_init_step=is_init
                )
                
                # 等待运动完成
                time.sleep(0.1)
                while self.angle_ctrl._running_tasks.get(joint, False):
                    if not self.script_running:
                        self.angle_ctrl.stop_all()
                        break
                    if getattr(self.angle_ctrl, "fatal_stop", False):
                        fatal_reason = getattr(self.angle_ctrl, "fatal_reason", "已触发越限急停")
                        self.root.after(0, lambda r=fatal_reason: messagebox.showerror("安全急停", f"剧本执行已中断：\n{r}"))
                        break
                    time.sleep(0.1)
                    
                # 动作之间强制加一个安全间隔 0.3s
                time.sleep(0.3)
                
        except Exception as e:
            print(f"执行异常: {e}")
        finally:
            self.script_running = False
            self.root.after(0, lambda: self.lbl_exec_status.config(text="当前状态: 执行完毕/已停止"))
            self.root.after(0, lambda: self.btn_load_script.config(state="normal"))
            
            save_dir = self.recorder.session_dir
            threading.Thread(target=self._save_live_gif, args=(filename, save_dir), daemon=True).start()
            
            if getattr(self, 'auto_started_recording', False):
                self.recorder.stop()
                self.auto_started_recording = False
                self.root.after(0, lambda: self.btn_dataset_record.config(text="🚀 启动端到端数据采集", bg="#ff9999"))

    def _save_script(self):
        if self.is_recording:
            messagebox.showwarning("警告", "请先停止录制，再进行保存。")
            return
            
        if not self.recorded_script:
            messagebox.showwarning("提示", "当前没有录制任何动作！")
            return
            
        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            initialdir=self.json_dir,
            title="保存闭环剧本",
            filetypes=[("JSON files", "*.json")]
        )
        
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump(self.recorded_script, f, ensure_ascii=False, indent=2)
                messagebox.showinfo("保存成功", f"成功保存 {len(self.recorded_script)} 步动作到:\n{file_path}")
            except Exception as e:
                messagebox.showerror("保存失败", str(e))

    def _record_current_angle(self, joint_name, label_text, target_var, is_init=False):
        """手动示教：读取当前传感器角度并记录到剧本中"""
        if not self.is_recording:
            messagebox.showwarning("提示", "请先点击下方的『🔴 开始录制剧本』按钮！")
            return
            
        current_val = round(self.current_angles.get(joint_name, 0.0), 1)
        
        # 将当前角度同步显示到输入框中
        target_var.set(current_val)
        
        ch1 = 0
        ch2 = 0
        ch3 = self.ch3_var.get()
        ramp_up = self.ramp_up_var.get()
        ramp_down = self.ramp_down_var.get()
        
        desc = f"{label_text}(手动示教{' - 初始位置' if is_init else ''})"
        
        record_item = {
            "step": len(self.recorded_script) + 1,
            "joint": joint_name,
            "description": desc,
            "ch1_mv": ch1,
            "ch2_mv": ch2,
            "ch3_mv": ch3,
            "ramp_up_s": ramp_up,
            "ramp_down_s": ramp_down,
            "target_val": current_val
        }
        
        if is_init:
            record_item["is_init_step"] = True
                
        self.recorded_script.append(record_item)
        print(f"[示教录制] 已记录: {desc} 当前角度: {current_val}°")

    def _handle_move(self, joint_name, label_text, target_val):
        """处理移动动作并录制剧本"""
        ch1 = 0
        ch2 = 0
        ch3 = self.ch3_var.get()
        ramp_up = self.ramp_up_var.get()
        ramp_down = self.ramp_down_var.get()
        
        if self.is_recording:
            record_item = {
                "step": len(self.recorded_script) + 1,
                "joint": joint_name,
                "description": label_text,
                "ch1_mv": ch1,
                "ch2_mv": ch2,
                "ch3_mv": ch3,
                "ramp_up_s": ramp_up,
                "ramp_down_s": ramp_down,
                "target_val": target_val
            }
                
            self.recorded_script.append(record_item)
            print(f"[录制] 已记录: {label_text} 参数: {target_val}")
            
        self.angle_ctrl.move_joint_to_angle(
            joint_name, target_val, tolerance=2.0, 
            ch1_mv=ch1, ch2_mv=ch2, ch3_mv=ch3,
            ramp_up_s=ramp_up, ramp_down_s=ramp_down
        )

    def _create_ctrl_row(self, parent, row, label_text, joint_name, target_var, entry_label):
        ttk.Label(parent, text=f"{label_text} {entry_label}").grid(row=row, column=0, padx=10, pady=10, sticky="e")
        ttk.Entry(parent, textvariable=target_var, width=15).grid(row=row, column=1, padx=5, pady=10)
        ttk.Button(
            parent, text=f"开始移动 {label_text}", 
            command=lambda: self._handle_move(joint_name, label_text, target_var.get())
        ).grid(row=row, column=2, padx=10, pady=10)
        
        # 对于角度控制的四个关节，添加“记录当前角度”和“记录初始位置”的示教按钮
        ttk.Button(
            parent, text=f"📍 记录当前角度", 
            command=lambda j=joint_name, l=label_text, v=target_var: self._record_current_angle(j, l, v, is_init=False)
        ).grid(row=row, column=3, padx=5, pady=10)
        
        ttk.Button(
            parent, text=f"🏠 记录为初始位置", 
            command=lambda j=joint_name, l=label_text, v=target_var: self._record_current_angle(j, l, v, is_init=True)
        ).grid(row=row, column=4, padx=5, pady=10)

    def _update_loop(self):
        # --- 更新硬件状态 UI ---
        def update_lbl(lbl, prefix, state):
            if state == "connected":
                lbl.config(text=f"{prefix}: ✅ 正常", fg="green")
            elif state == "failed":
                lbl.config(text=f"{prefix}: ❌ 失败", fg="red")
            elif state == "partial":
                lbl.config(text=f"{prefix}: ⚠️ 部分", fg="orange")
            else:
                lbl.config(text=f"{prefix}: ⏳ 等待", fg="blue")

        update_lbl(self.lbl_hw_ctrl, "CAN控制", self.hw_status["controller"])
        update_lbl(self.lbl_hw_sensors, "倾角传感器", self.hw_status["sensors"])
        update_lbl(self.lbl_hw_lidar, "激光雷达", self.hw_status["lidar"])
        update_lbl(self.lbl_hw_cam_hik, "海康相机", self.hw_status["cam_hik"])
        update_lbl(self.lbl_hw_cam1, "网络相机1", self.hw_status["cam1"])
        update_lbl(self.lbl_hw_cam2, "网络相机2", self.hw_status["cam2"])

        # 更新传感器数据给控制器
        self.angle_ctrl.update_sensor_data(self.sensor_data)
        
        # 更新界面显示 (计算真实相减的夹角，这与 v3 版本相符)
        d = self.sensor_data
        diff_ba = d['铲斗']['pitch'] - d['小臂']['pitch']
        diff_ab = d['小臂']['pitch'] - d['大臂']['pitch']
        diff_bs = d['大臂']['pitch'] - d['回转']['pitch']
        yaw_s = d['回转']['yaw']

        self.lbl_bucket_arm.config(text=f"铲斗-小臂 夹角: {diff_ba:6.1f}°")
        self.lbl_arm_boom.config(text=f"小臂-大臂 夹角: {diff_ab:6.1f}°")
        self.lbl_boom_swing.config(text=f"大臂-回转 夹角: {diff_bs:6.1f}°")
        self.lbl_swing_yaw.config(text=f"回转 偏航角: {yaw_s:6.1f}°")
        
        # 更新当前角度缓存，供示教录制使用
        self.current_angles["bucket_arm"] = diff_ba
        self.current_angles["arm_boom"] = diff_ab
        self.current_angles["boom_swing"] = diff_bs
        self.current_angles["swing_yaw"] = yaw_s

        # 记录多模态传感器状态 (10Hz~20Hz 左右)
        ts = time.time()
        yaw_rate = self.sensor_data["回转"].get("yaw_rate", 0.0)
        self.recorder.log_sensor_state(ts, diff_ab, diff_ba, diff_bs, yaw_s, yaw_rate)
        
        # 记录当前的控制指令输出
        if self.script_running:
            ch1, ch2, ch3 = self.angle_ctrl.get_current_outputs() if hasattr(self.angle_ctrl, 'get_current_outputs') else (0,0,0)
        else:
            ch1, ch2, ch3 = 0, 0, 3000
        self.recorder.log_control_cmd(ts, ch1, ch2, ch3)

        # 更新3D可视化
        if self.script_running:
            self.live_frames.append({
                'boom_swing': diff_bs,
                'arm_boom': diff_ab,
                'bucket_arm': diff_ba,
                'swing_yaw_deg': yaw_s
            })
            if self.live_3d_window and self.live_3d_window.winfo_exists() and self.live_3d_canvas:
                res = self.kin.forward_kinematics_v4(diff_bs, diff_ab, diff_ba)
                pts_2d = [
                    (self.kin.offset_x, self.kin.offset_z),
                    res['boom_bend'],
                    res['boom_tip'],
                    res['arm_tip'],
                    res['bucket_tip']
                ]
                yaw_rad = math.radians(yaw_s)
                pts_3d = [(x * math.cos(yaw_rad), x * math.sin(yaw_rad), z) for x, z in pts_2d]
                
                xs = [p[0] for p in pts_3d]
                ys = [p[1] for p in pts_3d]
                zs = [p[2] for p in pts_3d]
                
                self.line_top.set_data(xs, ys)
                self.live_traj_x.append(xs[-1])
                self.live_traj_y.append(ys[-1])
                self.traj_top.set_data(self.live_traj_x, self.live_traj_y)
                
                ds_signed = [math.hypot(p[0], p[1]) * (1 if p[0] >= 0 else -1) for p in pts_3d]
                
                self.line_side.set_data(ds_signed, zs)
                self.live_traj_d.append(ds_signed[-1])
                self.live_traj_z.append(zs[-1])
                self.traj_side.set_data(self.live_traj_d, self.live_traj_z)
                
                self.live_3d_canvas.draw_idle()

        self.root.after(50, self._update_loop)

    def on_closing(self):
        print("正在关闭...")
        self.is_running = False
        
        # 安全停止录制，确保 mp4 和 csv 被正确释放和保存
        if self.recorder.is_recording:
            self.recorder.stop()
            
        # 停止所有拉流线程，防止阻塞退出
        if hasattr(self, 'cams'):
            for cam in self.cams:
                cam.stop()
        
        # 1. 停止角度控制器的线程和控制指令
        if hasattr(self, 'angle_ctrl') and self.angle_ctrl:
            try:
                if hasattr(self.base_controller, 'transport') and getattr(self.base_controller.transport, 'ser', None) and self.base_controller.transport.ser.is_open:
                    self.angle_ctrl.stop_all()
            except Exception as e:
                print(f"关闭时急停异常: {e}")
            
        # 2. 通知所有传感器停止轮询
        for dev in self.devices:
            dev.stopLoopRead()
            
        time.sleep(0.5)
        
        # 3. 强制关闭串口并结束线程
        for dev in self.devices:
            dev.isOpen = False
            dev.closeDevice()
            
        # 4. 关闭底层 CAN 串口
        if hasattr(self, 'base_controller') and self.base_controller:
            try:
                self.base_controller.close()
            except:
                pass
                
        self.root.destroy()
        
        # 强制结束所有残留守护线程，防止 Ctrl+C 后进程卡住
        os._exit(0)

if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = V11MultimodalGUI(root)
        root.protocol("WM_DELETE_WINDOW", app.on_closing)
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 捕捉到退出信号，强制结束程序...")
        os._exit(0)
