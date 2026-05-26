import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import sys
import os
import time
import json

# 引入底层库
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v1_control_base")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v3_sensor_read_wit", "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v5_sensor_read_lidar")))

from zs_excavator_controller import build_controller
import device_model
from angle_controller import AngleController

# 引入 IMU 直接解算模块
import socket
import struct
import math
import threading
from imu_direct_swing_estimator import DirectSwingAngleEstimator, LISTEN_PORT

class V4ClosedLoopGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("V4 挖掘机角度闭环控制测试系统")
        self.root.geometry("800x600")

        # 1. 初始化物理控制器
        try:
            self.base_controller = build_controller(port="/dev/ttyUSB_Controller", baudrate=115200)
            if not self.base_controller.connect():
                messagebox.showwarning("连接失败", "无法打开 CAN 串口(/dev/ttyUSB_Controller)，当前处于离线模式。")
        except Exception as e:
            messagebox.showerror("CAN 初始化失败", str(e))
            self.base_controller = None

        # 2. 包装成闭环角度控制器
        self.angle_ctrl = AngleController(self.base_controller)

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
        
        # 保存当前实时计算的角度
        self.current_angles = {
            "bucket_arm": 0.0,
            "arm_boom": 0.0,
            "boom_swing": 0.0,
            "swing_yaw": 0.0
        }

        # 启动雷达 IMU 监听线程
        self.imu_running = True
        self.imu_thread = threading.Thread(target=self._imu_listener_loop, daemon=True)
        self.imu_thread.start()

        # 初始化传感器
        self._init_sensors()
        self._build_ui()
        self._update_loop()

    def _imu_listener_loop(self):
        print("Starting UDP Lidar IMU listener...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('0.0.0.0', LISTEN_PORT))
        except Exception as e:
            print(f"Failed to bind UDP socket for IMU: {e}")
            return

        estimator = DirectSwingAngleEstimator()
        
        while self.imu_running:
            try:
                # 使用超时防止阻塞线程退出
                sock.settimeout(0.5)
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except Exception:
                break
                
            if not data:
                continue
                
            if data[0] == 0xfa and data[1] == 0x88 and len(data) >= 27:
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
                        # 直接更新到传感器数据字典中
                        self.sensor_data["回转"]["yaw"] = swing_deg
                except struct.error:
                    pass
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
        
        for port in ports:
            try:
                # 注意这里传入 port，因为我们要通过 id_to_name 在回调里判断具体是哪个传感器
                dev = device_model.DeviceModel(port, port, baud, addrLis, self._sensor_callback(port))
                dev.openDevice()
                dev.startLoopRead()
                self.devices.append(dev)
                print(f"[{port}] 传感器初始化成功")
            except Exception as e:
                print(f"[{port}] 初始化失败: {e}")

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
                    
                    # 取出数据后清除缓存
                    dm.deviceData[addr].clear()
        return update

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # --- 顶部：传感器实时数据 ---
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
        
        ttk.Button(main_frame, text="【急停所有闭环动作】", command=self.angle_ctrl.stop_all).pack(pady=10, ipadx=20, ipady=10)

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

    def _save_script(self):
        if self.is_recording:
            messagebox.showwarning("警告", "请先停止录制，再进行保存。")
            return
            
        if not self.recorded_script:
            messagebox.showwarning("提示", "当前没有录制任何动作！")
            return
            
        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            initialdir=os.path.dirname(__file__),
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

        self.root.after(50, self._update_loop)

    def on_closing(self):
        print("正在关闭...")
        self.is_running = False
        
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
        app = V4ClosedLoopGUI(root)
        root.protocol("WM_DELETE_WINDOW", app.on_closing)
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 捕捉到退出信号，强制结束程序...")
        os._exit(0)
