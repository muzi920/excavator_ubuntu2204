import tkinter as tk
from tkinter import ttk, messagebox
import sys
import os
import time

# 引入之前的底层库
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v1")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v3", "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new")))

from zs_excavator_controller import build_controller
import device_model
from angle_controller import AngleController

class V4ClosedLoopGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("V4 挖掘机角度闭环控制测试系统")
        self.root.geometry("800x600")

        # 1. 初始化物理控制器
        try:
            self.base_controller = build_controller(port="COM5", baudrate=115200)
            if not self.base_controller.connect():
                messagebox.showwarning("连接失败", "无法打开 CAN 串口(COM5)，当前处于离线模式。")
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
        self.target_swing_yaw = tk.DoubleVar(value=0.0)
        self.ch1_var = tk.IntVar(value=2000)
        self.ch2_var = tk.IntVar(value=2000)
        self.ch3_var = tk.IntVar(value=2000)

        # 初始化传感器
        self._init_sensors()
        self._build_ui()
        self._update_loop()

    def _init_sensors(self):
        addrLis = [0x50]
        baud = 230400
        configs = [("大臂", "COM11"), ("小臂", "COM8"), ("铲斗", "COM7"), ("回转", "COM12")]
        
        for name, port in configs:
            try:
                dev = device_model.DeviceModel(name, port, baud, addrLis, self._sensor_callback(name))
                dev.openDevice()
                dev.startLoopRead()
                self.devices.append(dev)
                print(f"[{name}] {port} 传感器就绪")
            except Exception as e:
                print(f"[{name}] {port} 初始化失败: {e}")

    def _sensor_callback(self, sensor_name):
        def update(dm):
            addr = dm.addrLis[0]
            data = dm.deviceData.get(addr, {})
            if data:
                self.sensor_data[sensor_name]["pitch"] = data.get("AngY", 0.0)
                self.sensor_data[sensor_name]["yaw"] = data.get("AngZ", 0.0)
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
        analog_frame = ttk.LabelFrame(main_frame, text="模拟量推力配置", padding=10)
        analog_frame.pack(fill=tk.X, pady=5)
        ttk.Label(analog_frame, text="CH1(左):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch1_var, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="CH2(右):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch2_var, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="CH3(液压):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch3_var, width=8).pack(side=tk.LEFT, padx=5)

        # --- 下方：闭环目标控制 ---
        ctrl_frame = ttk.LabelFrame(main_frame, text="闭环角度目标控制", padding=10)
        ctrl_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self._create_ctrl_row(ctrl_frame, 0, "铲斗-小臂", "bucket_arm", self.target_bucket_arm)
        self._create_ctrl_row(ctrl_frame, 1, "小臂-大臂", "arm_boom", self.target_arm_boom)
        self._create_ctrl_row(ctrl_frame, 2, "大臂-回转", "boom_swing", self.target_boom_swing)
        self._create_ctrl_row(ctrl_frame, 3, "回转偏航", "swing_yaw", self.target_swing_yaw)

        ttk.Button(main_frame, text="【急停所有闭环动作】", command=self.angle_ctrl.stop_all).pack(pady=20, ipadx=20, ipady=10)

    def _create_ctrl_row(self, parent, row, label_text, joint_name, target_var):
        ttk.Label(parent, text=f"{label_text} 目标角度(°):").grid(row=row, column=0, padx=10, pady=10, sticky="e")
        ttk.Entry(parent, textvariable=target_var, width=10).grid(row=row, column=1, padx=5, pady=10)
        ttk.Button(
            parent, text=f"开始移动 {label_text}", 
            command=lambda: self.angle_ctrl.move_joint_to_angle(
                joint_name, target_var.get(), tolerance=2.0, 
                ch1_mv=self.ch1_var.get(), ch2_mv=self.ch2_var.get(), ch3_mv=self.ch3_var.get()
            )
        ).grid(row=row, column=2, padx=20, pady=10)

    def _update_loop(self):
        # 更新传感器数据给控制器
        self.angle_ctrl.update_sensor_data(self.sensor_data)
        
        # 更新界面显示
        d = self.sensor_data
        diff_ba = abs(d['铲斗']['pitch'] - d['小臂']['pitch'])
        diff_ab = abs(d['小臂']['pitch'] - d['大臂']['pitch'])
        diff_bs = abs(d['大臂']['pitch'] - d['回转']['pitch'])
        yaw_s = d['回转']['yaw']

        self.lbl_bucket_arm.config(text=f"铲斗-小臂 夹角: {diff_ba:6.1f}°")
        self.lbl_arm_boom.config(text=f"小臂-大臂 夹角: {diff_ab:6.1f}°")
        self.lbl_boom_swing.config(text=f"大臂-回转 夹角: {diff_bs:6.1f}°")
        self.lbl_swing_yaw.config(text=f"回转 偏航角: {yaw_s:6.1f}°")

        self.root.after(50, self._update_loop)

    def on_closing(self):
        print("正在关闭...")
        
        # 1. 停止角度控制器的线程和控制指令
        if hasattr(self, 'angle_ctrl') and self.angle_ctrl:
            self.angle_ctrl.stop_all()
            
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