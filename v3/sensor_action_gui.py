import tkinter as tk
import time
import threading
from tkinter import ttk, messagebox
import sys
import os

# 添加 v2 和 v3 的依赖路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v2")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v3", "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new")))

from action_scheduler import ActionScheduler
import device_model

class ExcavatorSensorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("挖掘机动作与角度传感器采集测试系统")
        self.root.geometry("1400x700")

        # 尝试连接 CAN 调度器 (v2 逻辑)
        self.scheduler = None
        self._init_controller()

        # --- 状态与缓存变量 ---
        self.is_running = False
        self.ch1_var = tk.IntVar(value=2000)
        self.ch2_var = tk.IntVar(value=2000)
        self.ch3_var = tk.IntVar(value=2000)
        self.duration_var = tk.DoubleVar(value=0.5)

        # 传感器最新数据缓存
        self.sensor_data = {
            "大臂": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "小臂": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "铲斗": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "回转": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }

        # 记录的历史列表
        self.history_records = []
        
        # 极限状态缓存
        self.extreme_states = {
            1: None,
            2: None
        }

        # --- 初始化传感器 ---
        self.devices = []
        self._init_sensors()

        # 构建 UI
        self._build_ui()
        
        # 启动定时刷新 UI 线程
        self.root.after(100, self._update_sensor_ui)

    def _init_controller(self):
        try:
            if self.scheduler is None:
                self.scheduler = ActionScheduler()
            
            if self.scheduler.connect():
                self.controller_connected = True
                print("挖掘机控制器连接成功！")
            else:
                self.controller_connected = False
                messagebox.showwarning("连接失败", "无法打开 CAN 串口，当前处于离线测试模式 (仅记录传感器)。")
        except Exception as e:
            self.controller_connected = False
            messagebox.showerror("初始化失败", f"CAN 初始化异常: {e}")

    def _reconnect_controller(self):
        if self.controller_connected:
            messagebox.showinfo("提示", "控制器已经是连接状态了。")
            return
        self._init_controller()
        if self.controller_connected:
            messagebox.showinfo("连接成功", "已成功连接到挖掘机控制板！")

    def _init_sensors(self):
        addrLis = [0x50]
        baud = 230400
        configs = [
            ("大臂", "COM11"),
            ("小臂", "COM8"),
            ("铲斗", "COM7"),
            ("回转", "COM12"),
        ]
        
        for name, port in configs:
            try:
                device = device_model.DeviceModel(name, port, baud, addrLis, self._create_sensor_callback(name))
                device.openDevice()
                device.startLoopRead()
                self.devices.append(device)
                print(f"[{name}] {port} 传感器初始化成功")
            except Exception as e:
                print(f"[{name}] {port} 传感器初始化失败: {e}")

    def _create_sensor_callback(self, sensor_name):
        def updateData(DeviceModel):
            addr = DeviceModel.addrLis[0]
            data = DeviceModel.deviceData.get(addr, {})
            if data:
                self.sensor_data[sensor_name]["roll"] = data.get("AngX", 0.0)
                self.sensor_data[sensor_name]["pitch"] = data.get("AngY", 0.0)
                self.sensor_data[sensor_name]["yaw"] = data.get("AngZ", 0.0)
        return updateData

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # === 左侧控制区 ===
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        # 1. 模拟量与时间
        top_frame = ttk.LabelFrame(left_frame, text="全局参数配置", padding=10)
        top_frame.pack(fill=tk.X, pady=(0, 10))
        self._create_analog_row(top_frame, "左履带", self.ch1_var, 0)
        self._create_analog_row(top_frame, "右履带", self.ch2_var, 1)
        self._create_analog_row(top_frame, "液压", self.ch3_var, 2)
        ttk.Button(top_frame, text="下发模拟量", command=self._sync_analogs).grid(row=0, column=3, rowspan=3, padx=10, sticky="ns")
        ttk.Label(top_frame, text="执行时间(s):", font=("", 10, "bold")).grid(row=3, column=0, pady=10)
        ttk.Entry(top_frame, textvariable=self.duration_var, width=10).grid(row=3, column=1, pady=10, sticky="w")

        # 2. 动作触发区
        controls_frame = ttk.LabelFrame(left_frame, text="机械臂动作控制", padding=10)
        controls_frame.pack(fill=tk.X)
        
        # --- 新增重新连接按钮 ---
        btn_reconnect = tk.Button(controls_frame, text="重新连接控制设备", bg="#b3e6ff", command=self._reconnect_controller)
        btn_reconnect.grid(row=0, column=0, columnspan=3, pady=(0, 10), sticky="ew")
        
        row_idx = 1
        ttk.Label(controls_frame, text="大臂:").grid(row=row_idx, column=0, pady=5)
        self._create_action_btn(controls_frame, "大臂 抬起", "boom_up", self.scheduler.controller.boom_up, row_idx, 1)
        self._create_action_btn(controls_frame, "大臂 落下", "boom_down", self.scheduler.controller.boom_down, row_idx, 2)
        
        row_idx += 1
        ttk.Label(controls_frame, text="小臂:").grid(row=row_idx, column=0, pady=5)
        self._create_action_btn(controls_frame, "小臂 回拉", "arm_pull", self.scheduler.controller.arm_pull, row_idx, 1)
        self._create_action_btn(controls_frame, "小臂 前推", "arm_push", self.scheduler.controller.arm_push, row_idx, 2)

        row_idx += 1
        ttk.Label(controls_frame, text="铲斗:").grid(row=row_idx, column=0, pady=5)
        self._create_action_btn(controls_frame, "铲斗 回拉", "bucket_in", self.scheduler.controller.bucket_in, row_idx, 1)
        self._create_action_btn(controls_frame, "铲斗 外推", "bucket_out", self.scheduler.controller.bucket_out, row_idx, 2)

        row_idx += 1
        ttk.Label(controls_frame, text="回转:").grid(row=row_idx, column=0, pady=5)
        self._create_action_btn(controls_frame, "回转 左转", "swing_left", self.scheduler.controller.swing_left, row_idx, 1)
        self._create_action_btn(controls_frame, "回转 右转", "swing_right", self.scheduler.controller.swing_right, row_idx, 2)

        row_idx += 1
        stop_btn = tk.Button(controls_frame, text="急停", width=12, bg="#ffcccc", fg="red", command=self._handle_stop_all)
        stop_btn.grid(row=row_idx, column=1, columnspan=2, pady=10)

        # === 右侧状态与记录区 ===
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # 1. 实时角度显示
        status_frame = ttk.LabelFrame(right_frame, text="实时倾角传感器状态", padding=10)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.lbl_boom = ttk.Label(status_frame, text="大臂: --", font=("Consolas", 12))
        self.lbl_boom.grid(row=0, column=0, sticky="w", pady=2, padx=10)
        self.lbl_arm = ttk.Label(status_frame, text="小臂: --", font=("Consolas", 12))
        self.lbl_arm.grid(row=1, column=0, sticky="w", pady=2, padx=10)
        self.lbl_bucket = ttk.Label(status_frame, text="铲斗: --", font=("Consolas", 12))
        self.lbl_bucket.grid(row=2, column=0, sticky="w", pady=2, padx=10)
        self.lbl_swing = ttk.Label(status_frame, text="回转: --", font=("Consolas", 12))
        self.lbl_swing.grid(row=3, column=0, sticky="w", pady=2, padx=10)

        # 2. 关系计算与打点记录
        record_frame = ttk.LabelFrame(right_frame, text="关节夹角关系与打点记录", padding=10)
        record_frame.pack(fill=tk.BOTH, expand=True)

        btn_frame = ttk.Frame(record_frame)
        btn_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(btn_frame, text="记录当前状态", command=self._record_current_state).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="记录极限状态 1 (收缩)", command=lambda: self._record_extreme(1)).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="记录极限状态 2 (伸展)", command=lambda: self._record_extreme(2)).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="计算极限落差", command=self._calculate_extreme_diff).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="清空记录", command=self._clear_records).pack(side=tk.LEFT, padx=5)

        self.tree = ttk.Treeview(record_frame, columns=("time", "action", "bucket", "arm", "boom", "swing"), show="headings")
        self.tree.heading("time", text="时间")
        self.tree.heading("action", text="触发动作")
        self.tree.heading("bucket", text="铲斗Pitch")
        self.tree.heading("arm", text="小臂Pitch")
        self.tree.heading("boom", text="大臂Pitch")
        self.tree.heading("swing", text="回转Yaw")
        
        self.tree.column("time", width=100)
        self.tree.column("action", width=150)
        self.tree.column("bucket", width=120)
        self.tree.column("arm", width=120)
        self.tree.column("boom", width=120)
        self.tree.column("swing", width=120)
        
        scrollbar = ttk.Scrollbar(record_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscroll=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    def _create_analog_row(self, parent, label_text, tk_var, row):
        ttk.Label(parent, text=label_text).grid(row=row, column=0, padx=5, pady=5)
        scale = ttk.Scale(parent, from_=0, to=5000, variable=tk_var, orient=tk.HORIZONTAL, length=150)
        scale.grid(row=row, column=1, padx=5, pady=5)
        ttk.Entry(parent, textvariable=tk_var, width=6).grid(row=row, column=2, padx=5)

    def _create_action_btn(self, parent, text, action_name, action_func, row, col):
        btn = tk.Button(parent, text=text, width=12, bg="#e0e0e0")
        btn.grid(row=row, column=col, padx=5, pady=5)
        btn.bind("<ButtonPress-1>", lambda e, name=action_name, f=action_func: self._trigger_action(name, f))

    def _sync_analogs(self):
        if not self.controller_connected:
            print("当前处于离线模式，无法下发模拟量。")
            return
        try:
            v1, v2, v3 = self.ch1_var.get(), self.ch2_var.get(), self.ch3_var.get()
            self.scheduler.controller.set_analog(v1, v2, v3)
        except Exception as e:
            print(f"同步模拟量失败: {e}")

    def _handle_stop_all(self):
        if self.controller_connected:
            self.scheduler.controller.stop_all()
        self.is_running = False

    def _trigger_action(self, action_name, action_func):
        if self.is_running:
            return
            
        duration = self.duration_var.get()
        if duration <= 0: return

        self.is_running = True
        # 执行动作前先打一个点
        self._record_current_state(f"Start: {action_name}")
        
        # 离线模式下，只打点记录，不去真实跑 CAN 任务
        if not self.controller_connected:
            print(f"[离线模式] 动作 '{action_name}' (耗时 {duration}s) 开始模拟执行...")
            self.root.after(int(duration * 1000), lambda: self._offline_action_finish(action_name))
        else:
            threading.Thread(target=self._run_scheduler_task, args=(action_name, action_func, duration), daemon=True).start()

    def _offline_action_finish(self, action_name):
        print(f"[离线模式] 动作 '{action_name}' 模拟执行结束。")
        self._record_current_state(f"End: {action_name}")
        self.is_running = False

    def _run_scheduler_task(self, action_name, action_func, duration):
        try:
            self.scheduler.run_action(
                action_name=action_name,
                action_func=action_func,
                duration_s=duration,
                ch1_mv=self.ch1_var.get(),
                ch2_mv=self.ch2_var.get(),
                ch3_mv=self.ch3_var.get()
            )
            # 动作执行完毕后再打一个点
            self.root.after(0, lambda: self._record_current_state(f"End: {action_name}"))
        finally:
            self.is_running = False

    def _update_sensor_ui(self):
        d = self.sensor_data
        self.lbl_boom.config(text=f"大臂: Roll={d['大臂']['roll']:7.1f}° Pitch={d['大臂']['pitch']:7.1f}° Yaw={d['大臂']['yaw']:7.1f}°")
        self.lbl_arm.config(text=f"小臂: Roll={d['小臂']['roll']:7.1f}° Pitch={d['小臂']['pitch']:7.1f}° Yaw={d['小臂']['yaw']:7.1f}°")
        self.lbl_bucket.config(text=f"铲斗: Roll={d['铲斗']['roll']:7.1f}° Pitch={d['铲斗']['pitch']:7.1f}° Yaw={d['铲斗']['yaw']:7.1f}°")
        self.lbl_swing.config(text=f"回转: Roll={d['回转']['roll']:7.1f}° Pitch={d['回转']['pitch']:7.1f}° Yaw={d['回转']['yaw']:7.1f}°")
        
        self.root.after(100, self._update_sensor_ui)

    def _record_current_state(self, action_desc="Manual Record"):
        d = self.sensor_data
        
        bucket_pitch = d['铲斗']['pitch']
        arm_pitch = d['小臂']['pitch']
        boom_pitch = d['大臂']['pitch']
        swing_yaw = d['回转']['yaw']

        now_str = time.strftime("%H:%M:%S")
        
        self.tree.insert("", tk.END, values=(
            now_str,
            action_desc,
            f"{bucket_pitch:.1f}°",
            f"{arm_pitch:.1f}°",
            f"{boom_pitch:.1f}°",
            f"{swing_yaw:.1f}°"
        ))
        self.tree.yview_moveto(1)

    def _clear_records(self):
        for item in self.tree.get_children():
            self.tree.delete(item)

    def _record_extreme(self, extreme_id):
        d = self.sensor_data
        
        bucket_pitch = d['铲斗']['pitch']
        arm_pitch = d['小臂']['pitch']
        boom_pitch = d['大臂']['pitch']
        swing_yaw = d['回转']['yaw']
        
        self.extreme_states[extreme_id] = {
            "bucket": bucket_pitch,
            "arm": arm_pitch,
            "boom": boom_pitch,
            "swing": swing_yaw
        }
        self._record_current_state(f"【记录 极限状态 {extreme_id}】")

    def _calculate_extreme_diff(self):
        e1 = self.extreme_states[1]
        e2 = self.extreme_states[2]
        if not e1 or not e2:
            messagebox.showwarning("数据不足", "请先将挖掘机移动到两个极限位置，并分别点击“记录极限状态 1”和“记录极限状态 2”。")
            return
        
        diff_bucket = abs(e1["bucket"] - e2["bucket"])
        diff_arm = abs(e1["arm"] - e2["arm"])
        diff_boom = abs(e1["boom"] - e2["boom"])
        diff_swing = abs(e1["swing"] - e2["swing"])
        
        # 针对偏航角的循环差值处理 (例如 350度 和 10度 实际相差 20度)
        if diff_swing > 180:
            diff_swing = 360 - diff_swing
            
        msg = (f"=== 各传感器自身极限范围计算 ===\n\n"
               f"铲斗 Pitch 活动范围: {diff_bucket:.1f}° (从 {e1['bucket']:.1f}° 到 {e2['bucket']:.1f}°)\n"
               f"小臂 Pitch 活动范围: {diff_arm:.1f}° (从 {e1['arm']:.1f}° 到 {e2['arm']:.1f}°)\n"
               f"大臂 Pitch 活动范围: {diff_boom:.1f}° (从 {e1['boom']:.1f}° 到 {e2['boom']:.1f}°)\n"
               f"回转 Yaw 活动范围: {diff_swing:.1f}° (从 {e1['swing']:.1f}° 到 {e2['swing']:.1f}°)")
               
        messagebox.showinfo("计算完成", msg)
        
        # 同样记录到表格中方便查看
        now_str = time.strftime("%H:%M:%S")
        self.tree.insert("", tk.END, values=(
            now_str,
            "【输出 自身极限范围】",
            f"差值 {diff_bucket:.1f}°",
            f"差值 {diff_arm:.1f}°",
            f"差值 {diff_boom:.1f}°",
            f"差值 {diff_swing:.1f}°"
        ))
        self.tree.yview_moveto(1)

    def on_closing(self):
        print("正在关闭传感器...")
        for device in self.devices:
            device.stopLoopRead()
            
        time.sleep(0.5)
        for device in self.devices:
            device.isOpen = False
            device.closeDevice()
            
        self.root.destroy()
        os._exit(0)

if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = ExcavatorSensorGUI(root)
        root.protocol("WM_DELETE_WINDOW", app.on_closing)
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 捕捉到退出信号，强制结束程序...")
        os._exit(0)
