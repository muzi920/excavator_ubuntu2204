import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time

from zs_excavator_controller import build_controller

class ExcavatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("挖掘机模型控制_lb")
        self.root.geometry("1000x450")  # 窗口加宽
        
        # 尝试连接控制器
        try:
            self.controller = build_controller()
            if not self.controller.connect():
                messagebox.showwarning("连接失败", "无法打开串口，当前处于离线测试模式 (指令仅会尝试打印，不会下发)。")
                # self.root.destroy()
                # return
        except Exception as e:
            messagebox.showerror("初始化失败", str(e))
            self.root.destroy()
            return

        # 当前动作状态，用于防止键盘自动重复发送信号
        self.active_actions = set()

        # 模拟量内部变量 (初始值改为 2000)
        self.ch1_var = tk.IntVar(value=2000)
        self.ch2_var = tk.IntVar(value=2000)
        self.ch3_var = tk.IntVar(value=2000)

        self._build_ui()
        self._bind_keys()

        # 启动时下发一次默认模拟量
        self._sync_analogs()

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ==========================================
        # 1. 模拟量控制区
        # ==========================================
        analog_frame = ttk.LabelFrame(main_frame, text="模拟量 (液压/速度) 控制", padding=10)
        analog_frame.pack(fill=tk.X, pady=(0, 10))

        self._create_analog_row(analog_frame, "通道 1 (左履带)", self.ch1_var, 0)
        self._create_analog_row(analog_frame, "通道 2 (右履带)", self.ch2_var, 1)
        self._create_analog_row(analog_frame, "通道 3 (液压/备用)", self.ch3_var, 2)

        # 添加一个专门用来实时显示底层模拟量状态的标签
        self.status_label = ttk.Label(analog_frame, text="当前系统输出: 左履带电机=2000, 右履带电机=2000, 液压=2000", foreground="blue", font=("", 10, "bold"))
        self.status_label.grid(row=3, column=0, columnspan=3, pady=5)

        # ==========================================
        # 2. 动作控制区
        # ==========================================
        controls_frame = ttk.Frame(main_frame)
        controls_frame.pack(fill=tk.BOTH, expand=True)

        # 左侧：底盘控制
        chassis_frame = ttk.LabelFrame(controls_frame, text="底盘行走 (W/A/S/D | Q/E/Z/C)", padding=10)
        chassis_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        self._create_action_btn(chassis_frame, "左前 (Q)", "left_track_forward", lambda: self.controller.left_track_forward(self.ch2_var.get()), self.controller.stop_chassis, 0, 0)
        self._create_action_btn(chassis_frame, "前进 (W)", "drive_forward", lambda: self.controller.drive_forward(self.ch2_var.get(), self.ch1_var.get()), self.controller.stop_chassis, 0, 1)
        self._create_action_btn(chassis_frame, "右前 (E)", "right_track_forward", lambda: self.controller.right_track_forward(self.ch1_var.get()), self.controller.stop_chassis, 0, 2)

        self._create_action_btn(chassis_frame, "左转 (A)", "turn_left", lambda: self.controller.turn_left(self.ch2_var.get(), self.ch1_var.get()), self.controller.stop_chassis, 1, 0)
        self._create_action_btn(chassis_frame, "急停 (Space)", "stop_all", self._handle_stop_all, None, 1, 1)
        self._create_action_btn(chassis_frame, "右转 (D)", "turn_right", lambda: self.controller.turn_right(self.ch2_var.get(), self.ch1_var.get()), self.controller.stop_chassis, 1, 2)

        self._create_action_btn(chassis_frame, "左后 (Z)", "left_track_backward", lambda: self.controller.left_track_backward(self.ch2_var.get()), self.controller.stop_chassis, 2, 0)
        self._create_action_btn(chassis_frame, "后退 (S)", "drive_backward", lambda: self.controller.drive_backward(self.ch2_var.get(), self.ch1_var.get()), self.controller.stop_chassis, 2, 1)
        self._create_action_btn(chassis_frame, "右后 (C)", "right_track_backward", lambda: self.controller.right_track_backward(self.ch1_var.get()), self.controller.stop_chassis, 2, 2)

        # 右侧：机械臂控制 (包含两个并排的十字键组)
        arm_frame = ttk.LabelFrame(controls_frame, text="机械臂控制", padding=10)
        arm_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))

        # --- 内部再分左右两块 ---
        left_arm_frame = ttk.Frame(arm_frame)
        left_arm_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        right_arm_frame = ttk.Frame(arm_frame)
        right_arm_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))

        # 1. 铲斗 / 大臂 (NumPad 4,6,8,2) - 放在左侧子区域
        ttk.Label(left_arm_frame, text="大臂/铲斗 (小键盘):", font=("", 9, "bold")).grid(row=0, column=0, columnspan=3, pady=(0, 10))
        self._create_action_btn(left_arm_frame, "大臂 抬起 (8)", "boom_up", self.controller.boom_up, self.controller.stop_boom_bucket, 1, 1)
        self._create_action_btn(left_arm_frame, "铲斗 回拉 (4)", "bucket_in", self.controller.bucket_in, self.controller.stop_boom_bucket, 2, 0)
        self._create_action_btn(left_arm_frame, "铲斗 外推 (6)", "bucket_out", self.controller.bucket_out, self.controller.stop_boom_bucket, 2, 2)
        self._create_action_btn(left_arm_frame, "大臂 落下 (2)", "boom_down", self.controller.boom_down, self.controller.stop_boom_bucket, 3, 1)

        # 2. 小臂 / 回转 (I, J, M, L) - 放在右侧子区域
        ttk.Label(right_arm_frame, text="小臂/回转 (I/J/M/L):", font=("", 9, "bold")).grid(row=0, column=0, columnspan=3, pady=(0, 10))
        self._create_action_btn(right_arm_frame, "小臂 回拉 (I)", "arm_pull", self.controller.arm_pull, self.controller.stop_arm_swing, 1, 1)
        self._create_action_btn(right_arm_frame, "回转 左转 (J)", "swing_left", self.controller.swing_left, self.controller.stop_arm_swing, 2, 0)
        self._create_action_btn(right_arm_frame, "回转 右转 (L)", "swing_right", self.controller.swing_right, self.controller.stop_arm_swing, 2, 2)
        self._create_action_btn(right_arm_frame, "小臂 前推 (M)", "arm_push", self.controller.arm_push, self.controller.stop_arm_swing, 3, 1)


    def _create_analog_row(self, parent, label_text, tk_var, row):
        ttk.Label(parent, text=label_text).grid(row=row, column=0, padx=5, pady=5, sticky="w")
        
        # 滑动条
        scale = ttk.Scale(parent, from_=0, to=5000, variable=tk_var, orient=tk.HORIZONTAL, length=300)
        scale.grid(row=row, column=1, padx=10, pady=5)
        # 滑动条松开时同步数据
        scale.bind("<ButtonRelease-1>", lambda e: self._sync_analogs())
        
        # 输入框
        entry = ttk.Entry(parent, textvariable=tk_var, width=6)
        entry.grid(row=row, column=2, padx=5, pady=5)
        # 回车键同步数据
        entry.bind("<Return>", lambda e: self._sync_analogs())

    def _create_action_btn(self, parent, text, action_id, start_func, stop_func, row, col):
        btn = tk.Button(parent, text=text, width=12, height=2, bg="#f0f0f0")
        btn.grid(row=row, column=col, padx=3, pady=3)
        
        if stop_func:
            btn.bind("<ButtonPress-1>", lambda e, a=action_id, sf=start_func, b=btn: self._start_action(a, sf, b))
            btn.bind("<ButtonRelease-1>", lambda e, a=action_id, stf=stop_func, b=btn: self._stop_action(a, stf, b))
        else: # 急停按钮
            btn.configure(bg="#ffcccc", fg="red")
            btn.bind("<ButtonPress-1>", lambda e, f=start_func: f())

    # ==========================================
    # 控制逻辑
    # ==========================================

    def _update_status_label(self):
        # 从底层控制器读取缓存值来显示
        v1 = self.controller._current_ch1_mv
        v2 = self.controller._current_ch2_mv
        v3 = self.controller._current_ch3_mv
        self.status_label.config(text=f"当前系统输出: 左履带电机={v1}mV, 右履带电机={v2}mV, 液压={v3}mV")

    def _sync_analogs(self):
        try:
            v1 = self.ch1_var.get()
            v2 = self.ch2_var.get()
            v3 = self.ch3_var.get()
            print(f"[GUI] 更新模拟量: CH1={v1}mV, CH2={v2}mV, CH3={v3}mV")
            self.controller.set_analog(v1, v2, v3)
            self._update_status_label()
        except Exception as e:
            print(f"[GUI Error] 同步模拟量失败: {e}")

    def _handle_stop_all(self):
        print("[GUI] 触发急停 (只停止动作，不再归零模拟量)")
        # 仅停止底盘和机械臂，保留模拟量状态
        self.controller.stop_chassis()
        self.controller.stop_boom_bucket()
        self.controller.stop_arm_swing()
        self._update_status_label()

    def _start_action(self, action_id, start_func, btn=None):
        if action_id not in self.active_actions:
            self.active_actions.add(action_id)
            if btn: btn.configure(bg="#cceeff")
            print(f"[GUI] 开始动作: {action_id}")
            try:
                start_func()
                self._update_status_label() # 更新一下状态显示，因为可能部分通道(履带)的值被修改了
            except Exception as e:
                print(f"[GUI Error] 启动动作异常: {e}")

    def _stop_action(self, action_id, stop_func, btn=None):
        if action_id in self.active_actions:
            self.active_actions.remove(action_id)
            if btn: btn.configure(bg="#f0f0f0")
            print(f"[GUI] 停止动作: {action_id}")
            try:
                stop_func()
            except Exception as e:
                print(f"[GUI Error] 停止动作异常: {e}")

    # ==========================================
    # 键盘绑定
    # ==========================================
    def _bind_keys(self):
        # 按键映射表: (键名, action_id, 启动函数, 停止函数)
        # 底盘
        self.key_map = {
            'w': ('drive_forward', lambda: self.controller.drive_forward(self.ch2_var.get(), self.ch1_var.get()), self.controller.stop_chassis),
            's': ('drive_backward', lambda: self.controller.drive_backward(self.ch2_var.get(), self.ch1_var.get()), self.controller.stop_chassis),
            'a': ('turn_left', lambda: self.controller.turn_left(self.ch2_var.get(), self.ch1_var.get()), self.controller.stop_chassis),
            'd': ('turn_right', lambda: self.controller.turn_right(self.ch2_var.get(), self.ch1_var.get()), self.controller.stop_chassis),
            'q': ('left_track_forward', lambda: self.controller.left_track_forward(self.ch2_var.get()), self.controller.stop_chassis),
            'z': ('left_track_backward', lambda: self.controller.left_track_backward(self.ch2_var.get()), self.controller.stop_chassis),
            'e': ('right_track_forward', lambda: self.controller.right_track_forward(self.ch1_var.get()), self.controller.stop_chassis),
            'c': ('right_track_backward', lambda: self.controller.right_track_backward(self.ch1_var.get()), self.controller.stop_chassis),
            
            # 大臂/铲斗 (小键盘 8,2,4,6 在 tkinter 中对应 KP_8, KP_2 等)
            '8': ('boom_up', self.controller.boom_up, self.controller.stop_boom_bucket),
            'KP_8': ('boom_up', self.controller.boom_up, self.controller.stop_boom_bucket),
            '2': ('boom_down', self.controller.boom_down, self.controller.stop_boom_bucket),
            'KP_2': ('boom_down', self.controller.boom_down, self.controller.stop_boom_bucket),
            '4': ('bucket_in', self.controller.bucket_in, self.controller.stop_boom_bucket),
            'KP_4': ('bucket_in', self.controller.bucket_in, self.controller.stop_boom_bucket),
            '6': ('bucket_out', self.controller.bucket_out, self.controller.stop_boom_bucket),
            'KP_6': ('bucket_out', self.controller.bucket_out, self.controller.stop_boom_bucket),
            
            # 小臂/回转
            'i': ('arm_pull', self.controller.arm_pull, self.controller.stop_arm_swing),
            'm': ('arm_push', self.controller.arm_push, self.controller.stop_arm_swing),
            'j': ('swing_left', self.controller.swing_left, self.controller.stop_arm_swing),
            'l': ('swing_right', self.controller.swing_right, self.controller.stop_arm_swing),
            
            # 急停 (空格键)
            'space': ('stop_all', self._handle_stop_all, None),
        }

        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)

    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key == 'space': key = 'space' # 空格特殊处理
        elif event.keysym.startswith('KP_'): key = event.keysym # 小键盘特殊处理

        if key in self.key_map:
            action_id, start_func, _ = self.key_map[key]
            self._start_action(action_id, start_func)

    def _on_key_release(self, event):
        key = event.keysym.lower()
        if key == 'space': key = 'space'
        elif event.keysym.startswith('KP_'): key = event.keysym

        if key in self.key_map:
            action_id, _, stop_func = self.key_map[key]
            if stop_func: # 急停没有 stop_func
                self._stop_action(action_id, stop_func)


if __name__ == "__main__":
    root = tk.Tk()
    app = ExcavatorGUI(root)
    root.mainloop()
