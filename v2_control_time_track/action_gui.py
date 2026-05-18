import tkinter as tk
from tkinter import ttk, messagebox
import threading
import sys
import os

# 引用 v2 下的 ActionScheduler
sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from action_scheduler import ActionScheduler

class ExcavatorGUIV2:
    def __init__(self, root):
        self.root = root
        self.root.title("挖掘机自动调度控制_v2")
        self.root.geometry("1200x500")  # 窗口继续加宽，给右侧日志留出空间
        
        # 尝试连接调度器 (使用 Ubuntu 默认的 /dev/ttyUSB_Controller，如果不对应可以修改)
        try:
            self.scheduler = ActionScheduler(port="/dev/ttyUSB_Controller")
            if not self.scheduler.connect():
                messagebox.showwarning("连接失败", "无法打开串口，当前处于离线测试模式 (指令仅打印不会下发)。")
        except Exception as e:
            messagebox.showerror("初始化失败", str(e))
            self.root.destroy()
            return

        # 控制运行状态，防止重复触发
        self.is_running = False

        # 模拟量内部变量 (初始值改为 2000)
        self.ch1_var = tk.IntVar(value=2000)
        self.ch2_var = tk.IntVar(value=2000)
        self.ch3_var = tk.IntVar(value=2000)
        self.duration_var = tk.DoubleVar(value=1.0) # 动作执行时间设定

        # --- 新增：预设动作序列与状态 ---
        self.sequence_index = 0
        # 格式: (动作ID, UI显示名称, 调用函数名, 执行时长)
        self.preset_sequence = [
            # ("arm_pull", "小臂回拉", "arm_pull", 0.2),
            ("boom_down", "大臂落下", "boom_down", 0.3),
            ("bucket_in", "铲斗回拉", "bucket_in", 1.8),
            ("arm_pull", "小臂回拉", "arm_pull", 0.7),
            ("bucket_in", "铲斗回拉", "bucket_in", 0.7),
            ("boom_up", "大臂抬起", "boom_up", 1.5),
            ("swing_left", "回转左转", "swing_left", 3.5),
            ("bucket_out", "铲斗外推", "bucket_out", 1),
            ("arm_push", "小臂前推", "arm_push", 1),
            ("bucket_out", "铲斗外推", "bucket_out", 1.5),
            ("swing_right", "回转右转", "swing_right", 3.5),
            # ("boom_down", "大臂落下", "boom_down", 0.4),
            # ("bucket_in", "铲斗回拉", "bucket_in", 1.5),
            # ("arm_pull", "小臂回拉", "arm_pull", 0.5),
            # ("bucket_in", "铲斗回拉", "bucket_in", 1.0),
            # ("boom_up", "大臂抬起", "boom_up", 1.5)
        ]

        self._build_ui()
        # self._bind_keys()  # 暂时屏蔽键盘控制功能，目前仅支持手动点击按钮操作

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ==========================================
        # 0. 全局左右分栏
        # ==========================================
        # 左半部分放控制区，右半部分放日志区
        left_main_frame = ttk.Frame(main_frame)
        left_main_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        right_main_frame = ttk.LabelFrame(main_frame, text="操作日志 (实时运动指令与时间)", padding=10)
        right_main_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False, ipadx=5)

        # 添加滚动条和文本框到右侧
        self.log_text = tk.Text(right_main_frame, width=45, state=tk.DISABLED, bg="#f8f8f8")
        scrollbar = ttk.Scrollbar(right_main_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # ==========================================
        # 1. 模拟量与时间控制区 (放左侧)
        # ==========================================
        top_frame = ttk.LabelFrame(left_main_frame, text="全局参数配置 (模拟量推力 & 执行时间)", padding=10)
        top_frame.pack(fill=tk.X, pady=(0, 10))

        # 模拟量设置
        self._create_analog_row(top_frame, "通道 1 (左履带)", self.ch1_var, 0)
        self._create_analog_row(top_frame, "通道 2 (右履带)", self.ch2_var, 1)
        self._create_analog_row(top_frame, "通道 3 (液压/备用)", self.ch3_var, 2)

        # 增加一个独立的“下发模拟量”按钮，用于手动应用拖动后的值
        ttk.Button(top_frame, text="确认并下发模拟量", command=self._sync_analogs).grid(row=0, column=3, rowspan=3, padx=15, sticky="ns")

        # 时间设置
        ttk.Label(top_frame, text="每次点击执行时间(秒):", font=("", 10, "bold")).grid(row=3, column=0, padx=5, pady=10, sticky="e")
        ttk.Entry(top_frame, textvariable=self.duration_var, width=10).grid(row=3, column=1, padx=10, pady=10, sticky="w")
        
        # 状态提示标签
        self.status_label = ttk.Label(top_frame, text="就绪。调整好参数后，可点击上方按钮应用模拟量，再点击下方动作按钮。", foreground="blue")
        self.status_label.grid(row=4, column=0, columnspan=4, pady=5)

        # ==========================================
        # 1.5 预设动作控制区 与 动作录制区
        # ==========================================
        seq_frame = ttk.LabelFrame(left_main_frame, text="预设序列执行 & 动作剧本录制", padding=10)
        seq_frame.pack(fill=tk.X, pady=(0, 10))

        self.seq_status_label = ttk.Label(seq_frame, text=f"当前进度: {self.sequence_index}/{len(self.preset_sequence)}", font=("", 10, "bold"))
        self.seq_status_label.pack(side=tk.LEFT, padx=10)

        ttk.Button(seq_frame, text="执行下一步", command=self._execute_next_sequence).pack(side=tk.LEFT, padx=5)
        ttk.Button(seq_frame, text="重置进度", command=self._reset_sequence).pack(side=tk.LEFT, padx=5)
        
        # 新增：动作录制与保存按钮
        self.is_recording = False
        self.recorded_actions = []
        
        self.record_btn = tk.Button(seq_frame, text="🔴 开始录制剧本", command=self._toggle_recording, bg="#ffcccc")
        self.record_btn.pack(side=tk.LEFT, padx=15)
        
        ttk.Button(seq_frame, text="💾 保存录制为 JSON", command=self._save_recorded_json).pack(side=tk.LEFT, padx=5)

        # ==========================================
        # 2. 动作触发区
        # ==========================================
        controls_frame = ttk.Frame(left_main_frame)
        controls_frame.pack(fill=tk.BOTH, expand=True)

        # 左侧：底盘控制
        chassis_frame = ttk.LabelFrame(controls_frame, text="底盘行走 (W/A/S/D | Q/E/Z/C)", padding=10)
        chassis_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        self._create_action_btn(chassis_frame, "左前 (Q)", "left_track_forward", lambda: self.scheduler.controller.left_track_forward(self.ch1_var.get()), 0, 0)
        self._create_action_btn(chassis_frame, "双侧前进 (W)", "drive_forward", lambda: self.scheduler.controller.drive_forward(self.ch1_var.get(), self.ch2_var.get()), 0, 1)
        self._create_action_btn(chassis_frame, "右前 (E)", "right_track_forward", lambda: self.scheduler.controller.right_track_forward(self.ch2_var.get()), 0, 2)

        self._create_action_btn(chassis_frame, "左转 (A)", "turn_left", lambda: self.scheduler.controller.turn_left(self.ch1_var.get(), self.ch2_var.get()), 1, 0)
        
        # 急停比较特殊
        stop_btn = tk.Button(chassis_frame, text="急停 (Space)", width=12, height=2, bg="#ffcccc", fg="red")
        stop_btn.grid(row=1, column=1, padx=3, pady=3)
        stop_btn.bind("<ButtonPress-1>", lambda e: self._handle_stop_all())
        
        self._create_action_btn(chassis_frame, "右转 (D)", "turn_right", lambda: self.scheduler.controller.turn_right(self.ch1_var.get(), self.ch2_var.get()), 1, 2)

        self._create_action_btn(chassis_frame, "左后 (Z)", "left_track_backward", lambda: self.scheduler.controller.left_track_backward(self.ch1_var.get()), 2, 0)
        self._create_action_btn(chassis_frame, "双侧后退 (S)", "drive_backward", lambda: self.scheduler.controller.drive_backward(self.ch1_var.get(), self.ch2_var.get()), 2, 1)
        self._create_action_btn(chassis_frame, "右后 (C)", "right_track_backward", lambda: self.scheduler.controller.right_track_backward(self.ch2_var.get()), 2, 2)

        # 右侧：机械臂控制
        arm_frame = ttk.LabelFrame(controls_frame, text="机械臂控制", padding=10)
        arm_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))

        left_arm_frame = ttk.Frame(arm_frame)
        left_arm_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        right_arm_frame = ttk.Frame(arm_frame)
        right_arm_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))

        # 铲斗 / 大臂
        ttk.Label(left_arm_frame, text="大臂/铲斗 (小键盘):", font=("", 9, "bold")).grid(row=0, column=0, columnspan=3, pady=(0, 10))
        self._create_action_btn(left_arm_frame, "大臂 抬起 (8)", "boom_up", self.scheduler.controller.boom_up, 1, 1)
        self._create_action_btn(left_arm_frame, "铲斗 回拉 (4)", "bucket_in", self.scheduler.controller.bucket_in, 2, 0)
        self._create_action_btn(left_arm_frame, "铲斗 外推 (6)", "bucket_out", self.scheduler.controller.bucket_out, 2, 2)
        self._create_action_btn(left_arm_frame, "大臂 落下 (2)", "boom_down", self.scheduler.controller.boom_down, 3, 1)

        # 小臂 / 回转
        ttk.Label(right_arm_frame, text="小臂/回转 (I/J/M/L):", font=("", 9, "bold")).grid(row=0, column=0, columnspan=3, pady=(0, 10))
        self._create_action_btn(right_arm_frame, "小臂 回拉 (I)", "arm_pull", self.scheduler.controller.arm_pull, 1, 1)
        self._create_action_btn(right_arm_frame, "回转 左转 (J)", "swing_left", self.scheduler.controller.swing_left, 2, 0)
        self._create_action_btn(right_arm_frame, "回转 右转 (L)", "swing_right", self.scheduler.controller.swing_right, 2, 2)
        self._create_action_btn(right_arm_frame, "小臂 前推 (M)", "arm_push", self.scheduler.controller.arm_push, 3, 1)


    def _create_analog_row(self, parent, label_text, tk_var, row):
        ttk.Label(parent, text=label_text).grid(row=row, column=0, padx=5, pady=5, sticky="w")
        scale = ttk.Scale(parent, from_=0, to=5000, variable=tk_var, orient=tk.HORIZONTAL, length=300)
        scale.grid(row=row, column=1, padx=10, pady=5)
        entry = ttk.Entry(parent, textvariable=tk_var, width=6)
        entry.grid(row=row, column=2, padx=5, pady=5)
        # 注意: 移除了松开拖动条或按回车自动下发的绑定，改为由新增的确认按钮控制

    def _sync_analogs(self):
        try:
            v1 = self.ch1_var.get()
            v2 = self.ch2_var.get()
            v3 = self.ch3_var.get()
            print(f"[GUI] 手动下发模拟量: CH1={v1}mV, CH2={v2}mV, CH3={v3}mV")
            self.scheduler.controller.set_analog(v1, v2, v3)
            self.status_label.config(text=f"模拟量已更新。准备就绪。", foreground="green")
        except Exception as e:
            print(f"[GUI Error] 同步模拟量失败: {e}")

    def _create_action_btn(self, parent, text, action_name, action_func, row, col):
        btn = tk.Button(parent, text=text, width=12, height=2, bg="#f0f0f0")
        btn.grid(row=row, column=col, padx=3, pady=3)
        # 点击按钮时触发 v2 调度逻辑
        btn.bind("<ButtonPress-1>", lambda e, name=action_name, f=action_func: self._trigger_action(name, f))


    # ==========================================
    # 预设动作序列逻辑
    # ==========================================
    def _execute_next_sequence(self):
        if self.is_running:
            print("[GUI] 当前有动作正在执行，请稍后再试...")
            return
            
        if self.sequence_index >= len(self.preset_sequence):
            messagebox.showinfo("执行完毕", "预设动作序列已全部执行完毕！")
            return

        action_id, ui_name, func_name, duration = self.preset_sequence[self.sequence_index]
        
        action_func = getattr(self.scheduler.controller, func_name, None)
        if not action_func:
            messagebox.showerror("错误", f"找不到控制函数: {func_name}")
            return
            
        self.is_running = True
        self.status_label.config(text=f"正在执行序列第 {self.sequence_index + 1} 步: {ui_name} (预计 {duration}s) ...", foreground="orange")
        
        v1, v2, v3 = self.ch1_var.get(), self.ch2_var.get(), self.ch3_var.get()
        log_msg = f"▶ [序列 {self.sequence_index + 1}/{len(self.preset_sequence)}] {action_id}\n  时长: {duration}s\n  推力: [{v1}, {v2}, {v3}]\n"
        self._append_log(log_msg)
        
        threading.Thread(target=self._run_sequence_task, args=(ui_name, action_func, duration), daemon=True).start()

    def _run_sequence_task(self, action_name, action_func, duration):
        try:
            self.scheduler.run_action(
                action_name=action_name,
                action_func=action_func,
                duration_s=duration,
                ch1_mv=self.ch1_var.get(),
                ch2_mv=self.ch2_var.get(),
                ch3_mv=self.ch3_var.get()
            )
            self.sequence_index += 1
            
            def update_ui():
                self.status_label.config(text=f"序列动作 {action_name} 执行完毕。", foreground="green")
                self.seq_status_label.config(text=f"当前进度: {self.sequence_index}/{len(self.preset_sequence)}")
                
            self.root.after(0, update_ui)
        except Exception as e:
            print(f"[GUI Error] 执行序列任务失败: {e}")
            self.root.after(0, lambda: self.status_label.config(text=f"动作 {action_name} 执行出错！", foreground="red"))
        finally:
            self.is_running = False

    def _reset_sequence(self):
        self.sequence_index = 0
        self.seq_status_label.config(text=f"当前进度: {self.sequence_index}/{len(self.preset_sequence)}")
        self._append_log("--- 预设序列已重置 ---")

    # ==========================================
    # 动作录制与 JSON 保存逻辑
    # ==========================================
    def _toggle_recording(self):
        if not self.is_recording:
            # 开始录制
            self.is_recording = True
            self.recorded_actions = []
            self.record_btn.config(text="⏹ 停止录制剧本", bg="#ccffcc")
            self._append_log("=========== 开始录制新剧本 ===========")
            messagebox.showinfo("录制已开始", "现在您在下方点击的所有动作（及其模拟量和时间参数）都会被记录下来。")
        else:
            # 停止录制
            self.is_recording = False
            self.record_btn.config(text="🔴 开始录制剧本", bg="#ffcccc")
            self._append_log(f"=========== 录制结束 (共 {len(self.recorded_actions)} 步) ===========")

    def _save_recorded_json(self):
        if self.is_recording:
            messagebox.showwarning("警告", "请先点击停止录制，然后再保存！")
            return
            
        if not self.recorded_actions:
            messagebox.showinfo("提示", "当前没有录制任何动作。")
            return
            
        import json
        from tkinter import filedialog
        
        # 弹出保存文件对话框
        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            initialdir=os.path.dirname(__file__),
            title="保存动作剧本",
            filetypes=[("JSON files", "*.json")]
        )
        
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump(self.recorded_actions, f, ensure_ascii=False, indent=2)
                messagebox.showinfo("保存成功", f"动作剧本已成功保存至:\n{file_path}")
                self._append_log(f"[保存] 剧本已导出到 JSON 文件。")
            except Exception as e:
                messagebox.showerror("保存失败", str(e))

    # ==========================================
    # 控制与调度逻辑
    # ==========================================

    def _append_log(self, msg: str):
        """向右侧日志文本框追加消息"""
        # 利用 after 保证在主线程更新 UI
        def update_ui():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, msg + "\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        self.root.after(0, update_ui)

    def _handle_stop_all(self):
        print("[GUI] 触发急停")
        self.scheduler.controller.stop_all()
        self.status_label.config(text="已急停！所有继电器动作已切断。", foreground="red")
        self._append_log("[紧急停止] 立即切断所有动作及动力。")
        self.is_running = False

    def _trigger_action(self, action_name, action_func):
        if self.is_running:
            print("[GUI] 动作正在执行中，忽略重复触发...")
            return
            
        duration = self.duration_var.get()
        if duration <= 0:
            messagebox.showwarning("时间错误", "执行时间必须大于 0 秒！")
            return

        self.is_running = True
        self.status_label.config(text=f"正在执行: {action_name} (预计 {duration}s) ...", foreground="orange")
        
        # 写入日志
        v1, v2, v3 = self.ch1_var.get(), self.ch2_var.get(), self.ch3_var.get()
        log_msg = f"▶ {action_name}\n  时长: {duration}s\n  推力: [{v1}, {v2}, {v3}]\n"
        self._append_log(log_msg)
        
        # 录制动作到剧本中
        if hasattr(self, 'is_recording') and self.is_recording:
            # 找到 action_func 对应的英文函数名（即我们在 self.preset_sequence 里使用的 action_id）
            action_id = action_func.__name__
            self.recorded_actions.append({
                "step": len(self.recorded_actions) + 1,
                "action": action_id,
                "description": action_name,
                "duration_s": duration,
                "ch1_mv": v1,
                "ch2_mv": v2,
                "ch3_mv": v3
            })
            self._append_log(f"[录制] 已将 {action_name} 记录到剧本序列中。")
        
        # 使用线程去调用 scheduler，避免阻塞 GUI
        threading.Thread(target=self._run_scheduler_task, args=(action_name, action_func, duration), daemon=True).start()

    def _run_scheduler_task(self, action_name, action_func, duration):
        try:
            # 这里的 run_action 已经封装了推力设定、动作下发、等待、停止的完整闭环
            self.scheduler.run_action(
                action_name=action_name,
                action_func=action_func,
                duration_s=duration,
                ch1_mv=self.ch1_var.get(),
                ch2_mv=self.ch2_var.get(),
                ch3_mv=self.ch3_var.get()
            )
            # 执行完成后恢复 UI 状态
            self.root.after(0, lambda: self.status_label.config(text=f"动作 {action_name} 执行完毕。", foreground="green"))
        except Exception as e:
            print(f"[GUI Error] 执行调度任务失败: {e}")
            self.root.after(0, lambda: self.status_label.config(text=f"动作 {action_name} 执行出错！", foreground="red"))
        finally:
            self.is_running = False

    # ==========================================
    # 键盘绑定
    # ==========================================
    def _bind_keys(self):
        self.key_map = {
            'w': ('drive_forward', lambda: self.scheduler.controller.drive_forward(self.ch1_var.get(), self.ch2_var.get())),
            's': ('drive_backward', lambda: self.scheduler.controller.drive_backward(self.ch1_var.get(), self.ch2_var.get())),
            'a': ('turn_left', lambda: self.scheduler.controller.turn_left(self.ch1_var.get(), self.ch2_var.get())),
            'd': ('turn_right', lambda: self.scheduler.controller.turn_right(self.ch1_var.get(), self.ch2_var.get())),
            'q': ('left_track_forward', lambda: self.scheduler.controller.left_track_forward(self.ch1_var.get())),
            'z': ('left_track_backward', lambda: self.scheduler.controller.left_track_backward(self.ch1_var.get())),
            'e': ('right_track_forward', lambda: self.scheduler.controller.right_track_forward(self.ch2_var.get())),
            'c': ('right_track_backward', lambda: self.scheduler.controller.right_track_backward(self.ch2_var.get())),
            
            '8': ('boom_up', self.scheduler.controller.boom_up),
            'KP_8': ('boom_up', self.scheduler.controller.boom_up),
            '2': ('boom_down', self.scheduler.controller.boom_down),
            'KP_2': ('boom_down', self.scheduler.controller.boom_down),
            '4': ('bucket_in', self.scheduler.controller.bucket_in),
            'KP_4': ('bucket_in', self.scheduler.controller.bucket_in),
            '6': ('bucket_out', self.scheduler.controller.bucket_out),
            'KP_6': ('bucket_out', self.scheduler.controller.bucket_out),
            
            'i': ('arm_pull', self.scheduler.controller.arm_pull),
            'm': ('arm_push', self.scheduler.controller.arm_push),
            'j': ('swing_left', self.scheduler.controller.swing_left),
            'l': ('swing_right', self.scheduler.controller.swing_right),
            
            'space': ('stop_all', self._handle_stop_all),
        }

        self.root.bind("<KeyPress>", self._on_key_press)

    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key == 'space': key = 'space'
        elif event.keysym.startswith('KP_'): key = event.keysym

        if key in self.key_map:
            action_name, action_func = self.key_map[key]
            if key == 'space':
                action_func() # 直接急停
            else:
                self._trigger_action(action_name, action_func)

if __name__ == "__main__":
    root = tk.Tk()
    app = ExcavatorGUIV2(root)
    root.mainloop()