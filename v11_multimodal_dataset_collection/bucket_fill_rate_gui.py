#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import subprocess
import os
import threading
import signal
import sys

class ScriptRunner:
    def __init__(self, name, command):
        self.name = name
        self.command = command
        self.process = None
        self.thread = None
        self.is_running = False

    def start(self, log_callback, status_callback):
        if self.is_running:
            return
        
        self.is_running = True
        self.thread = threading.Thread(
            target=self._run_process,
            args=(log_callback, status_callback),
            daemon=True
        )
        self.thread.start()

    def _run_process(self, log_callback, status_callback):
        try:
            log_callback(f"[{self.name}] 正在启动: {self.command}\n")
            status_callback(self.name, "starting")
            
            self.process = subprocess.Popen(
                self.command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid  # To kill process group later
            )
            
            status_callback(self.name, "running")
            
            for line in iter(self.process.stdout.readline, ''):
                if line:
                    log_callback(f"[{self.name}] {line}")
                    
            self.process.stdout.close()
            return_code = self.process.wait()
            
            if return_code == 0 or return_code == -15:  # SIGTERM
                log_callback(f"[{self.name}] 已停止\n")
            else:
                log_callback(f"[{self.name}] 异常退出, 状态码: {return_code}\n")
                
        except Exception as e:
            log_callback(f"[{self.name}] 运行出错: {e}\n")
            
        finally:
            self.is_running = False
            status_callback(self.name, "stopped")

    def stop(self, log_callback):
        if self.process and self.is_running:
            log_callback(f"[{self.name}] 正在停止...\n")
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception as e:
                log_callback(f"[{self.name}] 停止出错: {e}\n")

class BucketFillRateGUI:
    def __init__(self, root, ros_node=None):
        self.root = root
        self.ros_node = ros_node
        self.root.title("满斗率数据采集控制面板 (Bucket Fill Rate GUI)")
        self.root.geometry("950x700")

        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(current_dir)
        
        hikvision_cmd = f"python3 {os.path.join(current_dir, 'hikvision_cam_node.py')}"
        imu_cmd = f"python3 {os.path.join(parent_dir, 'v5_sensor_read_lidar', 'imu_direct_swing_estimator.py')}"
        lidar_cmd = f"python3 {os.path.join(parent_dir, 'v5_sensor_read_lidar', 'lidar_direct_reader.py')}"

        # 默认的启动命令
        self.scripts = {
            "d435i": ScriptRunner("D435i相机", "ros2 launch realsense2_camera rs_launch.py"),
            "hikvision": ScriptRunner("海康相机", hikvision_cmd),
            "imu": ScriptRunner("IMU", imu_cmd),
            "lidar": ScriptRunner("激光雷达", lidar_cmd)
        }
        
        self.status_labels = {}
        self.buttons = {}
        
        self._build_ui()
        
    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 顶部：标题
        title_label = ttk.Label(main_frame, text="满斗率数据采集控制面板", font=("Arial", 16, "bold"))
        title_label.pack(pady=(0, 15))
        
        # 硬件控制区
        ctrl_frame = ttk.LabelFrame(main_frame, text="硬件设备运行控制", padding=10)
        ctrl_frame.pack(fill=tk.X, pady=5)
        
        # 创建网格
        for i, (key, runner) in enumerate(self.scripts.items()):
            # 状态指示
            status_lbl = tk.Label(ctrl_frame, text="⏹ 已停止", fg="gray", width=12, anchor="w")
            status_lbl.grid(row=i, column=0, padx=10, pady=5)
            self.status_labels[key] = status_lbl
            
            # 名称
            name_lbl = ttk.Label(ctrl_frame, text=runner.name, width=12)
            name_lbl.grid(row=i, column=1, padx=10, pady=5)
            
            # 命令输入框（允许修改）
            cmd_var = tk.StringVar(value=runner.command)
            cmd_entry = tk.Entry(ctrl_frame, textvariable=cmd_var, width=55)
            cmd_entry.grid(row=i, column=2, padx=10, pady=5)
            # 保存引用以便更新命令
            runner.cmd_var = cmd_var
            
            # 启动按钮
            start_btn = tk.Button(ctrl_frame, text="▶ 启动", bg="#ccffcc", width=10,
                                  command=lambda k=key: self._start_script(k))
            start_btn.grid(row=i, column=3, padx=5, pady=5)
            
            # 停止按钮
            stop_btn = tk.Button(ctrl_frame, text="⏹ 停止", bg="#ffcccc", width=10, state=tk.DISABLED,
                                 command=lambda k=key: self._stop_script(k))
            stop_btn.grid(row=i, column=4, padx=5, pady=5)
            
            self.buttons[key] = {"start": start_btn, "stop": stop_btn}
            
        # 一键控制区
        all_frame = ttk.Frame(main_frame, padding=5)
        all_frame.pack(fill=tk.X, pady=10)
        
        tk.Button(all_frame, text="▶ 一键启动所有设备", bg="#aaffaa", width=20, font=("Arial", 10, "bold"),
                  command=self._start_all).pack(side=tk.LEFT, padx=10)
                  
        tk.Button(all_frame, text="⏹ 一键停止所有设备", bg="#ffaaaa", width=20, font=("Arial", 10, "bold"),
                  command=self._stop_all).pack(side=tk.LEFT, padx=10)
                  
        # 日志区
        log_frame = ttk.LabelFrame(main_frame, text="终端运行日志", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, height=15)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # 清空日志按钮
        ttk.Button(log_frame, text="清空日志", command=lambda: self.log_text.delete(1.0, tk.END)).pack(pady=5, anchor="e")

    def _log(self, message):
        def append():
            self.log_text.insert(tk.END, message)
            self.log_text.see(tk.END)
        self.root.after(0, append)

    def _update_status(self, key, state):
        def update():
            lbl = self.status_labels[key]
            btns = self.buttons[key]
            
            if state == "running":
                lbl.config(text="▶ 运行中", fg="green")
                btns["start"].config(state=tk.DISABLED)
                btns["stop"].config(state=tk.NORMAL)
            elif state == "stopped":
                lbl.config(text="⏹ 已停止", fg="gray")
                btns["start"].config(state=tk.NORMAL)
                btns["stop"].config(state=tk.DISABLED)
            elif state == "starting":
                lbl.config(text="⏳ 启动中...", fg="orange")
                btns["start"].config(state=tk.DISABLED)
                btns["stop"].config(state=tk.DISABLED)
        self.root.after(0, update)

    def _start_script(self, key):
        runner = self.scripts[key]
        # 获取输入框中最新的命令
        runner.command = runner.cmd_var.get()
        status_cb = lambda name, state: self._update_status(key, state)
        runner.start(self._log, status_cb)

    def _stop_script(self, key):
        runner = self.scripts[key]
        runner.stop(self._log)

    def _start_all(self):
        for key in self.scripts:
            if not self.scripts[key].is_running:
                self._start_script(key)

    def _stop_all(self):
        for key in self.scripts:
            if self.scripts[key].is_running:
                self._stop_script(key)

    def on_closing(self):
        self._log("正在停止所有脚本并退出...\n")
        self._stop_all()
        import time
        time.sleep(0.5)
        self.root.destroy()

def main():
    root = tk.Tk()
    app = BucketFillRateGUI(root, None)
    
    def on_closing():
        app.on_closing()
        # 强制退出，防止僵尸线程
        os._exit(0)
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
