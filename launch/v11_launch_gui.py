import os
import subprocess
import signal
import time
import tkinter as tk
from tkinter import messagebox

class V11LaunchGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("V11 数据集采集控制台")
        self.root.geometry("450x350")
        self.root.resizable(False, False)

        # 状态变量
        self.gui_process = None
        self.record_process = None

        self._setup_ui()

    def _setup_ui(self):
        # 1. 传感器/控制 GUI 启动区域
        frame_gui = tk.LabelFrame(self.root, text="ROS 2 传感器与控制节点", padx=10, pady=10)
        frame_gui.pack(fill="x", padx=10, pady=10)

        self.btn_gui = tk.Button(frame_gui, text="▶ 启动 V11 传感器 GUI", 
                                 bg="#a2d5f2", font=("Arial", 11, "bold"), height=2, command=self.toggle_gui)
        self.btn_gui.pack(fill="x", pady=5)

        self.lbl_gui_status = tk.Label(frame_gui, text="GUI 状态: 未启动", fg="gray")
        self.lbl_gui_status.pack()

        # 2. Rosbag 录制区域
        frame_record = tk.LabelFrame(self.root, text="V11 纯净数据集录制 (Rosbag)", padx=10, pady=10)
        frame_record.pack(fill="x", padx=10, pady=10)

        tk.Label(frame_record, text="输入 Bag 包名称 (无需扩展名):").pack(anchor="w")
        self.entry_bag_name = tk.Entry(frame_record, font=("Arial", 11))
        self.entry_bag_name.insert(0, "v11_test_bag")
        self.entry_bag_name.pack(fill="x", pady=5)

        self.btn_record = tk.Button(frame_record, text="🔴 开始录制", 
                                    bg="#ffb3b3", font=("Arial", 11, "bold"), height=2, command=self.toggle_record)
        self.btn_record.pack(fill="x", pady=5)

        self.lbl_record_status = tk.Label(frame_record, text="录制状态: 未录制", fg="gray")
        self.lbl_record_status.pack()

        # 3. 退出按钮
        tk.Button(self.root, text="安全退出程序", bg="#f0f0f0", command=self.on_closing).pack(pady=10)

    def toggle_gui(self):
        if self.gui_process is None:
            self.btn_gui.config(text="⏹ 停止 V11 传感器 GUI", bg="#ff9999")
            self.lbl_gui_status.config(text="GUI 状态: 运行中...", fg="green")
            
            gui_script = "ros2_multimodal_gui.py"
            # 必须 source 环境，且必须在 v11 目录下运行，否则找不到同级的其他 python 模块
            cmd = f"source /media/libo/libo_sn7100/ubuntu2204/shandong_ws/install/setup.bash && cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v11_multimodal_dataset_collection && python3 {gui_script}"
            self.gui_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
            print(f"[INFO] 已启动 V11 GUI: {gui_script}")
        else:
            self._stop_gui()
            
    def _stop_gui(self):
        if self.gui_process:
            print("[INFO] 正在停止 V11 GUI...")
            try:
                os.killpg(os.getpgid(self.gui_process.pid), signal.SIGINT)
                time.sleep(0.5)
                os.killpg(os.getpgid(self.gui_process.pid), signal.SIGTERM)
                self.gui_process.wait(timeout=3)
            except Exception as e:
                print(f"[ERROR] 停止 GUI 异常: {e}")
            finally:
                self.gui_process = None
                self.btn_gui.config(text="▶ 启动 V11 传感器 GUI", bg="#a2d5f2")
                self.lbl_gui_status.config(text="GUI 状态: 已停止", fg="red")

    def toggle_record(self):
        if self.record_process is None:
            bag_name = self.entry_bag_name.get().strip()
            if not bag_name:
                messagebox.showwarning("警告", "请输入有效的 Bag 包名称！")
                return

            self.btn_record.config(text="⏹ 停止录制", bg="#99ff99")
            self.lbl_record_status.config(text=f"录制状态: 正在录制 ({bag_name})", fg="red")
            
            # 仅录制指定的 5 个 V11 话题
            topics = [
                "/camera_hik/image_raw",
                "/lidar/points",
                "/lidar/points_odom",
                "/excavator/joint_states",
                "/lidar/elevation_map",
            ]
            
            save_dir = "/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/bag"
            os.makedirs(save_dir, exist_ok=True)
            bag_path = os.path.join(save_dir, bag_name)
            
            topics_str = " ".join(topics)
            cmd = f"source /media/libo/libo_sn7100/ubuntu2204/shandong_ws/install/setup.bash && ros2 bag record -o {bag_path} {topics_str}"
            
            self.record_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
            print(f"[INFO] 已开始录制 rosbag: {bag_path}")
        else:
            self._stop_record()

    def _stop_record(self):
        if self.record_process:
            print("[INFO] 正在停止 rosbag 录制...")
            try:
                os.killpg(os.getpgid(self.record_process.pid), signal.SIGINT)
                time.sleep(0.5)
                os.killpg(os.getpgid(self.record_process.pid), signal.SIGTERM)
                self.record_process.wait(timeout=3)
            except Exception as e:
                print(f"[ERROR] 停止录制异常: {e}")
            finally:
                self.record_process = None
                self.btn_record.config(text="🔴 开始录制", bg="#ffb3b3")
                self.lbl_record_status.config(text="录制状态: 已保存", fg="green")

    def on_closing(self):
        self._stop_record()
        self._stop_gui()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = V11LaunchGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()