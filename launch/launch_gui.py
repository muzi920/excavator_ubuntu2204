import os
import subprocess
import threading
import signal
import time
import tkinter as tk
from tkinter import messagebox, ttk

class Ros2LaunchGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS 2 多传感器启动与录制工具")
        self.root.geometry("500x450")
        self.root.resizable(False, False)

        # 状态变量
        self.launch_process = None
        self.record_process = None

        self._setup_ui()

    def _setup_ui(self):
        # 1. Launch 启动区域
        frame_launch = tk.LabelFrame(self.root, text="ROS 2 Launch 节点管理", padx=10, pady=10)
        frame_launch.pack(fill="x", padx=10, pady=10)

        self.btn_launch = tk.Button(frame_launch, text="▶ 启动所有传感器 (all_sensors.launch.py)", 
                                    bg="#a2d5f2", font=("Arial", 11, "bold"), height=2, command=self.toggle_launch)
        self.btn_launch.pack(fill="x", pady=5)

        self.lbl_launch_status = tk.Label(frame_launch, text="Launch 状态: 未启动", fg="gray")
        self.lbl_launch_status.pack()

        # 2. Rosbag 录制区域
        frame_record = tk.LabelFrame(self.root, text="Rosbag 数据录制", padx=10, pady=10)
        frame_record.pack(fill="x", padx=10, pady=10)

        tk.Label(frame_record, text="输入 Bag 包名称 (无需扩展名):").pack(anchor="w")
        self.entry_bag_name = tk.Entry(frame_record, font=("Arial", 11))
        self.entry_bag_name.insert(0, "lc1")
        self.entry_bag_name.pack(fill="x", pady=5)

        self.btn_record = tk.Button(frame_record, text="🔴 开始录制 Rosbag", 
                                    bg="#ffb3b3", font=("Arial", 11, "bold"), height=2, command=self.toggle_record)
        self.btn_record.pack(fill="x", pady=5)

        self.lbl_record_status = tk.Label(frame_record, text="录制状态: 未录制", fg="gray")
        self.lbl_record_status.pack()

        # 3. 退出按钮
        tk.Button(self.root, text="安全退出程序", bg="#f0f0f0", command=self.on_closing).pack(pady=15)

    def toggle_launch(self):
        if self.launch_process is None:
            # 启动
            self.btn_launch.config(text="⏹ 停止传感器节点", bg="#ff9999")
            self.lbl_launch_status.config(text="Launch 状态: 运行中...", fg="green")
            
            # 由于通过 shell 运行，且我们使用 Python 的包路径，我们不需要强行使用 ros2 launch shandong
            # 可以直接调用 python 的 launch 工具或者用绝对路径
            launch_file = "/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/launch/all_sensors.launch.py"
            cmd = f"source /media/libo/libo_sn7100/ubuntu2204/shandong_ws/install/setup.bash && ros2 launch {launch_file}"
            self.launch_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
            print(f"[INFO] 已启动 ros2 launch: {launch_file}")
        else:
            # 停止
            self._stop_launch()
            
    def _stop_launch(self):
        if self.launch_process:
            print("[INFO] 正在停止 ros2 launch...")
            try:
                # 必须连续发送两次 SIGINT 或者直接发送 SIGTERM，才能打破 ros2 launch 的 respawn/清理机制
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGINT)
                time.sleep(0.5)
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                self.launch_process.wait(timeout=3)
            except Exception as e:
                print(f"[ERROR] 停止 Launch 异常: {e}")
            finally:
                self.launch_process = None
                self.btn_launch.config(text="▶ 启动所有传感器 (all_sensors.launch.py)", bg="#a2d5f2")
                self.lbl_launch_status.config(text="Launch 状态: 已停止", fg="red")

    def toggle_record(self):
        if self.record_process is None:
            bag_name = self.entry_bag_name.get().strip()
            if not bag_name:
                messagebox.showwarning("警告", "请输入有效的 Bag 包名称！")
                return

            self.btn_record.config(text="⏹ 停止录制", bg="#99ff99")
            self.lbl_record_status.config(text=f"录制状态: 正在录制 ({bag_name})", fg="red")
            
            topics = [
                "/hikvision_cam/image_raw",
                "/imu",
                "/imu/arm_acc_x", "/imu/arm_acc_y", "/imu/arm_ang_x", "/imu/arm_ang_y",
                "/imu/boom_acc_x", "/imu/boom_acc_y", "/imu/boom_ang_x", "/imu/boom_ang_y",
                "/imu/bucket_acc_x", "/imu/bucket_acc_y", "/imu/bucket_ang_x", "/imu/bucket_ang_y",
                "/imu/relative_ang_x",
                "/imu/swing_acc_x", "/imu/swing_acc_y", "/imu/swing_ang_x", "/imu/swing_ang_y", "/imu/swing_angle",
                "/network_cam/image_raw",
                "/network_cam2/image_raw",
                "/pointcloud",
                "/pointcloud_base_link",
                # "/camera_hik/image_raw",
                # "/camera1/image_raw",
                # "/camera2/image_raw",
                # "/lidar/points",
                # "/excavator/joint_states",
                "/tf",
                "/tf_static"
            ]
            
            # 将录制的包保存在 src/bag/ 目录下
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
                self.record_process.wait(timeout=3)
            except Exception as e:
                print(f"[ERROR] 停止录制异常: {e}")
            finally:
                self.record_process = None
                self.btn_record.config(text="🔴 开始录制 Rosbag", bg="#ffb3b3")
                self.lbl_record_status.config(text="录制状态: 已保存", fg="green")

    def on_closing(self):
        self._stop_record()
        self._stop_launch()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = Ros2LaunchGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()