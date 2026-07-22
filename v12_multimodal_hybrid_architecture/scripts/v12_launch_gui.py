#!/usr/bin/env python3
import os
import signal
import subprocess
import time
import tkinter as tk
from tkinter import messagebox, ttk


WORKSPACE = "/media/libo/libo_sn7100/ubuntu2204/shandong_ws"
SETUP_CMD = (
    f"mkdir -p /tmp/roslogs && export ROS_LOG_DIR=/tmp/roslogs && "
    f"source /opt/ros/humble/setup.bash && source {WORKSPACE}/install/setup.bash"
)
LAUNCH_CMD = f"{SETUP_CMD} && ros2 launch v12_multimodal_hybrid_architecture v12_launch.py"
DEFAULT_BAG_DIR = os.path.join(WORKSPACE, "src", "bag")
DEFAULT_TOPICS = [
    "/cam_hik/image_raw",
    "/cam1/image_raw",
    "/cam2/image_raw",
    "/imu",
    "/pointcloud",
    "/lidar/points",
    "/lidar/points_odom",
    "/lidar/elevation_map",
    "/excavator/inclinometer_pitch_deg",
    "/excavator/inclinometer_relative_deg",
    "/excavator/joint_states",
    "/excavator/joint_angles_deg",
    "/excavator/target_joint_angles_deg",
    "/tf",
    "/tf_static",
]


class V12LaunchGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("V12 Launch And Recording Console")
        self.root.geometry("560x430")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.launch_process = None
        self.record_process = None

        self._setup_ui()

    def _setup_ui(self):
        frame_launch = ttk.LabelFrame(self.root, text="V12 System Launch", padding=10)
        frame_launch.pack(fill="x", padx=10, pady=10)

        self.btn_launch = tk.Button(
            frame_launch,
            text="Start V12 Launch",
            bg="#a2d5f2",
            font=("Arial", 11, "bold"),
            height=2,
            command=self.toggle_launch,
        )
        self.btn_launch.pack(fill="x", pady=5)
        self.lbl_launch_status = tk.Label(frame_launch, text="Launch status: stopped", fg="gray")
        self.lbl_launch_status.pack(anchor="w")

        frame_record = ttk.LabelFrame(self.root, text="Rosbag Recording", padding=10)
        frame_record.pack(fill="x", padx=10, pady=10)

        tk.Label(frame_record, text="Bag name:").pack(anchor="w")
        self.entry_bag_name = tk.Entry(frame_record, font=("Arial", 11))
        self.entry_bag_name.insert(0, "v12_session")
        self.entry_bag_name.pack(fill="x", pady=5)

        tk.Label(frame_record, text="Bag directory:").pack(anchor="w")
        self.entry_bag_dir = tk.Entry(frame_record, font=("Arial", 10))
        self.entry_bag_dir.insert(0, DEFAULT_BAG_DIR)
        self.entry_bag_dir.pack(fill="x", pady=5)

        tk.Label(frame_record, text="Topics to record:").pack(anchor="w")
        self.txt_topics = tk.Text(frame_record, height=10, width=60)
        self.txt_topics.insert("1.0", "\n".join(DEFAULT_TOPICS))
        self.txt_topics.pack(fill="x", pady=5)

        self.btn_record = tk.Button(
            frame_record,
            text="Start Recording",
            bg="#ffb3b3",
            font=("Arial", 11, "bold"),
            height=2,
            command=self.toggle_record,
        )
        self.btn_record.pack(fill="x", pady=5)
        self.lbl_record_status = tk.Label(frame_record, text="Record status: idle", fg="gray")
        self.lbl_record_status.pack(anchor="w")

        ttk.Button(self.root, text="Safe Exit", command=self.on_closing).pack(pady=10)

    def _spawn_process(self, command):
        return subprocess.Popen(
            command,
            shell=True,
            executable="/bin/bash",
            cwd=WORKSPACE,
            preexec_fn=os.setsid,
        )

    def toggle_launch(self):
        if self.launch_process is None:
            self.launch_process = self._spawn_process(LAUNCH_CMD)
            self.btn_launch.config(text="Stop V12 Launch", bg="#ff9999")
            self.lbl_launch_status.config(text="Launch status: running", fg="green")
        else:
            self._stop_process(self.launch_process)
            self.launch_process = None
            self.btn_launch.config(text="Start V12 Launch", bg="#a2d5f2")
            self.lbl_launch_status.config(text="Launch status: stopped", fg="red")

    def toggle_record(self):
        if self.record_process is None:
            bag_name = self.entry_bag_name.get().strip()
            bag_dir = self.entry_bag_dir.get().strip()
            topics = [line.strip() for line in self.txt_topics.get("1.0", "end").splitlines() if line.strip()]
            if not bag_name:
                messagebox.showwarning("Warning", "Bag name cannot be empty.")
                return
            if not topics:
                messagebox.showwarning("Warning", "Please provide at least one topic.")
                return

            os.makedirs(bag_dir, exist_ok=True)
            bag_path = os.path.join(bag_dir, bag_name)
            topics_str = " ".join(topics)
            record_cmd = f"{SETUP_CMD} && ros2 bag record -o {bag_path} {topics_str}"
            self.record_process = self._spawn_process(record_cmd)
            self.btn_record.config(text="Stop Recording", bg="#99ff99")
            self.lbl_record_status.config(text=f"Record status: recording -> {bag_path}", fg="red")
        else:
            self._stop_process(self.record_process)
            self.record_process = None
            self.btn_record.config(text="Start Recording", bg="#ffb3b3")
            self.lbl_record_status.config(text="Record status: saved", fg="green")

    def _stop_process(self, process):
        if process is None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            time.sleep(0.5)
            if process.poll() is None:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3)
        except Exception:
            pass

    def on_closing(self):
        if self.record_process is not None:
            self._stop_process(self.record_process)
            self.record_process = None
        if self.launch_process is not None:
            self._stop_process(self.launch_process)
            self.launch_process = None
        self.root.destroy()


def main():
    root = tk.Tk()
    V12LaunchGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
