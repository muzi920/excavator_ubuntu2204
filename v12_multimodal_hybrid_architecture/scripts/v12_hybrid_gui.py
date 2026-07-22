#!/usr/bin/env python3
import os
import queue
import threading
import time
import tkinter as tk
import json
from tkinter import filedialog, messagebox, ttk

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import Float64MultiArray, Int32, String


os.environ.setdefault("ROS_LOG_DIR", "/tmp/roslogs")
os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)


def generate_elevation_map(
    points,
    x_range=(-3.0, 3.0),
    y_range=(-3.0, 3.0),
    resolution=0.03,
    z_range=(-0.4, 0.7),
):
    width = int((x_range[1] - x_range[0]) / resolution)
    height = int((y_range[1] - y_range[0]) / resolution)
    flat_map = np.full(width * height, z_range[0], dtype=np.float32)

    if points is not None and len(points) > 0:
        u = np.floor((points[:, 1] - y_range[0]) / resolution).astype(int)
        v = np.floor((x_range[1] - points[:, 0]) / resolution).astype(int) - 1
        z = points[:, 2]

        valid_idx = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u = u[valid_idx]
        v = v[valid_idx]
        z = z[valid_idx]

        flat_indices = v * width + u
        np.maximum.at(flat_map, flat_indices, z)

    elevation_map = flat_map.reshape((height, width))
    z_min, z_max = z_range
    elevation_map = np.clip(elevation_map, z_min, z_max)
    elevation_img = ((elevation_map - z_min) / (z_max - z_min) * 255.0).astype(np.uint8)
    return cv2.cvtColor(elevation_img, cv2.COLOR_GRAY2BGR)


def pointcloud2_to_xyz(msg):
    field_offsets = {field.name: field.offset for field in msg.fields}
    required = ("x", "y", "z")
    if not all(name in field_offsets for name in required):
        raise ValueError("PointCloud2 missing x/y/z fields")

    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [f"{endian}f4", f"{endian}f4", f"{endian}f4"],
            "offsets": [field_offsets["x"], field_offsets["y"], field_offsets["z"]],
            "itemsize": msg.point_step,
        }
    )
    count = msg.width * msg.height
    cloud = np.frombuffer(msg.data, dtype=dtype, count=count)
    xyz = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def format_age(now_ts, sample_ts):
    if sample_ts <= 0.0:
        return "waiting"
    dt = now_ts - sample_ts
    if dt < 0.5:
        return f"ok ({dt:.1f}s)"
    if dt < 2.0:
        return f"slow ({dt:.1f}s)"
    return f"stale ({dt:.1f}s)"


class V12HybridNode(Node):
    def __init__(self):
        super().__init__("v12_hybrid_gui")
        self.bridge = CvBridge()
        self.state_lock = threading.Lock()
        self.pc_queue = queue.Queue(maxsize=2)
        self.worker_running = True

        self.topic_ts = {
            "cam_hik": 0.0,
            "cam1": 0.0,
            "cam2": 0.0,
            "pointcloud": 0.0,
            "points_odom": 0.0,
            "joint_states": 0.0,
            "joint_angles_deg": 0.0,
            "elevation_map": 0.0,
        }
        self.last_cam_shapes = {"cam_hik": None, "cam1": None, "cam2": None}
        self.last_point_count = 0
        self.last_elevation_shape = None
        self.last_joint_rad = [0.0, 0.0, 0.0, 0.0]
        self.last_joint_deg = [0.0, 0.0, 0.0, 0.0]
        self.last_target_deg = [0.0, 0.0, 0.0, 0.0]
        self.last_command_text = "Last command: none"
        self.first_msg_seen = set()

        self.create_subscription(Image, "/cam_hik/image_raw", self.make_camera_callback("cam_hik"), 10)
        self.create_subscription(Image, "/cam1/image_raw", self.make_camera_callback("cam1"), 10)
        self.create_subscription(Image, "/cam2/image_raw", self.make_camera_callback("cam2"), 10)
        self.create_subscription(PointCloud2, "/pointcloud", self.raw_pc_callback, 10)
        self.create_subscription(PointCloud2, "/lidar/points_odom", self.pc_callback, 10)
        self.create_subscription(JointState, "/excavator/joint_states", self.joint_callback, 10)
        self.create_subscription(
            Float64MultiArray,
            "/excavator/joint_angles_deg",
            self.joint_angles_deg_callback,
            10,
        )

        self.pub_elevation = self.create_publisher(Image, "/lidar/elevation_map", 10)
        self.pub_target_joint_deg = self.create_publisher(
            Float64MultiArray,
            "/excavator/target_joint_angles_deg",
            10,
        )
        self.pub_target_ch3 = self.create_publisher(
            Int32,
            "/excavator/target_ch3_mv",
            10,
        )
        self.pub_joint_command = self.create_publisher(
            String,
            "/excavator/joint_command_json",
            10,
        )

        self.worker = threading.Thread(target=self.elevation_worker, daemon=True)
        self.worker.start()
        self.get_logger().info("V12 Python GUI & Elevation Map Node Started (Subscriber-only mode)")

    def make_camera_callback(self, camera_name):
        def _callback(msg):
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                shape = image.shape[:2]
            except Exception as exc:
                self.get_logger().warning(f"{camera_name} image decode failed: {exc}")
                return

            with self.state_lock:
                self.topic_ts[camera_name] = time.time()
                self.last_cam_shapes[camera_name] = shape
            self.mark_first_message(camera_name)

        return _callback

    def pc_callback(self, msg):
        try:
            points = pointcloud2_to_xyz(msg)
        except Exception as exc:
            self.get_logger().warning(f"PointCloud2 parse failed: {exc}")
            return

        with self.state_lock:
            self.topic_ts["points_odom"] = time.time()
            self.last_point_count = int(points.shape[0])
        self.mark_first_message("points_odom")

        if self.pc_queue.full():
            try:
                self.pc_queue.get_nowait()
            except queue.Empty:
                pass
        self.pc_queue.put_nowait(points)

    def raw_pc_callback(self, _msg):
        with self.state_lock:
            self.topic_ts["pointcloud"] = time.time()
        self.mark_first_message("pointcloud")

    def joint_callback(self, msg):
        positions = list(msg.position[:4])
        while len(positions) < 4:
            positions.append(0.0)
        with self.state_lock:
            self.topic_ts["joint_states"] = time.time()
            self.last_joint_rad = positions
        self.mark_first_message("joint_states")

    def joint_angles_deg_callback(self, msg):
        positions = list(msg.data[:4])
        while len(positions) < 4:
            positions.append(0.0)
        with self.state_lock:
            self.topic_ts["joint_angles_deg"] = time.time()
            self.last_joint_deg = positions
        self.mark_first_message("joint_angles_deg")

    def elevation_worker(self):
        while self.worker_running:
            try:
                points = self.pc_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                elevation_img = generate_elevation_map(points)
                msg = self.bridge.cv2_to_imgmsg(elevation_img, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "odom"
                self.pub_elevation.publish(msg)
                with self.state_lock:
                    self.topic_ts["elevation_map"] = time.time()
                    self.last_elevation_shape = elevation_img.shape[:2]
                self.mark_first_message("elevation_map")
            except Exception as exc:
                self.get_logger().warning(f"Elevation map generation failed: {exc}")

    def publish_target_joint_angles_deg(self, target_deg, ch3_mv=None):
        msg = Float64MultiArray()
        msg.data = list(target_deg)
        self.pub_target_joint_deg.publish(msg)
        if ch3_mv is not None:
            ch3_msg = Int32()
            ch3_msg.data = int(ch3_mv)
            self.pub_target_ch3.publish(ch3_msg)
        with self.state_lock:
            self.last_target_deg = list(target_deg)
        if ch3_mv is None:
            self.get_logger().info(f"Published target joint angles (deg): {target_deg}")
        else:
            self.get_logger().info(f"Published target joint angles (deg): {target_deg}, CH3={int(ch3_mv)} mV")

    def publish_joint_command(self, joint_name, target_deg, ch3_mv=2000, ramp_up_s=0.2, ramp_down_s=0.2, is_init_step=False, description=""):
        command = {
            "joint": joint_name,
            "target_val": float(target_deg),
            "ch1_mv": 0,
            "ch2_mv": 0,
            "ch3_mv": int(ch3_mv),
            "ramp_up_s": float(ramp_up_s),
            "ramp_down_s": float(ramp_down_s),
            "is_init_step": bool(is_init_step),
            "description": description,
        }
        msg = String()
        msg.data = json.dumps(command, ensure_ascii=False)
        self.pub_joint_command.publish(msg)
        with self.state_lock:
            self.last_command_text = f"{joint_name} -> {float(target_deg):.2f} deg | CH3={int(ch3_mv)} | up={float(ramp_up_s):.2f}s | down={float(ramp_down_s):.2f}s"
        self.get_logger().info(f"Published joint command: {msg.data}")

    def publish_stop_command(self):
        msg = String()
        msg.data = json.dumps({"command": "stop_all"}, ensure_ascii=False)
        self.pub_joint_command.publish(msg)
        with self.state_lock:
            self.last_command_text = "stop_all"
        self.get_logger().info("Published stop_all command")

    def mark_first_message(self, topic_name):
        if topic_name not in self.first_msg_seen:
            self.first_msg_seen.add(topic_name)
            self.get_logger().info(f"Received first message on {topic_name}")

    def get_snapshot(self):
        with self.state_lock:
            return {
                "topic_ts": dict(self.topic_ts),
                "cam_shapes": dict(self.last_cam_shapes),
                "point_count": self.last_point_count,
                "elevation_shape": self.last_elevation_shape,
                "joint_rad": list(self.last_joint_rad),
                "joint_deg": list(self.last_joint_deg),
                "target_deg": list(self.last_target_deg),
                "last_command_text": self.last_command_text,
            }

    def destroy_node(self):
        self.worker_running = False
        if hasattr(self, "worker") and self.worker.is_alive():
            self.worker.join(timeout=0.5)
        super().destroy_node()


class V12HybridConsole:
    JOINT_LABELS = ["boom_joint", "arm_joint", "bucket_joint", "swing_joint"]
    JOINT_DISPLAY_NAMES = {
        "boom_joint": "大臂",
        "arm_joint": "小臂",
        "bucket_joint": "铲斗",
        "swing_joint": "回转",
    }

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("V12 Python Control Console")
        self.root.geometry("1120x760")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.status_vars = {}
        self.joint_vars = {}
        self.target_vars = [tk.DoubleVar(value=0.0) for _ in range(4)]
        self.ch3_var = tk.IntVar(value=2000)
        self.ramp_up_var = tk.DoubleVar(value=0.2)
        self.ramp_down_var = tk.DoubleVar(value=0.2)
        self.last_target_var = tk.StringVar(value="Last command: none")
        self.point_count_var = tk.StringVar(value="points_odom: waiting")
        self.exec_status_var = tk.StringVar(value="当前状态: 未执行")
        self.ros_spin_ok = True
        self.is_recording = False
        self.script_running = False
        self.recorded_script = []
        self.json_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "json"))
        os.makedirs(self.json_dir, exist_ok=True)
        self.btn_record = None
        self.btn_load_script = None

        self.build_ui()
        self.pump_ros()
        self.refresh_loop()

    def build_ui(self):
        frame_topics = ttk.LabelFrame(self.root, text="Topic Status", padding=10)
        frame_topics.pack(fill="x", padx=10, pady=10)

        topic_columns = [
            ["cam_hik", "cam1", "cam2"],
            ["pointcloud", "points_odom", "elevation_map"],
            ["joint_states", "joint_angles_deg"],
        ]
        columns_frame = ttk.Frame(frame_topics)
        columns_frame.pack(fill="x")
        for col_topics in topic_columns:
            col_frame = ttk.Frame(columns_frame)
            col_frame.pack(side="left", fill="both", expand=True, padx=8)
            for topic_name in col_topics:
                var = tk.StringVar(value=f"{topic_name}: waiting")
                self.status_vars[topic_name] = var
                ttk.Label(col_frame, textvariable=var).pack(anchor="w", pady=2)

        frame_elev = ttk.LabelFrame(self.root, text="Elevation Map", padding=10)
        frame_elev.pack(fill="x", padx=10, pady=10)
        ttk.Label(frame_elev, textvariable=self.point_count_var).pack(anchor="w")

        frame_joint = ttk.LabelFrame(self.root, text="关节角度", padding=10)
        frame_joint.pack(fill="x", padx=10, pady=10)
        for index, joint_name in enumerate(self.JOINT_LABELS):
            joint_var = tk.StringVar(value=f"{self.JOINT_DISPLAY_NAMES[joint_name]}: 0.000 度 | 0.000 弧度")
            self.joint_vars[joint_name] = joint_var
            ttk.Label(frame_joint, textvariable=joint_var).grid(
                row=index // 2,
                column=index % 2,
                padx=20,
                pady=6,
                sticky="w",
            )

        frame_target = ttk.LabelFrame(self.root, text="闭环角度目标控制", padding=10)
        frame_target.pack(fill="x", padx=10, pady=10)
        analog_row = ttk.Frame(frame_target)
        analog_row.pack(fill="x", pady=4)
        ttk.Label(analog_row, text="CH3(液压 mV):", width=18).pack(side="left")
        ttk.Entry(analog_row, textvariable=self.ch3_var, width=12).pack(side="left", padx=6)
        ttk.Label(analog_row, text="柔性启动(s):", width=12).pack(side="left", padx=(20, 0))
        ttk.Entry(analog_row, textvariable=self.ramp_up_var, width=8).pack(side="left", padx=6)
        ttk.Label(analog_row, text="柔性停止(s):", width=12).pack(side="left", padx=(20, 0))
        ttk.Entry(analog_row, textvariable=self.ramp_down_var, width=8).pack(side="left", padx=6)

        for index, joint_name in enumerate(self.JOINT_LABELS):
            row = ttk.Frame(frame_target)
            row.pack(fill="x", pady=2)
            display_name = self.JOINT_DISPLAY_NAMES[joint_name]
            ttk.Label(row, text=f"{display_name}目标角度(°):", width=18).pack(side="left")
            ttk.Entry(row, textvariable=self.target_vars[index], width=12).pack(side="left", padx=6)
            ttk.Button(
                row,
                text=f"开始移动{display_name}",
                command=lambda i=index, j=joint_name: self.handle_move(i, j),
                width=16,
            ).pack(side="left", padx=6)
            ttk.Button(
                row,
                text="记录当前角度",
                command=lambda i=index, j=joint_name: self.record_current_angle(i, j, is_init=False),
                width=14,
            ).pack(side="left", padx=6)
            ttk.Button(
                row,
                text="记录为初始位置",
                command=lambda i=index, j=joint_name: self.record_current_angle(i, j, is_init=True),
                width=16,
            ).pack(side="left", padx=6)

        frame_buttons = ttk.Frame(self.root)
        frame_buttons.pack(fill="x", padx=10, pady=10)
        self.btn_record = tk.Button(
            frame_buttons,
            text="开始录制剧本",
            command=self.toggle_recording,
            bg="#ffcccc",
            width=16,
        )
        self.btn_record.pack(side="left", padx=8)
        ttk.Button(frame_buttons, text="保存为 JSON 剧本", command=self.save_script, width=18).pack(side="left", padx=8)
        self.btn_load_script = tk.Button(
            frame_buttons,
            text="选择并执行 JSON 剧本",
            command=self.load_and_run_script,
            bg="#ccccff",
            width=22,
        )
        self.btn_load_script.pack(side="left", padx=8)
        ttk.Label(frame_buttons, textvariable=self.exec_status_var).pack(side="left", padx=10)

        frame_bottom = ttk.Frame(self.root)
        frame_bottom.pack(fill="x", padx=10, pady=5)
        ttk.Button(frame_bottom, text="使用当前角度填充目标", command=self.load_current_as_target).pack(side="left", padx=5)
        ttk.Button(frame_bottom, text="全部目标清零", command=self.zero_target).pack(side="left", padx=5)
        ttk.Button(frame_bottom, text="急停/停止剧本", command=self.emergency_stop).pack(side="left", padx=5)
        ttk.Label(frame_bottom, textvariable=self.last_target_var).pack(side="left", padx=10)

    def get_target_values(self):
        try:
            return [var.get() for var in self.target_vars]
        except tk.TclError:
            messagebox.showerror("输入错误", "目标关节角度必须是数字。")
            return None

    def get_ch3_value(self):
        try:
            return int(self.ch3_var.get())
        except tk.TclError:
            messagebox.showerror("输入错误", "CH3(液压) 必须是整数。")
            return None

    def get_ramp_values(self):
        try:
            return float(self.ramp_up_var.get()), float(self.ramp_down_var.get())
        except tk.TclError:
            messagebox.showerror("输入错误", "柔性启动/停止时间必须是数字。")
            return None, None

    def publish_single_joint_command(self, joint_name, target_deg, ch3_mv, ramp_up_s, ramp_down_s, is_init_step=False, description=""):
        self.node.publish_joint_command(
            joint_name,
            target_deg,
            ch3_mv=ch3_mv,
            ramp_up_s=ramp_up_s,
            ramp_down_s=ramp_down_s,
            is_init_step=is_init_step,
            description=description,
        )
        self.last_target_var.set(
            f"Last command: {self.JOINT_DISPLAY_NAMES.get(joint_name, joint_name)} -> {float(target_deg):.2f}° | CH3={int(ch3_mv)}"
        )

    def handle_move(self, joint_index, joint_name):
        target = self.get_target_values()
        if target is None:
            return
        ch3_mv = self.get_ch3_value()
        if ch3_mv is None:
            return
        ramp_up_s, ramp_down_s = self.get_ramp_values()
        if ramp_up_s is None:
            return
        target_deg = target[joint_index]
        self.publish_single_joint_command(
            joint_name,
            target_deg,
            ch3_mv,
            ramp_up_s,
            ramp_down_s,
            is_init_step=False,
            description=f"开始移动{self.JOINT_DISPLAY_NAMES[joint_name]}",
        )
        if self.is_recording:
            self.recorded_script.append(
                {
                    "step": len(self.recorded_script) + 1,
                    "joint": joint_name,
                    "description": f"开始移动{self.JOINT_DISPLAY_NAMES[joint_name]}",
                    "target_val": target_deg,
                    "ch3_mv": ch3_mv,
                    "ramp_up_s": ramp_up_s,
                    "ramp_down_s": ramp_down_s,
                }
            )
        self.exec_status_var.set(
            f"当前状态: 已下发 {self.JOINT_DISPLAY_NAMES[joint_name]} 目标 {target_deg:.2f}° | CH3={ch3_mv}"
        )

    def load_current_as_target(self):
        snapshot = self.node.get_snapshot()
        for index, value in enumerate(snapshot["joint_deg"]):
            self.target_vars[index].set(round(value, 3))

    def zero_target(self):
        for var in self.target_vars:
            var.set(0.0)

    def toggle_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.recorded_script = []
            self.btn_record.config(text="停止录制剧本", bg="#ccffcc")
            messagebox.showinfo("开始录制", "已开始录制剧本。现在每次“开始移动”或“记录当前角度”都会写入剧本。")
        else:
            self.is_recording = False
            self.btn_record.config(text="开始录制剧本", bg="#ffcccc")
            messagebox.showinfo("停止录制", f"录制已停止，当前共记录 {len(self.recorded_script)} 个动作。")

    def record_current_angle(self, joint_index, joint_name, is_init=False):
        if not self.is_recording:
            messagebox.showwarning("提示", "请先点击“开始录制剧本”。")
            return
        snapshot = self.node.get_snapshot()
        current_deg = round(snapshot["joint_deg"][joint_index], 3)
        self.target_vars[joint_index].set(current_deg)
        current_target = self.get_target_values()
        if current_target is None:
            return
        ch3_mv = self.get_ch3_value()
        if ch3_mv is None:
            return
        ramp_up_s, ramp_down_s = self.get_ramp_values()
        if ramp_up_s is None:
            return
        desc = f"{self.JOINT_DISPLAY_NAMES[joint_name]}手动示教"
        if is_init:
            desc += " - 初始位置"
        self.recorded_script.append(
            {
                "step": len(self.recorded_script) + 1,
                "joint": joint_name,
                "description": desc,
                "target_val": current_deg,
                "ch3_mv": ch3_mv,
                "ramp_up_s": ramp_up_s,
                "ramp_down_s": ramp_down_s,
                "is_init_step": is_init,
            }
        )
        self.exec_status_var.set(f"当前状态: 已记录 {desc} = {current_deg:.2f}° | CH3={ch3_mv}")

    def save_script(self):
        if self.is_recording:
            messagebox.showwarning("警告", "请先停止录制，再进行保存。")
            return
        if not self.recorded_script:
            messagebox.showwarning("提示", "当前没有录制任何动作。")
            return
        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            initialdir=self.json_dir,
            title="保存闭环剧本",
            filetypes=[("JSON files", "*.json")],
        )
        if not file_path:
            return
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(self.recorded_script, f, ensure_ascii=False, indent=2)
        messagebox.showinfo("保存成功", f"成功保存 {len(self.recorded_script)} 步动作到:\n{file_path}")

    def load_and_run_script(self):
        if self.script_running:
            messagebox.showwarning("警告", "当前已有剧本正在执行，请先急停。")
            return
        file_path = filedialog.askopenfilename(
            initialdir=self.json_dir,
            title="选择要执行的 JSON 剧本",
            filetypes=[("JSON files", "*.json")],
        )
        if not file_path:
            return
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                script_data = json.load(f)
        except Exception as exc:
            messagebox.showerror("读取失败", f"无法解析 JSON 剧本:\n{exc}")
            return

        self.script_running = True
        self.btn_load_script.config(state="disabled")
        self.exec_status_var.set(f"当前状态: 正在执行 {os.path.basename(file_path)}")
        threading.Thread(
            target=self.execute_script_thread,
            args=(script_data, os.path.basename(file_path)),
            daemon=True,
        ).start()

    def execute_script_thread(self, script_data, filename):
        try:
            for idx, step in enumerate(script_data):
                if not self.script_running:
                    break

                joint = step.get("joint", "")
                target_deg = float(step.get("target_val", step.get("target_deg", 0.0)))
                ch3_mv = int(step.get("ch3_mv", 2000))
                ramp_up_s = float(step.get("ramp_up_s", 0.2))
                ramp_down_s = float(step.get("ramp_down_s", 0.2))
                timeout_s = float(step.get("timeout_s", 20.0))
                tolerance_deg = float(step.get("tolerance_deg", 2.0))
                hold_time_s = float(step.get("hold_time_s", 0.3))
                is_init_step = bool(step.get("is_init_step", False))
                description = step.get("description", f"Step {idx + 1}")

                self.root.after(
                    0,
                    lambda d=description, i=idx + 1: self.exec_status_var.set(
                        f"当前状态: 执行第{i}步 {d} | CH3={ch3_mv}"
                    ),
                )
                self.root.after(
                    0,
                    lambda j=joint, t=target_deg, c=ch3_mv, up=ramp_up_s, down=ramp_down_s, init=is_init_step, d=description:
                        self.apply_single_step(j, t, c, up, down, init, d),
                )

                joint_index = self.JOINT_LABELS.index(joint) if joint in self.JOINT_LABELS else None
                start_time = time.time()
                while self.script_running:
                    if joint_index is None:
                        break
                    current_deg = self.node.get_snapshot()["joint_deg"][joint_index]
                    if abs(current_deg - target_deg) <= tolerance_deg:
                        time.sleep(hold_time_s)
                        break
                    if time.time() - start_time > timeout_s:
                        self.root.after(
                            0,
                            lambda d=description: self.exec_status_var.set(
                                f"当前状态: 超时跳过 {d}"
                            ),
                        )
                        break
                    time.sleep(0.1)
        finally:
            self.script_running = False
            self.root.after(0, lambda: self.btn_load_script.config(state="normal"))
            self.root.after(0, lambda: self.exec_status_var.set(f"当前状态: {filename} 执行完毕/已停止"))

    def apply_single_step(self, joint_name, target_deg, ch3_mv, ramp_up_s, ramp_down_s, is_init_step=False, description=""):
        if joint_name in self.JOINT_LABELS:
            self.target_vars[self.JOINT_LABELS.index(joint_name)].set(round(float(target_deg), 3))
        self.ch3_var.set(int(ch3_mv))
        self.ramp_up_var.set(round(float(ramp_up_s), 3))
        self.ramp_down_var.set(round(float(ramp_down_s), 3))
        self.publish_single_joint_command(
            joint_name,
            target_deg,
            ch3_mv,
            ramp_up_s,
            ramp_down_s,
            is_init_step=is_init_step,
            description=description,
        )

    def emergency_stop(self):
        self.script_running = False
        self.node.publish_stop_command()
        self.last_target_var.set("Last command: stop_all")
        self.exec_status_var.set("当前状态: 已急停/停止剧本")

    def refresh_loop(self):
        snapshot = self.node.get_snapshot()
        now_ts = time.time()

        for topic_name, var in self.status_vars.items():
            var.set(f"{topic_name}: {format_age(now_ts, snapshot['topic_ts'][topic_name])}")

        self.point_count_var.set(f"points_odom count: {snapshot['point_count']}")

        for index, joint_name in enumerate(self.JOINT_LABELS):
            self.joint_vars[joint_name].set(
                f"{self.JOINT_DISPLAY_NAMES[joint_name]}: {snapshot['joint_deg'][index]:.3f} 度 | {snapshot['joint_rad'][index]:.3f} 弧度"
            )

        self.last_target_var.set(snapshot["last_command_text"])
        self.root.after(200, self.refresh_loop)

    def pump_ros(self):
        if not self.ros_spin_ok:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            self.ros_spin_ok = False
            self.node.get_logger().warning(f"ROS spin_once stopped: {exc}")
            return
        self.root.after(10, self.pump_ros)

    def run(self):
        self.root.mainloop()

    def on_close(self):
        self.root.quit()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = V12HybridNode()

    gui = None
    try:
        gui = V12HybridConsole(node)
        gui.run()
    except tk.TclError as exc:
        node.get_logger().warning(f"Tk GUI unavailable, running headless: {exc}")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
