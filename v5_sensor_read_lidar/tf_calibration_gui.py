#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import threading
import datetime
import os

class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        from tf2_ros import StaticTransformBroadcaster
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 初始默认值 (倒装)
        self.x = -0.5500
        self.y = -0.2000
        self.z = 1.2712
        self.yaw = 0.0532
        self.pitch = 0.0349
        self.roll = 3.0316
        
        # 启动时先发一次静态的打底
        self.publish_tf()
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def publish_tf(self):
        # 1. 发布雷达静态 TF (map -> base_link)
        t_lidar = TransformStamped()
        # 静态 TF 时间戳设置为当前时间即可，因为它是 static，所以对所有时间有效
        t_lidar.header.stamp = self.get_clock().now().to_msg()
        t_lidar.header.frame_id = 'map'       # 父坐标系
        t_lidar.child_frame_id = 'base_link'  # 子坐标系
        
        t_lidar.transform.translation.x = self.x
        t_lidar.transform.translation.y = self.y
        t_lidar.transform.translation.z = self.z
        
        q = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        t_lidar.transform.rotation.x = q[0]
        t_lidar.transform.rotation.y = q[1]
        t_lidar.transform.rotation.z = q[2]
        t_lidar.transform.rotation.w = q[3]

        # 2. 静态发布网络摄像头1 TF
        t_net1 = TransformStamped()
        t_net1.header.stamp = self.get_clock().now().to_msg()
        t_net1.header.frame_id = 'base_link'
        t_net1.child_frame_id = 'network_cam_frame'
        t_net1.transform.rotation.w = 1.0

        # 3. 静态发布网络摄像头2 TF
        t_net2 = TransformStamped()
        t_net2.header.stamp = self.get_clock().now().to_msg()
        t_net2.header.frame_id = 'base_link'
        t_net2.child_frame_id = 'network_cam2_frame'
        t_net2.transform.rotation.w = 1.0

        # 4. 静态发布海康摄像头 TF
        t_hik = TransformStamped()
        t_hik.header.stamp = self.get_clock().now().to_msg()
        t_hik.header.frame_id = 'base_link'
        t_hik.child_frame_id = 'hikvision_cam_frame'
        t_hik.transform.rotation.w = 1.0
        
        # 一次性发布所有静态 TF
        self.static_tf_broadcaster.sendTransform([t_lidar, t_net1, t_net2, t_hik])

class TFCalibrationGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title("雷达 TF 动态标定工具")
        self.root.geometry("500x650")
        
        # 变量绑定
        self.var_x = tk.DoubleVar(value=-0.5500)
        self.var_y = tk.DoubleVar(value=-0.2000)
        self.var_z = tk.DoubleVar(value=1.2712)
        self.var_yaw = tk.DoubleVar(value=math.degrees(0.0532))
        self.var_pitch = tk.DoubleVar(value=math.degrees(0.0349))
        self.var_roll = tk.DoubleVar(value=math.degrees(3.0316)) # 界面上显示度数，底层转换弧度
        
        self.step_var = tk.DoubleVar(value=0.05) # 默认步长
        
        self._build_ui()
        self._update_ros_node() # 初始化同步一次

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=20)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        ttk.Label(main_frame, text="雷达 (map) -> base_link (map作为父节点)", font=("Helvetica", 14, "bold")).pack(pady=10)
        
        # 参数调整区
        params_frame = ttk.LabelFrame(main_frame, text="动态参数调整", padding=10)
        params_frame.pack(fill=tk.X, pady=10)
        
        self._create_slider(params_frame, 0, "X (前/后 米):", self.var_x, -5.0, 5.0)
        self._create_slider(params_frame, 1, "Y (左/右 米):", self.var_y, -5.0, 5.0)
        self._create_slider(params_frame, 2, "Z (上/下 米):", self.var_z, -5.0, 5.0)
        
        self._create_slider(params_frame, 3, "Yaw (偏航 度):", self.var_yaw, -180.0, 180.0)
        self._create_slider(params_frame, 4, "Pitch (俯仰 度):", self.var_pitch, -180.0, 180.0)
        self._create_slider(params_frame, 5, "Roll (翻滚 度):", self.var_roll, 0.0, 360.0)
        
        # 步长设置
        step_frame = ttk.Frame(main_frame)
        step_frame.pack(fill=tk.X, pady=5)
        ttk.Label(step_frame, text="微调步长:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(step_frame, textvariable=self.step_var, width=8).pack(side=tk.LEFT)
        
        # 按钮区
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(pady=20)
        
        ttk.Button(btn_frame, text="重置为最新标定参数", command=self._reset_defaults).grid(row=0, column=0, padx=10)
        ttk.Button(btn_frame, text="记录并导出配置", command=self._save_record).grid(row=0, column=1, padx=10)
        
        # 提示信息
        log_frame = ttk.LabelFrame(main_frame, text="操作日志", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=8, width=50)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self._log("工具启动。请在 Rviz2 中查看动态变化的 TF。")
        self._log("提示：在调整本工具时，请务必【注释掉/关闭】launch 文件中原有的静态 TF，避免冲突跳闪！")

    def _create_slider(self, parent, row, label_text, var, min_val, max_val):
        ttk.Label(parent, text=label_text).grid(row=row, column=0, sticky="w", pady=5)
        
        # 减按钮
        ttk.Button(parent, text="-", width=2, command=lambda: self._adjust_val(var, -1)).grid(row=row, column=1, padx=5)
        
        # 滑动条
        scale = ttk.Scale(parent, from_=min_val, to=max_val, orient=tk.HORIZONTAL, variable=var, length=150, command=lambda _: self._update_ros_node())
        scale.grid(row=row, column=2, padx=5)
        
        # 加按钮
        ttk.Button(parent, text="+", width=2, command=lambda: self._adjust_val(var, 1)).grid(row=row, column=3, padx=5)
        
        # 数值显示
        lbl_val = ttk.Label(parent, width=6)
        lbl_val.grid(row=row, column=4, padx=5)
        
        # 动态绑定显示
        def update_lbl(*args):
            lbl_val.config(text=f"{var.get():.3f}")
        var.trace_add("write", update_lbl)
        update_lbl()

    def _adjust_val(self, var, direction):
        current = var.get()
        step = self.step_var.get()
        var.set(current + direction * step)
        self._update_ros_node()

    def _reset_defaults(self):
        self.var_x.set(-0.5500)
        self.var_y.set(-0.2000)
        self.var_z.set(1.2712)
        self.var_yaw.set(math.degrees(0.0532))
        self.var_pitch.set(math.degrees(0.0349))
        self.var_roll.set(math.degrees(3.0316))
        self._update_ros_node()
        self._log("已重置为最新的标定参数。")

    def _update_ros_node(self):
        """将界面上的参数（度）同步给 ROS 节点（弧度），并发布一次静态 TF"""
        self.node.x = self.var_x.get()
        self.node.y = self.var_y.get()
        self.node.z = self.var_z.get()
        self.node.yaw = math.radians(self.var_yaw.get())
        self.node.pitch = math.radians(self.var_pitch.get())
        self.node.roll = math.radians(self.var_roll.get())
        
        # 参数更新后，发布新的静态 TF 覆盖旧的
        self.node.publish_tf()

    def _save_record(self):
        x = self.var_x.get()
        y = self.var_y.get()
        z = self.var_z.get()
        yaw_rad = math.radians(self.var_yaw.get())
        pitch_rad = math.radians(self.var_pitch.get())
        roll_rad = math.radians(self.var_roll.get())
        
        record_str = f"""
# 记录时间: {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
# 请将以下代码复制并替换 sensors_tf.launch.py 中的雷达 Node 节点:

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=['{x:.4f}', '{y:.4f}', '{z:.4f}', '{yaw_rad:.4f}', '{pitch_rad:.4f}', '{roll_rad:.4f}', 'map', 'base_link']
    )
--------------------------------------------------
"""
        
        save_path = os.path.join(os.path.dirname(__file__), "tf_calibration_record.txt")
        try:
            with open(save_path, "a") as f:
                f.write(record_str)
            self._log(f"记录已保存至: {save_path}")
            messagebox.showinfo("保存成功", f"配置已保存！\n文件路径: {save_path}\n\n可以直接打开文件复制 launch 代码。")
        except Exception as e:
            self._log(f"保存失败: {e}")

    def _log(self, msg):
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = TFBroadcasterNode()
    
    # 在后台线程运行 ROS 循环，不阻塞 tkinter
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()
    
    # 启动 GUI
    root = tk.Tk()
    app = TFCalibrationGUI(root, node)
    root.mainloop()
    
    # 退出清理
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
