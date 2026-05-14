#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class ImuVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        
        self.max_points = 200
        
        # 数据存储 Buffer
        self.data_buffers = {
            'imu': {'wx': [], 'wy': [], 'wz': []},
            'bucket_x': [],
            'rotation_x': [],
            'arm_sub_x': []
        }
        self.time_buffers = {k: [] for k in self.data_buffers.keys()}
        
        # 1. 订阅标准雷达 IMU
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # 2. 订阅三个机械臂 X 轴角度
        self.create_subscription(Float32MultiArray, '/imu/bucket_x', self.create_float_callback('bucket_x'), 10)
        self.create_subscription(Float32MultiArray, '/imu/rotation_x', self.create_float_callback('rotation_x'), 10)
        self.create_subscription(Float32MultiArray, '/imu/arm_sub_x', self.create_float_callback('arm_sub_x'), 10)
        
        self.get_logger().info('Subscribed to /imu, /imu/bucket_x, /imu/rotation_x, /imu/arm_sub_x')
            
        # matplotlib 设置：2x2 网格
        self.fig, self.axs = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.canvas.manager.set_window_title('Sensors Real-time Visualization')
        self.fig.suptitle('Lidar IMU (Angular Velocity) & Joints Angle X', fontsize=16)
        
        # 分配子图
        self.ax_imu = self.axs[0, 0]
        self.ax_bucket = self.axs[0, 1]
        self.ax_rotation = self.axs[1, 0]
        self.ax_arm = self.axs[1, 1]
        
        # --- 图 1: Lidar IMU (Angular Velocity) ---
        self.ax_imu.set_title('Lidar /imu (Angular Velocity)')
        self.ax_imu.set_ylim(-4, 4)  # 修改纵坐标限制不超过 10
        self.ax_imu.set_xlim(0, self.max_points)
        self.ax_imu.grid(True)
        self.ax_imu.set_xlabel('Time Frames')
        self.ax_imu.set_ylabel('Angular Velocity')
        self.line_x, = self.ax_imu.plot([], [], label='Angular Vel X', color='red', lw=2)
        self.line_y, = self.ax_imu.plot([], [], label='Angular Vel Y', color='green', lw=2)
        self.line_z, = self.ax_imu.plot([], [], label='Angular Vel Z', color='blue', lw=2)
        self.ax_imu.legend(loc='upper right')
        
        # --- 图 2: Bucket X ---
        self._setup_float_ax(self.ax_bucket, '/bucket')
        self.line_bucket, = self.ax_bucket.plot([], [], label='Bucket X', color='orange', lw=2)
        self.ax_bucket.legend(loc='upper right')
        
        # --- 图 3: Rotation X ---
        self._setup_float_ax(self.ax_rotation, '/boom')
        self.line_rotation, = self.ax_rotation.plot([], [], label='Rotation X', color='purple', lw=2)
        self.ax_rotation.legend(loc='upper right')
        
        # --- 图 4: Arm Sub X ---
        self._setup_float_ax(self.ax_arm, '/arm')
        self.line_arm, = self.ax_arm.plot([], [], label='Arm Sub X', color='brown', lw=2)
        self.ax_arm.legend(loc='upper right')

    def _setup_float_ax(self, ax, title):
        ax.set_title(title)
        ax.set_ylim(-180, 180)  
        ax.set_xlim(0, self.max_points)
        ax.grid(True)
        ax.set_xlabel('Time Frames')
        ax.set_ylabel('Angle (Degrees)')

    def imu_callback(self, msg):
        # 提取角速度 angular_velocity 数据
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        
        self.data_buffers['imu']['wx'].append(wx)
        self.data_buffers['imu']['wy'].append(wy)
        self.data_buffers['imu']['wz'].append(wz)
        
        if len(self.data_buffers['imu']['wx']) > self.max_points:
            self.data_buffers['imu']['wx'].pop(0)
            self.data_buffers['imu']['wy'].pop(0)
            self.data_buffers['imu']['wz'].pop(0)
            
        self.time_buffers['imu'] = list(range(len(self.data_buffers['imu']['wx'])))

    def create_float_callback(self, key):
        def callback(msg):
            if not msg.data:
                return
            val = msg.data[0]
            self.data_buffers[key].append(val)
            if len(self.data_buffers[key]) > self.max_points:
                self.data_buffers[key].pop(0)
            self.time_buffers[key] = list(range(len(self.data_buffers[key])))
        return callback

    def update_plot(self, frame):
        lines = []
        # 更新 IMU
        if self.time_buffers['imu']:
            self.line_x.set_data(self.time_buffers['imu'], self.data_buffers['imu']['wx'])
            self.line_y.set_data(self.time_buffers['imu'], self.data_buffers['imu']['wy'])
            self.line_z.set_data(self.time_buffers['imu'], self.data_buffers['imu']['wz'])
            lines.extend([self.line_x, self.line_y, self.line_z])
            
        # 更新 Bucket X
        if self.time_buffers['bucket_x']:
            self.line_bucket.set_data(self.time_buffers['bucket_x'], self.data_buffers['bucket_x'])
            lines.append(self.line_bucket)
            
        # 更新 Rotation X
        if self.time_buffers['rotation_x']:
            self.line_rotation.set_data(self.time_buffers['rotation_x'], self.data_buffers['rotation_x'])
            lines.append(self.line_rotation)
            
        # 更新 Arm Sub X
        if self.time_buffers['arm_sub_x']:
            self.line_arm.set_data(self.time_buffers['arm_sub_x'], self.data_buffers['arm_sub_x'])
            lines.append(self.line_arm)
            
        return lines

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = ImuVisualizer()
    
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,))
    spin_thread.daemon = True
    spin_thread.start()
    
    ani = FuncAnimation(node.fig, node.update_plot, interval=30, blit=False)
    plt.tight_layout()
    plt.show()  
    
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
