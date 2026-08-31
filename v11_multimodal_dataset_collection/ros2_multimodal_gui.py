import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import sys
import os
import time
import json
import threading
import math
import struct
import socket
import queue

# 引入底层库
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)

sys.path.append(os.path.join(parent_dir, "v1_control_base"))
sys.path.append(os.path.join(parent_dir, "v3_sensor_read_wit", "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new"))
sys.path.append(os.path.join(parent_dir, "v5_sensor_read_lidar"))
sys.path.append(os.path.join(parent_dir, "v4_control_closed"))
sys.path.append(os.path.join(parent_dir, "v10_cailbration_arm"))

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from matplotlib.figure import Figure
from kinematics import ExcavatorKinematics

from zs_excavator_controller import build_controller
import device_model
from angle_controller import AngleController
from imu_direct_swing_estimator import DirectSwingAngleEstimator, LISTEN_PORT

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, JointState
from cv_bridge import CvBridge
import std_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from templates.imu_preintegration import TiltCompensator
from templates.pointcloud_transform import PointCloudTransform

# LIDAR 协议常量
LIDARPOINTCLOUD = 0x01
LIDAR_IP = "192.168.158.99"
LIDAR_PORT = 6543

class Ros2DataPublisher(Node):
    def __init__(self):
        super().__init__('multimodal_excavator_gui')
        self.bridge = CvBridge()
        
        # Publishers
        self.pub_cam1 = self.create_publisher(Image, 'camera1/image_raw', 10)
        self.pub_cam2 = self.create_publisher(Image, 'camera2/image_raw', 10)
        self.pub_cam_hik = self.create_publisher(Image, 'camera_hik/image_raw', 10)
        
        # 发布: /lidar/points 保持为 base_link 坐标系 (随车体转动)
        # 发布: /lidar/points_odom 为 odom 坐标系 (根据 IMU 计算抗旋补偿后，环境静止)
        self.pub_lidar = self.create_publisher(PointCloud2, 'lidar/points', 10)
        self.pub_lidar_odom = self.create_publisher(PointCloud2, 'lidar/points_odom', 10)
        self.pub_elevation = self.create_publisher(Image, 'lidar/elevation_map', 10)
        
        self.pub_joint = self.create_publisher(JointState, 'excavator/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # LIDAR 到 base_link 的静态 TF (从 sensors_tf.launch.py 中提取 map -> base_link 逆矩阵等效)
        # tf2 args: [-0.5500, -0.2000, 1.2712, 0.0532, 0.0349, 3.0316] map base_link
        # 这意味着：base_link 在 map (LIDAR) 坐标系下的位姿。
        # 我们要将 LIDAR (map) 坐标系下的点云，转换到 base_link 坐标系下。
        # 注意: ros2 static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
        # args=['-0.5500', '-0.2000', '1.2712', '0.0532', '0.0349', '3.0316', 'map', 'base_link']
        # 这意味着它发布的是 map -> base_link 的正向变换。
        # 如果点 P_map 在 map 中，它在 base_link 中的坐标 P_base = T_base_map * P_map
        # TF 树中的 T_map_base (即把 base_link 里的点转到 map 里的矩阵) 就是我们从参数构建的矩阵。
        # 因此，P_map = T_map_base * P_base  =>  P_base = (T_map_base)^-1 * P_map
        
        tx = -0.5500
        ty = -0.2000
        tz = 1.2712
        yaw = 0.0532
        pitch = 0.0349
        roll = 3.0316
        
        # ROS 的 static_transform_publisher 接受的欧拉角是 yaw, pitch, roll，也就是围绕 z, y, x 轴旋转
        # 且是 fixed axis 旋转 (extrinsic, 'xyz')，等价于 intrinsic 'ZYX'
        # scipy 中的 'XYZ' extrinsic == 'zyx' intrinsic
        # 我们按照 ROS TF 的标准方式构建 map->base_link 的齐次矩阵
        self.lidar_rot = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        self.lidar_trans = np.array([tx, ty, tz])
        
        # base_link 到 map 的齐次矩阵:
        # T_map_base = [ R   t ]
        #              [ 0   1 ]
        # P_map = T_map_base * P_base
        # 所以 P_base = T_map_base^-1 * P_map = R^T * P_map - R^T * t
        self.lidar_rot_inv = self.lidar_rot.T
        self.lidar_trans_inv = -self.lidar_rot_inv @ self.lidar_trans
        
        
    def publish_odom_tf(self, quaternion):
        """
        发布 odom -> base_link 的动态 TF。
        这样在 RViz 中固定 odom 坐标系时，base_link 就会跟随 IMU 旋转，而周围的点云环境将保持静止！
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # 挖掘机履带不动，平移保持 0
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        
        self.tf_broadcaster.sendTransform(t)

    def publish_image(self, cam_name, frame):
        """
        发布相机图像数据
        - Topic: /camera1/image_raw, /camera2/image_raw, /camera_hik/image_raw
        - 类型: sensor_msgs/msg/Image
        - 频率: 约 10Hz (受 VideoStreamThread 中的 sleep 限制)
        - 数据: BGR8 编码的原始图像帧，带系统时间戳
        """
        if frame is None:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = cam_name
        if cam_name == "cam1":
            self.pub_cam1.publish(msg)
        elif cam_name == "cam2":
            self.pub_cam2.publish(msg)
        elif cam_name == "cam_hik":
            self.pub_cam_hik.publish(msg)

    def publish_pointcloud(self, pts_array):
        """
        发布雷达点云数据
        - Topic: /lidar/points (base_link 系下)
        - Topic: /lidar/points_odom (odom 系下)
        """
        if pts_array is None or len(pts_array) == 0:
            return
            
        # 将点云从 lidar 本地(map)坐标系转换到 base_link
        # pts_array shape is (N, 3)
        pts_base_link = (self.lidar_rot_inv @ pts_array.T).T + self.lidar_trans_inv
        
        # --- 根据 base_link 坐标系进行空间有效区域过滤 ---
        # 仅保留 x, y 在 (-3, 3) 之间，且 z > -0.1 的点
        mask = (
            (pts_base_link[:, 0] > -3.0) & (pts_base_link[:, 0] < 3.0) &
            (pts_base_link[:, 1] > -3.0) & (pts_base_link[:, 1] < 3.0) &
            (pts_base_link[:, 2] > -0.1)
        )
        pts_base_link = pts_base_link[mask]
        
        # 如果过滤后没有点了，直接跳过发布
        if len(pts_base_link) == 0:
            return
            
        # --- 根据 Z 轴高度 ([-0.4, 0.7]) 映射为 0-255 的灰度颜色 ---
        z_min, z_max = -0.4, 0.7
        z_vals = np.clip(pts_base_link[:, 2], z_min, z_max)
        gray_vals = ((z_vals - z_min) / (z_max - z_min) * 255.0).astype(np.uint32)
        rgb_vals = (gray_vals << 16) | (gray_vals << 8) | gray_vals
        
        dt = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)])
        
        # 1. 发布 base_link 下的点云 (原始标定后，随车体转动)
        msg_base = PointCloud2()
        msg_base.header.stamp = self.get_clock().now().to_msg()
        msg_base.header.frame_id = "base_link"
        msg_base.height = 1
        msg_base.width = len(pts_base_link)
        msg_base.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        msg_base.is_bigendian = False
        msg_base.point_step = 16
        msg_base.row_step = msg_base.point_step * msg_base.width
        msg_base.is_dense = True
        
        pc_data_base = np.empty(len(pts_base_link), dtype=dt)
        pc_data_base['x'] = pts_base_link[:, 0]
        pc_data_base['y'] = pts_base_link[:, 1]
        pc_data_base['z'] = pts_base_link[:, 2]
        pc_data_base['rgb'] = rgb_vals
        msg_base.data = pc_data_base.tobytes()
        self.pub_lidar.publish(msg_base)
        
        # 2. 计算并发布 odom 下的点云 (根据雷达IMU旋转跟随，使得点云环境静止)
        msg_odom = PointCloud2()
        msg_odom.header.stamp = msg_base.header.stamp
        msg_odom.header.frame_id = "odom"
        msg_odom.height = 1
        msg_odom.width = len(pts_base_link)
        msg_odom.fields = msg_base.fields
        msg_odom.is_bigendian = False
        msg_odom.point_step = 16
        msg_odom.row_step = msg_odom.point_step * msg_odom.width
        msg_odom.is_dense = True
        
        # 获取当前的 base_link -> odom 的四元数 (正向)
        quat = np.array([0.0, 0.0, 0.0, 1.0])
        if hasattr(self, 'tilt_compensator'):
            quat = self.tilt_compensator.get_quaternion(getattr(self, 'last_yaw_rad', 0.0))
        
        # --- 高效旋转矩阵计算 ---
        r_matrix = R.from_quat(quat).as_matrix()
        # pts_odom = R * P + T (T 是 0)
        pts_odom = (r_matrix @ pts_base_link.T).T
        
        pc_data_odom = np.empty(len(pts_odom), dtype=dt)
        pc_data_odom['x'] = pts_odom[:, 0]
        pc_data_odom['y'] = pts_odom[:, 1]
        pc_data_odom['z'] = pts_odom[:, 2]
        pc_data_odom['rgb'] = rgb_vals
        msg_odom.data = pc_data_odom.tobytes()
        
        # 移除时间锁限制，单帧点云数量很小，完全可以实时双发
        self.pub_lidar_odom.publish(msg_odom)
        
        # 将过滤后的点云数据送入高程图线程进行 2D 转换 (依然用 base_link 的，保证高程图一直居中)
        if hasattr(self, 'elevation_callback') and self.elevation_callback:
            self.elevation_callback(pts_base_link)

    def publish_elevation_map(self, elevation_img):
        """
        发布由点云转换而来的 2D 高程图 (Elevation Map)
        - Topic: /lidar/elevation_map
        - 类型: sensor_msgs/msg/Image (BGR8 三通道灰度图，对齐网易标准)
        """
        if elevation_img is None:
            return
        msg = self.bridge.cv2_to_imgmsg(elevation_img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.pub_elevation.publish(msg)

    def publish_joint_state(self, diff_ab, diff_ba, diff_bs, yaw_s):
        """
        发布本体关节角度与姿态数据
        - Topic: /excavator/joint_states
        - 类型: sensor_msgs/msg/JointState
        - 频率: 20Hz (由 _update_loop 主循环按 50ms 周期发布)
        - 数据: 包含 4 个关节角度，以标准弧度 (Radians) 表示
        
        关节数组顺序及对应关系 (msg.position):
          [0] boom_joint   : 大臂-回转 夹角 (diff_bs)
          [1] arm_joint    : 小臂-大臂 夹角 (diff_ab)
          [2] bucket_joint : 铲斗-小臂 夹角 (diff_ba)
          [3] swing_joint  : 回转偏航角 (yaw_s, 由 IMU 解算)
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Joint names should match URDF if you have one
        msg.name = ['boom_joint', 'arm_joint', 'bucket_joint', 'swing_joint']
        
        # Convert degrees to radians for standard ROS format
        msg.position = [
            math.radians(diff_bs),
            math.radians(diff_ab),
            math.radians(diff_ba),
            math.radians(yaw_s)
        ]
        self.pub_joint.publish(msg)

import cv2
class VideoStreamThread(threading.Thread):
    def __init__(self, name, rtsp_url, ros_node, transport="udp", hw_status_dict=None):
        super().__init__(daemon=True)
        self.name = name
        self.rtsp_url = rtsp_url
        self.ros_node = ros_node
        self.transport = transport
        self.hw_status_dict = hw_status_dict
        self.running = True
        # 增加 fflags;nobuffer 和 flags;low_delay 强制降低 FFMPEG 解码延迟
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = f"rtsp_transport;{self.transport}|stimeout;3000000|fflags;nobuffer|flags;low_delay"

    def run(self):
        print(f"[Camera {self.name}] 尝试连接 RTSP流: {self.rtsp_url}")
        cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        # 强制将 OpenCV 内部缓冲设为 1
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if not cap.isOpened():
            if self.hw_status_dict is not None:
                self.hw_status_dict[self.name] = "failed"
            return
            
        if self.hw_status_dict is not None:
            self.hw_status_dict[self.name] = "connected"
        
        last_pub_time = 0
        
        while self.running:
            # 必须全速读取！不能在读取后 sleep，否则底层缓冲区会疯狂积压旧画面，导致花屏和巨大延迟
            ret, frame = cap.read()
            if not ret:
                time.sleep(1)
                cap.release()
                cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                continue
            
            # 发布 ROS 2 Image 消息，控制发布频率在 10Hz 左右以降低网络和CPU负载
            current_time = time.time()
            if current_time - last_pub_time >= 0.1:
                self.ros_node.publish_image(self.name, frame)
                last_pub_time = current_time

        cap.release()

    def stop(self):
        self.running = False

def generate_elevation_map(points, x_range=(-3.0, 3.0), y_range=(-3.0, 3.0), resolution=0.03, z_range=(-0.4, 0.7), bucket_tip=None):
    """
    点云高程图算法 (Elevation Map)
    将过滤后的点云投影到 2D 网格上，取每个网格内的最大 Z 值作为高程。
    将 Z 值 [z_range[0], z_range[1]] 映射到 [0, 255] 灰度图像。
    
    参数:
        points: (N, 3) Numpy Array
        x_range, y_range: 网格的物理范围 (米)
        resolution: 每个像素代表的物理大小 (米/像素)
        z_range: 映射的最低和最高物理高度
        bucket_tip: 铲尖 3D 坐标 (x, y, z)，若提供则在图上绘制铲尖位置
    返回:
        2D numpy array (uint8)，三通道灰度图 (对齐网易标准)
    """
    width = int((x_range[1] - x_range[0]) / resolution)
    height = int((y_range[1] - y_range[0]) / resolution)
    
    # 初始化一个值为 z_range[0] 的一维数组
    flat_map = np.full(width * height, z_range[0], dtype=np.float32)
    
    if points is not None and len(points) > 0:
        # 映射到像素坐标
        u = np.floor((points[:, 1] - y_range[0]) / resolution).astype(int)
        v = np.floor((x_range[1] - points[:, 0]) / resolution).astype(int) - 1 # X反向，确保前方在图像上方
        z = points[:, 2]
        
        # 过滤越界点（虽然前面过滤过，但防止边界精度误差）
        valid_idx = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u = u[valid_idx]
        v = v[valid_idx]
        z = z[valid_idx]
        
        # 计算在一维数组中的索引
        flat_indices = v * width + u
        
        # 相同网格内的多个点，取最大的 Z 值
        np.maximum.at(flat_map, flat_indices, z)
        
    elevation_map = flat_map.reshape((height, width))
    
    # 归一化到 0-255 灰度值
    z_min, z_max = z_range
    elevation_map = np.clip(elevation_map, z_min, z_max)
    elevation_map_img = ((elevation_map - z_min) / (z_max - z_min) * 255.0).astype(np.uint8)
    
    # 转为 3 通道纯灰度图 (对齐网易的 3 通道格式，R=G=B)
    import cv2
    elevation_map_3ch = cv2.cvtColor(elevation_map_img, cv2.COLOR_GRAY2BGR)
    
    if bucket_tip is not None:
        bx, by, bz = bucket_tip
        bu = int(np.floor((by - y_range[0]) / resolution))
        bv = int(np.floor((x_range[1] - bx) / resolution)) - 1
        
        # 在图像上绘制铲尖位置 (红色圆点，外加白圈提升对比度)
        if 0 <= bu < width and 0 <= bv < height:
            cv2.circle(elevation_map_3ch, (bu, bv), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.circle(elevation_map_3ch, (bu, bv), radius=4, color=(255, 255, 255), thickness=1)
            
    return elevation_map_3ch

class V11MultimodalGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        self.root.title("V11 ROS2 控制与传感器发布节点")
        self.root.geometry("800x700")
        
        # 初始化 ROS 2 发布器作为 recorder 替代
        self.camera_threads = []
        
        # 硬件状态字典
        self.hw_status = {
            "controller": "waiting",
            "sensors": "waiting",
            "lidar": "waiting",
            "cam_hik": "waiting",
            "cam1": "waiting",
            "cam2": "waiting"
        }

        # 1. 初始化物理控制器
        try:
            self.base_controller = build_controller(port="/dev/ttyUSB_Controller", baudrate=115200)
            if not self.base_controller.connect():
                self.hw_status["controller"] = "failed"
                messagebox.showwarning("连接失败", "无法打开 CAN 串口(/dev/ttyUSB_Controller)，当前处于离线模式 (离线模式下仅能读取传感器并发布话题)。")
            else:
                self.hw_status["controller"] = "connected"
        except Exception as e:
            self.hw_status["controller"] = "failed"
            print(f"CAN 初始化失败: {e}")
            self.base_controller = None

        # 2. 包装成闭环角度控制器
        if self.base_controller and self.hw_status["controller"] == "connected":
            self.angle_ctrl = AngleController(self.base_controller)
        else:
            self.angle_ctrl = None
            print("[警告] CAN 控制器离线，控制指令将被忽略。")
        self.kin = ExcavatorKinematics()

        # 实时3D可视化相关变量
        self.live_3d_window = None
        self.live_3d_canvas = None
        self.live_frames = []
        self.live_traj_x = []
        self.live_traj_y = []
        self.live_traj_d = []
        self.live_traj_z = []

        # 传感器缓存与设备列表
        self.sensor_data = {
            "大臂": {"pitch": 0.0, "yaw": 0.0},
            "小臂": {"pitch": 0.0, "yaw": 0.0},
            "铲斗": {"pitch": 0.0, "yaw": 0.0},
            "回转": {"pitch": 0.0, "yaw": 0.0},
        }
        self.devices = []

        # UI 变量
        self.target_bucket_arm = tk.DoubleVar(value=90.0)
        self.target_arm_boom = tk.DoubleVar(value=90.0)
        self.target_boom_swing = tk.DoubleVar(value=90.0)
        self.target_swing_yaw = tk.DoubleVar(value=0.0) # 现为目标角度，正右负左
        
        self.ch1_var = tk.IntVar(value=0)
        self.ch2_var = tk.IntVar(value=0)
        self.ch3_var = tk.IntVar(value=2000)
        
        # 柔性控制加减速参数
        self.ramp_up_var = tk.DoubleVar(value=0.2)
        self.ramp_down_var = tk.DoubleVar(value=0.2)
        
        # 新增剧本录制相关变量
        self.is_recording = False
        self.recorded_script = []
        self.recording_history = {
            "bucket_arm": [],
            "arm_boom": [],
            "boom_swing": [],
            "swing_yaw": []
        }
        self.script_running = False
        
        # 保存当前实时计算的角度
        self.current_angles = {
            "bucket_arm": 0.0,
            "arm_boom": 0.0,
            "boom_swing": 0.0,
            "swing_yaw": 0.0
        }
        self.current_bucket_tip_3d = None

        # 设置 JSON 统一保存目录
        self.json_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "json"))
        os.makedirs(self.json_dir, exist_ok=True)

        # 实例化 IMU 倾斜补偿与预积分器
        self.tilt_compensator = TiltCompensator(alpha=0.98)

        # 启动雷达 IMU 监听线程
        self.imu_running = True
        self.imu_thread = threading.Thread(target=self._imu_listener_loop, daemon=True)
        self.imu_thread.start()

        # 启动点云高程图处理线程
        self.elevation_queue = queue.Queue(maxsize=5)
        self.elevation_running = True
        self.elevation_thread = threading.Thread(target=self._elevation_map_loop, daemon=True)
        self.elevation_thread.start()

        # 提供给 Ros2DataPublisher 的回调，用于接收过滤后的点云
        self.ros_node.elevation_callback = self._on_pointcloud_received

        # 启动三个摄像头的视频流拉取线程
        self._start_camera_threads()

        # 初始化传感器
        self._init_sensors()
        self._build_ui()
        self._update_loop()

    def _start_camera_threads(self):
        # 海康摄像头 (建议 TCP)
        hik_url = "rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101"
        t_hik = VideoStreamThread("cam_hik", hik_url, self.ros_node, transport="tcp", hw_status_dict=self.hw_status)
        
        # 网络摄像头 1 (UDP)
        net1_url = "rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream"
        t_net1 = VideoStreamThread("cam1", net1_url, self.ros_node, transport="udp", hw_status_dict=self.hw_status)
        
        # 网络摄像头 2 (UDP)
        net2_url = "rtsp://admin:@192.168.158.103:554/stream"
        t_net2 = VideoStreamThread("cam2", net2_url, self.ros_node, transport="udp", hw_status_dict=self.hw_status)
        
        self.camera_threads = [t_hik, t_net1, t_net2]
        self.cams = self.camera_threads
        for t in self.camera_threads:
            t.start()

    def _on_pointcloud_received(self, pts_base_link):
        """接收已过滤的点云，进行重力纠正后，送入队列"""
        if self.elevation_running and not self.elevation_queue.full():
            # 获取重力对齐的四元数 (仅 Roll, Pitch，忽略 Yaw)
            quat_gravity = self.tilt_compensator.get_gravity_aligned_quaternion()
            # 纠正点云，使得高程图永远水平
            pts_horizontal = PointCloudTransform.gravity_align_only(pts_base_link, quat_gravity)
            
            self.elevation_queue.put((pts_horizontal, self.current_bucket_tip_3d))

    def _elevation_map_loop(self):
        """高程图生成线程：消费点云，生成灰度图并发布"""
        while self.elevation_running:
            try:
                # 阻塞等待点云数据，超时时间0.1秒
                item = self.elevation_queue.get(timeout=0.1)
                if isinstance(item, tuple) and len(item) == 2:
                    pts, bucket_tip = item
                else:
                    pts = item
                    bucket_tip = None
                # 生成高程图
                elevation_img = generate_elevation_map(pts, bucket_tip=bucket_tip)
                # 发布到 ROS Topic
                self.ros_node.publish_elevation_map(elevation_img)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[高程图生成异常]: {e}")

    def _send_lidar_start_command(self, sock):
        """模拟 C++ 驱动发送启动雷达的点云/IMU 推送指令 (LSTARH)"""
        def crc32_stm32(data):
            crc = 0xFFFFFFFF
            for i in range(0, len(data), 4):
                word = struct.unpack_from('>I', data, i)[0] if i + 4 <= len(data) else 0
                crc ^= word
                for _ in range(32):
                    if crc & 0x80000000:
                        crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
                    else:
                        crc = (crc << 1) & 0xFFFFFFFF
            return crc

        import random
        sn = random.randint(0, 65535)
        sign = 0x484C
        cmd_type = 0x0043
        cmd_str = "LSTARH"
        length = len(cmd_str)
        len4 = ((length + 3) >> 2) * 4
        padded_payload = cmd_str.encode('ascii') + b'\x00' * (len4 - length)
        
        header = struct.pack('<H H H H', sign, cmd_type, sn, length)
        packet_without_crc = header + padded_payload
        crc = crc32_stm32(packet_without_crc)
        packet = packet_without_crc + struct.pack('<I', crc)
        
        for i in range(5):
            sock.sendto(packet, (LIDAR_IP, LIDAR_PORT))
            time.sleep(0.1)
        print("[Lidar] 已发送启动指令 (LSTARH)")

    def _imu_listener_loop(self):
        print("Starting UDP Lidar IMU & PointCloud listener...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Increase UDP receive buffer size to prevent dropping packets
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024 * 8)
        try:
            sock.bind(('0.0.0.0', LISTEN_PORT))
        except Exception as e:
            print(f"Failed to bind UDP socket for Lidar: {e}")
            self.hw_status["lidar"] = "failed"
            return
            
        # 发送启动雷达指令
        self._send_lidar_start_command(sock)

        estimator = DirectSwingAngleEstimator()
        pc_count = 0
        
        while self.imu_running:
            try:
                sock.settimeout(0.5)
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except Exception:
                break
                
            if not data:
                continue
            
            # 只要收到雷达任意有效包，就认为 Lidar 连接成功
            self.hw_status["lidar"] = "connected"
                
            # === 1. 解析 IMU 报文 ===
            if len(data) >= 27 and data[0] == 0xfa and data[1] == 0x88:
                imu_fmt = '<B h h h h h h b H Q'
                try:
                    imu_data = struct.unpack_from(imu_fmt, data, 8 + 1)
                    accel_x = imu_data[1] * 4.0 / 0x10000
                    accel_y = imu_data[2] * 4.0 / 0x10000
                    accel_z = imu_data[3] * 4.0 / 0x10000
                    gyro_x = imu_data[4] * 4000.0 / 0x10000 * math.pi / 180
                    gyro_y = imu_data[5] * 4000.0 / 0x10000 * math.pi / 180
                    gyro_z = imu_data[6] * 4000.0 / 0x10000 * math.pi / 180
                    timestamp = imu_data[9]
                    
                    # --- 1. 原有的 DirectSwingAngleEstimator (计算高精度 Yaw) ---
                    if not hasattr(self, 'last_yaw_rad'):
                        self.last_yaw_rad = 0.0
                        
                    res = estimator.process_imu((accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), timestamp)
                    if res is not None:
                        swing_deg, w_yaw = res
                        self.sensor_data["回转"]["yaw"] = swing_deg
                        self.sensor_data["回转"]["yaw_rate"] = w_yaw
                        self.sensor_data["回转"]["ts"] = time.time()
                        
                        # 修复: 当左旋转(逆时针)时，如果环境反而向右旋转(反转了180度)，
                        # 说明计算矩阵时传入的 yaw 符号反了，导致过补偿/反向补偿。
                        # 这里去掉负号，翻转用于生成 R_odom_base 的偏航角方向。
                        self.last_yaw_rad = math.radians(swing_deg)
                        
                    # --- 2. 更新 TiltCompensator 并发布 Odom TF ---
                    # 转换雷达坐标系下的 IMU 数据到 base_link
                    accel_base = self.ros_node.lidar_rot_inv @ np.array([accel_x, accel_y, accel_z])
                    gyro_base = self.ros_node.lidar_rot_inv @ np.array([gyro_x, gyro_y, gyro_z])
                    
                    current_time = time.time()
                    quat = self.tilt_compensator.update(current_time, accel_base, gyro_base, external_yaw=self.last_yaw_rad)
                    
                    if not hasattr(self, 'last_tf_time'):
                        self.last_tf_time = 0
                    if current_time - self.last_tf_time > 0.02: # 50Hz TF 频率
                        self.ros_node.publish_odom_tf(quat)
                        self.last_tf_time = current_time
                except struct.error:
                    pass
                    
            # === 2. 解析 PointCloud 点云报文 ===
            elif len(data) >= 36 and (data[0] == 0x00 or data[0] == 0x01):
                header = struct.unpack_from('<B H H H H B B B 12s I Q', data, 0)
                dot_num = header[3]
                data_type = header[6]
                
                # 打印出雷达数据包类型，让我们知道是不是雷达发过来的数据不是 LIDARPOINTCLOUD
                # 不用 pc_count 限制打印，因为如果根本没进点云循环，pc_count 就不会增加
                # 为了防止刷屏，我们用时间控制
                if not hasattr(self, '_last_print_time'):
                    self._last_print_time = 0
                if time.time() - self._last_print_time > 2.0:
                    print(f"[雷达数据监听] UDP 接收中... 当前收到的报文类型: {data_type}, 数据点数量: {dot_num}")
                    self._last_print_time = time.time()
                
                # C++ 驱动实际上并不检查 data_type，只要包头是 0 或 1 均认为是点云
                try:
                    # 恢复 0.1s 一帧的累积逻辑 (雷达旋转一周大约需要 0.1s，这是一帧完整的 360 度点云)
                    if not hasattr(self, 'pc_buffer'):
                        self.pc_buffer = []
                        self.last_pc_save_time = time.time()

                    if dot_num > 0 and len(data) >= 36 + dot_num * 10:
                        buf = data[36:36 + dot_num * 10]
                        dt = np.dtype([('word1', '<u4'), ('word2', '<u4'), ('ref', 'u1'), ('tag', 'u1')])
                        arr = np.frombuffer(buf, dtype=dt)
                        
                        depth = arr['word1'] & 0xFFFFFF
                        theta_hi = (arr['word1'] >> 24) & 0xFF
                        theta_lo = arr['word2'] & 0xFFF
                        phi = (arr['word2'] >> 12) & 0xFFFFF
                        
                        theta = (theta_hi << 12) | theta_lo
                        ang = (90000 - theta) * (math.pi / 180000.0)
                        depth_m = depth / 1000.0
                        
                        r = depth_m * np.cos(ang)
                        z = depth_m * np.sin(ang)
                        phi_ang = phi * (math.pi / 180000.0)
                        x = np.cos(phi_ang) * r
                        y = np.sin(phi_ang) * r
                        
                        pts = np.column_stack((x, y, z))
                        
                        if len(pts) > 0:
                            self.pc_buffer.extend(pts.tolist())

                        # 每 0.1 秒 (10Hz) 发布一次完整的一帧点云
                        if time.time() - self.last_pc_save_time >= 0.1:
                            if self.pc_buffer:
                                pts_arr = np.array(self.pc_buffer, dtype=np.float32)
                                self.ros_node.publish_pointcloud(pts_arr)
                                # 发布完成后清空，准备累积下一帧
                                self.pc_buffer = []
                            self.last_pc_save_time = time.time()
                except Exception as e:
                    print(f"[雷达数据监听] 解析点云异常: {e}")

        sock.close()

    def _init_sensors(self):
        addrLis = [0x50, 0x51, 0x52, 0x53]
        baud = 230400
        
        # 使用 Ubuntu 下 udev 规则绑定的软链接名称
        ports = [
            "/dev/ttyUSB_Sensor1",
            "/dev/ttyUSB_Sensor2",
            "/dev/ttyUSB_Sensor3",
            "/dev/ttyUSB_Sensor4",
        ]
        
        success_count = 0
        for port in ports:
            try:
                # 注意这里传入 port，因为我们要通过 id_to_name 在回调里判断具体是哪个传感器
                dev = device_model.DeviceModel(port, port, baud, addrLis, self._sensor_callback(port))
                dev.openDevice()
                dev.startLoopRead()
                self.devices.append(dev)
                print(f"[{port}] 传感器初始化成功")
                success_count += 1
            except Exception as e:
                print(f"[{port}] 初始化失败: {e}")
                
        if success_count == 4:
            self.hw_status["sensors"] = "connected"
        elif success_count > 0:
            self.hw_status["sensors"] = "partial"
        else:
            self.hw_status["sensors"] = "failed"

    def _sensor_callback(self, port_name):
        id_to_name = {
            0x50: "铲斗",
            0x51: "小臂",
            0x52: "大臂",
            0x53: "回转"
        }
        
        def update(dm):
            for addr, name in id_to_name.items():
                data = dm.deviceData.get(addr, {})
                # 我们这里获取 AngX(Roll) 代替之前的 AngY，保持和 v3 ROS2 一致
                if data and "AngX" in data:
                    self.sensor_data[name]["pitch"] = data.get("AngX", 0.0)
                    # 仅当不是回转传感器时才更新 yaw，因为回转 yaw 现由 IMU 专门接管提供
                    if name != "回转":
                        self.sensor_data[name]["yaw"] = data.get("AngZ", 0.0)
                    
                    self.sensor_data[name]["ts"] = time.time()
                    
                    # 取出数据后清除缓存
                    dm.deviceData[addr].clear()
        return update

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        
        # 移除 UI 中与直接写入磁盘相关的数据集录制按钮
        # 顶部：硬件连接状态区
        hw_frame = ttk.LabelFrame(main_frame, text="硬件连接状态", padding=10)
        hw_frame.pack(fill=tk.X, pady=5)
        
        self.lbl_hw_ctrl = tk.Label(hw_frame, text="CAN控制: ⏳", width=15, anchor="w")
        self.lbl_hw_ctrl.grid(row=0, column=0, padx=5)
        
        self.lbl_hw_sensors = tk.Label(hw_frame, text="倾角传感器: ⏳", width=18, anchor="w")
        self.lbl_hw_sensors.grid(row=0, column=1, padx=5)
        
        self.lbl_hw_lidar = tk.Label(hw_frame, text="激光雷达: ⏳", width=15, anchor="w")
        self.lbl_hw_lidar.grid(row=0, column=2, padx=5)
        
        self.lbl_hw_cam_hik = tk.Label(hw_frame, text="海康相机: ⏳", width=15, anchor="w")
        self.lbl_hw_cam_hik.grid(row=1, column=0, padx=5, pady=5)
        
        self.lbl_hw_cam1 = tk.Label(hw_frame, text="网络相机1: ⏳", width=15, anchor="w")
        self.lbl_hw_cam1.grid(row=1, column=1, padx=5, pady=5)
        
        self.lbl_hw_cam2 = tk.Label(hw_frame, text="网络相机2: ⏳", width=15, anchor="w")
        self.lbl_hw_cam2.grid(row=1, column=2, padx=5, pady=5)
        
        # --- 传感器实时数据 ---
        status_frame = ttk.LabelFrame(main_frame, text="传感器实时状态", padding=10)
        status_frame.pack(fill=tk.X, pady=5)
        self.lbl_bucket_arm = ttk.Label(status_frame, text="铲斗-小臂 夹角: --°")
        self.lbl_bucket_arm.grid(row=0, column=0, padx=20, pady=5, sticky="w")
        self.lbl_arm_boom = ttk.Label(status_frame, text="小臂-大臂 夹角: --°")
        self.lbl_arm_boom.grid(row=1, column=0, padx=20, pady=5, sticky="w")
        self.lbl_boom_swing = ttk.Label(status_frame, text="大臂-回转 夹角: --°")
        self.lbl_boom_swing.grid(row=0, column=1, padx=20, pady=5, sticky="w")
        self.lbl_swing_yaw = ttk.Label(status_frame, text="回转 偏航角: --°")
        self.lbl_swing_yaw.grid(row=1, column=1, padx=20, pady=5, sticky="w")

        # --- 中间：推力配置 ---
        analog_frame = ttk.LabelFrame(main_frame, text="模拟量与柔性参数配置", padding=10)
        analog_frame.pack(fill=tk.X, pady=5)
        ttk.Label(analog_frame, text="CH1(左):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch1_var, width=6, state="disabled").pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="CH2(右):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch2_var, width=6, state="disabled").pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="CH3(液压):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch3_var, width=6).pack(side=tk.LEFT, padx=5)
        
        ttk.Label(analog_frame, text="|  加速时间(s):").pack(side=tk.LEFT, padx=(15, 5))
        ttk.Entry(analog_frame, textvariable=self.ramp_up_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="减速时间(s):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ramp_down_var, width=5).pack(side=tk.LEFT, padx=5)

        # --- 下方：闭环目标控制 ---
        ctrl_frame = ttk.LabelFrame(main_frame, text="闭环角度目标控制", padding=10)
        ctrl_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self._create_ctrl_row(ctrl_frame, 0, "铲斗-小臂", "bucket_arm", self.target_bucket_arm, "目标角度(°):")
        self._create_ctrl_row(ctrl_frame, 1, "小臂-大臂", "arm_boom", self.target_arm_boom, "目标角度(°):")
        self._create_ctrl_row(ctrl_frame, 2, "大臂-回转", "boom_swing", self.target_boom_swing, "目标角度(°):")
        
        # 回转改为基于 IMU 角度控制
        self._create_ctrl_row(ctrl_frame, 3, "回转动作", "swing_yaw", self.target_swing_yaw, "目标角度(°): (正右负左)")

        # --- 底部：剧本录制与保存区 ---
        record_frame = ttk.Frame(main_frame)
        record_frame.pack(fill=tk.X, pady=10)
        
        self.btn_record_manual = tk.Button(record_frame, text="🔴 开始手动录制 (记录按钮动作)", command=self._toggle_recording_manual, bg="#ffcccc", width=25)
        self.btn_record_manual.pack(side=tk.LEFT, padx=5)

        self.btn_record_auto = tk.Button(record_frame, text="🔴 开始自动提取 (基于遥控动作)", command=self._toggle_recording_auto, bg="#ffebcc", width=28)
        self.btn_record_auto.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(record_frame, text="💾 保存为 JSON 剧本", command=self._save_script, width=20).pack(side=tk.LEFT, padx=10)
        
        # --- 新增：剧本一键执行区 ---
        self.btn_load_script = tk.Button(record_frame, text="📂 选择并执行 JSON 剧本", command=self._load_and_run_script, bg="#ccccff", width=22)
        self.btn_load_script.pack(side=tk.LEFT, padx=10)
        
        self.lbl_exec_status = ttk.Label(record_frame, text="当前状态: 未执行", font=("Arial", 11))
        self.lbl_exec_status.pack(side=tk.LEFT, padx=10)
        
        ttk.Button(main_frame, text="【急停所有闭环动作】", command=self._emergency_stop).pack(pady=10, ipadx=20, ipady=10)

    def _emergency_stop(self):
        self.script_running = False
        if self.angle_ctrl:
            self.angle_ctrl.stop_all()

    def _toggle_dataset_recording(self):
        pass # The UI button was removed

    def _toggle_recording_manual(self):
        """传统手动录制模式：只记录点击按钮时的目标值"""
        if getattr(self, 'is_recording_auto', False):
            messagebox.showwarning("警告", "当前正在进行自动提取录制，请先停止！")
            return
            
        if not self.is_recording:
            self.is_recording = True
            self.recorded_script = []
            self.btn_record_manual.config(text="⏹ 停止手动录制", bg="#ccffcc")
            messagebox.showinfo("开始录制", "已开始【手动】录制剧本。现在您下发的每一次【开始移动】或【记录当前角度】都会被记录。")
        else:
            self.is_recording = False
            self.btn_record_manual.config(text="🔴 开始手动录制 (记录按钮动作)", bg="#ffcccc")
            messagebox.showinfo("停止录制", f"手动录制已停止，当前共记录了 {len(self.recorded_script)} 个动作，请点击保存。")

    def _toggle_recording_auto(self):
        """基于传感器轨迹的自动防抖提取模式"""
        if self.is_recording and not getattr(self, 'is_recording_auto', False):
            messagebox.showwarning("警告", "当前正在进行手动录制，请先停止！")
            return
            
        if not getattr(self, 'is_recording_auto', False):
            self.is_recording = True
            self.is_recording_auto = True
            self.recorded_script = []
            for k in self.recording_history:
                self.recording_history[k].clear()
            self.btn_record_auto.config(text="⏹ 停止自动提取", bg="#ccffcc")
            messagebox.showinfo("开始录制", "已开始【自动】提取。请遥控挖掘机，系统将自动过滤抖动并提取动作终点生成剧本。")
        else:
            self.is_recording = False
            self.is_recording_auto = False
            self.btn_record_auto.config(text="🔴 开始自动提取 (基于遥控动作)", bg="#ffebcc")
            self._auto_generate_script_from_history()
            messagebox.showinfo("停止录制", f"自动提取已停止，共提取了 {len(self.recorded_script)} 个动作，请点击保存。")

    def _auto_generate_script_from_history(self):
        """基于滑动窗口方差的动作终点自动提取算法，有效过滤抖动"""
        window_size = 10         # 约 0.5 秒 (20Hz)
        stable_threshold = 1.0   # 极差 > 1.0 度认为是运动中
        steady_time_s = 1.0      # 连续静止 1.0 秒才认为动作彻底结束
        min_move_deg = 2.0       # 动作前后角度差 > 2.0 度才记录 (过滤手抖)
        
        script_events = []
        
        for joint_name, history in self.recording_history.items():
            if len(history) < window_size:
                continue
                
            is_moving = False
            steady_start_time = None
            last_recorded_val = sum(x[1] for x in history[:window_size]) / window_size
            
            for i in range(len(history) - window_size):
                window = history[i:i+window_size]
                vals = [x[1] for x in window]
                ts = window[-1][0]
                
                val_max = max(vals)
                val_min = min(vals)
                val_avg = sum(vals)/len(vals)
                
                if val_max - val_min > stable_threshold:
                    is_moving = True
                    steady_start_time = None
                else:
                    if is_moving:
                        if steady_start_time is None:
                            steady_start_time = ts
                        elif ts - steady_start_time > steady_time_s:
                            # 动作确实结束了，检查是否产生实质位移
                            if abs(val_avg - last_recorded_val) > min_move_deg:
                                action = {
                                    "joint": joint_name,
                                    "target_val": round(val_avg, 1),
                                    "ch1_mv": 0,
                                    "ch2_mv": 0,
                                    "ch3_mv": self.ch3_var.get(),
                                    "ramp_up_s": self.ramp_up_var.get(),
                                    "ramp_down_s": self.ramp_down_var.get(),
                                    "is_init_step": False,
                                    "description": f"自动提取: {joint_name} 运动至 {val_avg:.1f}°"
                                }
                                script_events.append((ts, action))
                                last_recorded_val = val_avg
                            
                            is_moving = False
                            steady_start_time = None
                            
        # 按时间戳排序以保证剧本执行顺序
        script_events.sort(key=lambda x: x[0])
        
        self.recorded_script = []
        for i, (ts, act) in enumerate(script_events):
            act["step"] = i + 1
            self.recorded_script.append(act)

    def _load_and_run_script(self):
        if self.script_running:
            messagebox.showwarning("警告", "当前已有剧本正在执行，请先急停！")
            return
            
        file_path = filedialog.askopenfilename(
            initialdir=self.json_dir,
            title="选择要执行的 JSON 剧本",
            filetypes=[("JSON files", "*.json")]
        )
        if not file_path:
            return
            
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                script_data = json.load(f)
        except Exception as e:
            messagebox.showerror("读取失败", f"无法解析 JSON 剧本:\n{e}")
            return
            
        self.script_running = True
        self.btn_load_script.config(state="disabled")
        
        self.live_frames = []
        self.live_traj_x.clear()
        self.live_traj_y.clear()
        self.live_traj_d.clear()
        self.live_traj_z.clear()
        self._open_live_3d_window()
        
        threading.Thread(target=self._execute_script_thread, args=(script_data, os.path.basename(file_path)), daemon=True).start()

    def _open_live_3d_window(self):
        if self.live_3d_window is not None:
            try:
                self.live_3d_window.destroy()
            except Exception:
                pass
            
        self.live_3d_window = tk.Toplevel(self.root)
        self.live_3d_window.title("实时 3D 挖掘机位姿可视化")
        self.live_3d_window.geometry("1000x500")
        
        self.fig = Figure(figsize=(10, 4.5))
        self.fig.suptitle('Real-time Excavator 3D Trajectory', fontsize=14)
        
        self.ax_top = self.fig.add_subplot(121)
        self.ax_side = self.fig.add_subplot(122)
        
        # --- 俯视图 ---
        self.ax_top.set_xlim(-2.0, 2.0)
        self.ax_top.set_ylim(-2.0, 2.0)
        self.ax_top.set_aspect('equal')
        self.ax_top.grid(True)
        self.ax_top.set_title('Top View (Swing X-Y)')
        self.ax_top.plot([0], [0], 'rX', markersize=10)
        
        self.line_top, = self.ax_top.plot([], [], 'o-', lw=4, markersize=6, color='blue')
        self.traj_top, = self.ax_top.plot([], [], 'r-', lw=1.5, alpha=0.6)
        
        # --- 侧视图 ---
        self.ax_side.set_xlim(-0.5, 2.0)
        self.ax_side.set_ylim(-1.0, 1.8)
        self.ax_side.set_aspect('equal')
        self.ax_side.grid(True)
        self.ax_side.set_title('Side View (Profile D-Z)')
        self.ax_side.axhline(0, color='brown', linestyle='--')
        self.ax_side.plot([0], [0], 'rX', markersize=10)
        
        self.line_side, = self.ax_side.plot([], [], 'o-', lw=4, markersize=6, color='green')
        self.traj_side, = self.ax_side.plot([], [], 'r-', lw=1.5, alpha=0.6)
        
        self.fig.tight_layout()
        
        self.live_3d_canvas = FigureCanvasTkAgg(self.fig, master=self.live_3d_window)
        self.live_3d_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _save_live_gif(self, filename, save_dir):
        if not self.live_frames:
            return
        print("[GIF] 开始生成并保存实时运动轨迹 GIF...")
        try:
            fig = Figure(figsize=(10, 4.5))
            fig.suptitle(f'Trajectory: {filename}', fontsize=14)
            
            ax_top = fig.add_subplot(121)
            ax_side = fig.add_subplot(122)
            
            ax_top.set_xlim(-2.0, 2.0)
            ax_top.set_ylim(-2.0, 2.0)
            ax_top.set_aspect('equal')
            ax_top.grid(True)
            ax_top.plot([0], [0], 'rX', markersize=10)
            line_top, = ax_top.plot([], [], 'o-', lw=4, color='blue')
            traj_top, = ax_top.plot([], [], 'r-', lw=1.5, alpha=0.6)
            
            ax_side.set_xlim(-0.5, 2.0)
            ax_side.set_ylim(-1.0, 1.8)
            ax_side.set_aspect('equal')
            ax_side.grid(True)
            ax_side.axhline(0, color='brown', linestyle='--')
            ax_side.plot([0], [0], 'rX', markersize=10)
            line_side, = ax_side.plot([], [], 'o-', lw=4, color='green')
            traj_side, = ax_side.plot([], [], 'r-', lw=1.5, alpha=0.6)
            
            traj_x, traj_y, traj_d, traj_z = [], [], [], []
            
            # 降采样，防止生成 GIF 过大或过慢
            frames_to_render = self.live_frames[::2] if len(self.live_frames) > 200 else self.live_frames
            
            def init():
                line_top.set_data([], [])
                traj_top.set_data([], [])
                line_side.set_data([], [])
                traj_side.set_data([], [])
                return line_top, traj_top, line_side, traj_side
                
            def update(frame_state):
                res = self.kin.forward_kinematics_v4(frame_state['boom_swing'], frame_state['arm_boom'], frame_state['bucket_arm'])
                pts_2d = [
                    (self.kin.offset_x, self.kin.offset_z),
                    res['boom_bend'],
                    res['boom_tip'],
                    res['arm_tip'],
                    res['bucket_tip']
                ]
                yaw_rad = math.radians(frame_state['swing_yaw_deg'])
                pts_3d = [(x * math.cos(yaw_rad), x * math.sin(yaw_rad), z) for x, z in pts_2d]
                
                xs = [p[0] for p in pts_3d]
                ys = [p[1] for p in pts_3d]
                zs = [p[2] for p in pts_3d]
                
                line_top.set_data(xs, ys)
                traj_x.append(xs[-1])
                traj_y.append(ys[-1])
                traj_top.set_data(traj_x, traj_y)
                
                ds_signed = [math.hypot(p[0], p[1]) * (1 if p[0] >= 0 else -1) for p in pts_3d]
                line_side.set_data(ds_signed, zs)
                traj_d.append(ds_signed[-1])
                traj_z.append(zs[-1])
                traj_side.set_data(traj_d, traj_z)
                
                return line_top, traj_top, line_side, traj_side
                
            ani = animation.FuncAnimation(fig, update, frames=frames_to_render, init_func=init, blit=True, interval=50)
            
            if save_dir and os.path.exists(save_dir):
                gif_path = os.path.join(save_dir, f"{os.path.splitext(filename)[0]}_realtime.gif")
            else:
                gif_path = os.path.join(self.json_dir, f"{os.path.splitext(filename)[0]}_realtime.gif")
                
            ani.save(gif_path, writer='pillow', fps=20)
            print(f"[GIF] 实时 3D GIF 已保存至: {gif_path}")
            self.root.after(0, lambda: messagebox.showinfo("保存成功", f"执行过程的实时 3D GIF 已保存至:\n{gif_path}"))
        except Exception as e:
            print(f"[GIF] 保存失败: {e}")

    def _execute_script_thread(self, script_data, filename):
        if not self.angle_ctrl:
            self.root.after(0, lambda: messagebox.showerror("错误", "当前处于离线模式，无法执行控制剧本！"))
            self.script_running = False
            self.root.after(0, lambda: self.btn_load_script.config(state="normal"))
            self.root.after(0, lambda: self.lbl_exec_status.config(text="当前状态: 执行失败 (离线)"))
            return

        try:
            for idx, step in enumerate(script_data):
                if not self.script_running:
                    break
                if getattr(self.angle_ctrl, "fatal_stop", False):
                    fatal_reason = getattr(self.angle_ctrl, "fatal_reason", "已触发越限急停")
                    self.root.after(0, lambda r=fatal_reason: messagebox.showerror("安全急停", f"剧本执行已中断：\n{r}"))
                    break
                    
                step_num = step.get('step', idx + 1)
                joint = step.get('joint', '')
                desc = step.get('description', '')
                
                # 兼容时间回转
                if joint == "swing_yaw":
                    if 'duration_s' in step and 'target_val' not in step:
                        joint = "swing_time"
                        target_val = step.get('duration_s', 0.0)
                    else:
                        target_val = step.get('target_val', 0.0)
                else:
                    target_val = step.get('target_val', 0.0)
                    
                ch1 = step.get('ch1_mv', 0)
                ch2 = step.get('ch2_mv', 0)
                ch3 = step.get('ch3_mv', 2000)
                ramp_up = step.get('ramp_up_s', 0.0)
                ramp_down = step.get('ramp_down_s', 0.0)
                
                is_init = step.get('is_init_step', False)
                if not is_init and step_num <= 3 and ("初始" in desc or "归位" in desc):
                    is_init = True
                    
                # 更新 UI 状态
                status_text = f"正在执行 [{filename}]: 第{step_num}步 {desc} (目标: {target_val})"
                self.root.after(0, lambda t=status_text: self.lbl_exec_status.config(text=t))
                
                # 触发运动
                self.angle_ctrl.move_joint_to_angle(
                    joint, target_val, tolerance=2.0, 
                    ch1_mv=ch1, ch2_mv=ch2, ch3_mv=ch3,
                    ramp_up_s=ramp_up, ramp_down_s=ramp_down,
                    is_init_step=is_init
                )
                
                # 等待运动完成
                time.sleep(0.1)
                while self.angle_ctrl._running_tasks.get(joint, False):
                    if not self.script_running:
                        self.angle_ctrl.stop_all()
                        break
                    if getattr(self.angle_ctrl, "fatal_stop", False):
                        fatal_reason = getattr(self.angle_ctrl, "fatal_reason", "已触发越限急停")
                        self.root.after(0, lambda r=fatal_reason: messagebox.showerror("安全急停", f"剧本执行已中断：\n{r}"))
                        break
                    time.sleep(0.1)
                    
                # 动作之间强制加一个安全间隔 0.3s
                time.sleep(0.3)
                
        except Exception as e:
            print(f"执行异常: {e}")
        finally:
            self.script_running = False
            self.root.after(0, lambda: self.lbl_exec_status.config(text="当前状态: 执行完毕/已停止"))
            self.root.after(0, lambda: self.btn_load_script.config(state="normal"))
            
            save_dir = self.json_dir
            threading.Thread(target=self._save_live_gif, args=(filename, save_dir), daemon=True).start()

    def _save_script(self):
        if self.is_recording:
            messagebox.showwarning("警告", "请先停止录制，再进行保存。")
            return
            
        if not self.recorded_script:
            messagebox.showwarning("提示", "当前没有录制任何动作！")
            return
            
        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            initialdir=self.json_dir,
            title="保存闭环剧本",
            filetypes=[("JSON files", "*.json")]
        )
        
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump(self.recorded_script, f, ensure_ascii=False, indent=2)
                messagebox.showinfo("保存成功", f"成功保存 {len(self.recorded_script)} 步动作到:\n{file_path}")
            except Exception as e:
                messagebox.showerror("保存失败", str(e))

    def _record_current_angle(self, joint_name, label_text, target_var, is_init=False):
        """手动示教：读取当前传感器角度并记录到剧本中"""
        if not self.is_recording:
            messagebox.showwarning("提示", "请先点击下方的『🔴 开始录制剧本』按钮！")
            return
            
        current_val = round(self.current_angles.get(joint_name, 0.0), 1)
        
        # 将当前角度同步显示到输入框中
        target_var.set(current_val)
        
        ch1 = 0
        ch2 = 0
        ch3 = self.ch3_var.get()
        ramp_up = self.ramp_up_var.get()
        ramp_down = self.ramp_down_var.get()
        
        desc = f"{label_text}(手动示教{' - 初始位置' if is_init else ''})"
        
        record_item = {
            "step": len(self.recorded_script) + 1,
            "joint": joint_name,
            "description": desc,
            "ch1_mv": ch1,
            "ch2_mv": ch2,
            "ch3_mv": ch3,
            "ramp_up_s": ramp_up,
            "ramp_down_s": ramp_down,
            "target_val": current_val
        }
        
        if is_init:
            record_item["is_init_step"] = True
                
        self.recorded_script.append(record_item)
        print(f"[示教录制] 已记录: {desc} 当前角度: {current_val}°")

    def _handle_move(self, joint_name, label_text, target_val):
        """处理移动动作并录制剧本"""
        ch1 = 0
        ch2 = 0
        ch3 = self.ch3_var.get()
        ramp_up = self.ramp_up_var.get()
        ramp_down = self.ramp_down_var.get()
        
        if self.is_recording and not getattr(self, 'is_recording_auto', False):
            record_item = {
                "step": len(self.recorded_script) + 1,
                "joint": joint_name,
                "description": label_text,
                "ch1_mv": ch1,
                "ch2_mv": ch2,
                "ch3_mv": ch3,
                "ramp_up_s": ramp_up,
                "ramp_down_s": ramp_down,
                "target_val": target_val
            }
                
            self.recorded_script.append(record_item)
            print(f"[录制] 已记录: {label_text} 参数: {target_val}")
            
        if self.angle_ctrl:
            self.angle_ctrl.move_joint_to_angle(
                joint_name, target_val, tolerance=2.0, 
                ch1_mv=ch1, ch2_mv=ch2, ch3_mv=ch3,
                ramp_up_s=ramp_up, ramp_down_s=ramp_down
            )
        else:
            print(f"[警告] 离线模式：无法移动 {joint_name} 至 {target_val}")

    def _create_ctrl_row(self, parent, row, label_text, joint_name, target_var, entry_label):
        ttk.Label(parent, text=f"{label_text} {entry_label}").grid(row=row, column=0, padx=10, pady=10, sticky="e")
        ttk.Entry(parent, textvariable=target_var, width=15).grid(row=row, column=1, padx=5, pady=10)
        ttk.Button(
            parent, text=f"开始移动 {label_text}", 
            command=lambda: self._handle_move(joint_name, label_text, target_var.get())
        ).grid(row=row, column=2, padx=10, pady=10)
        
        # 对于角度控制的四个关节，添加“记录当前角度”和“记录初始位置”的示教按钮
        ttk.Button(
            parent, text=f"📍 记录当前角度", 
            command=lambda j=joint_name, l=label_text, v=target_var: self._record_current_angle(j, l, v, is_init=False)
        ).grid(row=row, column=3, padx=5, pady=10)
        
        ttk.Button(
            parent, text=f"🏠 记录为初始位置", 
            command=lambda j=joint_name, l=label_text, v=target_var: self._record_current_angle(j, l, v, is_init=True)
        ).grid(row=row, column=4, padx=5, pady=10)

    def _update_loop(self):
        # --- 更新硬件状态 UI ---
        def update_lbl(lbl, prefix, state):
            if state == "connected":
                lbl.config(text=f"{prefix}: ✅ 正常", fg="green")
            elif state == "failed":
                lbl.config(text=f"{prefix}: ❌ 失败", fg="red")
            elif state == "partial":
                lbl.config(text=f"{prefix}: ⚠️ 部分", fg="orange")
            else:
                lbl.config(text=f"{prefix}: ⏳ 等待", fg="blue")

        update_lbl(self.lbl_hw_ctrl, "CAN控制", self.hw_status["controller"])
        update_lbl(self.lbl_hw_sensors, "倾角传感器", self.hw_status["sensors"])
        update_lbl(self.lbl_hw_lidar, "激光雷达", self.hw_status["lidar"])
        update_lbl(self.lbl_hw_cam_hik, "海康相机", self.hw_status["cam_hik"])
        update_lbl(self.lbl_hw_cam1, "网络相机1", self.hw_status["cam1"])
        update_lbl(self.lbl_hw_cam2, "网络相机2", self.hw_status["cam2"])

        # 更新传感器数据给控制器
        if self.angle_ctrl:
            self.angle_ctrl.update_sensor_data(self.sensor_data)
        
        # 更新界面显示 (计算真实相减的夹角，这与 v3 版本相符)
        d = self.sensor_data
        diff_ba = d['铲斗']['pitch'] - d['小臂']['pitch']
        diff_ab = d['小臂']['pitch'] - d['大臂']['pitch']
        diff_bs = d['大臂']['pitch'] - d['回转']['pitch']
        yaw_s = d['回转']['yaw']

        self.lbl_bucket_arm.config(text=f"铲斗-小臂 夹角: {diff_ba:6.1f}°")
        self.lbl_arm_boom.config(text=f"小臂-大臂 夹角: {diff_ab:6.1f}°")
        self.lbl_boom_swing.config(text=f"大臂-回转 夹角: {diff_bs:6.1f}°")
        self.lbl_swing_yaw.config(text=f"回转 偏航角: {yaw_s:6.1f}°")
        
        # 更新当前角度缓存，供示教录制使用
        self.current_angles["bucket_arm"] = diff_ba
        self.current_angles["arm_boom"] = diff_ab
        self.current_angles["boom_swing"] = diff_bs
        self.current_angles["swing_yaw"] = yaw_s
        
        # 如果正在录制，记录历史轨迹以备自动提取剧本
        if self.is_recording and getattr(self, 'is_recording_auto', False):
            ts = time.time()
            self.recording_history["bucket_arm"].append((ts, diff_ba))
            self.recording_history["arm_boom"].append((ts, diff_ab))
            self.recording_history["boom_swing"].append((ts, diff_bs))
            self.recording_history["swing_yaw"].append((ts, yaw_s))

        # 计算铲尖 3D 坐标并缓存给高程图线程
        res = self.kin.forward_kinematics_v4(diff_bs, diff_ab, diff_ba)
        bx_2d, bz = res['bucket_tip']
        yaw_rad = math.radians(yaw_s)
        self.current_bucket_tip_3d = (bx_2d * math.cos(yaw_rad), bx_2d * math.sin(yaw_rad), bz)

        # 记录多模态传感器状态 (10Hz~20Hz 左右)
        ts = time.time()
        yaw_rate = self.sensor_data["回转"].get("yaw_rate", 0.0)
        
        # 发布 ROS 2 JointState
        self.ros_node.publish_joint_state(diff_ab, diff_ba, diff_bs, yaw_s)

        # 之前因为高频更新（10Hz~20Hz）和大量 matplotlib 重绘操作
        # 容易导致 Tkinter 主线程拥堵，进而引发 GUI 界面响应迟钝（一顿一顿的卡顿现象）。
        #
        # 为了解决这个问题，我们对可视化更新频率进行限流降频：
        # 设定 GUI 3D 画布最大更新频率为 10Hz（每 100ms 更新一次）。
        # 注意：这不影响底层数据记录、点云和 ROS TF/话题 的高频（20Hz）发布。
        
        if not hasattr(self, 'last_gui_update_time'):
            self.last_gui_update_time = 0
            
        current_gui_time = time.time()
        
        if self.script_running and (current_gui_time - self.last_gui_update_time >= 0.1):
            self.last_gui_update_time = current_gui_time
            
            self.live_frames.append({
                'boom_swing': diff_bs,
                'arm_boom': diff_ab,
                'bucket_arm': diff_ba,
                'swing_yaw_deg': yaw_s
            })
            if self.live_3d_window and self.live_3d_window.winfo_exists() and self.live_3d_canvas:
                res = self.kin.forward_kinematics_v4(diff_bs, diff_ab, diff_ba)
                pts_2d = [
                    (self.kin.offset_x, self.kin.offset_z),
                    res['boom_bend'],
                    res['boom_tip'],
                    res['arm_tip'],
                    res['bucket_tip']
                ]
                yaw_rad = math.radians(yaw_s)
                pts_3d = [(x * math.cos(yaw_rad), x * math.sin(yaw_rad), z) for x, z in pts_2d]
                
                xs = [p[0] for p in pts_3d]
                ys = [p[1] for p in pts_3d]
                zs = [p[2] for p in pts_3d]
                
                self.line_top.set_data(xs, ys)
                self.live_traj_x.append(xs[-1])
                self.live_traj_y.append(ys[-1])
                
                # 限制轨迹拖尾长度，防止随着时间推移绘图越来越慢
                if len(self.live_traj_x) > 300:
                    self.live_traj_x = self.live_traj_x[-300:]
                    self.live_traj_y = self.live_traj_y[-300:]
                
                self.traj_top.set_data(self.live_traj_x, self.live_traj_y)
                
                ds_signed = [math.hypot(p[0], p[1]) * (1 if p[0] >= 0 else -1) for p in pts_3d]
                
                self.line_side.set_data(ds_signed, zs)
                self.live_traj_d.append(ds_signed[-1])
                self.live_traj_z.append(zs[-1])
                
                # 限制轨迹拖尾长度
                if len(self.live_traj_d) > 300:
                    self.live_traj_d = self.live_traj_d[-300:]
                    self.live_traj_z = self.live_traj_z[-300:]
                    
                self.traj_side.set_data(self.live_traj_d, self.live_traj_z)
                
                self.live_3d_canvas.draw_idle()
                
        # 维持 Tkinter 定时循环，建议主循环保持在 50ms (20Hz) 以满足传感器读取需求
        self.root.after(50, self._update_loop)

    def on_closing(self):
        print("正在关闭...")
        self.is_running = False
            
        # 停止所有拉流线程，防止阻塞退出
        if hasattr(self, 'cams'):
            for cam in self.cams:
                cam.stop()
        
        # 1. 停止角度控制器的线程和控制指令
        if hasattr(self, 'angle_ctrl') and self.angle_ctrl:
            try:
                if hasattr(self.base_controller, 'transport') and getattr(self.base_controller.transport, 'ser', None) and self.base_controller.transport.ser.is_open:
                    self.angle_ctrl.stop_all()
            except Exception as e:
                print(f"关闭时急停异常: {e}")
            
        # 2. 通知所有传感器停止轮询
        for dev in self.devices:
            dev.stopLoopRead()
            
        time.sleep(0.5)
        
        # 强制关闭串口并结束线程
        for dev in self.devices:
            dev.isOpen = False
            dev.closeDevice()
            
        # 关闭高程图线程
        self.elevation_running = False
            
        # 4. 关闭底层 CAN 串口
        if hasattr(self, 'base_controller') and self.base_controller:
            try:
                self.base_controller.close()
            except:
                pass
                
        self.root.destroy()
        
        # 强制结束所有残留守护线程，防止 Ctrl+C 后进程卡住
        os._exit(0)

def main():
    rclpy.init()
    ros_node = Ros2DataPublisher()
    
    # Run tkinter in the main thread and ros spin in a background thread
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    try:
        root = tk.Tk()
        app = V11MultimodalGUI(root, ros_node)
        
        def on_closing():
            app.on_closing()
            ros_node.destroy_node()
            rclpy.shutdown()
            os._exit(0)
            
        root.protocol("WM_DELETE_WINDOW", on_closing)
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 捕捉到退出信号，强制结束程序...")
        os._exit(0)

if __name__ == "__main__":
    main()
