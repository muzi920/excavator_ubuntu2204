import os
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32
from cv_bridge import CvBridge
import threading

# LeRobot 0.4.x 的导入方式可能有所不同，使用核心类
from lerobot.datasets.lerobot_dataset import LeRobotDataset

def setup_dataset(repo_id: str, local_dir: str):
    """
    初始化 LeRobot 数据集环境。
    根据当前挖掘机模型，定义观测 (observation) 和动作 (action) 的特征空间。
    """
    features = {
        # --- 观测状态 (Observation State) ---
        "observation.state": {
            "dtype": "float32",
            "shape": (4,), 
            "names": ["swing_yaw", "boom_pitch", "arm_pitch", "bucket_pitch"]
        },
        
        # --- 视觉观测 (Observation Images) ---
        # 1. 海康相机 (全局覆盖视角)
        "observation.images.hikvision_cam": {
            "dtype": "video",
            "shape": (3, 720, 1280), 
            "names": ["channels", "height", "width"]
        },
        # 2. 网络摄像头 .102 (覆盖工作空间)
        "observation.images.network_cam_102": {
            "dtype": "video",
            "shape": (3, 720, 1280),
            "names": ["channels", "height", "width"]
        },
        # 3. 网络摄像头 .103 (铲斗特写视角)
        "observation.images.bucket_cam_103": {
            "dtype": "video",
            "shape": (3, 720, 1280),
            "names": ["channels", "height", "width"]
        },
        
        # --- 动作 (Action) ---
        "action": {
            "dtype": "float32",
            "shape": (4,),
            "names": ["target_swing", "target_boom", "target_arm", "target_bucket"]
        }
    }

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=10,             # 考虑到多路高分辨率相机，建议采集频率设为 10Hz
        root=local_dir,
        features=features,
        use_videos=True     # 将图像序列保存为 mp4 视频以节省空间并对齐 lerobot 标准
    )
    return dataset

class DatasetCollectorNode(Node):
    """
    ROS 2 节点：负责订阅图像数据并汇总其他传感器状态，
    最后保存到 LeRobot 数据集。
    """
    def __init__(self, dataset):
        super().__init__('lerobot_dataset_collector')
        self.dataset = dataset
        self.bridge = CvBridge()
        
        # 图像缓存 (C, H, W 格式)
        # 初始化为空，或提供一个零矩阵，防止因为掉帧导致的报错
        self.img_hik = np.zeros((3, 720, 1280), dtype=np.uint8)
        self.img_102 = np.zeros((3, 720, 1280), dtype=np.uint8)
        self.img_103 = np.zeros((3, 720, 1280), dtype=np.uint8)
        
        # 初始化角度缓存 (大臂、小臂、铲斗)，默认回转为0
        self.current_boom = 0.0
        self.current_arm = 0.0
        self.current_bucket = 0.0
        self.current_swing = 0.0  # 预留，如果以后有回转传感器可以更新
        
        # 订阅三个相机的 Topic (名称需根据实际 ros2_all_cams_pub.py 发布的 Topic 进行修改)
        self.sub_hik = self.create_subscription(Image, '/hikvision_cam/image_raw', self.hik_callback, 10)
        self.sub_102 = self.create_subscription(Image, '/network_cam/image_raw', self.cam102_callback, 10)
        self.sub_103 = self.create_subscription(Image, '/network_cam2/image_raw', self.cam103_callback, 10)
        
        # 订阅相对角度 Topic
        self.sub_ang = self.create_subscription(Float32MultiArray, '/imu/relative_ang_x', self.ang_callback, 10)
        
        # 订阅回转角度 Topic (由 swing_angle_estimator.py 发布)
        self.sub_swing = self.create_subscription(Float32, '/imu/swing_angle', self.swing_callback, 10)
        
        # 控制采集状态的标志位
        self.is_recording = False
        self.step_count = 0
        self.episode_idx = 0
        
        # 设置采集定时器 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("Dataset Collector Node Started.")
        self.get_logger().info("Please use terminal input to control recording: type 'start' to begin an episode, 'stop' to end it, or 'quit' to exit.")

    def start_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.step_count = 0
            print(f"\n[🔴 RECORDING] 开始录制 Episode {self.episode_idx}...")
        else:
            print("\n[⚠️ WARNING] 已经在录制中了！")

    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            try:
                self.dataset.save_episode()
                print(f"\n[✅ SAVED] Episode {self.episode_idx} 已保存 (共记录 {self.step_count} 帧).")
                self.episode_idx += 1
            except Exception as e:
                print(f"\n[❌ ERROR] 保存 Episode 失败: {e}")
            self.step_count = 0
        else:
            print("\n[⚠️ WARNING] 当前没有在录制数据！")

    def hik_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 确保尺寸严格匹配 (H, W, C) = (720, 1280, 3) 
        if cv_img.shape[:2] != (720, 1280):
            cv_img = cv2.resize(cv_img, (1280, 720))
        # LeRobot 要求格式为 (C, H, W)
        self.img_hik = np.transpose(cv_img, (2, 0, 1))

    def cam102_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if cv_img.shape[:2] != (720, 1280):
            cv_img = cv2.resize(cv_img, (1280, 720))
        self.img_102 = np.transpose(cv_img, (2, 0, 1))

    def cam103_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if cv_img.shape[:2] != (720, 1280):
            cv_img = cv2.resize(cv_img, (1280, 720))
        self.img_103 = np.transpose(cv_img, (2, 0, 1))

    def ang_callback(self, msg):
        # 根据 ros2_readRad_pub.py 第166-167行约定，数据格式为 [大臂相对角, 小臂相对角, 铲斗相对角]
        if len(msg.data) >= 3:
            self.current_boom = msg.data[0]
            self.current_arm = msg.data[1]
            self.current_bucket = msg.data[2]

    def swing_callback(self, msg):
        # 接收基于雷达 IMU 预估并转换到 base_link 下的回转角度
        self.current_swing = msg.data

    def timer_callback(self):
        # 如果没有开始录制，则不采集数据
        if not self.is_recording:
            return

        # 1. 抓取当前物理状态与动作 (由于暂时没有统一底层接口，这里用占位符替代)
        # 根据特征字典定义的顺序 ["swing_yaw", "boom_pitch", "arm_pitch", "bucket_pitch"]
        current_state = np.array([
            self.current_swing,
            self.current_boom,
            self.current_arm,
            self.current_bucket
        ], dtype=np.float32)
        
        # 动作暂时用0占位，等有下发指令的接口时再替换
        current_action = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        
        # 2. 将数据推入 Dataset
        try:
            self.dataset.add_frame({
                "observation.state": current_state,
                "observation.images.hikvision_cam": self.img_hik,
                "observation.images.network_cam_102": self.img_102,
                "observation.images.bucket_cam_103": self.img_103,
                "action": current_action,
                "task": "digging" # 在最新版本 LeRobot 中，添加帧可能需要提供当前任务说明标签
            })
            self.step_count += 1
            
            # 打印简易进度 (每 10 帧即 1 秒打印一次)
            if self.step_count % 10 == 0:
                print(f"  ... 录制中: {self.step_count} 帧 ({self.step_count/10.0:.1f} 秒)")
                
        except Exception as e:
            self.get_logger().error(f"Error adding frame: {e}")

def terminal_input_thread(node):
    """
    独立线程用于监听终端输入
    """
    import sys
    print("\n" + "="*50)
    print(" 🎬 LeRobot 数据采集控制器 🎬 ")
    print(" - 输入 'start' : 开始录制一个 Episode")
    print(" - 输入 'stop'  : 结束当前 Episode 并保存")
    print(" - 输入 'quit'  : 退出程序")
    print("="*50 + "\n")
    
    while rclpy.ok():
        try:
            cmd = input("").strip().lower()
            if cmd == 'start':
                node.start_recording()
            elif cmd == 'stop':
                node.stop_recording()
            elif cmd == 'quit':
                print(">>> 准备退出程序...")
                # 触发关闭
                if node.is_recording:
                    node.stop_recording()
                
                # 退出 ROS2
                rclpy.shutdown()
                break
            elif cmd != '':
                print("未知指令，请输入 'start', 'stop' 或 'quit'.")
        except EOFError:
            break
        except Exception as e:
            print(f"输入异常: {e}")
            break

def main():
    # 数据集存放的本地目录
    local_dataset_dir = os.path.join(os.path.dirname(__file__), "data", "excavator_dataset")
    
    # 如果目录已存在，说明可能之前创建过了
    # 在 LeRobot 中，对于已存在的空数据集或者未 consolidate 的数据，可以选择覆盖或删除
    import shutil
    if os.path.exists(local_dataset_dir):
        shutil.rmtree(local_dataset_dir)
        print(f">>> 清理已存在的数据集目录: {local_dataset_dir}")
        
    print(">>> 正在初始化 LeRobot 数据集环境...")
    # 由于 lerobot 内部包结构变更，如果在导入时报错，我们在此捕获并给与提示
    try:
        dataset = setup_dataset(
            repo_id="local/excavator_teleop", 
            local_dir=local_dataset_dir
        )
        print(f">>> 数据集初始化成功，保存在: {local_dataset_dir}")
        
        import sys
        # 防止 ROS2 log 文件系统只读报错
        os.environ['ROS_LOG_DIR'] = '/tmp/ros2_logs'
        rclpy.init(args=sys.argv)
        collector_node = DatasetCollectorNode(dataset)
        
        # 启动终端输入监听线程
        input_thread = threading.Thread(target=terminal_input_thread, args=(collector_node,), daemon=True)
        input_thread.start()
        
        try:
            rclpy.spin(collector_node)
        except KeyboardInterrupt:
            print("\n>>> 用户中断，停止采集。")
        finally:
            collector_node.destroy_node()
            rclpy.shutdown()
            
            print(">>> 正在整合并写入数据集文件...")
            dataset.consolidate()
            print(">>> LeRobot 数据集构建完成！")
            
    except ImportError as e:
        print(f"LeRobot 导入失败: {e}")
        print("请检查 LeRobot 的版本或安装路径是否正确。部分最新版本 API 可能变动。")

if __name__ == "__main__":
    main()