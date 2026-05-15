import os
import sys
import time
import threading
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# 引入我们刚才封装好的模块
from inclinometer_reader import InclinometerReader
from camera_reader import MultiCameraReader

def setup_dataset(repo_id: str, local_dir: str):
    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (4,), 
            "names": ["swing_yaw", "boom_pitch", "arm_pitch", "bucket_pitch"]
        },
        "observation.images.hikvision_cam": {
            "dtype": "video",
            "shape": (3, 720, 1280), 
            "names": ["channels", "height", "width"]
        },
        "observation.images.network_cam_102": {
            "dtype": "video",
            "shape": (3, 720, 1280),
            "names": ["channels", "height", "width"]
        },
        "observation.images.bucket_cam_103": {
            "dtype": "video",
            "shape": (3, 720, 1280),
            "names": ["channels", "height", "width"]
        },
        "action": {
            "dtype": "float32",
            "shape": (4,),
            "names": ["target_swing", "target_boom", "target_arm", "target_bucket"]
        }
    }

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=10,             
        root=local_dir,
        features=features,
        use_videos=True     
    )
    return dataset

class DirectCollector:
    def __init__(self, dataset):
        self.dataset = dataset
        self.fps = 10
        self.dt = 1.0 / self.fps
        
        # 初始化传感器读取模块
        self.inclinometer = InclinometerReader()
        self.cameras = MultiCameraReader(target_width=1280, target_height=720)
        
        self.is_recording = False
        self.episode_idx = 0
        self.step_count = 0
        
        # 回转角度暂无直接底层读取接口，预留 0.0
        self.current_swing = 0.0 
        
        self.running = False

    def start_hardware(self):
        print(">>> 正在启动硬件读取模块...")
        self.inclinometer.start()
        self.cameras.start()
        
        # 等待传感器和相机初始化 (获取第一帧)
        print(">>> 等待传感器和相机缓冲初始化 (3秒)...")
        time.sleep(3.0)

    def stop_hardware(self):
        self.inclinometer.stop()
        self.cameras.stop()

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

    def collect_loop(self):
        self.running = True
        
        # 提供给如果没有图像时的纯黑兜底画面
        blank_img = np.zeros((3, 720, 1280), dtype=np.uint8)
        
        while self.running:
            start_time = time.time()
            
            if self.is_recording:
                # 1. 获取最新角度
                boom_rel, arm_rel, bucket_rel = self.inclinometer.get_relative_angles()
                if boom_rel is None:
                    boom_rel, arm_rel, bucket_rel = 0.0, 0.0, 0.0
                    
                current_state = np.array([
                    self.current_swing,
                    boom_rel,
                    arm_rel,
                    bucket_rel
                ], dtype=np.float32)
                
                # 2. 获取最新画面
                frames = self.cameras.get_frames()
                
                def process_frame(frame):
                    if frame is None:
                        return blank_img
                    # LeRobot 要求格式为 (C, H, W)
                    return np.transpose(frame, (2, 0, 1))
                
                img_hik = process_frame(frames["hikvision"])
                img_102 = process_frame(frames["net_102"])
                img_103 = process_frame(frames["net_103"])
                
                # 3. 动作暂位
                current_action = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
                
                # 4. 压入数据集
                try:
                    self.dataset.add_frame({
                        "observation.state": current_state,
                        "observation.images.hikvision_cam": img_hik,
                        "observation.images.network_cam_102": img_102,
                        "observation.images.bucket_cam_103": img_103,
                        "action": current_action,
                        "task": "digging"
                    })
                    self.step_count += 1
                    
                    if self.step_count % 10 == 0:
                        print(f"  ... 录制中: {self.step_count} 帧 ({self.step_count/10.0:.1f} 秒)")
                except Exception as e:
                    print(f"[ERROR] 压入数据帧失败: {e}")
            
            # 精确延时控制 (10Hz)
            elapsed = time.time() - start_time
            if elapsed < self.dt:
                time.sleep(self.dt - elapsed)

def terminal_input_thread(collector):
    print("\n" + "="*50)
    print(" 🎬 LeRobot 直连底层数据采集控制器 🎬 ")
    print(" - 输入 'start' : 开始录制一个 Episode")
    print(" - 输入 'stop'  : 结束当前 Episode 并保存")
    print(" - 输入 'quit'  : 退出程序")
    print("="*50 + "\n")
    
    while True:
        try:
            cmd = input("").strip().lower()
            if cmd == 'start':
                collector.start_recording()
            elif cmd == 'stop':
                collector.stop_recording()
            elif cmd == 'quit':
                print(">>> 准备退出程序...")
                if collector.is_recording:
                    collector.stop_recording()
                collector.running = False
                break
            elif cmd != '':
                print("未知指令，请输入 'start', 'stop' 或 'quit'.")
        except Exception as e:
            print(f"输入异常: {e}")
            break

def main():
    local_dataset_dir = os.path.join(os.path.dirname(__file__), "data", "excavator_dataset")
    
    import shutil
    if os.path.exists(local_dataset_dir):
        shutil.rmtree(local_dataset_dir)
        print(f">>> 清理已存在的数据集目录: {local_dataset_dir}")
        
    print(">>> 正在初始化 LeRobot 数据集环境...")
    dataset = setup_dataset("local/excavator_teleop", local_dataset_dir)
    print(f">>> 数据集初始化成功，保存在: {local_dataset_dir}")
    
    collector = DirectCollector(dataset)
    
    # 启动硬件读取
    collector.start_hardware()
    
    # 启动终端监听
    input_thread = threading.Thread(target=terminal_input_thread, args=(collector,), daemon=True)
    input_thread.start()
    
    try:
        # 在主线程中运行采集循环
        collector.collect_loop()
    except KeyboardInterrupt:
        print("\n>>> 用户中断，停止采集。")
    finally:
        collector.running = False
        collector.stop_hardware()
        print(">>> 正在整合并写入数据集文件...")
        dataset.consolidate()
        print(">>> LeRobot 数据集构建完成！")

if __name__ == "__main__":
    main()
