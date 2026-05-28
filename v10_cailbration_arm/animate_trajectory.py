import json
import math
import os
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

import sys
from kinematics import ExcavatorKinematics

class TrajectoryAnimator:
    def __init__(self, json_path):
        self.json_path = json_path
        self.kin = ExcavatorKinematics()

    def generate_frames(self):
        with open(self.json_path, 'r') as f:
            data = json.load(f)
            
        # 初始化状态
        state = {'boom_swing': 0.7, 'arm_boom': -1.0, 'bucket_arm': -92.7}
        
        # 提取初始值 (前三步)
        for step in data:
            if step.get('is_init_step'):
                j = step.get('joint')
                v = step.get('target_val')
                if j in state and v is not None:
                    state[j] = v

        frames = []
        # 将初始状态加入
        frames.append(dict(state))
        
        # 插值帧数 (每个动作生成几帧，使得动画平滑)
        interp_steps = 5 
        
        # 我们只提取前50步(约5轮动作)进行动画展示，避免GIF过大
        max_steps_to_show = min(len(data), 60)
        
        for step in data[3:max_steps_to_show]:
            j = step.get('joint')
            v = step.get('target_val')
            
            if j in state and v is not None:
                start_val = state[j]
                end_val = v
                # 线性插值
                for i in range(1, interp_steps + 1):
                    current_val = start_val + (end_val - start_val) * (i / interp_steps)
                    state[j] = current_val
                    frames.append(dict(state))
                    
        return frames

    def animate(self, output_path):
        frames_data = self.generate_frames()
        
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_xlim(-1.0, 2.0)
        ax.set_ylim(-1.5, 1.5)
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title('Excavator Trajectory Animation')
        ax.set_xlabel('X (Forward) [m]')
        ax.set_ylabel('Z (Upward) [m]')
        
        # 画地平线
        ax.axhline(0, color='brown', linestyle='--', label='Ground Level')
        # 画原点
        ax.plot(0, 0, 'rX', markersize=10, label='Swing Center')
        
        line, = ax.plot([], [], 'o-', lw=4, markersize=8, label='Excavator Arm')
        trajectory_line, = ax.plot([], [], 'r-', lw=1.5, alpha=0.6, label='Bucket Tip Trajectory')
        
        ax.legend(loc='upper left')
        
        trajectory_x = []
        trajectory_z = []

        def init():
            line.set_data([], [])
            trajectory_line.set_data([], [])
            return line, trajectory_line

        def update(frame_state):
            res = self.kin.forward_kinematics_v4(frame_state['boom_swing'], frame_state['arm_boom'], frame_state['bucket_arm'])
            pts = [
                (self.kin.offset_x, self.kin.offset_z),
                res['boom_bend'],
                res['boom_tip'],
                res['arm_tip'],
                res['bucket_tip']
            ]
            x_vals = [p[0] for p in pts]
            z_vals = [p[1] for p in pts]
            
            line.set_data(x_vals, z_vals)
            
            # 记录铲尖轨迹
            trajectory_x.append(pts[-1][0])
            trajectory_z.append(pts[-1][1])
            trajectory_line.set_data(trajectory_x, trajectory_z)
            
            return line, trajectory_line

        ani = animation.FuncAnimation(
            fig, update, frames=frames_data,
            init_func=init, blit=True, interval=50 # 50ms per frame
        )
        
        print(f"正在保存动画到 {output_path} ...")
        ani.save(output_path, writer='pillow', fps=20)
        print("动画保存完成！")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="生成挖掘机运动轨迹 GIF 动画")
    parser.add_argument(
        "--json", 
        type=str, 
        required=True, 
        help="V4 控制器的剧本 JSON 文件路径 (支持相对于 src/shandong 目录的路径或绝对路径)"
    )
    args = parser.parse_args()
    
    json_file = args.json
    
    # 尝试解析相对路径：如果输入的文件不存在，且不是绝对路径，
    # 假设它是相对于 `src/shandong` 根目录的路径
    if not os.path.exists(json_file) and not os.path.isabs(json_file):
        # 当前脚本在 src/shandong/v10_calibration/ 下
        # 所以 shandong 根目录是 os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        shandong_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        json_file = os.path.join(shandong_root, args.json)
    
    if not os.path.exists(json_file):
        print(f"错误: 找不到文件 {json_file} (也尝试了相对于 shandong 根目录的路径)")
        exit(1)
        
    # 获取与 JSON 同名的 GIF 输出路径
    base_name = os.path.splitext(os.path.basename(json_file))[0]
    output_dir = os.path.dirname(os.path.abspath(__file__))
    output_gif = os.path.join(output_dir, f"{base_name}.gif")
    
    animator = TrajectoryAnimator(json_file)
    animator.animate(output_gif)
