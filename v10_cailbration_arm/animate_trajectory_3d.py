import json
import math
import os
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

import sys
from kinematics import ExcavatorKinematics

class TrajectoryAnimator3D:
    def __init__(self, json_path):
        self.json_path = json_path
        self.kin = ExcavatorKinematics()

    def fk_3d(self, boom_swing, arm_boom, bucket_arm, swing_yaw_deg):
        """计算各关节的 3D 坐标点"""
        # 1. 计算 2D 坐标 (X, Z)
        res = self.kin.forward_kinematics_v4(boom_swing, arm_boom, bucket_arm)
        pts_2d = [
            (self.kin.offset_x, self.kin.offset_z),
            res['boom_bend'],
            res['boom_tip'],
            res['arm_tip'],
            res['bucket_tip']
        ]
        
        # 2. 绕 Z 轴旋转生成 3D 坐标
        # 设定：正角度为向左(逆时针)，负角度为向右(顺时针)
        yaw_rad = math.radians(swing_yaw_deg)
        pts_3d = []
        for x, z in pts_2d:
            x_3d = x * math.cos(yaw_rad)
            y_3d = x * math.sin(yaw_rad)
            pts_3d.append((x_3d, y_3d, z))
            
        return pts_3d

    def generate_frames(self):
        with open(self.json_path, 'r') as f:
            data = json.load(f)
            
        # 初始状态
        state = {'boom_swing': 0.7, 'arm_boom': -1.0, 'bucket_arm': -92.7, 'swing_yaw_deg': 0.0}
        
        # 提取初始化配置
        for step in data:
            if step.get('is_init_step'):
                j = step.get('joint')
                v = step.get('target_val')
                if j in state and v is not None:
                    state[j] = v

        frames = [dict(state)]
        interp_steps = 10 
        
        # 提取前若干步用于动画展示，避免 GIF 过大
        max_steps_to_show = min(len(data), 150)
        
        for step in data[3:max_steps_to_show]:
            j = step.get('joint')
            
            if j == "swing_yaw":
                # 解析回转时间到角度：CH3=3000 时，2.5s 约等于 90度 => 36度/s
                duration = step.get('duration_s', step.get('target_val', 0))
                
                # 挖掘机控制约定：正时间=向右转，负时间=向左转
                # 数学坐标系约定：向右转(顺时针)角度减小
                delta_angle = -duration * 36.0
                
                start_val = state['swing_yaw_deg']
                end_val = start_val + delta_angle
                
                for i in range(1, interp_steps + 1):
                    state['swing_yaw_deg'] = start_val + (end_val - start_val) * (i / interp_steps)
                    frames.append(dict(state))
            else:
                v = step.get('target_val')
                if j in state and v is not None:
                    start_val = state[j]
                    end_val = v
                    for i in range(1, interp_steps + 1):
                        state[j] = start_val + (end_val - start_val) * (i / interp_steps)
                        frames.append(dict(state))
                        
        return frames

    def animate(self, output_path):
        frames_data = self.generate_frames()
        
        fig, (ax_top, ax_side) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle('Excavator 3D Trajectory (Dual View)', fontsize=16)
        
        # --- 左侧：俯视图 (Top View, X-Y 平面) ---
        ax_top.set_xlim(-2.0, 2.0)
        ax_top.set_ylim(-2.0, 2.0)
        ax_top.set_aspect('equal')
        ax_top.grid(True)
        ax_top.set_title('Top View (Swing X-Y)')
        ax_top.set_xlabel('X (Forward) [m]')
        ax_top.set_ylabel('Y (Left) [m]')
        ax_top.plot([0], [0], 'rX', markersize=10, label='Swing Center')
        
        line_top, = ax_top.plot([], [], 'o-', lw=4, markersize=6, color='blue', label='Arm')
        traj_top, = ax_top.plot([], [], 'r-', lw=1.5, alpha=0.6, label='Bucket Tip')
        ax_top.legend(loc='upper left')
        
        # --- 右侧：侧视图 (Side View, Distance-Z 平面) ---
        ax_side.set_xlim(-0.5, 2.0)
        ax_side.set_ylim(-1.0, 1.8)
        ax_side.set_aspect('equal')
        ax_side.grid(True)
        ax_side.set_title('Side View (Profile D-Z)')
        ax_side.set_xlabel('Distance from Center [m]')
        ax_side.set_ylabel('Z (Upward) [m]')
        ax_side.axhline(0, color='brown', linestyle='--', label='Ground Level')
        ax_side.plot([0], [0], 'rX', markersize=10, label='Swing Center')
        
        line_side, = ax_side.plot([], [], 'o-', lw=4, markersize=6, color='green', label='Arm Profile')
        traj_side, = ax_side.plot([], [], 'r-', lw=1.5, alpha=0.6, label='Bucket Tip')
        ax_side.legend(loc='upper left')
        
        trajectory_x, trajectory_y, trajectory_z, trajectory_d = [], [], [], []

        def init():
            line_top.set_data([], [])
            traj_top.set_data([], [])
            line_side.set_data([], [])
            traj_side.set_data([], [])
            return line_top, traj_top, line_side, traj_side

        def update(frame_state):
            pts = self.fk_3d(frame_state['boom_swing'], frame_state['arm_boom'], frame_state['bucket_arm'], frame_state['swing_yaw_deg'])
            
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            zs = [p[2] for p in pts]
            
            # 计算平面上的投影距离 (用于侧视图)
            # 因为挖掘机总是朝向它当前回转的角度，所以直接用 sqrt(x^2 + y^2) 作为前伸距离
            ds = [math.hypot(p[0], p[1]) for p in pts]
            # 但是考虑到基座偏移，为了防止折叠错乱，我们直接保留带符号的投影距离
            # 这里简单起见，侧视图直接展示沿着当前悬臂方向的切面展开
            
            # 更新俯视图
            line_top.set_data(xs, ys)
            trajectory_x.append(xs[-1])
            trajectory_y.append(ys[-1])
            traj_top.set_data(trajectory_x, trajectory_y)
            
            # 更新侧视图
            # 为了侧视图物理上总是合理的，我们把大臂旋转到统一的平面
            # 实际上直接使用最初计算出来的未经 Yaw 旋转的 2D 坐标是最完美的切面图
            # 但既然我们已经算出了 3D，可以把 3D 旋转回去得到切面
            # 其实我们可以直接调用一次 2D fk
            # 简单起见，我们重新算一下 ds，保证正负号逻辑
            sign = 1 if frame_state['swing_yaw_deg'] > -90 and frame_state['swing_yaw_deg'] < 90 else -1
            ds_signed = [math.hypot(p[0], p[1]) * (1 if p[0] >= 0 else -1) for p in pts]
            
            line_side.set_data(ds_signed, zs)
            trajectory_d.append(ds_signed[-1])
            trajectory_z.append(zs[-1])
            traj_side.set_data(trajectory_d, trajectory_z)
            
            return line_top, traj_top, line_side, traj_side

        ani = animation.FuncAnimation(
            fig, update, frames=frames_data,
            init_func=init, blit=True, interval=50
        )
        
        print(f"正在保存动画到 {output_path} ...")
        ani.save(output_path, writer='pillow', fps=20)
        print("动画保存完成！")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="生成挖掘机 3D 运动轨迹 GIF 动画")
    parser.add_argument("--json", type=str, required=True, help="V4 剧本 JSON 文件路径")
    args = parser.parse_args()
    
    json_file = args.json
    if not os.path.exists(json_file) and not os.path.isabs(json_file):
        shandong_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        json_file = os.path.join(shandong_root, args.json)
    
    if not os.path.exists(json_file):
        print(f"错误: 找不到文件 {json_file}")
        exit(1)
        
    base_name = os.path.splitext(os.path.basename(json_file))[0]
    output_dir = os.path.dirname(os.path.abspath(__file__))
    output_gif = os.path.join(output_dir, f"{base_name}_3d.gif")
    
    animator = TrajectoryAnimator3D(json_file)
    animator.animate(output_gif)
