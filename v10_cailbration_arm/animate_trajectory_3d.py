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
                if 'duration_s' in step:
                    # 兼容老版本：解析回转时间到角度：CH3=3000 时，2.5s 约等于 90度 => 36度/s
                    duration = step['duration_s']
                    delta_angle = -duration * 36.0
                    end_val = state['swing_yaw_deg'] + delta_angle
                else:
                    # V11 新版本 JSON 直接给出了绝对目标角度 target_val
                    end_val = step.get('target_val', state['swing_yaw_deg'])
                
                start_val = state['swing_yaw_deg']
                
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
        
        line_boom_top, = ax_top.plot([], [], 'o-', lw=4, markersize=6, color='#1f77b4', label='Boom')
        line_arm_top, = ax_top.plot([], [], 'o-', lw=4, markersize=6, color='#2ca02c', label='Arm')
        line_bucket_top, = ax_top.plot([], [], 'o-', lw=4, markersize=6, color='#ff7f0e', label='Bucket')
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
        
        line_boom_side, = ax_side.plot([], [], 'o-', lw=4, markersize=6, color='#1f77b4', label='Boom')
        line_arm_side, = ax_side.plot([], [], 'o-', lw=4, markersize=6, color='#2ca02c', label='Arm')
        line_bucket_side, = ax_side.plot([], [], 'o-', lw=4, markersize=6, color='#ff7f0e', label='Bucket')
        traj_side, = ax_side.plot([], [], 'r-', lw=1.5, alpha=0.6, label='Bucket Tip')
        ax_side.legend(loc='upper left')
        
        trajectory_x, trajectory_y, trajectory_z, trajectory_d = [], [], [], []

        def init():
            line_boom_top.set_data([], [])
            line_arm_top.set_data([], [])
            line_bucket_top.set_data([], [])
            traj_top.set_data([], [])
            line_boom_side.set_data([], [])
            line_arm_side.set_data([], [])
            line_bucket_side.set_data([], [])
            traj_side.set_data([], [])
            return line_boom_top, line_arm_top, line_bucket_top, traj_top, line_boom_side, line_arm_side, line_bucket_side, traj_side

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
            line_boom_top.set_data(xs[0:3], ys[0:3])
            line_arm_top.set_data(xs[2:4], ys[2:4])
            line_bucket_top.set_data(xs[3:5], ys[3:5])
            
            trajectory_x.append(xs[-1])
            trajectory_y.append(ys[-1])
            traj_top.set_data(trajectory_x, trajectory_y)
            
            # 更新侧视图
            # 侧视图只关心挖掘机大臂向前伸展的“水平距离”和“高度Z”
            # 因为挖掘机回转时，大臂始终在它自身的对称面上，所以在侧切面图中，我们应该直接使用
            # 原始正向动力学计算出的 2D X 坐标。
            # 直接使用 math.hypot() 会丢失基座偏移(offset_x)的相对关系。
            # 我们重新调用一次 2D fk_2d 获取绝对正确的前伸距离 (全部为正向)
            res_2d = self.kin.forward_kinematics_v4(frame_state['boom_swing'], frame_state['arm_boom'], frame_state['bucket_arm'])
            pts_2d_only = [
                (self.kin.offset_x, self.kin.offset_z),
                res_2d['boom_bend'],
                res_2d['boom_tip'],
                res_2d['arm_tip'],
                res_2d['bucket_tip']
            ]
            
            ds_signed = [p[0] for p in pts_2d_only]
            
            line_boom_side.set_data(ds_signed[0:3], zs[0:3])
            line_arm_side.set_data(ds_signed[2:4], zs[2:4])
            line_bucket_side.set_data(ds_signed[3:5], zs[3:5])
            
            trajectory_d.append(ds_signed[-1])
            trajectory_z.append(zs[-1])
            traj_side.set_data(trajectory_d, trajectory_z)
            
            return line_boom_top, line_arm_top, line_bucket_top, traj_top, line_boom_side, line_arm_side, line_bucket_side, traj_side

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
