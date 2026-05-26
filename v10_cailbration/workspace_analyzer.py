import math
import numpy as np
import matplotlib.pyplot as plt
import os
from kinematics import ExcavatorKinematics

class WorkspaceAnalyzer:
    """
    挖掘机工作空间分析类
    基于物理模型和各关节夹角的运动限位，计算并绘制铲尖可达的二维工作区域。
    """
    def __init__(self):
        self.kin = ExcavatorKinematics()

    def calculate_forward_kinematics_from_relative(self, boom_swing, arm_boom, bucket_arm):
        """
        基于 V4 控制器的相对角计算铲尖绝对坐标
        """
        res = self.kin.forward_kinematics_v4(boom_swing, arm_boom, bucket_arm)
        return res['bucket_tip'][0], res['bucket_tip'][1]

    def generate_point_cloud(self, resolution=2.0):
        """
        穷举限位范围内的所有组合，生成工作空间点云
        :param resolution: 角度遍历步长(度)
        """
        points = []
        
        # 使用基于真实工作场景的限位 (test3_generated_30.json 中提取的极限)
        # 大臂: 0.7 到 47.3
        # 小臂: -1.0 到 57.1
        # 铲斗: -92.7 到 5.7
        for boom_swing in np.arange(0.7, 47.3, resolution):
            for arm_boom in np.arange(-1.0, 57.1, resolution):
                for bucket_arm in np.arange(-92.7, 5.7, resolution):
                    
                    res = self.kin.forward_kinematics_v4(boom_swing, arm_boom, bucket_arm)
                    points.append(res['bucket_tip'])
                    
        return points

    def plot_workspace(self, points):
        """
        使用 matplotlib 绘制并保存工作空间图
        """
        x_vals = [p[0] for p in points]
        z_vals = [p[1] for p in points]
        
        plt.figure(figsize=(10, 8))
        plt.scatter(x_vals, z_vals, s=1, alpha=0.5, c=z_vals, cmap='viridis')
        
        # 绘制原点(大臂底座)
        plt.plot(0, 0, 'r+', markersize=15, label='Base Pivot (0,0)')
        
        plt.title('Excavator 2D Workspace (Bucket Tip)')
        plt.xlabel('X (Forward) [m]')
        plt.ylabel('Z (Upward) [m]')
        plt.axis('equal')
        plt.grid(True)
        plt.legend(loc='upper left')
        
        save_path = "/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v10_cailbration/workspace_plot.png"
        plt.savefig(save_path)
        print(f"工作空间图表已保存至: {save_path}")
        
        # 计算边界
        print(f"\n--- 工作空间边界分析 ---")
        print(f"最大前伸距离 (Max X): {max(x_vals):.3f} m")
        print(f"最小收缩距离 (Min X): {min(x_vals):.3f} m")
        print(f"最高挖掘高度 (Max Z): {max(z_vals):.3f} m")
        print(f"最大挖掘深度 (Min Z): {min(z_vals):.3f} m")

if __name__ == "__main__":
    analyzer = WorkspaceAnalyzer()
    points = analyzer.generate_point_cloud(resolution=3) # 步长3度
    analyzer.plot_workspace(points)
