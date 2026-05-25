import math
import numpy as np
import matplotlib.pyplot as plt

class WorkspaceAnalyzer:
    """
    挖掘机工作空间分析类
    基于物理模型和各关节夹角的运动限位，计算并绘制铲尖可达的二维工作区域。
    """
    def __init__(self):
        # 坐标系偏移 (相对于回转中心地面投影点)
        self.offset_x = -0.25
        self.offset_y = 0.0
        self.offset_z = 0.40
        
        # 物理参数 (米)
        self.L1 = 0.35
        self.L2 = 0.60
        self.boom_bend_angle_deg = 46.0
        
        self.L_arm = 0.44
        self.L_bucket = 0.26
        
        # 计算大臂等效长度和偏置角
        inner_angle_rad = math.radians(180.0 - self.boom_bend_angle_deg)
        self.L_boom = math.sqrt(self.L1**2 + self.L2**2 - 2 * self.L1 * self.L2 * math.cos(inner_angle_rad))
        sin_beta = (self.L1 * math.sin(inner_angle_rad)) / self.L_boom
        self.beta_deg = math.degrees(math.asin(sin_beta))
        
        # 根据 test3_generated_30.json 中实际运动范围更新限位
        # 注意：这里的范围基于剧本中的 target_val
        # boom_swing (大臂pitch)
        self.boom_limit = (0.7, 47.3)
        # arm_boom (小臂pitch - 大臂pitch)
        self.arm_limit = (-1.0, 57.1)
        # bucket_arm (铲斗pitch - 小臂pitch)
        self.bucket_limit = (-92.7, 5.7)

    def calculate_forward_kinematics_from_relative(self, boom_swing, arm_boom, bucket_arm):
        """
        基于相对夹角计算绝对坐标
        在 v4 控制器中：
        1. 角度定义: boom_swing = 大臂pitch, arm_boom = 小臂pitch - 大臂pitch, bucket_arm = 铲斗pitch - 小臂pitch
        2. 极性定义: 【向下/向内 运动时，pitch数值增大（为正）】。
           - 大臂降下，boom_swing 变大 (-5 到 55)
           - 小臂回收，arm_boom 变大 (-5 到 95)
           - 铲斗回收，bucket_arm 变大 (-95 到 20)
        
        因此绝对角度传感器读数 (向下为正) 为：
        大臂绝对角 = boom_swing
        小臂绝对角 = arm_boom + boom_swing
        铲斗绝对角 = bucket_arm + 小臂绝对角
        """
        # 1. 相对角推导绝对角 (向下为正的传感器读数)
        # 根据 V4 逻辑，arm_boom 和 bucket_arm 都是直接相减得到的 diff
        # diff_ab = 小臂pitch - 大臂pitch => 小臂pitch = 大臂pitch + diff_ab
        # diff_ba = 铲斗pitch - 小臂pitch => 铲斗pitch = 小臂pitch + diff_ba
        sensor_boom_deg = boom_swing
        sensor_arm_deg = sensor_boom_deg + arm_boom
        sensor_bucket_deg = sensor_arm_deg + bucket_arm
        
        # 2. 转换为标准几何系 (向上为正)
        # 加上大臂传感器的物理标定偏置：读数为0时，L2实际几何角度为 +40度
        abs_boom_L2_deg = 40.0 - sensor_boom_deg
        abs_arm_deg = -sensor_arm_deg  # 暂未标定偏置
        abs_bucket_deg = -sensor_bucket_deg  # 暂未标定偏置

        # 3. 结合物理偏置计算实际连杆的绝对角度
        # 大臂等效直线的角度 = 几何角L2 + beta (因为连线在L2上方)
        theta1 = math.radians(abs_boom_L2_deg + self.beta_deg)
        # 小臂和铲斗的传感器我们假设是平行于连杆安装的
        # 但需要注意，当小臂垂直向下时，传感器读数可能是 90度（向下为正），标准几何系就是 -90度
        # 我们这里先假设传感器的 0度 就是水平向前
        theta2 = math.radians(abs_arm_deg)
        theta3 = math.radians(abs_bucket_deg)
        
        # 3. 坐标解算 (加上底座偏移)
        x1 = self.offset_x + self.L_boom * math.cos(theta1)
        z1 = self.offset_z + self.L_boom * math.sin(theta1)
        
        x2 = x1 + self.L_arm * math.cos(theta2)
        z2 = z1 + self.L_arm * math.sin(theta2)
        
        x3 = x2 + self.L_bucket * math.cos(theta3)
        z3 = z2 + self.L_bucket * math.sin(theta3)
        
        return x3, z3

    def generate_workspace(self, resolution=5):
        """
        穷举限位范围内的所有组合，生成工作空间点云
        :param resolution: 角度遍历步长
        """
        points = []
        
        # 遍历所有可能的夹角组合
        for boom in np.arange(self.boom_limit[0], self.boom_limit[1] + resolution, resolution):
            for arm in np.arange(self.arm_limit[0], self.arm_limit[1] + resolution, resolution):
                for bucket in np.arange(self.bucket_limit[0], self.bucket_limit[1] + resolution, resolution):
                    x, z = self.calculate_forward_kinematics_from_relative(boom, arm, bucket)
                    points.append((x, z))
                    
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
        plt.legend()
        
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
    points = analyzer.generate_workspace(resolution=3) # 步长3度
    analyzer.plot_workspace(points)
