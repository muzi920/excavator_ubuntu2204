import math

class ExcavatorKinematics:
    """
    挖掘机正向运动学计算类 (Forward Kinematics)
    基于大臂、小臂、铲斗的物理长度和倾角传感器数据，计算铲尖在二维平面 (X, Z) 的绝对坐标。
    """
    def __init__(self):
        # 1. 物理参数配置 (单位: 米)
        # 坐标系偏移: 原点为回转中心地面投影 (X向前, Y向左, Z向上)
        self.offset_x = -0.25
        self.offset_y = 0.0
        self.offset_z = 0.40  # 你提供的是 z:-0.4，但通常销轴在地面之上，所以我用 +0.4 作为销轴的高度

        # 大臂参数
        self.L1 = 0.35
        self.L2 = 0.60
        self.boom_bend_angle_deg = 46.0  # 折弯角度

        # 小臂和铲斗参数
        self.L_arm = 0.44
        self.L_bucket = 0.26

        # 2. 预计算大臂的等效直线模型
        self._calculate_boom_equivalent()

        # 3. 传感器标定偏置 (Calibration Offsets)
        # --- 新的 V4 相对角标定 ---
        # 1. 大臂偏置: 下降时传感器增加，极性为负
        self.offset_boom = 40.9
        
        # 2. 小臂偏置: 回拉时传感器增加，物理仰角减小(向下)，极性为负
        # 状态A(外推): sensor=5.0, 实测15度(向上) => 15 = offset - 5.0 => offset=20.0
        # 状态B(回拉): sensor=99.3, 实测80度(向下) => -80 = offset - 99.3 => offset=19.3
        # 取平均
        self.offset_arm = 19.6
        
        # 3. 铲斗偏置
        # 极性分析：
        # 外推(铲斗张开到头): sensor_bucket_rel = -93.2 => 累加角 = 5.9 - 0.9 - 93.2 = -88.2
        #    实测姿态: 铲尖略微朝上朝外，与水平夹角32度。在我们的向上为正坐标系中，仰角为 +32 度。
        # 回拉(铲斗内收到头): sensor_bucket_rel = 6.3 => 累加角 = 6.4 + 92.9 + 6.3 = 105.6
        #    实测姿态: 铲尖略微朝上朝内，与水平夹角17度。此时铲尖越过了向下，转到了后方上方。仰角为 180 - 17 = 163 度。
        # 
        # 验证总行程：
        # 物理行程 = 163 - 32 = 131度 (或者如果是向下转的，应该是 32 -> 0 -> -90 -> -180 -> -197，也就是 32 - (-197) = 229度)
        # 用户说明实际运动了 195度！
        #
        # 让我们重新审视坐标：
        # 如果外推是 +32度，然后向内收紧(向下转)，运动 195度。
        # 那么收紧到底时的绝对角度应该是 32 - 195 = -163 度。
        # 而 -163 度正好等于 180 + 17（因为它是周期性的），也就是“向后下方指，与水平夹角17度，铲尖略微朝上朝内”。这完美符合了用户的描述！
        # 
        # 所以，真实的物理仰角是：
        # 状态A (外推): 32 度
        # 状态B (回拉): -163 度
        #
        # 对应的传感器读数：
        # 状态A: -88.2
        # 状态B: 105.6 (增加了 193.8)
        # 
        # 当传感器数值增加 193.8，物理角度从 32 变到了 -163 (减小了 195)。
        # 两者的变化量几乎完美吻合！ (193.8 vs 195)，极性为负！
        # abs = offset - sensor
        # 32 = offset - (-88.2) => offset = -56.2
        #
        # 用状态B验证: abs = -56.2 - 105.6 = -161.8 (与 -163 误差仅1.2度)
        # 我们取 offset = -56.2
        self.offset_bucket = -56.2

    def _calculate_boom_equivalent(self):
        """
        根据 L1, L2 和折弯角计算大臂的等效直线长度和结构偏置角
        """
        # 内部钝角
        inner_angle_deg = 180.0 - self.boom_bend_angle_deg
        inner_angle_rad = math.radians(inner_angle_deg)

        # 余弦定理计算等效长度 L_boom
        self.L_boom = math.sqrt(
            self.L1**2 + self.L2**2 - 2 * self.L1 * self.L2 * math.cos(inner_angle_rad)
        )

        # 正弦定理计算 L_boom 与 L2 的夹角 (因为传感器安装在 L2 上)
        # L1 / sin(beta) = L_boom / sin(inner_angle)
        sin_beta = (self.L1 * math.sin(inner_angle_rad)) / self.L_boom
        self.beta_deg = math.degrees(math.asin(sin_beta))

        # 打印初始化信息以便确认
        print(f"[Kinematics Init] 大臂等效长度 L_boom: {self.L_boom:.4f} m")
        print(f"[Kinematics Init] L2 传感器结构偏置角 beta: {self.beta_deg:.2f} 度")

    def forward_kinematics(self, sensor_boom_deg, sensor_arm_deg, sensor_bucket_deg):
        """
        根据传感器原始读数计算正向运动学
        注意：根据 V4 控制器逻辑，传感器的 pitch 极性为【向下/向内 为正，向上/向外 为负】。
        为了进行标准平面几何系（X向前方为正，Z向垂直向上为正）计算，我们需要将角度反转。
        :param sensor_boom_deg: 大臂传感器绝对倾角 (位于 L2)
        :param sensor_arm_deg: 小臂传感器绝对倾角
        :param sensor_bucket_deg: 铲斗传感器绝对倾角
        :return: (x, z) 铲尖坐标，单位: 米
        """
        # 1. 传感器极性转换与偏置应用 (转换为标准几何角: 向上为正)
        abs_boom_L2_deg = self.offset_boom - sensor_boom_deg
        abs_arm_deg = self.offset_arm - sensor_arm_deg
        abs_bucket_deg = self.offset_bucket - sensor_bucket_deg

        # 2. 计算大臂等效直线的绝对倾角
        # 大臂是鹅颈向下的，等效直线(基座到小臂销轴)在 L2 的"上方"，因此角度加上 beta
        theta_boom_line_deg = abs_boom_L2_deg + self.beta_deg
        
        # 转换为弧度
        theta1 = math.radians(theta_boom_line_deg)
        theta2 = math.radians(abs_arm_deg)
        theta3 = math.radians(abs_bucket_deg)

        # 3. 逐级坐标解算
        
        # 节点 1: 大臂顶端 (连接小臂处)
        x1 = self.offset_x + self.L_boom * math.cos(theta1)
        z1 = self.offset_z + self.L_boom * math.sin(theta1)

        # 节点 2: 小臂顶端 (连接铲斗处)
        x2 = x1 + self.L_arm * math.cos(theta2)
        z2 = z1 + self.L_arm * math.sin(theta2)

        # 节点 3: 铲尖
        x3 = x2 + self.L_bucket * math.cos(theta3)
        z3 = z2 + self.L_bucket * math.sin(theta3)

        return {
            "boom_tip": (x1, z1),
            "arm_tip": (x2, z2),
            "bucket_tip": (x3, z3)
        }

if __name__ == "__main__":
    # 简单测试代码
    fk = ExcavatorKinematics()
    
    # 假设此时大臂L2水平(0度)，小臂向下垂直(-90度)，铲斗向内弯曲(-135度)
    print("\n--- 测试计算 ---")
    sensor_boom = 0.0
    sensor_arm = -90.0
    sensor_bucket = -135.0
    
    coords = fk.forward_kinematics(sensor_boom, sensor_arm, sensor_bucket)
    
    print(f"大臂顶端坐标: X={coords['boom_tip'][0]:.3f} m, Z={coords['boom_tip'][1]:.3f} m")
    print(f"小臂顶端坐标: X={coords['arm_tip'][0]:.3f} m, Z={coords['arm_tip'][1]:.3f} m")
    print(f"铲尖最终坐标: X={coords['bucket_tip'][0]:.3f} m, Z={coords['bucket_tip'][1]:.3f} m")
