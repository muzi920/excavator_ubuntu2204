import math

class ExcavatorIK:
    """
    挖掘机逆向运动学 (Inverse Kinematics)
    已知目标铲尖的 (X, Z) 坐标和铲斗的绝对倾角，反推需要下发给 V4 控制器的三个关节相对角度。
    """
    def __init__(self):
        # 1. 坐标系与物理参数 (与 FK 保持完全一致)
        self.offset_x = -0.25
        self.offset_z = 0.40
        self.L1 = 0.35
        self.L2 = 0.60
        self.boom_bend_angle_deg = 46.0
        self.L_arm = 0.44
        self.L_bucket = 0.26

        # 大臂等效计算
        inner_angle_rad = math.radians(180.0 - self.boom_bend_angle_deg)
        self.L_boom = math.sqrt(self.L1**2 + self.L2**2 - 2 * self.L1 * self.L2 * math.cos(inner_angle_rad))
        sin_beta = (self.L1 * math.sin(inner_angle_rad)) / self.L_boom
        self.beta_deg = math.degrees(math.asin(sin_beta))

        # 运动限位 (参考 test3)
        self.boom_limit = (0.7, 47.3)
        self.arm_limit = (-1.0, 57.1)
        self.bucket_limit = (-92.7, 5.7)

    def calculate_ik(self, target_x, target_z, bucket_angle_deg):
        """
        计算逆向运动学
        由于机械臂在 2D 平面有 3 个自由度 (大臂、小臂、铲斗)，但目标坐标只有 (X, Z) 两个约束。
        因此我们需要第三个约束：铲斗的绝对倾角 (即你希望铲斗以什么姿态切入土中)。
        
        :param target_x: 铲尖目标 X 坐标 (相对于回转中心)
        :param target_z: 铲尖目标 Z 坐标 (相对于地面)
        :param bucket_angle_deg: 铲斗的绝对几何倾角 (向上为正，水平为0，向下挖掘通常为负)
        :return: (boom_swing, arm_boom, bucket_arm) 或 None (如果不可达)
        """
        # 1. 几何系中的目标角度
        theta3 = math.radians(bucket_angle_deg)

        # 2. 扣除铲斗长度，求出“小臂顶端(腕关节)”的目标坐标 (相对于大臂底座)
        # 注意要减去底座偏移，转为纯连杆坐标系
        x_wrist = (target_x - self.offset_x) - self.L_bucket * math.cos(theta3)
        z_wrist = (target_z - self.offset_z) - self.L_bucket * math.sin(theta3)

        # 3. 求解两连杆 (L_boom 和 L_arm) 到达 (x_wrist, z_wrist) 的逆解
        distance = math.sqrt(x_wrist**2 + z_wrist**2)

        # 检查是否在可达范围内
        if distance > (self.L_boom + self.L_arm) or distance < abs(self.L_boom - self.L_arm):
            # print(f"目标点 ({target_x}, {target_z}) 无法到达！")
            return None

        # 使用余弦定理求夹角
        # alpha: 原点到腕关节的向量与 X 轴的夹角
        alpha = math.atan2(z_wrist, x_wrist)
        
        # gamma: L_boom 与 distance 连线的夹角
        cos_gamma = (self.L_boom**2 + distance**2 - self.L_arm**2) / (2 * self.L_boom * distance)
        # 防止浮点数精度越界
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma = math.acos(cos_gamma)

        # 挖掘机的标准姿态是“肘部朝上 (Elbow Up)”，即大臂关节在直线连线上方
        theta1 = alpha + gamma

        # 求小臂的绝对几何角度 theta2
        # 小臂向量 = 腕关节坐标 - 大臂顶端坐标
        x_elbow = self.L_boom * math.cos(theta1)
        z_elbow = self.L_boom * math.sin(theta1)
        theta2 = math.atan2(z_wrist - z_elbow, x_wrist - x_elbow)

        # 转换为角度
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        theta3_deg = math.degrees(theta3)

        # ---------------------------------------------------------
        # 4. 将标准几何绝对角，逆推回 V4 传感器的读数和相对角
        # ---------------------------------------------------------
        # 根据 FK 的映射公式:
        # theta1_deg = abs_boom_L2_deg + beta_deg
        # abs_boom_L2_deg = 40.0 - sensor_boom_deg
        abs_boom_L2_deg = theta1_deg - self.beta_deg
        sensor_boom_deg = 40.0 - abs_boom_L2_deg
        
        # abs_arm_deg = -sensor_arm_deg
        sensor_arm_deg = -theta2_deg
        
        # abs_bucket_deg = -sensor_bucket_deg
        sensor_bucket_deg = -theta3_deg

        # V4 相对角 (JSON 剧本中下发的参数)
        boom_swing = sensor_boom_deg
        arm_boom = sensor_arm_deg - sensor_boom_deg
        bucket_arm = sensor_bucket_deg - sensor_arm_deg

        return {
            "boom_swing": round(boom_swing, 2),
            "arm_boom": round(arm_boom, 2),
            "bucket_arm": round(bucket_arm, 2),
            "sensor_boom": round(sensor_boom_deg, 2),
            "sensor_arm": round(sensor_arm_deg, 2),
            "sensor_bucket": round(sensor_bucket_deg, 2)
        }

if __name__ == "__main__":
    ik = ExcavatorIK()
    
    # 假设我们要在前方 1.0 米，地面深度 -0.2 米处挖掘，
    # 此时铲斗的姿态设定为与地面呈 -60度 (向下切入土中)
    target_x = 1.0
    target_z = -0.2
    bucket_angle = -60.0
    
    print(f"目标位置: X={target_x}m, Z={target_z}m, 铲斗姿态={bucket_angle}°")
    result = ik.calculate_ik(target_x, target_z, bucket_angle)
    
    if result:
        print("\n计算出的 V4 剧本参数：")
        print(f"  boom_swing (大臂): {result['boom_swing']}°")
        print(f"  arm_boom   (小臂): {result['arm_boom']}°")
        print(f"  bucket_arm (铲斗): {result['bucket_arm']}°")
        
        print("\n这对应的物理传感器读数应为：")
        print(f"  大臂传感器: {result['sensor_boom']}°")
        print(f"  小臂传感器: {result['sensor_arm']}°")
        print(f"  铲斗传感器: {result['sensor_bucket']}°")
        
        # 简单检查一下是否越界
        print("\n限位检查:")
        print(f"  大臂 {ik.boom_limit}: {'OK' if ik.boom_limit[0] <= result['boom_swing'] <= ik.boom_limit[1] else '越界'}")
        print(f"  小臂 {ik.arm_limit}: {'OK' if ik.arm_limit[0] <= result['arm_boom'] <= ik.arm_limit[1] else '越界'}")
        print(f"  铲斗 {ik.bucket_limit}: {'OK' if ik.bucket_limit[0] <= result['bucket_arm'] <= ik.bucket_limit[1] else '越界'}")
    else:
        print("无法到达该目标位置！")
