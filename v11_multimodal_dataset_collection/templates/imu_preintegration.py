import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class TiltCompensator:
    """
    倾斜补偿器（基于互补滤波 Complementary Filter）+ 启动零偏校准。
    专门针对挖掘机履带不动、原地挖土伴随车体晃动的场景。
    """
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.last_timestamp = None
        
        self.roll = 0.0
        self.pitch = 0.0
        
        # 开机校准相关
        self.is_calibrated = False
        self.calib_samples = []
        self.CALIB_COUNT = 50  # 收集 50 帧进行初始水平面校准
        
        self.roll0 = 0.0
        self.pitch0 = 0.0

    def update(self, timestamp, accel, gyro, external_yaw=0.0):
        """
        输入:
            timestamp: 当前时间戳 (秒)
            accel: [ax, ay, az] (加速度 m/s^2)
            gyro: [wx, wy, wz] (角速度 rad/s)
            external_yaw: 外部传入的准确 Yaw 角 (弧度)，因为单靠陀螺仪积分 Yaw 会漂移。
        返回:
            quaternion: [x, y, z, w] 修正后的姿态四元数 (相对初始水平面的 Roll/Pitch，加上 external_yaw)
        """
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            self._update_from_accel_only(accel)
            return self.get_quaternion(external_yaw)

        dt = timestamp - self.last_timestamp
        if dt <= 0:
            return self.get_quaternion(external_yaw)

        # 1. 陀螺仪积分更新 (高频)
        self.roll += gyro[0] * dt
        self.pitch += gyro[1] * dt

        # 2. 加速度计解算绝对重力方向 (低频)
        ax, ay, az = accel
        g_magnitude = math.sqrt(ax**2 + ay**2 + az**2)
        
        if 8.0 < g_magnitude < 11.5:
            accel_roll = math.atan2(ay, az)
            accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            
            # 3. 互补滤波融合
            self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch

            # 4. 开机静止零点校准 (消除安装倾斜误差)
            if not self.is_calibrated:
                self.calib_samples.append((accel_roll, accel_pitch))
                if len(self.calib_samples) >= self.CALIB_COUNT:
                    self.roll0 = sum(s[0] for s in self.calib_samples) / self.CALIB_COUNT
                    self.pitch0 = sum(s[1] for s in self.calib_samples) / self.CALIB_COUNT
                    # 将当前融合值也对齐到 0
                    self.roll = self.roll0
                    self.pitch = self.pitch0
                    self.is_calibrated = True
                    print(f"[TiltCompensator] 初始化零偏校准完成: roll0={math.degrees(self.roll0):.2f}°, pitch0={math.degrees(self.pitch0):.2f}°")

        self.last_timestamp = timestamp
        return self.get_quaternion(external_yaw)

    def _update_from_accel_only(self, accel):
        ax, ay, az = accel
        self.roll = math.atan2(ay, az)
        self.pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

    def get_quaternion(self, yaw=0.0):
        """返回相对初始水平面 (消除安装误差后) 的四元数"""
        if not self.is_calibrated:
            return R.from_euler('xyz', [0.0, 0.0, yaw], degrees=False).as_quat()
            
        rel_roll = self.roll - self.roll0
        rel_pitch = self.pitch - self.pitch0
        r = R.from_euler('xyz', [rel_roll, rel_pitch, yaw], degrees=False)
        return r.as_quat()

    def get_gravity_aligned_quaternion(self):
        """
        获取仅包含相对 Roll 和 Pitch 的四元数（强制 Yaw=0）。
        用于高程图点云纠正时，保持 Z 轴垂直于地平面，同时不改变原始朝向。
        """
        if not self.is_calibrated:
            return R.identity().as_quat()
            
        rel_roll = self.roll - self.roll0
        rel_pitch = self.pitch - self.pitch0
        r = R.from_euler('xyz', [rel_roll, rel_pitch, 0.0], degrees=False)
        return r.as_quat()
