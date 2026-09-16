import numpy as np
from scipy.spatial.transform import Rotation as R

class PointCloudTransform:
    """
    点云坐标系变换模板类。
    用于将 base_link (第一视角) 下的点云，利用 IMU 或 SLAM 提供的位姿，
    投影到 odom/map (绝对世界) 坐标系下。
    """
    def __init__(self):
        pass

    @staticmethod
    def transform_to_world(points_local, translation, quaternion):
        """
        将第一视角的点云转换到世界坐标系。
        
        参数:
            points_local: numpy array (N, 3)，在 base_link 下的点云坐标
            translation: numpy array (3,)，base_link 在 odom 下的平移 [x, y, z]
            quaternion: numpy array (4,)，base_link 在 odom 下的旋转四元数 [x, y, z, w]
            
        返回:
            points_world: numpy array (N, 3)，在 odom 坐标系下的绝对点云
        """
        if len(points_local) == 0:
            return points_local
            
        # 解析旋转
        rot = R.from_quat(quaternion)
        
        # 坐标变换： P_world = R * P_local + T
        points_world = rot.apply(points_local) + translation
        return points_world

    @staticmethod
    def gravity_align_only(points_local, quaternion):
        """
        仅做重力对齐（不平移，不考虑偏航角的绝对朝向）。
        用于：挖掘机车体倾斜时，保证点云的高程图 Z 轴永远垂直于真实地平面。
        
        参数:
            points_local: numpy array (N, 3)
            quaternion: numpy array (4,) 包含准确的 Roll 和 Pitch
        """
        if len(points_local) == 0:
            return points_local
            
        rot = R.from_quat(quaternion)
        # 提取 Roll 和 Pitch，忽略 Yaw
        euler = rot.as_euler('xyz', degrees=False)
        euler[2] = 0.0 # 强制 Yaw 为 0
        rot_aligned = R.from_euler('xyz', euler)
        
        points_aligned = rot_aligned.apply(points_local)
        return points_aligned
