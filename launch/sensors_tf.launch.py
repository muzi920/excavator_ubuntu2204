import math
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    独立发布多传感器到车体(base_link)的静态 TF 树的 Launch 文件。
    
    使用方法:
    ros2 launch sensors_tf.launch.py
    """
    
    # 1. 雷达 TF (map -> base_link)
    # 雷达是倒着安装的，所以 roll 设置为 180度 (math.pi = 3.14159)
    # 可在 arguments 中微调物理安装位置 [x, y, z, yaw, pitch, roll]
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=['-0.5500', '-0.2000', '1.2712', '0.0532', '0.0349', '3.0316', 'map', 'base_link']
    )

    # 2. 普通网络摄像头1 TF (IP: .102) (network_cam_frame -> base_link)
    # 由 base_link 坐标系下测量的相对于雷达的绝对偏移：x: -0.18, y: 0.13, z: 0.20
    # 姿态：俯仰角 53度 (向下)
    net_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='net_cam_static_tf',
        arguments=['0.4239', '-0.1768', '1.4246', '0.0000', '0.9250', '0.0000', 'base_link', 'network_cam_frame']
    )

    # 3. 普通网络摄像头2 TF (IP: .103) (network_cam2_frame -> base_link)
    net_cam2_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='net_cam2_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'network_cam2_frame']
    )

    # 4. 海康摄像头 TF (hikvision_cam_frame -> base_link)
    # 由 base_link 坐标系下测量的相对于雷达的绝对偏移：x: -0.15, y: 0.46, z: 0.30
    # 姿态：俯仰角 66度 (向下)
    hik_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hik_cam_static_tf',
        arguments=['0.4539', '0.1532', '1.5246', '0.0000', '1.1519', '0.0000', 'base_link', 'hikvision_cam_frame']
    )

    return LaunchDescription([
        lidar_tf,
        net_cam_tf,
        net_cam2_tf,
        hik_cam_tf
    ])
