import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. 包含雷达的 Launch 文件 (通过包名引用)
    # 因为 pacecat_m300_driver 已经是一个标准的 ROS2 包并被编译，我们可以直接引用它
    lidar_launch_dir = os.path.join(get_package_share_directory('pacecat_m300_driver'), 'launch')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_launch_dir, 'LDS-M300-E.launch')
        )
    )

    # 2. 启动 IMU 节点 (因为这个脚本不在标准的 ROS2 包 install 里，我们使用 ExecuteProcess 直接运行 Python)
    imu_node = ExecuteProcess(
        cmd=['python3', '/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v3/ros2_readRad_pub.py'],
        output='screen'
    )

    # 3. 启动三个摄像头的合并节点 (同上，直接执行 Python 脚本)
    cams_node = ExecuteProcess(
        cmd=['python3', '/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v6/ros2_all_cams_pub.py'],
        output='screen'
    )

    return LaunchDescription([
        lidar_launch,
        imu_node,
        cams_node
    ])
