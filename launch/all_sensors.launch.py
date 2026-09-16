import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 获取当前工作空间路径
    # 假设该 launch 文件位于 /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/launch/
    pkg_path = os.path.join(os.path.dirname(__file__), '..')
    
    # 1. 启动雷达驱动 (M300-E)
    lidar_launch_dir = os.path.join(get_package_share_directory('pacecat_m300_driver'), 'launch')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_launch_dir, 'LDS-M300-E.launch')
        )
    )

    # 2. 启动全系统传感器 TF 坐标系发布
    sensors_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sensors_tf.launch.py')
        )
    )

    # 3. 启动回转角度预估节点 (依赖于雷达 /imu 和 TF)
    swing_estimator_node = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_path, 'v5_sensor_read_lidar', 'swing_angle_estimator.py')],
        output='screen'
    )

    # 4. 启动点云坐标系转换节点 (将 /pointcloud 从 map 转换到 base_link)
    pointcloud_transformer_node = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_path, 'v5_sensor_read_lidar', 'pointcloud_transformer.py')],
        output='screen'
    )

    # 5. 启动 WIT 倾角传感器节点 (大臂、小臂、铲斗相对角度)
    imu_node = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_path, 'v3_sensor_read_wit', 'ros2_readRad_pub.py')],
        output='screen'
    )

    # 6. 启动所有摄像头合并节点 (海康主视角 + 两个网络摄像头覆盖视角)
    cams_node = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_path, 'v6_sensor_read_camera', 'ros2_net_cams_pub.py')],
        output='screen'
    )

    return LaunchDescription([
        # 1. 底层硬件与坐标系
        lidar_launch,
        sensors_tf_launch,
        
        # 2. 角度解算与传感器采集
        swing_estimator_node,
        pointcloud_transformer_node,
        imu_node,
        
        # 3. 视觉采集
        cams_node
    ])
