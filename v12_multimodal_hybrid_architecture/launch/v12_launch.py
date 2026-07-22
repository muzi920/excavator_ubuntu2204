from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 启动官方 M300 雷达驱动 (发布 /pointcloud 和 /imu)
    # 注意: 确保 pacecat_m300_driver 已经 source
    m300_driver_share = get_package_share_directory("pacecat_m300_driver")
    m300_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m300_driver_share, 'launch', 'LDS-M300-E.launch')
        )
    )

    # 2. 启动 C++ IMU & Sensor 节点 (监听 /imu 和相对倾角数据，发布 TF odom->base_link 和 JointState)
    imu_sensor_node = Node(
        package="v12_multimodal_hybrid_architecture",
        executable="imu_sensor_node",
        name="imu_sensor_node",
        output="screen"
    )

    # 3. 启动 C++ Lidar Processor 节点 (将 /pointcloud 从 map 转换过滤至 /lidar/points)
    lidar_processor_node = Node(
        package="v12_multimodal_hybrid_architecture",
        executable="lidar_processor_node",
        name="lidar_processor_node",
        output="screen"
    )

    # 5. 启动 C++ RTSP 相机硬件加速节点
    rtsp_cam_hik = Node(
        package="v12_multimodal_hybrid_architecture",
        executable="rtsp_camera_node",
        name="cam_hik_node",
        parameters=[{
            "camera_name": "cam_hik",
            "rtsp_url": "rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101",
            "fps": 25,
            "target_width": 1920,
            "target_height": 1080,
        }],
        output="screen"
    )

    rtsp_cam1 = Node(
        package="v12_multimodal_hybrid_architecture",
        executable="rtsp_camera_node",
        name="cam1_node",
        parameters=[{
            "camera_name": "cam1",
            "rtsp_url": "rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream",
            "fps": 25,
            "target_width": 1920,
            "target_height": 1080,
        }],
        output="screen"
    )

    rtsp_cam2 = Node(
        package="v12_multimodal_hybrid_architecture",
        executable="rtsp_camera_node",
        name="cam2_node",
        parameters=[{
            "camera_name": "cam2",
            "rtsp_url": "rtsp://admin:@192.168.158.103:554/stream",
            "fps": 25,
            "target_width": 1920,
            "target_height": 1080,
        }],
        output="screen"
    )

    # 5. 启动 Python GUI 与 高程图逻辑
    python_gui_node = Node(
        package="v12_multimodal_hybrid_architecture",
        executable="v12_hybrid_gui.py",
        name="v12_hybrid_gui",
        output="screen"
    )

    # 6. 启动闭环控制执行节点
    control_executor_node = Node(
        package="v12_multimodal_hybrid_architecture",
        executable="control_executor_node.py",
        name="control_executor_node",
        parameters=[{
            "sensor_ports": [
                "/dev/ttyUSB_Sensor1",
                "/dev/ttyUSB_Sensor2",
                "/dev/ttyUSB_Sensor3",
                "/dev/ttyUSB_Sensor4",
            ]
        }],
        output="screen"
    )

    # 7. 静态 TF: map -> odom (如果需要的话), 以及 base_link -> lidar
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'] 
        # 注意: 官方驱动发布的 frame_id 默认是 map (可以在 LDS-M300-E.yaml 修改为 lidar_link)
    )

    return LaunchDescription([
        m300_driver,
        imu_sensor_node,
        lidar_processor_node,
        rtsp_cam_hik,
        rtsp_cam1,
        rtsp_cam2,
        python_gui_node,
        control_executor_node,
        static_tf_lidar
    ])
