"""DSVT/PointPillars 实时推理 Launch 文件。

用法:
    source /opt/ros/humble/setup.bash
    ros2 launch dsvt_ros2 dsvt_inference.launch.py \
        ckpt_path:=/home/libo/PointPillars/soil_logs/checkpoints/best.pth \
        data_path:=/home/libo/Point_recognition/DSVT/pcd_npy/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 声明参数 ----
    ckpt_path = LaunchConfiguration('ckpt_path')
    data_path = LaunchConfiguration('data_path')
    class_names = LaunchConfiguration('class_names')
    score_thresh = LaunchConfiguration('score_thresh')
    rate = LaunchConfiguration('rate')

    declare_ckpt = DeclareLaunchArgument(
        'ckpt_path',
        default_value='/home/libo/PointPillars/soil_logs/checkpoints/best.pth',
        description='Model checkpoint (.pth)')

    declare_data = DeclareLaunchArgument(
        'data_path',
        default_value='/home/libo/Point_recognition/DSVT/pcd_npy/',
        description='Point cloud file or directory')

    declare_classes = DeclareLaunchArgument(
        'class_names',
        default_value='Soil',
        description='Class names, comma-separated')

    declare_score = DeclareLaunchArgument(
        'score_thresh',
        default_value='0.1',
        description='Detection score threshold')

    declare_rate = DeclareLaunchArgument(
        'rate',
        default_value='10.0',
        description='Point cloud publish rate (Hz)')

    # ---- 节点 1: 点云发布器 ----
    pc_pub_node = Node(
        package='dsvt_ros2',
        executable='pc_publisher',
        name='pc_publisher',
        namespace='perception',
        parameters=[{
            'data_path': data_path,
            'rate': 10.0,
            'loop': True,
            'frame_id': 'lidar',
        }],
        output='screen',
    )

    # ---- 节点 2: DSVT 推理 (延迟 2 秒等模型加载) ----
    infer_node = TimerAction(
        period=0.5,
        actions=[
            Node(
                package='dsvt_ros2',
                executable='inference_node',
                name='dsvt_inference',
                namespace='perception',
                parameters=[{
                    'ckpt_path': ckpt_path,
                    'class_names': class_names,
                    'score_thresh': 0.0 if score_thresh == '0.1' else 0.1,
                    'device': 'cuda',
                    'input_topic': '/lidar/points',
                    'output_topic': '/perception/markers',
                }],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        declare_ckpt,
        declare_data,
        declare_classes,
        declare_score,
        declare_rate,
        LogInfo(msg='🚀 DSVT ROS2 实时推理 Pipeline 启动中...'),
        LogInfo(msg=f'   点云数据: {data_path}'),
        LogInfo(msg=f'   模型权重: {ckpt_path}'),
        pc_pub_node,
        infer_node,
    ])
