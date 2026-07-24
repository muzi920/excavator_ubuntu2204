#!/usr/bin/env python3
"""
单路网络摄像头 RTSP → ROS 2 启动文件
======================================
启动单路网络摄像头 RTSP 流，默认对应摄像头 1，可按需要配置。

用法:
  ros2 launch v11_multimodal_dataset_collection camera_net1.launch.py

覆盖参数:
  ros2 launch v11_multimodal_dataset_collection camera_net1.launch.py \
    rtsp_url:="rtsp://admin:pass@192.168.1.102:554/stream" \
    camera_name:="cam2" \
    topic:="/camera2/image_raw"
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream',
        description='网络摄像头 RTSP 地址'
    )

    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/camera1/image_raw',
        description='ROS2 发布 topic'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='cam1',
        description='摄像头 frame_id'
    )

    transport_arg = DeclareLaunchArgument(
        'transport',
        default_value='udp',
        description='RTSP 传输协议'
    )

    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate_hz',
        default_value='10.0',
        description='图像发布频率 (Hz)'
    )

    cam_node = Node(
        package='v11_multimodal_dataset_collection',
        executable='network_cam_node',
        name='network_cam_node',
        output='screen',
        parameters=[{
            'rtsp_url': LaunchConfiguration('rtsp_url'),
            'topic': LaunchConfiguration('topic'),
            'camera_name': LaunchConfiguration('camera_name'),
            'transport': LaunchConfiguration('transport'),
            'pub_rate_hz': LaunchConfiguration('pub_rate_hz'),
        }],
    )

    return LaunchDescription([
        rtsp_url_arg,
        topic_arg,
        camera_name_arg,
        transport_arg,
        pub_rate_arg,
        cam_node,
    ])
