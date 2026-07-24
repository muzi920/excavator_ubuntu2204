#!/usr/bin/env python3
"""
双路网络摄像头 RTSP → ROS 2 启动文件
======================================
同时启动两路网络摄像头 RTSP 流:
  - 网络摄像头 1: /camera1/image_raw
  - 网络摄像头 2: /camera2/image_raw

用法:
  ros2 launch v11_multimodal_dataset_collection camera_net.launch.py

自定义参数（摄像头1）:
  ros2 launch v11_multimodal_dataset_collection camera_net.launch.py \
    net1_url:="rtsp://admin:pass@192.168.1.102:554/stream" \
    net2_url:="rtsp://admin:pass@192.168.1.103:554/stream"
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ========= 网络摄像头 1 =========
    net1_url_arg = DeclareLaunchArgument(
        'net1_url',
        default_value='rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream',
        description='网络摄像头 1 的 RTSP 地址'
    )
    net1_topic_arg = DeclareLaunchArgument(
        'net1_topic',
        default_value='/camera1/image_raw',
        description='网络摄像头 1 的 ROS2 topic'
    )

    # ========= 网络摄像头 2 =========
    net2_url_arg = DeclareLaunchArgument(
        'net2_url',
        default_value='rtsp://admin:@192.168.158.103:554/stream',
        description='网络摄像头 2 的 RTSP 地址'
    )
    net2_topic_arg = DeclareLaunchArgument(
        'net2_topic',
        default_value='/camera2/image_raw',
        description='网络摄像头 2 的 ROS2 topic'
    )

    # 公共参数
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

    # 节点 1: 网络摄像头 1
    net1_node = Node(
        package='v11_multimodal_dataset_collection',
        executable='network_cam_node',
        name='network_cam1_node',
        output='screen',
        parameters=[{
            'rtsp_url': LaunchConfiguration('net1_url'),
            'topic': LaunchConfiguration('net1_topic'),
            'camera_name': 'cam1',
            'transport': LaunchConfiguration('transport'),
            'pub_rate_hz': LaunchConfiguration('pub_rate_hz'),
        }],
    )

    # 节点 2: 网络摄像头 2
    net2_node = Node(
        package='v11_multimodal_dataset_collection',
        executable='network_cam_node',
        name='network_cam2_node',
        output='screen',
        parameters=[{
            'rtsp_url': LaunchConfiguration('net2_url'),
            'topic': LaunchConfiguration('net2_topic'),
            'camera_name': 'cam2',
            'transport': LaunchConfiguration('transport'),
            'pub_rate_hz': LaunchConfiguration('pub_rate_hz'),
        }],
    )

    return LaunchDescription([
        net1_url_arg,
        net1_topic_arg,
        net2_url_arg,
        net2_topic_arg,
        transport_arg,
        pub_rate_arg,
        net1_node,
        net2_node,
    ])
