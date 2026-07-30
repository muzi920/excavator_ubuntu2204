#!/usr/bin/env python3
"""
全部摄像头一键启动 (海康 + 双路网络摄像头)
=========================================
同时启动:
  - 海康摄像头:    /camera_hik/image_raw
  - 网络摄像头 1:  /camera1/image_raw
  - 网络摄像头 2:  /camera2/image_raw

用法:
  ros2 launch v11_multimodal_dataset_collection camera_all.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 海康摄像头参数
    hik_url_arg = DeclareLaunchArgument(
        'hik_url',
        default_value='rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101',
        description='海康摄像头 RTSP 地址'
    )
    # 网络摄像头参数
    net1_url_arg = DeclareLaunchArgument(
        'net1_url',
        default_value='rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream',
        description='网络摄像头 1 RTSP 地址'
    )
    # net2_url_arg = DeclareLaunchArgument(
    #     'net2_url',
    #     default_value='rtsp://admin:@192.168.158.103:554/stream',
    #     description='网络摄像头 2 RTSP 地址'
    # )

    # 海康摄像头节点 (TCP)
    hik_node = Node(
        package='v11_multimodal_dataset_collection',
        executable='hikvision_cam_node',
        name='hikvision_cam_node',
        output='screen',
        parameters=[{
            'rtsp_url': LaunchConfiguration('hik_url'),
            'topic': '/camera_hik/image_raw',
            'camera_name': 'cam_hik',
            'transport': 'tcp',
            'pub_rate_hz': 10.0,
        }],
    )

    # 网络摄像头 1 (UDP)
    net1_node = Node(
        package='v11_multimodal_dataset_collection',
        executable='network_cam_node',
        name='network_cam1_node',
        output='screen',
        parameters=[{
            'rtsp_url': LaunchConfiguration('net1_url'),
            'topic': '/camera1/image_raw',
            'camera_name': 'cam1',
            'transport': 'udp',
            'pub_rate_hz': 10.0,
        }],
    )

    # 网络摄像头 2 (UDP)
    net2_node = Node(
        package='v11_multimodal_dataset_collection',
        executable='network_cam_node',
        name='network_cam2_node',
        output='screen',
        parameters=[{
            'rtsp_url': LaunchConfiguration('net2_url'),
            'topic': '/camera2/image_raw',
            'camera_name': 'cam2',
            'transport': 'udp',
            'pub_rate_hz': 10.0,
        }],
    )

    return LaunchDescription([
        hik_url_arg,
        net1_url_arg,
        # net2_url_arg,
        hik_node,
        net1_node,
        net2_node,
    ])
