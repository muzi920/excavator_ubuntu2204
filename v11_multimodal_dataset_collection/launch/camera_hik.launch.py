#!/usr/bin/env python3
"""
海康威视摄像头 RTSP → ROS 2 启动文件
======================================
启动单路海康摄像头 RTSP 流，发布到 /camera_hik/image_raw。

用法:
  ros2 launch v11_multimodal_dataset_collection camera_hik.launch.py

自定义参数:
  ros2 launch v11_multimodal_dataset_collection camera_hik.launch.py \
    rtsp_url:="rtsp://admin:password@192.168.1.100:554/Streaming/Channels/101" \
    topic:="/my_hik_cam/image_raw"
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明可配置参数
    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101',
        description='海康摄像头 RTSP 地址'
    )

    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/camera_hik/image_raw',
        description='ROS2 发布 topic'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='cam_hik',
        description='摄像头 frame_id'
    )

    transport_arg = DeclareLaunchArgument(
        'transport',
        default_value='tcp',
        description='RTSP 传输协议 (海康建议 tcp)'
    )

    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate_hz',
        default_value='10.0',
        description='图像发布频率 (Hz)'
    )

    # 海康摄像头节点
    hikvision_node = Node(
        package='v11_multimodal_dataset_collection',
        executable='hikvision_cam_node',
        name='hikvision_cam_node',
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
        hikvision_node,
    ])
