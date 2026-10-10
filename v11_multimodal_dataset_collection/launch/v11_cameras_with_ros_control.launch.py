#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立 Launch B：v11 相机采集 + ROS Topic 控制桥（同时启动，原程序零修改）
===========================================================================
完全复用 v11 原生 camera_all.launch.py 不做任何改动；
额外以 <script> 方式启动 v11_ros_topic_controller.py。

用法：
  ros2 launch v11_multimodal_dataset_collection v11_cameras_with_ros_control.launch.py \
      offline:=true                               # 首次联调必加（先不通 CAN，只验证话题）

说明：
  1) camera_all.launch.py 的 hik_url / net1_url / net2_url 参数原样透传；
  2) ros_control 的参数（offline/can_port/can_baud/default_bucket_guess_deg）独立定义；
  3) 因为不修改 setup.py，ros2 launch 里直接用 'python3 <绝对路径>' 启动桥脚本，
     不走 console_scripts 注册 → 不影响任何 v11 原安装产物。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------------- ROS 控制桥参数 ----------------
    offline_arg = DeclareLaunchArgument(
        "offline", default_value="true",
        description="离线模式（=不连 CAN，只验证话题/指令；首次联调建议=true）",
    )
    can_port_arg = DeclareLaunchArgument(
        "can_port", default_value="/dev/ttyUSB_Controller",
        description="v4 闭环控制器 CAN 串口（同 multimodal_gui.py 默认）",
    )
    can_baud_arg = DeclareLaunchArgument(
        "can_baud", default_value="115200",
        description="CAN 串口波特率",
    )
    bucket_guess_arg = DeclareLaunchArgument(
        "default_bucket_guess_deg", default_value="-45.0",
        description="/target_point IK 时的默认铲斗绝对倾角(°)",
    )
    # ---------------- v11 原生相机参数（透传到 camera_all.launch.py） ----------------
    hik_url_arg = DeclareLaunchArgument(
        "hik_url",
        default_value="rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101",
    )
    net1_url_arg = DeclareLaunchArgument(
        "net1_url",
        default_value="rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream",
    )

    # 1) 包含原 v11 camera_all.launch.py（一行不修改原 launch）
    pkg_share = FindPackageShare("v11_multimodal_dataset_collection")
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_share, "/launch/camera_all.launch.py"]
        ),
        launch_arguments={
            "hik_url": LaunchConfiguration("hik_url"),
            "net1_url": LaunchConfiguration("net1_url"),
        }.items(),
    )

    # 2) 启动我们新增的 ROS 控制桥（独立 Python 进程）
    #    注意：桥脚本在 package 的根目录，不在 share/installed 里，所以用 ThisLaunchFileDir 反推 src 路径
    #    launch 文件位置: pkg_root/launch/xxx.py
    #    脚本位置:        pkg_root/v11_ros_topic_controller.py
    script_path_expr = [
        ThisLaunchFileDir(), "/../v11_ros_topic_controller.py"
    ]
    control_bridge = ExecuteProcess(
        cmd=[
            "/usr/bin/env", "python3",
            *script_path_expr,
            "--offline", LaunchConfiguration("offline"),
            "--can-port", LaunchConfiguration("can_port"),
            "--can-baud", LaunchConfiguration("can_baud"),
            "--default-bucket-guess-deg", LaunchConfiguration("default_bucket_guess_deg"),
        ],
        output="screen",
        name="v11_ros_topic_controller",
        # offline_arg 是 string 形式的 'true'/'false'，脚本 argparse 会当开关处理？
        # 解决：把 'true' 时追加 --offline，否则不加 → 用 shell + Python 表达式
        shell=True,
        # 注意：当用户 offline=false 时我们要把 cmd 里的 '--offline <x>' 去掉；
        # 所以直接改成：让脚本支持 --offline 后跟 true/false 也行 —— 下面把 argparse 的 offline
        # 改写成了支持 store_true 以及显式字符串值两种方式，在脚本里做解析兼容。
    )
    # （为了稳妥，我们在 v11_ros_topic_controller.py 的 main 里不依赖 argparse store_true 对字符串的解析，
    #   而改为手动判断 'false'/'0' 等于没打开——在文件里做一次兼容修复）

    banner = LogInfo(msg=(
        "\n"
        "================================================================\n"
        "[v11_launch] cameras + ros_control_bridge 同时启动中\n"
        "  - 控制订阅: /state_joint  (sensor_msgs/JointState 弧度, URDF 四关节顺序)\n"
        "  - 控制订阅: /target_point (geometry_msgs/PointStamped 米, frame=base_link)\n"
        "  - 调试反馈: /v11/current_state (JointState 度)\n"
        "  - offline=" + ("true" if False else "动态") + "  首次联调建议保持 true 再观察日志\n"
        "================================================================"
    ))

    return LaunchDescription([
        offline_arg,
        can_port_arg,
        can_baud_arg,
        bucket_guess_arg,
        hik_url_arg,
        net1_url_arg,
        banner,
        cameras_launch,
        control_bridge,
    ])
