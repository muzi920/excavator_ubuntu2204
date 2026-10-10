#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立 Launch A：只启动 v11 ROS 控制桥（不启相机，方便和 v15 / RViz URDF 联调）
=============================================================================
用法：
  ros2 launch v11_multimodal_dataset_collection v11_ros_control_only.launch.py  \
      offline:=true

提示：与 RViz URDF 联调时，建议 offline=true；真实车联调时改 offline=false 且确
      认 CAN 串口 /dev/ttyUSB_Controller 存在。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    offline_arg = DeclareLaunchArgument("offline", default_value="true")
    can_port_arg = DeclareLaunchArgument("can_port", default_value="/dev/ttyUSB_Controller")
    can_baud_arg = DeclareLaunchArgument("can_baud", default_value="115200")
    bucket_guess_arg = DeclareLaunchArgument("default_bucket_guess_deg", default_value="-45.0")

    bridge = ExecuteProcess(
        cmd=[
            "/bin/bash", "-c",
            (
                "OFFLINE='$(var offline)' ; "
                "ARGS='--can-port \"$(var can_port)\" "
                "--can-baud \"$(var can_baud)\" "
                "--default-bucket-guess-deg \"$(var default_bucket_guess_deg)\"' ; "
                "case \"$OFFLINE\" in 1|true|TRUE|True|on|ON|yes) ARGS=\"--offline $ARGS\" ;; esac ; "
                "exec /usr/bin/env python3 "
                "$(python3 -c \"import os, sys; print(os.path.abspath(sys.argv[1]))\" "
                "'$(dirname $(dirname $0))'/v11_ros_topic_controller.py)"
            ),
        ],
        output="screen",
        name="v11_ros_topic_controller",
    )
    # 上面复杂的 bash 表达式 + shell cmd 太绕，直接用 ThisLaunchFileDir 拼绝对路径 + Python 条件开关
    script_abs = [ThisLaunchFileDir(), "/../v11_ros_topic_controller.py"]
    bridge_simple = ExecuteProcess(
        cmd=[
            "/usr/bin/env", "python3",
            *script_abs,
            "--offline", LaunchConfiguration("offline"),
            "--can-port", LaunchConfiguration("can_port"),
            "--can-baud", LaunchConfiguration("can_baud"),
            "--default-bucket-guess-deg", LaunchConfiguration("default_bucket_guess_deg"),
        ],
        output="screen",
        name="v11_ros_topic_controller",
    )
    banner = LogInfo(msg=(
        "\n[v11_launch] ros_control_only 启动中\n"
        "  订阅: /state_joint     (弧度; name=swing/boom/arm/bucket_joint)\n"
        "  订阅: /target_point    (米; frame_id=base_link)\n"
        "  反馈: /v11/current_state (度, 调试)\n"
    ))
    return LaunchDescription([
        offline_arg, can_port_arg, can_baud_arg, bucket_guess_arg,
        banner,
        bridge_simple,
    ])
