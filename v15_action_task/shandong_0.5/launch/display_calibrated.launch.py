"""launch shandong_0.5 挖掘机 URDF (calibrated 版本) + RViz2 显示（纯复用 describe_60FED_calibrated.urdf）。

与 v15_main / describe_60FED/display.launch.py 启动的 calibrated 模型 100% 同一个 URDF。
本 launch 独立给 shandong_0.5 两个功能库（角度控制/目标点控制 单关节串行）做真机联调显示。

⚠️ 关键 launch 参数（顺序别写反）：use_joint_state_publisher:=false
     —— shandong_0.5.scripts.smoke_* 或你的控制脚本自己会发 /joint_states（4 关节顺序 [swing,boom,arm,bucket]），
        不要让 joint_state_publisher_gui 抢着发，否则心跳会被抢导致关节不动。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # URDF: 直接走 describe_60FED 的 calibrated 版本（用户指定，不新建副本不修改原文件）
    pkg_urdf = get_package_share_directory("describe_60FED")
    urdf_path = os.path.join(pkg_urdf, "urdf", "describe_60FED_calibrated.urdf")
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    use_jsp = LaunchConfiguration("use_joint_state_publisher", default="false")
    use_rviz = LaunchConfiguration("rviz", default="true")
    gui = LaunchConfiguration("gui", default="false")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_joint_state_publisher",
            default_value="false",
            description="必须 false（否则 joint_state_publisher_gui 会抢发 /joint_states，导致 shandong_0.5 单关节指令被覆盖）",
        ),
        DeclareLaunchArgument("rviz", default_value="true", description="是否打开 RViz2"),
        DeclareLaunchArgument("gui", default_value="false", description="RViz 显示 GUI"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="shandong_0_5_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="shandong_0_5_jsp",
            output="screen",
            condition=IfCondition(use_jsp),
            parameters=[{"source_list": ["/joint_states"]}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="shandong_0_5_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=(["-d", ""] if gui.lower() in ("1", "true", "yes") else []),
        ),
    ])
