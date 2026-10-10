# File: describe_60FED/launch/display.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
import xacro

def generate_launch_description():
    # 获取功能包的路径
    pkg_path = get_package_share_directory('describe_60FED')
    # 构建URDF文件的完整路径
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'describe_60FED.urdf')

    # 读取URDF文件内容
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 定义机器人状态发布者节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]
    )

    # 定义joint_state_publisher节点（可选，用于发布虚拟关节状态）
    # 如果没有真实的硬件或仿真器提供/joint_states话题，这个节点允许你用GUI滑块控制关节
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # 定义rviz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', ''], # 启动时不加载特定配置，用户可手动配置或保存后指定
        parameters=[{'use_sim_time': False}]
    )

    # 将所有节点加入LaunchDescription
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node, # 如果不需要GUI控制关节，可以注释掉这一行
        rviz_node
    ])
