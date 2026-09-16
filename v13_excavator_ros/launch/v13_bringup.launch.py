from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('v13_excavator_ros')
    params = os.path.join(pkg_share, 'config', 'v13_topics.yaml')

    return LaunchDescription([
        Node(package='v13_excavator_ros', executable='inclinometer_reader', name='v13_inclinometer_reader', output='screen', parameters=[params]),
        Node(package='v13_excavator_ros', executable='lidar_reader', name='v13_lidar_reader', output='screen', parameters=[params]),
        Node(package='v13_excavator_ros', executable='lidar_imu_reader', name='v13_lidar_imu_reader', output='screen', parameters=[params]),
        Node(package='v13_excavator_ros', executable='controller_node', name='v13_controller_node', output='screen', parameters=[params]),
        Node(package='v13_excavator_ros', executable='v13_main_node', name='v13_main_node', output='screen', parameters=[params]),
    ])
