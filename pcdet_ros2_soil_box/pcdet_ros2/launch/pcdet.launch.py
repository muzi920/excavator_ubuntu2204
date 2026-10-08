"""Launch the OpenPCDet ROS 2 inference node."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _runtime_pythonpath():
    """Exclude Ubuntu test packages that conflict with venv runtime libs."""
    system_dist_packages = os.path.normpath(
        '/usr/lib/python3/dist-packages'
    )
    python_paths = os.environ.get('PYTHONPATH', '').split(os.pathsep)
    return os.pathsep.join(
        path for path in python_paths
        if os.path.normpath(path) != system_dist_packages
    )


def generate_launch_description():
    """Create the configurable PCDet launch description."""
    package_name = 'pcdet_ros2'
    package_dir = get_package_share_directory(package_name)
    config_file = 'pcdet_pvrcnn.param.yaml'

    params_file = LaunchConfiguration('params_file')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    highest_points_topic = LaunchConfiguration('highest_points_topic')
    highest_point_markers_topic = LaunchConfiguration(
        'highest_point_markers_topic'
    )
    box_highest_points_topic = LaunchConfiguration('box_highest_points_topic')
    box_highest_point_markers_topic = LaunchConfiguration(
        'box_highest_point_markers_topic'
    )
    # cmd_dig_point_topic = LaunchConfiguration('cmd_dig_point_topic')  # cmd_dig_point disabled
    cmd_full_cycle_topic = LaunchConfiguration('cmd_full_cycle_topic')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'config', config_file),
        description=(
            'Full path to the ROS 2 parameters file used by the node'
        )
    )

    declare_input_topic_cmd = DeclareLaunchArgument(
        'input_topic',
        default_value='/kitti/point_cloud',
        description='Input Point Cloud'
    )

    declare_output_topic_cmd = DeclareLaunchArgument(
        'output_topic',
        default_value='cloud_detections',
        description='Output Object Detections'
    )

    declare_highest_points_topic_cmd = DeclareLaunchArgument(
        'highest_points_topic',
        default_value='soil_highest_points',
        description='Output coordinates of the highest point in each box'
    )

    declare_highest_point_markers_topic_cmd = DeclareLaunchArgument(
        'highest_point_markers_topic',
        default_value='soil_highest_point_markers',
        description='RViz markers for detected soil highest points'
    )

    declare_box_highest_points_topic_cmd = DeclareLaunchArgument(
        'box_highest_points_topic',
        default_value='box_highest_points',
        description='Output coordinates of the Box top-center point'
    )

    declare_box_highest_point_markers_topic_cmd = DeclareLaunchArgument(
        'box_highest_point_markers_topic',
        default_value='box_highest_point_markers',
        description='RViz markers for the Box top-center point'
    )

    # declare_cmd_dig_point_topic_cmd = DeclareLaunchArgument(
    #     'cmd_dig_point_topic',
    #     default_value='/excavator/cmd_dig_point',
    #     description='Digging point command sent to the excavator'
    # )

    declare_cmd_full_cycle_topic_cmd = DeclareLaunchArgument(
        'cmd_full_cycle_topic',
        default_value='/excavator/cmd_full_cycle',
        description='Full dig/swing/dump/return cycle command for the excavator'
    )

    pcdet = Node(
        package=package_name,
        executable='pcdet',
        name='pcdet',
        output='screen',
        parameters=[
            params_file,
            {'package_folder_path': package_dir},
        ],
        remappings=[
            ('input', input_topic),
            ('output', output_topic),
            ('highest_points', highest_points_topic),
            ('highest_point_markers', highest_point_markers_topic),
            ('box_highest_points', box_highest_points_topic),
            ('box_highest_point_markers', box_highest_point_markers_topic),
            # ('cmd_dig_point', cmd_dig_point_topic),
            ('cmd_full_cycle', cmd_full_cycle_topic),
        ],
    )

    ld = LaunchDescription()

    # `/usr/lib/python3/dist-packages` is needed only while the system colcon
    # command builds this package. At runtime it exposes Ubuntu's coverage.py
    # to venv Numba, and incompatible releases make OpenPCDet fail to import.
    ld.add_action(SetEnvironmentVariable('PYTHONPATH', _runtime_pythonpath()))
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_input_topic_cmd)
    ld.add_action(declare_output_topic_cmd)
    ld.add_action(declare_highest_points_topic_cmd)
    ld.add_action(declare_highest_point_markers_topic_cmd)
    ld.add_action(declare_box_highest_points_topic_cmd)
    ld.add_action(declare_box_highest_point_markers_topic_cmd)
    # ld.add_action(declare_cmd_dig_point_topic_cmd)
    ld.add_action(declare_cmd_full_cycle_topic_cmd)
    ld.add_action(pcdet)

    return ld
