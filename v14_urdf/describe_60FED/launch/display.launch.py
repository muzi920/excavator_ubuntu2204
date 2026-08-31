import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('describe_60FED')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(pkg_path, 'urdf', 'describe_60FED_calibrated.urdf'),
        description='Absolute path to the URDF model file to load.'
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Disable RViz and GUI tools for SSH or headless environments.'
    )
    use_joint_state_publisher_arg = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='true',
        description='Enable joint_state_publisher or joint_state_publisher_gui.'
    )

    urdf_file_path = LaunchConfiguration('model')
    headless = LaunchConfiguration('headless')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    robot_desc = ParameterValue(Command(['cat ', urdf_file_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        arguments=[urdf_file_path],
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }],
        condition=IfCondition(
            PythonExpression([
                "'", headless, "' == 'false' and '",
                use_joint_state_publisher, "' == 'true'"
            ])
        )
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        arguments=[urdf_file_path],
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }],
        condition=IfCondition(
            PythonExpression([
                "'", headless, "' == 'true' and '",
                use_joint_state_publisher, "' == 'true'"
            ])
        )
    )

    rviz_config_path = os.path.join(pkg_path, 'config', 'describe_60FED.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=UnlessCondition(headless)
    )

    return LaunchDescription([
        model_arg,
        headless_arg,
        use_joint_state_publisher_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node
    ])
