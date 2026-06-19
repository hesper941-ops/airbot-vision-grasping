"""Launch the single-owner AIRBOT arm executor node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    default_config = os.path.join(bringup_dir, 'config', 'arm_executor.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='arm_executor_node parameter YAML file',
    )
    executor_log_level_arg = DeclareLaunchArgument(
        'executor_log_level',
        default_value='warn',
        description='ROS log level for arm_executor_node',
    )

    return LaunchDescription([
        config_arg,
        executor_log_level_arg,
        Node(
            package='robot_arm_driver',
            executable='arm_executor_node',
            name='arm_executor_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            arguments=[
                '--ros-args',
                '--log-level',
                LaunchConfiguration('executor_log_level'),
            ],
        ),
    ])
