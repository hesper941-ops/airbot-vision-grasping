"""Launch the default open-loop grasp pipeline.

This launch file starts:
  1. arm_executor_node
  2. pose_switch_node
  3. grasp_task_open_loop

camera_to_base_transform.py is still part of the recommended main pipeline,
but it is started by the user in another terminal. It converts /duck_position
to /visual_target_base for grasp_task_open_loop.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    default_config = os.path.join(bringup_dir, 'config', 'open_loop_grasp.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Open-loop grasp parameter YAML file',
    )
    task_log_level_arg = DeclareLaunchArgument(
        'task_log_level',
        default_value='info',
        description='ROS log level for grasp_task_open_loop',
    )
    executor_log_level_arg = DeclareLaunchArgument(
        'executor_log_level',
        default_value='warn',
        description='ROS log level for arm_executor_node',
    )

    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'arm_bringup.launch.py')),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'executor_log_level': LaunchConfiguration('executor_log_level'),
        }.items(),
    )

    open_loop_node = Node(
        package='robot_tasks',
        executable='grasp_task_open_loop',
        name='grasp_task_open_loop',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        arguments=[
            '--ros-args',
            '--log-level',
            LaunchConfiguration('task_log_level'),
        ],
    )

    pose_switch_node = Node(
        package='robot_arm_driver',
        executable='pose_switch_node',
        name='pose_switch_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_arg,
        task_log_level_arg,
        executor_log_level_arg,
        arm_bringup,
        pose_switch_node,
        open_loop_node,
    ])
