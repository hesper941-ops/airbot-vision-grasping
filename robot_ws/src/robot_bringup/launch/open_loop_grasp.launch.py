"""Launch the default open-loop grasp pipeline.

Default pipeline:
  1. arm_executor_node owns the robot hardware.
  2. Orbbec_ws publishes /visual_target_base in base_link separately.
  3. grasp_task_open_loop consumes /visual_target_base and sends commands.

The old camera-to-base bridge is removed from this flow; publish
/visual_target_base directly from the vision workspace.
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

    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'arm_bringup.launch.py')),
        launch_arguments={'config_file': LaunchConfiguration('config_file')}.items(),
    )

    open_loop_node = Node(
        package='robot_tasks',
        executable='grasp_task_open_loop',
        name='grasp_task_open_loop',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_arg,
        arm_bringup,
        open_loop_node,
    ])
