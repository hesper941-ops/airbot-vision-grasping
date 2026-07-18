from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    arm_executor_launch = os.path.join(bringup_dir, 'launch', 'arm_executor_demo.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_executor_launch)
        ),
        Node(
            package='robot_tasks',
            executable='camera_target_executor',
            name='camera_target_executor',
            output='screen'
        ),
    ])
