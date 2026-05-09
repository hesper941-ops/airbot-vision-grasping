from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# 机械臂视觉引导抓取方案启动文件。
def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    arm_bringup_launch = os.path.join(bringup_dir, 'launch', 'arm_bringup.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_bringup_launch)
        ),
        Node(
            package='robot_tasks',
            executable='camera_target_executor',
            name='camera_target_executor',
            output='screen'
        ),
    ])
