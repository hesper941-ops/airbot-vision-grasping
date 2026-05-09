"""机械臂执行层启动文件。

只启动 arm_executor_node(唯一硬件 owner)。
可接收外部 config_file 参数来配置初始位姿等。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 生成启动描述
def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    default_config = os.path.join(bringup_dir, 'config', 'arm_executor.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='arm_executor_node 参数 YAML 文件路径'
    )

    # 启动 arm_executor_node，传入配置文件参数
    return LaunchDescription([
        config_arg,
        Node(
            package='robot_arm_driver',
            executable='arm_executor_node',
            name='arm_executor_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
