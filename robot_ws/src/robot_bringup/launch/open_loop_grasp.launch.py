"""方案一：开环抓取启动文件。

启动内容：
  1. arm_executor_node（执行层，唯一硬件 owner）
  2. grasp_task_open_loop（任务层，一次识别 + 分阶段抓取）

使用方法：
  ros2 launch robot_bringup open_loop_grasp.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 包路径
    bringup_dir = get_package_share_directory('robot_bringup')
    tasks_dir = get_package_share_directory('robot_tasks')

    # 配置文件路径
    default_config = os.path.join(bringup_dir, 'config', 'open_loop_grasp.yaml')

    # 可覆盖的参数文件路径
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='方案一（开环抓取）参数 YAML 文件路径'
    )

    # 包含执行层（arm_executor_node），传入同一份配置文件
    arm_bringup_launch = os.path.join(
        bringup_dir, 'launch', 'arm_bringup.launch.py')
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(arm_bringup_launch),
        launch_arguments={'config_file': LaunchConfiguration('config_file')}.items(),
    )

    # 开环抓取任务节点
    bridge_node = Node(
        package='robot_tasks',
        executable='visual_target_bridge',
        name='visual_target_bridge',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
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
        bridge_node,
        open_loop_node,
    ])
