"""方案二：视觉伺服抓取启动文件。

启动内容：
  1. arm_executor_node（执行层，唯一硬件 owner）
  2. grasp_task_visual_servo（任务层，粗定位 + 眼在手上闭环微调）

使用方法：
  ros2 launch robot_bringup visual_servo_grasp.launch.py
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

    # 配置文件路径
    default_config = os.path.join(
        bringup_dir, 'config', 'visual_servo_grasp.yaml')

    # 可覆盖的参数文件路径
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='方案二（视觉伺服抓取）参数 YAML 文件路径'
    )

    # 包含执行层（arm_executor_node），传入同一份配置文件
    arm_bringup_launch = os.path.join(
        bringup_dir, 'launch', 'arm_bringup.launch.py')
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(arm_bringup_launch),
        launch_arguments={'config_file': LaunchConfiguration('config_file')}.items(),
    )

    # 视觉伺服抓取任务节点
    visual_servo_node = Node(
        package='robot_tasks',
        executable='grasp_task_visual_servo',
        name='grasp_task_visual_servo',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_arg,
        arm_bringup,
        visual_servo_node,
    ])
