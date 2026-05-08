from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm_driver',
            executable='arm_executor_node',
            name='arm_executor_node',
            output='screen'
        ),
    ])
