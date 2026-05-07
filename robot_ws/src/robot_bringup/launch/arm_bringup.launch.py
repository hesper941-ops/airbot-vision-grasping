from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm_driver',
            executable='state_node',
            name='arm_state_node',
            output='screen'
        ),
        Node(
            package='robot_arm_driver',
            executable='move_node',
            name='arm_move_node',
            output='screen'
        ),
        Node(
            package='robot_arm_driver',
            executable='gripper_node',
            name='arm_gripper_node',
            output='screen'
        ),
        Node(
            package='robot_arm_driver',
            executable='init_arm',
            name='arm_init_arm',
            output='screen'
        ),
    ])