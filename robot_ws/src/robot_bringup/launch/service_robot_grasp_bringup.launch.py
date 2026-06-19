"""Unified service-robot grasp bringup entrypoint.

Default startup includes only the main ROS2 grasp chain:
  1. arm_executor_node
  2. grasp_task_open_loop

External LLM or task modules must publish robot_msgs/msg/VisualTarget to
/visual_target_base with header.frame_id == base_link. They must not publish
directly to /robot_arm/cart_target, /robot_arm/cart_waypoints,
/robot_arm/target_joint, or /robot_arm/gripper_cmd.

/robot_arm/cart_waypoints is an internal execution topic from
grasp_task_open_loop to arm_executor_node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    default_config = os.path.join(bringup_dir, 'config', 'open_loop_grasp.yaml')

    enable_arm = LaunchConfiguration('enable_arm')
    enable_grasp_task = LaunchConfiguration('enable_grasp_task')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_detector = LaunchConfiguration('enable_detector')
    enable_transform = LaunchConfiguration('enable_transform')
    enable_llm_interface = LaunchConfiguration('enable_llm_interface')
    enable_mock_target = LaunchConfiguration('enable_mock_target')
    config_file = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')

    use_open_loop_launch = PythonExpression([
        "'", enable_arm, "'.lower() == 'true' and '",
        enable_grasp_task, "'.lower() == 'true'",
    ])
    arm_only = PythonExpression([
        "'", enable_arm, "'.lower() == 'true' and '",
        enable_grasp_task, "'.lower() != 'true'",
    ])
    grasp_only = PythonExpression([
        "'", enable_arm, "'.lower() != 'true' and '",
        enable_grasp_task, "'.lower() == 'true'",
    ])
    neither_arm_nor_grasp = PythonExpression([
        "'", enable_arm, "'.lower() != 'true' and '",
        enable_grasp_task, "'.lower() != 'true'",
    ])

    arguments = [
        DeclareLaunchArgument('enable_arm', default_value='true'),
        DeclareLaunchArgument('enable_grasp_task', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('enable_detector', default_value='false'),
        DeclareLaunchArgument('enable_transform', default_value='false'),
        DeclareLaunchArgument('enable_llm_interface', default_value='true'),
        DeclareLaunchArgument('enable_mock_target', default_value='false'),
        DeclareLaunchArgument(
            'target_source',
            default_value='llm',
            description='Documented target source: llm, camera, or mock.',
        ),
        DeclareLaunchArgument('visual_target_topic', default_value='/visual_target_base'),
        DeclareLaunchArgument('camera_target_topic', default_value='/duck_position'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'transform_script',
            default_value='/home/sunrise/robot/hand_to_eye/camera_to_base_transform.py',
            description='Site-specific path to hand_to_eye/camera_to_base_transform.py.',
        ),
    ]

    open_loop_grasp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'open_loop_grasp.launch.py')
        ),
        launch_arguments={'config_file': config_file}.items(),
        condition=IfCondition(use_open_loop_launch),
    )

    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'arm_bringup.launch.py')
        ),
        launch_arguments={'config_file': config_file}.items(),
        condition=IfCondition(arm_only),
    )

    grasp_task = Node(
        package='robot_tasks',
        executable='grasp_task_open_loop',
        name='grasp_task_open_loop',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(grasp_only),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'gemini2.launch.py',
            ])
        ),
        condition=IfCondition(enable_camera),
    )

    detector = Node(
        package='detector',
        executable='duck_detector_node',
        name='duck_detector_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(enable_detector),
    )

    transform = ExecuteProcess(
        cmd=['python3', LaunchConfiguration('transform_script')],
        output='screen',
        condition=IfCondition(enable_transform),
    )

    return LaunchDescription(arguments + [
        LogInfo(
            msg=[
                'LLM interface enabled: publish robot_msgs/msg/VisualTarget to ',
                LaunchConfiguration('visual_target_topic'),
                ' with frame_id=',
                LaunchConfiguration('target_frame'),
                '. Do not publish robot arm internal command topics.',
            ],
            condition=IfCondition(enable_llm_interface),
        ),
        LogInfo(
            msg=[
                'Camera target topic is documented as ',
                LaunchConfiguration('camera_target_topic'),
                '; adjust detector/transform launch commands on site if needed.',
            ],
            condition=IfCondition(enable_camera),
        ),
        LogInfo(
            msg='enable_mock_target=true requested, but no mock target node exists in this repo yet. Add the site-specific node name before enabling it.',
            condition=IfCondition(enable_mock_target),
        ),
        LogInfo(
            msg='Both enable_arm and enable_grasp_task are false; no grasp ROS2 node will be started.',
            condition=IfCondition(neither_arm_nor_grasp),
        ),
        open_loop_grasp,
        arm_bringup,
        grasp_task,
        camera,
        detector,
        transform,
    ])
