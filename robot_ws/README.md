# robot_ws

AIRBOT Play 机械臂侧 ROS 2 工作区。

主链路中，`arm_executor_node` 是唯一调用 AIRBOT SDK 的节点。它发布 `/robot_arm/end_pose`、`/robot_arm/joint_state` 和 `/robot_arm/executor_status`，并接收任务节点发来的运动、夹爪和速度命令。

## 主链路

```text
/visual_target_base
-> grasp_task_open_loop
-> /robot_arm/cart_target、/robot_arm/target_joint、/robot_arm/gripper_cmd
-> arm_executor_node
-> AIRBOT SDK
```

`/visual_target_base` 由 `hand_to_eye/camera_to_base_transform.py` 发布。该脚本需要同时 source `robot_ws`，否则 `robot_msgs/msg/VisualTarget` 不可见。

不要在主链路中启动 `hand_to_eye/end_position_publisher.py`。它会直接连接 AIRBOT SDK，只能用于旧调试链路。

## 编译

```bash
cd /home/sunrise/robot/airbot-vision-grasping/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动

终端 0：

```bash
sudo airbot_server -i can1 -p 50001
```

终端 1：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/airbot-vision-grasping/robot_ws/install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

## Topic

`grasp_task_open_loop` 订阅：

- `/visual_target_base` (`robot_msgs/msg/VisualTarget`)
- `/robot_arm/joint_state` (`robot_msgs/msg/ArmJointState`)
- `/robot_arm/end_pose` (`geometry_msgs/msg/PoseStamped`)
- `/robot_arm/executor_status` (`std_msgs/msg/String`)

`grasp_task_open_loop` 发布：

- `/robot_arm/target_joint` (`std_msgs/msg/Float64MultiArray`)
- `/robot_arm/cart_target` (`geometry_msgs/msg/PointStamped`)
- `/robot_arm/gripper_cmd` (`std_msgs/msg/String`)
- `/robot_arm/speed_profile` (`std_msgs/msg/String`)
- `/robot_arm/reset_executor` (`std_msgs/msg/String`)

`arm_executor_node` 额外支持结构化夹爪命令：

- `/robot_arm/gripper_command` (`robot_msgs/msg/GripperCommand`)

当前 `grasp_task_open_loop` 仍使用兼容的 String 版本 `/robot_arm/gripper_cmd`。后续需要使用视觉估计夹爪宽度时，可以切换到 `/robot_arm/gripper_command`。

执行器状态包括：

- `IDLE`
- `BUSY`
- `DONE`
- `ERROR`
- `REJECTED_BUSY`
- `REJECTED_INVALID_JOINT_LIMIT`

## 最小验证

```bash
ros2 topic echo /robot_arm/end_pose --once
ros2 topic echo /robot_arm/joint_state --once
ros2 topic echo /robot_arm/executor_status
ros2 topic echo /visual_target_base --once
```

如果 `/visual_target_base` 有数据但机械臂不动，优先检查：

- `airbot_server`
- CAN 口，例如 `can1`
- `arm_executor_node`
- `/robot_arm/executor_status`
