# robot_ws

AIRBOT Play 机械臂 ROS 2 工作空间。当前主线是：

```text
/visual_target_base
  -> grasp_task_open_loop
  -> arm_executor_node
  -> AIRBOT SDK
```

`arm_executor_node` 是主链路中唯一直接调用 AIRBOT SDK 的节点；抓取任务节点只发布运动、夹爪、速度和恢复命令。

## 功能包

```text
robot_ws/
└── src/
    ├── robot_arm_driver/      # AIRBOT 执行器节点
    ├── robot_arm_interface/   # AIRBOT SDK 封装
    ├── robot_bringup/         # launch 与配置
    ├── robot_msgs/            # 自定义消息
    └── robot_tasks/           # 抓取任务状态机
```

## 编译

```bash
cd /home/sunrise/robot/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动

先启动 AIRBOT server：

```bash
sudo airbot_server -i can1 -p 50001
```

确认监听：

```bash
sudo ss -lntp | grep 50001
```

启动抓取主链路：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py task_log_level:=info executor_log_level:=warn
```

视觉坐标桥由 `hand_to_eye/camera_to_base_transform.py` 提供，需另开终端运行。

## 主链路 Topic

`grasp_task_open_loop` 订阅：

```text
/visual_target_base        robot_msgs/msg/VisualTarget
/robot_arm/joint_state     robot_msgs/msg/ArmJointState
/robot_arm/end_pose        geometry_msgs/msg/PoseStamped
/robot_arm/executor_status std_msgs/msg/String
```

`grasp_task_open_loop` 发布：

```text
/robot_arm/cart_waypoints  geometry_msgs/msg/PoseArray
/robot_arm/cart_target     geometry_msgs/msg/PointStamped
/robot_arm/target_joint    std_msgs/msg/Float64MultiArray
/robot_arm/gripper_cmd     std_msgs/msg/String
/robot_arm/speed_profile   std_msgs/msg/String
/robot_arm/reset_executor  std_msgs/msg/String
```

执行状态：

```text
IDLE
BUSY
DONE
ERROR
TIMEOUT
REJECTED_BUSY
REJECTED_INVALID_JOINT_LIMIT
```

## Open Loop 抓取流程

```text
WAIT_PRE_TARGET
  -> PRE_OPEN_GRIPPER
  -> MOVE_APPROACH_BLEND
  -> CLOSE_GRIPPER
  -> MOVE_LIFT
  -> RETURN_INIT_POSE
```

默认使用 AIRBOT 官方多段轨迹融合 `/robot_arm/cart_waypoints`。`MOVE_APPROACH_BLEND` 发送 `pre_grasp -> grasp`，到达 grasp 点后先闭合夹爪，再进入 `MOVE_LIFT`。

默认抓取方式优先级：

```yaml
approach_priority: ["front", "top_down"]
```

front 失败时会尝试 top_down。

## 关节限位

`arm_executor_node` 会检查 `/robot_arm/target_joint`，超限时发布 `REJECTED_INVALID_JOINT_LIMIT`，不会自动 clamp。

保守 AIRBOT Play 限位：

| 关节 | 角度范围 | 弧度范围 |
| --- | --- | --- |
| J1 | [-180 deg, +120 deg] | [-3.1416, +2.0944] |
| J2 | [-170 deg, +10 deg] | [-2.9671, +0.1745] |
| J3 | [-5 deg, +180 deg] | [-0.0873, +3.1416] |
| J4 | [-148 deg, +148 deg] | [-2.5831, +2.5831] |
| J5 | [-100 deg, +100 deg] | [-1.7453, +1.7453] |
| J6 | [-170 deg, +170 deg] | [-2.9671, +2.9671] |

## 常用调试

```bash
ros2 topic echo /robot_arm/end_pose --once
ros2 topic echo /robot_arm/joint_state --once
ros2 topic echo /robot_arm/executor_status
ros2 topic echo /visual_target_base --once
```

夹爪：

```bash
ros2 topic pub --once /robot_arm/gripper_cmd std_msgs/msg/String "{data: 'open'}"
ros2 topic pub --once /robot_arm/gripper_cmd std_msgs/msg/String "{data: 'close'}"
```

速度和错误复位：

```bash
ros2 topic pub --once /robot_arm/speed_profile std_msgs/msg/String "{data: 'slow'}"
ros2 topic pub --once /robot_arm/reset_executor std_msgs/msg/String "{data: 'clear_error'}"
```
