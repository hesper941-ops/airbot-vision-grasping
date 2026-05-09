# robot_ws

这是 AIRBOT Play 机械臂侧 ROS 2 工作区。

当前默认架构保持简单清晰：

- `robot_arm_driver`：唯一硬件执行层，只有 `arm_executor_node` 会调用 AIRBOT SDK。
- `robot_tasks`：只负责任务和状态机逻辑，不直接调用 SDK。
- `robot_msgs`：共享消息定义。
- `robot_bringup`：启动文件和参数配置。

## 默认开环流程

1. 启动 `airbot_server`。
2. 启动 `arm_executor_node`。
3. 启动 Orbbec 视觉流程，由视觉侧直接发布 `base_link` 坐标系下的 `/visual_target_base`。
4. 启动 `grasp_task_open_loop`。

旧的 camera-to-base 桥接节点和 camera-target demo 节点已经删除。请从视觉工作区直接发布 `/visual_target_base`。

## `/visual_target_base` 接口约定

Topic：`/visual_target_base`

类型：`robot_msgs/msg/VisualTarget`

字段语义：

- `header.frame_id`：必须是 `base_link`
- `header.stamp`：视觉结果的最新时间戳
- `x`, `y`, `z`：机器人 `base_link` 坐标系下的目标中心点或抓取点，单位米
- `confidence`：检测置信度，通常范围为 `0.0` 到 `1.0`
- `depth`：目标深度，单位米；如果可用，应保持有限且稳定
- `is_stable`：视觉侧可选稳定标志；机械臂任务节点仍会做自己的稳定性判断

默认流程中，机械臂侧不做 camera-to-base 坐标转换。

## 机械臂侧 Topic

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
- `/robot_arm/reset_executor` (`std_msgs/msg/String`，恢复流程中用于发布 `clear_error`)

执行器状态包括：

- `IDLE`
- `BUSY`
- `DONE`
- `ERROR`
- `TIMEOUT`
- `REJECTED_BUSY`

## 开环状态机

```text
IDLE
  -> WAIT_PRE_TARGET
  -> SET_GRIPPER_ORIENTATION
  -> MOVE_PRE_GRASP
  -> WAIT_GRASP_TARGET
  -> MOVE_GRASP
  -> CLOSE_GRIPPER
  -> MOVE_LIFT
  -> MOVE_RETREAT
  -> IDLE
```

所有异常统一进入：

```text
RECOVER -> open gripper -> move safe_pose -> IDLE
```

任务节点会等待两次稳定视觉目标：

- `WAIT_PRE_TARGET`：稳定目标只用于生成预抓取位姿。
- `WAIT_GRASP_TARGET`：机械臂到达 pre-grasp 并停稳后，重新等待新稳定目标，用于真正抓取。

运动阶段中新视觉结果只会更新 `latest_target`，不会修改当前已经下发的 `active_motion_goal`。

## 编译

```bash
cd ~/robot/airbot-vision-grasping/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 运行

终端 1：

```bash
sudo airbot_server -i can1 -p 50001
```

终端 2：

```bash
cd ~/robot/airbot-vision-grasping/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

终端 3：启动 Orbbec 视觉流程，让它发布 `base_link` 坐标系下的 `/visual_target_base`。

## Topic 手动测试

查看执行器状态：

```bash
ros2 topic echo /robot_arm/executor_status
```

查看机械臂末端位姿：

```bash
ros2 topic echo /robot_arm/end_pose
```

发布重复的假稳定目标，用于测试状态机：

```bash
ros2 topic pub -r 10 /visual_target_base robot_msgs/msg/VisualTarget \
"{header: {frame_id: 'base_link'}, target_id: 'test_1', object_name: 'test', x: 0.35, y: 0.0, z: 0.12, confidence: 0.90, is_stable: true, u: 0.0, v: 0.0, depth: 0.12, image_width: 640, image_height: 480}"
```

手动发送笛卡尔目标：

```bash
ros2 topic pub --once /robot_arm/cart_target geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'base_link'}, point: {x: 0.35, y: 0.0, z: 0.35}}"
```

切换速度档位：

```bash
ros2 topic pub --once /robot_arm/speed_profile std_msgs/msg/String "{data: 'slow'}"
```

打开或关闭夹爪：

```bash
ros2 topic pub --once /robot_arm/gripper_cmd std_msgs/msg/String "{data: 'open'}"
ros2 topic pub --once /robot_arm/gripper_cmd std_msgs/msg/String "{data: 'close'}"
```
