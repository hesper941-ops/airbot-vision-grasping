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
- `REJECTED_INVALID_JOINT_LIMIT`

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

- `WAIT_PRE_TARGET`：稳定目标只用于生成预抓取位姿。**没有视觉目标时不会触发 RECOVER**，
  只会周期性 warning 并清空旧目标窗口，持续等待 `/visual_target_base`。
- `WAIT_GRASP_TARGET`：机械臂到达 pre-grasp 并停稳后，重新等待新稳定目标，用于真正抓取。
  此阶段目标丢失会进入 RECOVER（因为机械臂已离开安全位姿）。

运动阶段中新视觉结果只会更新 `latest_target`，不会修改当前已经下发的 `active_motion_goal`。

## Cartesian 分段运动

所有 Cartesian 运动阶段都会自动按 `max_cartesian_step`（默认 0.08 m）分段执行。
每次只发布一小步，到位并停稳后才发下一步，绝不直接发送最终目标。

`max_cartesian_step` 必须小于 AirbotWrapper 的 0.100 m 单步安全限制。

以下状态都使用 Cartesian 分段：

- `MOVE_PRE_GRASP`
- `MOVE_GRASP`
- `MOVE_LIFT`
- `MOVE_RETREAT`
- `RECOVER_RETREAT`

例如 safe_pose 距离当前末端 0.268 m 时，会分成约 4 步执行（0.08 + 0.08 + 0.08 + 0.028）。

## 关节限位保护

### arm_executor_node 是最后一道安全防线

`arm_executor_node` 对所有 `/robot_arm/target_joint` 命令执行全关节限位检查。
超出限位的 joint target 会被拒绝并发布 `REJECTED_INVALID_JOINT_LIMIT`，**不会**自动裁剪（clamp）。

自动裁剪只适用于 Cartesian waypoint 的工作空间裁剪，不适用于 joint target。

### 当前保守 AIRBOT Play 关节限位

确认真实硬件版本后，可以在 `open_loop_grasp.yaml` 中调整 `joint_min_rad` / `joint_max_rad`。

| 关节 | 角度范围 | 弧度范围 |
|------|----------|----------|
| J1 | [-180°, +120°] | [-3.1416, +2.0944] |
| J2 | [-170°, +10°] | [-2.9671, +0.1745] |
| J3 | [-5°, +180°] | [-0.0873, +3.1416] |
| J4 | [-148°, +148°] | [-2.5831, +2.5831] |
| J5 | [-100°, +100°] | [-1.7453, +1.7453] |
| J6 | [-170°, +170°] | [-2.9671, +2.9671] |

### init pose 也会检查

`_move_to_init_pose()` 中会将 `init_joint_pos_deg` 转换为弧度后调用同一个 joint limit 检查函数。
如果 init pose 超限或长度不是 6，executor 进入 ERROR，不执行 move_joints。

## REJECTED_BUSY 处理

单次 `REJECTED_BUSY` 不会立即触发 RECOVER。任务节点会累计连续 `REJECTED_BUSY` 次数，
仅当达到 `rejected_busy_recover_threshold`（默认 2）时才进入 RECOVER。

当 executor 回到 `IDLE` / `DONE`、任务状态切换、或成功发布新命令后，计数器清零。

## RECOVER 阶段 clear_error 周期重发

当 executor 处于 `ERROR` 状态时，任务节点每 0.5 秒重复发布 `clear_error`，
直到 executor 不再处于 `ERROR`。这避免了"只发一次但 executor 未收到"的问题。

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

### 关节限位测试

测试合法 joint target：

```bash
ros2 topic pub --once /robot_arm/target_joint std_msgs/msg/Float64MultiArray \
"{data: [0.0, -0.785, 2.094, -1.571, 1.571, 1.571]}"
```

测试非法 joint target（J2 超限，期望被拒绝并发布 REJECTED_INVALID_JOINT_LIMIT）：

```bash
ros2 topic pub --once /robot_arm/target_joint std_msgs/msg/Float64MultiArray \
"{data: [0.0, 1.0, 2.094, -1.571, 1.571, 1.571]}"
```

预期：
- 非法 joint target 被拒绝。
- executor_status 发布 `REJECTED_INVALID_JOINT_LIMIT`。
- 不调用 SDK move_joints。

测试 clear_error：

```bash
ros2 topic pub --once /robot_arm/reset_executor std_msgs/msg/String "{data: 'clear_error'}"
```
