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
cd /home/sunrise/robot/robot_ws
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
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

终端 2（视觉坐标桥）：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
python3 /home/sunrise/robot/hand_to_eye/camera_to_base_transform.py
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

## 状态机

```text
IDLE
  -> WAIT_PRE_TARGET      (等待第一个稳定视觉目标，不会超时进入 RECOVER)
  -> SET_GRIPPER_ORIENTATION  (J6 ±90° 旋转补偿)
  -> MOVE_PRE_GRASP       (分段移动到预抓取点)
  -> MOVE_GRASP           (分段下降到抓取点)
  -> CLOSE_GRIPPER
  -> MOVE_LIFT            (分段抬升)
  -> MOVE_RETREAT         (分段返回 safe_pose)
  -> IDLE
```

`require_second_visual_confirm=true` 时，`MOVE_PRE_GRASP` 和 `MOVE_GRASP` 之间会额外启用 `WAIT_GRASP_TARGET` 二次视觉确认。

异常统一进入：

```text
RECOVER -> clear_error (周期重发) -> open gripper -> move safe_pose (分段) -> IDLE
```

## 眼在手上 last-seen fallback 抓取策略

分段靠近过程（`MOVE_PRE_GRASP`、`MOVE_GRASP`）中，目标可能因为相机视角变化而短暂离开视野。为应对这一问题，任务层实现了 last-seen fallback。

### 行为

- 如果 `/visual_target_base` 持续更新（即 detector 持续检测到目标），任务节点持续更新 `active_target_base`，每次分段都使用最新的 `base_link` 坐标。
- 如果目标短暂离开相机视野，任务节点使用 `last_seen_target_base`（最后一次真实检测到目标的 `base_link` 坐标）继续分段靠近。
- 如果超过 `last_seen_target_max_age_sec`（默认 5 s）仍没有新目标到达，状态机进入 `RECOVER`，避免盲抓。

### 职责划分

- **视觉桥接层** `hand_to_eye/camera_to_base_transform.py` 只发布真实 detector 输入产生的新 `/visual_target_base`。不 republish 旧目标、不伪造旧坐标。
- **任务层** `robot_tasks/grasp_task_open_loop.py` 订阅 `/visual_target_base`，维护 `last_seen_target_base` 和 `active_target_base`，在目标短暂丢失时自行 fallback。
- 视觉层仅提供 `no_recent_detector_target` 和 `target_timeout` warning 用于健康监控，不参与抓取决策。

### 相关参数（任务层）

- `use_last_seen_target_on_loss`（默认 `true`）：目标丢失时是否启用 last-seen fallback。
- `visual_lost_grace_sec`（默认 `0.5` s）：短于此时间的目标丢失视为瞬时遮挡，不打印 warning。
- `last_seen_target_max_age_sec`（默认 `5.0` s）：last-seen 目标的最大有效年龄。超过后 `_get_active_target_or_last_seen()` 返回 `None`，触发 `RECOVER`。
- `update_target_during_motion`（默认 `true`）：分段靠近过程中是否根据新到达的 `/visual_target_base` 更新 `active_target_base`。
- `freeze_target_before_close`（默认 `true`）：夹爪闭合前冻结 `active_target_base`，避免闭合瞬间目标坐标跳动。

## Cartesian 分段运动

所有 Cartesian 阶段都自动按 `max_cartesian_step`（默认 0.08 m）分段。每次只发布一小步，到位并停稳后才发下一步，绝不直接发送最终目标。

`max_cartesian_step` 必须小于 AirbotWrapper 的 0.100 m 单步安全限制。

受影响的阶段：`MOVE_PRE_GRASP`、`MOVE_GRASP`、`MOVE_LIFT`、`MOVE_RETREAT`、`RECOVER_RETREAT`。

## WAIT_PRE_TARGET 行为

`WAIT_PRE_TARGET` 是等待第一个视觉目标的阶段，机械臂尚未开始运动。**没有目标时不会触发 RECOVER**，只会周期性 warning（默认 15 s），清空旧目标窗口，并持续等待 `/visual_target_base`。

默认流程不再强制进入 `WAIT_GRASP_TARGET`。启用 `require_second_visual_confirm` 后，`WAIT_GRASP_TARGET`（到达 pre-grasp 后的二次确认）目标丢失**会**进入 RECOVER，因为机械臂已离开安全位姿。

## REJECTED_BUSY

单次 `REJECTED_BUSY` 不立即触发 RECOVER。连续达到 `rejected_busy_recover_threshold`（默认 2）后才进入。executor 回到 `IDLE`/`DONE`、任务状态切换、或成功发布命令后计数器清零。

## RECOVER 中 clear_error 周期重发

executor 处于 `ERROR` 时，每 0.5 s 重复发布 `clear_error`，直到 ERROR 清除，避免"只发一次但 executor 未收到"。

## 关节限位保护

`arm_executor_node` 对所有 `/robot_arm/target_joint` 做全关节限位检查。超限的 joint target 被拒绝并发布 `REJECTED_INVALID_JOINT_LIMIT`，**不自动 clamp**（clamp 仅用于 Cartesian waypoint 的工作空间裁剪）。

保守 AIRBOT Play 限位（确认真实硬件后可放宽）：

| 关节 | 角度范围 | 弧度范围 |
|------|----------|----------|
| J1 | [-180°, +120°] | [-3.1416, +2.0944] |
| J2 | [-170°, +10°] | [-2.9671, +0.1745] |
| J3 | [-5°, +180°] | [-0.0873, +3.1416] |
| J4 | [-148°, +148°] | [-2.5831, +2.5831] |
| J5 | [-100°, +100°] | [-1.7453, +1.7453] |
| J6 | [-170°, +170°] | [-2.9671, +2.9671] |

J6 方向选择：`GraspPlanner.compute_joint6_target()` 同时检测 +90° 和 -90° 两个候选方向，只使用在限位内的方向。都不合法时返回 None，状态机进入 RECOVER。

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

## 手动测试

发布假稳定目标（测试状态机）：

```bash
ros2 topic pub -r 10 /visual_target_base robot_msgs/msg/VisualTarget \
"{header: {frame_id: 'base_link'}, x: 0.35, y: 0.0, z: 0.12, confidence: 0.90, depth: 0.12, image_width: 640, image_height: 480}"
```

Cartesian 目标：

```bash
ros2 topic pub --once /robot_arm/cart_target geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'base_link'}, point: {x: 0.35, y: 0.0, z: 0.35}}"
```

合法 joint target：

```bash
ros2 topic pub --once /robot_arm/target_joint std_msgs/msg/Float64MultiArray \
"{data: [0.0, -0.785, 2.094, -1.571, 1.571, 1.571]}"
```

非法 joint target（J2=1.0 > 0.1745，期望被拒绝 `REJECTED_INVALID_JOINT_LIMIT`）：

```bash
ros2 topic pub --once /robot_arm/target_joint std_msgs/msg/Float64MultiArray \
"{data: [0.0, 1.0, 2.094, -1.571, 1.571, 1.571]}"
```

速度 / 夹爪 / clear_error：

```bash
ros2 topic pub --once /robot_arm/speed_profile std_msgs/msg/String "{data: 'slow'}"
ros2 topic pub --once /robot_arm/gripper_cmd std_msgs/msg/String "{data: 'open'}"
ros2 topic pub --once /robot_arm/gripper_cmd std_msgs/msg/String "{data: 'close'}"
ros2 topic pub --once /robot_arm/reset_executor std_msgs/msg/String "{data: 'clear_error'}"
```
