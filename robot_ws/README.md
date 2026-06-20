# robot_ws

AIRBOT Play 机械臂侧 ROS 2 工作区。当前主线是 AIRBOT Play 机械臂 + Orbbec 相机的 open_loop 连续 waypoint 抓取。

主链路中，`arm_executor_node` 是唯一直接调用 AIRBOT SDK 的节点。`grasp_task_open_loop` 只发布运动、夹爪、速度和恢复命令，不直接访问 AIRBOT SDK。

## 主链路

```text
/visual_target_base
  -> grasp_task_open_loop
  -> /robot_arm/cart_waypoints   # main blended approach path
  -> /robot_arm/cart_target      # sequential fallback only
  -> /robot_arm/target_joint     # return-init / recover / non-approach stages
  -> /robot_arm/gripper_cmd
  -> /robot_arm/speed_profile
  -> arm_executor_node
  -> AIRBOT SDK move_cart_waypoints
```

`/visual_target_base` 由 `hand_to_eye/camera_to_base_transform.py` 发布，坐标必须已经在 `base_link` 下。不要在主链路中启动 `hand_to_eye/end_position_publisher.py`，它会直接连接 AIRBOT SDK，只能用于旧调试链路。

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

确认 50001 已监听：

```bash
sudo ss -lntp | grep 50001
```

再启动 open_loop 抓取：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py task_log_level:=info executor_log_level:=warn
```

视觉坐标桥另开终端：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
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

- `/robot_arm/cart_waypoints` (`geometry_msgs/msg/PoseArray`): main blended approach path
- `/robot_arm/cart_target` (`geometry_msgs/msg/PointStamped`): sequential fallback only
- `/robot_arm/target_joint` (`std_msgs/msg/Float64MultiArray`): return-init / recover / non-approach stages
- `/robot_arm/gripper_cmd` (`std_msgs/msg/String`)
- `/robot_arm/speed_profile` (`std_msgs/msg/String`)
- `/robot_arm/reset_executor` (`std_msgs/msg/String`)

执行器状态包括：

- `IDLE`
- `BUSY`
- `DONE`
- `ERROR`
- `TIMEOUT`
- `REJECTED_BUSY`
- `REJECTED_INVALID_JOINT_LIMIT`

## Open Loop 状态机

当前主流程：

```text
WAIT_PRE_TARGET
  -> PRE_OPEN_GRIPPER
  -> MOVE_APPROACH_BLEND
  -> CLOSE_GRIPPER
  -> MOVE_LIFT
  -> RETURN_INIT_POSE
```

当前推荐主线是 AIRBOT 官方多段轨迹融合 `/robot_arm/cart_waypoints`，由 `arm_executor_node` 调用 `move_cart_waypoints` 执行。open_loop 只在 `WAIT_PRE_TARGET` 阶段做目标稳定判断和 preflight planning。执行阶段使用冻结的 `selected_plan`，`MOVE_APPROACH_BLEND` 直接发送 `selected_plan.pre_grasp -> selected_plan.grasp`，不做二次视觉确认。

`CLOSE_GRIPPER` 不能融合进轨迹：机械臂必须在 grasp 点停下，先闭合夹爪，再进入 `MOVE_LIFT` 抬升。

`MOVE_PRE_GRASP -> MOVE_GRASP` sequential 路径仍保留，但只作为 `MOVE_APPROACH_BLEND` 失败或 `blend_approach_enabled=false` 时的 fallback。

当前相机安装位置下，open_loop 不再进入 `SET_GRIPPER_ORIENTATION`，抓取前不再发布 J6 补偿 joint target。

## J6 策略

当前 open_loop 初始/回位关节角：

```text
[0.0, -45.0, 110.0, -90.0, 90.0, 0.0]
```

也就是说，J6 初始/回位角为 `0 deg`。

`GraspPlanner.compute_joint6_target()` 仍作为 visual_servo/旧流程兼容 helper 保留，但不参与 open_loop 主流程。

## 抓取前预规划

`WAIT_PRE_TARGET` 收到稳定目标后，不会立即打开夹爪。任务节点会先按 `approach_priority` 做完整预规划检查：

- 检查目标点 `target_base`
- 生成并检查 `pre_grasp`
- 生成并检查 `grasp`
- 生成并检查 `lift_goal`
- 检查 workspace 和官方 reach radius

只有至少一个 approach mode 成功规划，才进入 `PRE_OPEN_GRIPPER`。如果所有 mode 都失败，夹爪保持当前状态，任务停留在 `WAIT_PRE_TARGET` 等待新的稳定目标，并输出 workspace 拒绝原因。

成功的规划会冻结为 `selected_plan`；默认主线中 `MOVE_APPROACH_BLEND` 直接执行其中的 `pre_grasp` 和 `grasp` waypoint，`MOVE_LIFT` 在夹爪闭合后执行 `lift_goal`。sequential fallback 也复用同一个 `selected_plan`，不在执行途中重新等待视觉确认或重新计算目标。

默认优先级：

```yaml
approach_priority: ["front", "top_down"]
```

front 失败时会尝试 top_down；全部失败才作为最终拒绝。

## TODO：实机验证后清理

实机验证 `move_cart_waypoints` 主线跑通后，计划清理：

1. 删除 visual_servo 方案
2. 删除 legacy J6 补偿路径
3. 删除未接入的 active_search / search_pose / old sequence 模块
4. 删除 sequential fallback 或保留为 debug mode
5. 精简 README 和 launch

这些内容本轮先保留，避免在主线实机验证前破坏仓库完整性。

## Front Pre-Grasp 自适应

front 模式下，预抓取点原始计算为：

```text
raw_pre_x = target_x + front_approach_x_offset
```

为避免近距离目标因为固定 `front_approach_x_offset=-0.10` 导致 `pre_grasp.x < workspace_limits.x_min`，当前启用自适应预抓取：

```yaml
adaptive_front_pre_grasp: true
workspace_soft_margin_m: 0.02
min_front_pre_grasp_distance_m: 0.04
```

逻辑：

- 如果 `raw_pre_x` 在 workspace 软边界内，直接使用。
- 如果 `raw_pre_x` 越过软边界，先把 `pre_x` 调整到软边界内。
- 调整后仍必须满足 `target_x - pre_x >= min_front_pre_grasp_distance_m`。
- 如果不满足，front mode 被拒绝，状态机继续尝试 top_down。
- 最终 `grasp` 点不会被静默 clamp；抓取点越界仍会明确报错。

## Workspace 拒绝日志

workspace 拒绝日志会尽量说明：

- 哪个点越界：`target_base`、`pre_grasp`、`grasp`、`lift_goal`、`retreat_goal`
- 哪个轴越界：`x`、`y`、`z`
- 当前 `target_base`
- 尝试的 `pre_grasp`
- 尝试的 `grasp`
- 当前末端 `end_pose`
- `workspace_limits`
- `approach_mode`
- `front_approach_x_offset`

如果实机确认某个区域可达，再调整 YAML 里的 `workspace_limits`。不要通过静默放宽或 clamp 最终抓取点来掩盖真实不可达问题。

## 日志参数

open_loop 任务节点：

```yaml
verbose_debug: false
status_log_period_sec: 2.0
log_waypoint_each_step: false
```

launch 支持：

```bash
ros2 launch robot_bringup open_loop_grasp.launch.py task_log_level:=info executor_log_level:=warn
```

默认 INFO 只保留关键流程：

- 节点启动参数摘要
- 状态切换，例如 `[GRASP] WAIT_PRE_TARGET -> PRE_OPEN_GRIPPER`
- 稳定目标
- 选中的 `approach_mode`
- 固定的 `pre_grasp` / `grasp` / `lift_goal`
- 开夹爪、闭夹爪
- 抓取成功、返回初始位、RECOVER 原因

每个 waypoint step、每次 publish 的详细 reason、安全检查细节默认在 DEBUG，或由 `verbose_debug` / `log_waypoint_each_step` 打开。

## 关节限位保护

`arm_executor_node` 对所有 `/robot_arm/target_joint` 做全关节限位检查。超限的 joint target 会被拒绝并发布 `REJECTED_INVALID_JOINT_LIMIT`，不会自动 clamp。

保守 AIRBOT Play 限位：

| 关节 | 角度范围 | 弧度范围 |
| --- | --- | --- |
| J1 | [-180 deg, +120 deg] | [-3.1416, +2.0944] |
| J2 | [-170 deg, +10 deg] | [-2.9671, +0.1745] |
| J3 | [-5 deg, +180 deg] | [-0.0873, +3.1416] |
| J4 | [-148 deg, +148 deg] | [-2.5831, +2.5831] |
| J5 | [-100 deg, +100 deg] | [-1.7453, +1.7453] |
| J6 | [-170 deg, +170 deg] | [-2.9671, +2.9671] |

## 最小验证

```bash
ros2 topic echo /robot_arm/end_pose --once
ros2 topic echo /robot_arm/joint_state --once
ros2 topic echo /robot_arm/executor_status
ros2 topic echo /visual_target_base --once
```

实机重点看：

- 启动后 `arm_executor_node` 初始位 J6 是否为 `0 deg`
- 抓取前是否不再出现 `SET_GRIPPER_ORIENTATION`
- 抓取前是否不再发布 J6 补偿 joint target
- 近距离目标是否先完成预规划，再打开夹爪
- front 预抓取越界时是否自适应调整，或 fallback 到 top_down
- workspace 拒绝日志是否能看出具体点、轴和参数
- launch 窗口是否只显示关键流程，而不是刷大量 waypoint INFO

## 手动测试

发布假稳定目标：

```bash
ros2 topic pub -r 10 /visual_target_base robot_msgs/msg/VisualTarget \
"{header: {frame_id: 'base_link'}, x: 0.35, y: 0.0, z: 0.12, confidence: 0.90, depth: 0.12, image_width: 640, image_height: 480}"
```

发布 Cartesian 目标：

```bash
ros2 topic pub --once /robot_arm/cart_target geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'base_link'}, point: {x: 0.35, y: 0.0, z: 0.35}}"
```

发布合法 joint target：

```bash
ros2 topic pub --once /robot_arm/target_joint std_msgs/msg/Float64MultiArray \
"{data: [0.0, -0.785, 2.094, -1.571, 1.571, 0.0]}"
```

发布非法 joint target，期望被拒绝为 `REJECTED_INVALID_JOINT_LIMIT`：

```bash
ros2 topic pub --once /robot_arm/target_joint std_msgs/msg/Float64MultiArray \
"{data: [0.0, 1.0, 2.094, -1.571, 1.571, 0.0]}"
```

速度、夹爪、clear_error：

```bash
ros2 topic pub --once /robot_arm/speed_profile std_msgs/msg/String "{data: 'slow'}"
ros2 topic pub --once /robot_arm/gripper_cmd std_msgs/msg/String "{data: 'open'}"
ros2 topic pub --once /robot_arm/gripper_cmd std_msgs/msg/String "{data: 'close'}"
ros2 topic pub --once /robot_arm/reset_executor std_msgs/msg/String "{data: 'clear_error'}"
```
