# 机械臂抓取模块调用说明 v0.2

适用对象：大语言模型模块、相机模块、底盘模块、队友调试人员。
当前阶段：不使用 ROS2 Action，不使用 MoveIt，不让外部模块直接控制机械臂底层运动话题。

一句话：外部模块只需要提供 base_link 坐标系下的 /visual_target_base，机械臂抓取任务节点 grasp_task_open_loop.py 会按 open-loop 状态机自动执行抓取。

---

## 0. 主链路强约束

当前唯一推荐主抓取入口是：

```bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

该 launch 启动：

```text
arm_executor_node.py
grasp_task_open_loop.py
```

当前主抓取链路：

```text
Orbbec 相机
  -> duck_detector_node
  -> /duck_position
  -> camera_to_base_transform.py
  -> /visual_target_base
  -> grasp_task_open_loop.py
  -> ArmCommandPort
  -> arm_executor_node.py
  -> AIRBOT SDK
```

必须注意：

```text
1. arm_executor_node.py 是唯一允许直接调用 AIRBOT SDK 的节点。
2. grasp_task_open_loop.py 是当前主抓取状态机入口。
3. 外部模块只需要提供 base_link 坐标系下的 /visual_target_base。
4. 大模型、相机模块、底盘模块不要直接发布机械臂底层运动话题。
```

---

## 1. 禁止作为主链路使用的旧脚本

以下脚本只能作为 legacy/debug 参考，禁止写成当前主抓取入口：

```text
hand_to_eye/auto_pick_from_base.py
hand_to_eye/end_position_publisher.py
任何直接调用 AIRBOT SDK 的旧脚本
```

原因：

```text
1. auto_pick_from_base.py 是 legacy/debug 脚本，不是当前主抓取入口。
2. end_position_publisher.py 可能和 arm_executor_node.py 争抢 SDK/状态发布职责。
3. 当前架构要求 arm_executor_node.py 是唯一 SDK owner。
4. 当前抓取状态机入口是 grasp_task_open_loop.py。
```

如果文档、代码注释或启动说明里出现：

```text
自动抓取入口：auto_pick_from_base.py
```

应立即改为：

```text
自动抓取入口：grasp_task_open_loop.py
启动方式：ros2 launch robot_bringup open_loop_grasp.launch.py
```

---

## 2. X5 实机路径

```text
仓库根目录：/home/sunrise/robot
ROS2 工作区：/home/sunrise/robot/robot_ws
Orbbec 工作区：/home/sunrise/robot/Orbbec_ws
转换脚本目录：/home/sunrise/robot/hand_to_eye
```

后续命令不要写成：

```text
/home/sunrise/robot/airbot-vision-grasping
```

---

## 3. 推荐启动顺序

### 终端 0：启动 AIRBOT 服务

```bash
sudo airbot_server -i can1 -p 50001
```

当前实机使用 can1。如果使用 can0 脚本，必须先确认机械臂实际连接的是 can0，否则不要使用。

### 终端 1：启动 Orbbec 相机

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini2.launch.py
```

### 终端 2：启动目标检测节点

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 run detector duck_detector_node
```

该节点发布 /duck_position。

### 终端 3：启动机械臂执行器与抓取状态机

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

该 launch 启动 arm_executor_node.py 和 grasp_task_open_loop.py。

### 终端 4：启动相机坐标到 base_link 的转换桥

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
python3 /home/sunrise/robot/hand_to_eye/camera_to_base_transform.py
```

该脚本负责 /duck_position -> /visual_target_base。

---

## 4. 话题汇总

| 话题 | 消息类型 | 方向 | 用途 |
|---|---|---|---|
| /duck_position | 以 detector 实际定义为准 | detector 发布；转换桥订阅 | 目标在相机坐标系下的位置 |
| /visual_target_base | robot_msgs/msg/VisualTarget | 转换桥/外部模块发布；抓取状态机订阅 | 当前机械臂抓取的核心输入，必须是 base_link 坐标系 |
| /robot_arm/executor_status | std_msgs/msg/String | 执行器发布；外部模块订阅 | 执行器状态 |
| /robot_arm/joint_state | robot_msgs/msg/ArmJointState | 执行器发布；外部模块订阅 | 当前关节状态 |
| /robot_arm/end_pose | geometry_msgs/msg/PoseStamped | 执行器发布；转换桥订阅 | 当前机械臂末端位姿 |
| /robot_arm/reset_executor | std_msgs/msg/String | 恢复时发布 | clear_error；增强后支持 recover_joint_limit |
| /robot_arm/cart_target | geometry_msgs/msg/PointStamped | 抓取状态机发布；执行器订阅 | 底层运动目标，不建议外部模块直接发布 |
| /robot_arm/target_joint | std_msgs/msg/Float64MultiArray | 抓取状态机发布；执行器订阅 | 底层关节目标，不建议外部模块直接发布 |
| /robot_arm/gripper_cmd | std_msgs/msg/String | 抓取状态机发布；执行器订阅 | 夹爪控制，不建议外部模块直接发布 |
| /robot_arm/speed_profile | std_msgs/msg/String | 抓取状态机发布；执行器订阅 | 速度配置，不建议外部模块直接发布 |

---

## 5. 各节点订阅与发布关系

| 节点 / 脚本 | 订阅话题 | 发布话题 | 作用 |
|---|---|---|---|
| duck_detector_node | 相机图像与深度话题 | /duck_position | 检测小黄鸭并输出相机坐标系下的位置 |
| camera_to_base_transform.py | /duck_position、/robot_arm/end_pose | /visual_target_base | 把相机坐标系目标转换到 base_link |
| grasp_task_open_loop.py | /visual_target_base、/robot_arm/executor_status、/robot_arm/end_pose | /robot_arm/cart_target、/robot_arm/target_joint、/robot_arm/gripper_cmd、/robot_arm/reset_executor | 当前主抓取状态机 |
| arm_executor_node.py | /robot_arm/cart_target、/robot_arm/target_joint、/robot_arm/gripper_cmd、/robot_arm/speed_profile、/robot_arm/reset_executor | /robot_arm/joint_state、/robot_arm/end_pose、/robot_arm/executor_status | 唯一 AIRBOT SDK owner |

---

## 6. 外部模块如何调用抓取

当前阶段，大模型/相机模块只需要发布 /visual_target_base。

不要直接发布：

```text
/robot_arm/cart_target
/robot_arm/target_joint
/robot_arm/gripper_cmd
/robot_arm/speed_profile
```

单次发布示例：

```bash
ros2 topic pub --once /visual_target_base robot_msgs/msg/VisualTarget "{
  header: {frame_id: 'base_link'},
  target_id: 'duck_001',
  object_name: 'duck',
  x: 0.35,
  y: 0.02,
  z: 0.06,
  confidence: 0.90,
  is_stable: true,
  u: 320.0,
  v: 240.0,
  depth: 0.35,
  image_width: 640,
  image_height: 480
}"
```

实际使用建议连续发布若干帧：

```bash
for i in $(seq 1 10); do
  ros2 topic pub --once /visual_target_base robot_msgs/msg/VisualTarget "{
    header: {frame_id: 'base_link'},
    target_id: 'duck_001',
    object_name: 'duck',
    x: 0.35, y: 0.02, z: 0.06,
    confidence: 0.90,
    is_stable: true,
    u: 320.0, v: 240.0, depth: 0.35,
    image_width: 640, image_height: 480
  }"
  sleep 0.2
done
```

---

## 7. 工作空间判断

当前硬边界按 AIRBOT Play 官网工作范围：

```text
official_reach_radius_m = 0.647
```

判断时不要只看目标中心点，要看最终抓取点。

当前候选参数：

```text
front_grasp_x_offset = 0.065
front_grasp_x_offset_max = 0.075
grasp_z_offset = 以当前 YAML 为准
```

计算公式：

```text
final_x = target_x + front_grasp_x_offset
final_y = target_y
final_z = target_z + grasp_z_offset
final_radius = sqrt(final_x^2 + final_y^2 + final_z^2)
```

如果 final_radius <= 0.647，可以进入抓取流程。  
如果 final_radius > 0.647，不要让机械臂抓取，应先让底盘调整位置，再重新识别目标。

---

## 8. 状态监控

查看执行器状态：

```bash
ros2 topic echo /robot_arm/executor_status
```

| 状态 | 含义 | 外部模块处理 |
|---|---|---|
| IDLE | 执行器空闲 | 可以等待目标或准备下一次抓取 |
| BUSY | 正在执行动作 | 不要重复发底层运动命令 |
| DONE | 当前动作完成 | 等待状态机进入下一阶段 |
| ERROR | 执行器错误 | 停止继续发布新目标，进入恢复流程 |
| REJECTED_BUSY | 执行器忙时收到新命令 | 等待 IDLE |
| REJECTED_INVALID_JOINT_LIMIT | 关节目标超出限制 | 不要继续抓，调整目标或底盘 |

---

## 9. 恢复命令

清除错误状态：

```bash
ros2 topic pub --once /robot_arm/reset_executor std_msgs/msg/String "{data: 'clear_error'}"
```

注意：clear_error 只清除 ERROR 状态，不保证机械臂物理位置安全。

增强后触发通用关节限位救援：

```bash
ros2 topic pub --once /robot_arm/reset_executor std_msgs/msg/String "{data: 'recover_joint_limit'}"
```

recover_joint_limit 的设计原则：

```text
1. 由 arm_executor_node.py 内部调用 SDK。
2. grasp_task_open_loop.py 只通过 /robot_arm/reset_executor 触发。
3. 针对 6 个关节通用处理，不写死 J3。
4. 救援过程中不主动打开夹爪。
5. 救援过程中不主动闭合夹爪。
6. 保持救援前夹爪状态，默认假设夹爪可能夹着东西。
```

---

## 10. 不建议大模型直接使用的底层话题

大模型/任务规划模块不要直接发布：

```text
/robot_arm/cart_target
/robot_arm/target_joint
/robot_arm/gripper_cmd
/robot_arm/speed_profile
```

原因：

```text
1. 会绕过目标稳定性检查。
2. 会绕过工作空间检查。
3. 会绕过抓取状态机。
4. 可能导致关节限位、夹爪误开合或物体掉落。
```

这些底层话题应由 grasp_task_open_loop.py 统一控制。

---

## 11. 给大语言模型的固定规则模板

```text
你不能直接控制机械臂关节、笛卡尔运动或夹爪。

当你需要机械臂抓取物体时，必须遵循：

1. 目标坐标必须是 base_link 坐标系下的 x/y/z，单位 m。
2. 先计算最终抓取点：
   final_x = target_x + 0.065
   final_y = target_y
   final_z = target_z + grasp_z_offset
   final_radius = sqrt(final_x^2 + final_y^2 + final_z^2)
3. 如果 final_radius > 0.647 m，不要请求机械臂抓取，应先让底盘调整。
4. 如果目标可抓，连续发布 /visual_target_base，消息类型为 robot_msgs/msg/VisualTarget。
5. 发布后监听 /robot_arm/executor_status。
6. 如果状态为 BUSY，等待；如果 DONE/IDLE，继续观察任务；如果 ERROR，停止发布新目标并请求恢复。
7. 不要直接发布 /robot_arm/cart_target、/robot_arm/target_joint、/robot_arm/gripper_cmd。
8. 当前主入口是 grasp_task_open_loop.py，不是 auto_pick_from_base.py。
```

---

## 12. 调试命令速查

```bash
ros2 topic list | grep -E "duck_position|visual_target_base|robot_arm"

ros2 topic echo /duck_position --once
ros2 topic echo /visual_target_base --once
ros2 topic echo /robot_arm/end_pose --once
ros2 topic echo /robot_arm/joint_state --once
ros2 topic echo /robot_arm/executor_status
```

---

## 13. 当前版本限制与下一阶段

| 项目 | 当前版本 | 下一阶段 |
|---|---|---|
| /grasp_object Action | 未实现 | 后续作为大模型正式调用入口 |
| /grasp/check_target 服务 | 未实现 | 后续用于底盘先问“能不能抓” |
| MoveIt | 未使用 | 暂不接入 |
| 关节限位救援 | clear_error 不等于物理救援 | 后续新增 recover_joint_limit |
| 抓取补偿 | front_grasp_x_offset 候选值 0.065 | 后续根据实机稳定性微调，并设置最大值保护 |
| 主抓取入口 | grasp_task_open_loop.py | 禁止把 auto_pick_from_base.py 写成主入口 |
