# 机械臂抓取模块调用说明

适用对象：大语言模型模块、相机模块、底盘模块、联调同学。

当前阶段：

- 不使用 ROS2 Action。
- 不使用 MoveIt。
- 不新增 `/grasp_object` Action。
- 不新增 `/grasp/check_target` 服务。
- 外部模块不要直接发布 `/robot_arm/cart_target`、`/robot_arm/target_joint`、`/robot_arm/gripper_cmd`。

一句话说明：外部模块只需要提供 `base_link` 坐标系下的 `/visual_target_base`，`grasp_task_open_loop.py` 会按 open-loop 状态机执行抓取，`arm_executor_node.py` 是唯一 AIRBOT SDK owner。

---

## 0. 主链路约束

当前唯一推荐启动入口：

```bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

该 launch 只应启动：

```text
arm_executor_node.py
grasp_task_open_loop.py
```

当前主链路：

```text
duck_detector_node
  -> /duck_position
  -> camera_to_base_transform.py
  -> /visual_target_base
  -> grasp_task_open_loop.py
  -> ArmCommandPort
  -> arm_executor_node.py
  -> AIRBOT SDK
```

必须注意：

1. `arm_executor_node.py` 是唯一允许直接调用 AIRBOT SDK 的节点。
2. `grasp_task_open_loop.py` 是当前主抓取入口。
3. 不要恢复 legacy auto-pick 脚本到当前入口。
4. 不要把实验性桥接节点接入当前主链路。

---

## 1. Legacy / Debug 脚本

以下脚本只能作为 legacy/debug 参考，禁止写成当前主抓取入口：

```text
hand_to_eye/auto_pick_from_base.py
任何直接调用 AIRBOT SDK 的旧脚本
```

正确表述：

```text
自动抓取主入口：grasp_task_open_loop.py
启动方式：ros2 launch robot_bringup open_loop_grasp.launch.py
```

禁止表述：

```text
把 legacy auto-pick 脚本写成当前自动抓取入口
把 legacy 调试脚本写进当前主数据流
```

---

## 2. X5 实机路径

```text
仓库根目录：/home/sunrise/robot
ROS2 工作区：/home/sunrise/robot/robot_ws
Orbbec 工作区：/home/sunrise/robot/Orbbec_ws
转换脚本：/home/sunrise/robot/hand_to_eye/camera_to_base_transform.py
```

文档中不要再出现旧仓库子目录路径写法。

---

## 3. 推荐启动顺序

终端 0：启动 AIRBOT 服务

```bash
sudo airbot_server -i can1 -p 50001
```

终端 1：启动 Orbbec 相机

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini2.launch.py
```

终端 2：启动检测节点

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 run detector duck_detector_node
```

终端 3：启动主抓取链路

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

终端 4：启动相机到 `base_link` 的转换桥

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
python3 /home/sunrise/robot/hand_to_eye/camera_to_base_transform.py
```

---

## 4. 关键话题

| 话题 | 消息类型 | 方向 | 用途 |
|---|---|---|---|
| `/duck_position` | 以 detector 实际定义为准 | detector 发布；转换桥订阅 | 相机坐标系下的目标 |
| `/visual_target_base` | `robot_msgs/msg/VisualTarget` | 转换桥或外部模块发布；抓取状态机订阅 | 主抓取输入，必须是 `base_link` 坐标系 |
| `/robot_arm/executor_status` | `std_msgs/msg/String` | 执行器发布 | 执行器状态 |
| `/robot_arm/joint_state` | `robot_msgs/msg/ArmJointState` | 执行器发布 | 当前关节状态 |
| `/robot_arm/end_pose` | `geometry_msgs/msg/PoseStamped` | 执行器发布 | 当前末端位姿 |
| `/robot_arm/reset_executor` | `std_msgs/msg/String` | 抓取状态机发布；执行器订阅 | 恢复命令，支持 `clear_error` 和 `recover_joint_limit` |
| `/robot_arm/cart_target` | `geometry_msgs/msg/PointStamped` | 抓取状态机发布；执行器订阅 | 底层笛卡尔目标 |
| `/robot_arm/target_joint` | `std_msgs/msg/Float64MultiArray` | 抓取状态机发布；执行器订阅 | 底层关节目标 |
| `/robot_arm/gripper_cmd` | `std_msgs/msg/String` | 抓取状态机发布；执行器订阅 | 夹爪控制 |

---

## 5. 当前边界

| 项目 | 当前状态 |
|---|---|
| ROS2 Action | 未接入 |
| `/grasp/check_target` 服务 | 未接入 |
| MoveIt | 未接入 |
| detector | 保持现有链路，不在本次任务改动 |
| 主抓取入口 | `grasp_task_open_loop.py` |
| 推荐启动方式 | `ros2 launch robot_bringup open_loop_grasp.launch.py` |
