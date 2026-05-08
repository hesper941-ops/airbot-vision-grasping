# robot_ws

机械臂侧 ROS 2 工作区，用于对接 **求之科技 AIRBOT Play 机械臂**，并与视觉侧目标点接口配合，实现“目标点接收 -> 机械臂执行”的基础链路。

当前仓库中的 `robot_ws` 主要聚焦在：

- 机械臂底层接口封装
- 机械臂状态发布
- 基于目标点的最小运动执行
- 后续完整抓取流程的任务层重构

---

## 1. 当前目标

本工作区当前的重构目标不是一次性把完整抓取全塞进去，而是先把机械臂侧整理成清晰的三层结构：

- **driver 层**：唯一负责机械臂真实执行
- **interface 层**：统一封装 AIRBOT SDK
- **task 层**：只写任务逻辑，不直接碰 SDK

对应包如下：

- `robot_arm_driver`：机械臂执行层 / 状态发布层
- `robot_arm_interface`：AIRBOT SDK Python 封装
- `robot_tasks`：任务节点
- `robot_msgs`：统一消息定义
- `robot_bringup`：启动文件

---

## 2. 工作区结构

```text
robot_ws/
├── src/
│   ├── robot_arm_driver/
│   ├── robot_arm_interface/
│   ├── robot_bringup/
│   ├── robot_msgs/
│   └── robot_tasks/
├── build/
├── install/
└── log/
```

### 2.1 robot_arm_driver

机械臂驱动与执行相关节点，当前主要包括：

- `init_arm.py`：机械臂初始化到预设关节位
- `sleep_arm.py`：机械臂移动到休眠位
- `end_position_publisher.py`：发布 `/robot_arm/end_pose`
- `arm_executor_node.py`：新的统一执行节点（重构方向）
- 兼容保留的旧节点：
  - `state_node.py`
  - `move_node.py`
  - `gripper_node.py`

### 2.2 robot_arm_interface

AIRBOT SDK 的统一 Python 封装层。

当前核心文件：

- `airbot_wrapper.py`

主要职责：

- 连接 / 断开 `airbot_server`
- 设置速度档
- 读取关节 / 末端状态
- 执行关节运动
- 执行笛卡尔 waypoint 运动
- 夹爪开合
- 补充动作等待、到位判断等能力

### 2.3 robot_tasks

任务层节点，不应长期直接操作 SDK。

当前已有：

- `camera_target_executor.py`

用途：

- 接收视觉侧给出的 base 坐标目标点
- 读取目标置信度和稳定性
- 规划候选小步路径
- 只向执行层下发第一小步
- 等待新的视觉反馈后再决定下一步

### 2.4 robot_msgs

机械臂相关统一消息定义。

当前 / 规划中的消息包括：

- `ArmJointTarget.msg`
- `ArmJointState.msg`
- `GripperCommand.msg`
- `ArmCommand.msg`
- `ArmState.msg`
- `CameraTarget.msg`

### 2.5 robot_bringup

负责不同模式下的启动组合，不写具体业务逻辑。

当前常用 launch：

- `arm_bringup.launch.py`
- `camera_target_demo.launch.py`
- `arm_executor_demo.launch.py`

---

## 3. 当前推荐运行模式

当前建议将机械臂侧分成两种模式：

### 3.1 直控模式（推荐当前开发使用）

这条链路用于当前重构中的机械臂侧开发。

特点：

- 直接连接 `airbot_server`
- 用统一执行节点接收命令
- 任务节点只发命令 / 读状态

推荐组成：

- `airbot_server -i can1 -p 50001`
- `arm_executor_node`
- `camera_target_executor`

### 3.2 官方控制栈模式

这一模式对应官方控制器 / MoveIt / ros2_control 相关链路。

当前建议：

- 不要和直控模式混跑
- 如果要调试官方控制栈，请单独起
- 如果要调试当前封装后的机械臂工作区，请优先使用直控模式

---

## 4. 环境依赖

### 4.1 系统
- Ubuntu
- ROS 2 Humble

### 4.2 机械臂
- 求之科技 AIRBOT Play
- 旧版 Python SDK / 对应 `airbot_server`
- 当前控制服务通过 `localhost:50001` 对外提供 SDK 接口

### 4.3 通信链路
当前项目中，机械臂真实控制前提是先启动 `airbot_server`。

示例：

```bash
sudo airbot_server -i can1 -p 50001
```

> 注意：`can1` 只是当前开发环境中的实例接口，实际以你的机器当前有效 CAN 接口为准。

---

## 5. 编译方法

在 `robot_ws` 下执行：

```bash
cd /home/sunrise/robot/robot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

如果只想编译机械臂相关包，可以使用：

```bash
cd /home/sunrise/robot/robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_msgs robot_arm_interface robot_arm_driver robot_tasks robot_bringup
source install/setup.bash
```

---

## 6. 当前常用测试方法

### 6.1 启动 `airbot_server`

```bash
sudo airbot_server -i can1 -p 50001
```

### 6.2 启动统一执行节点（直控模式）

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 launch robot_bringup arm_executor_demo.launch.py
```

### 6.3 查看机械臂状态

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 topic echo /robot_arm/state --once
```

### 6.4 发送回 home 命令

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 topic pub --once /robot_arm/cmd robot_msgs/msg/ArmCommand \
"{command_type: 'GO_HOME', source: 'manual_test'}"
```

### 6.5 启动目标点演示链路

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 launch robot_bringup camera_target_demo.launch.py
```

### 6.6 手动发送目标点

当前视觉目标消息已改为 `robot_msgs/msg/CameraTarget`，建议使用下面格式：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 topic pub --once /camera_target_base robot_msgs/msg/CameraTarget \
"{header: {frame_id: 'base_link'}, x: 0.35, y: 0.00, z: 0.20, confidence: 0.85, target_id: 'obj1', object_name: 'apple', is_stable: true, estimated_gripper_width: 0.06}"
```

---

## 7. 当前话题约定

### 7.1 机械臂状态输出

- `/robot_arm/state`
- `/robot_arm/end_pose`

### 7.2 机械臂命令输入

- `/robot_arm/cmd`
- `/robot_arm/cart_target`（本地笛卡尔小步命令，`PointStamped`）
- `/robot_arm/gripper_cmd`（夹爪开/关指令，`std_msgs/String`）

### 7.3 视觉目标输入

- `/camera_target_base`（`robot_msgs/msg/CameraTarget`）

当前 `camera_target_executor` 的目标是先完成：

- 接收视觉侧给出的 base 坐标目标，并读取置信度
- 做工作空间检查
- 规划一段候选小步路径
- 只下发第一个小步到执行层
- 等待新的视觉反馈后再决定下一步

---

## 8. 当前开发状态

当前机械臂工作区已经完成：

- 基础工作区搭建
- `AirbotWrapper` 的统一接口封装
- `/robot_arm/end_pose` 发布
- `init_arm` / `sleep_arm` 基础动作脚本
- `camera_target_executor` 已重构为纯任务节点，向执行层发布 `/robot_arm/cart_target`
- 新增 `robot_msgs/CameraTarget.msg`，统一视觉目标格式
- 新增 `cart_move_node.py`，执行层负责笛卡尔小步移动
- 统一消息接口的第一轮整理
- `arm_executor_node` 单节点执行模式的引入

---

## 9. 后续重构计划

接下来计划继续做：

1. 完善 `AirbotWrapper`
   - 到位判断
   - 超时 / 失败反馈
   - 统一动作等待逻辑

2. 稳定 `arm_executor_node`
   - 成为唯一硬件 owner
   - 统一发布状态与末端位姿

3. 继续增强任务层
   - 完善 `camera_target_executor` 的阶段机
   - 支持 `PREGRASP / ALIGN / FINAL_APPROACH / CLOSE_GRIPPER / LIFT / RETREAT`
   - 基于置信度、目标漂移、视觉稳定性做决策

4. 明确视觉侧目标接口
   - `/camera_target_base` 使用 `robot_msgs/CameraTarget`
   - 支持 `confidence`、`is_stable`、`target_id` 等字段

5. 继续保持执行层唯一性
   - 所有真实动作下发到 `robot_arm_driver`
   - 任务层只发布命令、读取状态，不直接调用 SDK

---

## 10. 注意事项

1. 不要默认把“打印 success”当成“实体一定到位”
2. 当前调试时优先使用**直控模式**
3. 官方控制栈与直控模式不要混跑
4. 如果机械臂不动，优先检查：
   - `airbot_server` 是否正常启动
   - CAN 接口是否正确
   - 当前是否有其他控制栈抢占
   - `/robot_arm/state` 是否返回真实错误信息

---

## 11. 说明

本 README 当前只针对 `robot_ws` 的机械臂部分。
相机、手眼标定、奥比中光相机工作区等内容不在本 README 范围内。
