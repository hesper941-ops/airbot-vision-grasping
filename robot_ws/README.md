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

任务层节点，不应直接操作 SDK，只通过 `/robot_arm/cart_target`、`/robot_arm/target_joint`、`/robot_arm/gripper_cmd` 等 topic 向执行层下发指令。

目录结构：
```
robot_tasks/
├── camera_target_executor.py       # 旧版视觉目标执行器（保留）
├── grasp_task_open_loop.py          # 方案一：开环抓取
├── grasp_task_visual_servo.py       # 方案二：视觉伺服抓取
└── shared/                          # 共用工具模块
    ├── target_filter.py             # 目标过滤与验证
    ├── grasp_planner.py             # 抓取路径点规划
    └── servo_utils.py               # 视觉伺服计算
```

#### 方案一：grasp_task_open_loop（开环抓取）

一次识别 + 坐标转换 + 分阶段抓取，适合作为保底方案。

状态机：
```
IDLE → WAIT_TARGET → PLAN_OPEN_LOOP → MOVE_PRE_GRASP
  → MOVE_DESCEND → CLOSE_GRIPPER → MOVE_LIFT → MOVE_RETREAT
  → DONE → IDLE
```

特点：
- 观察位识别目标后一次性规划所有路径点
- safe → pre_grasp → grasp → lift → safe
- 抓取前 joint6 固定 90° 补偿
- pre_grasp 和 grasp 使用同一个补偿方向

#### 方案二：grasp_task_visual_servo（视觉伺服抓取）

粗定位 + 眼在手上视觉闭环微调，适合作为主力方案。

状态机：
```
IDLE → WAIT_TARGET → PLAN_COARSE_PATH → MOVE_PRE_GRASP
  → WAIT_SERVO_TARGET → SERVO_ALIGN → WAIT_AFTER_SERVO_STEP
  → FINAL_DESCEND → CLOSE_GRIPPER → MOVE_LIFT → MOVE_RETREAT
  → DONE → IDLE
```

特点：
- 先用方案一逻辑粗定位到 pre_grasp
- 到达后进入视觉闭环，根据像素误差 (u,v) 做小步修正
- 步长限幅 + 死区 + 连续稳定帧判断
- 连续 N 帧满足阈值后才允许下降抓取
- 每步修正后必须等待新的 VisualTarget，不连续多步

### 2.4 robot_msgs

机械臂相关统一消息定义。

当前消息：

- `ArmJointTarget.msg`       —— 关节目标指令（6 个 float64）
- `ArmJointState.msg`        —— 关节状态（state, joint_pos, joint_vel, end_pose）
- `GripperCommand.msg`       —— 夹爪指令（command, target_width, speed）
- `CameraTarget.msg`         —— 相机目标（旧版，camera_target_executor 使用）
- `VisualTarget.msg`         —— 统一视觉目标（新版，两个抓取任务节点共用）

`VisualTarget.msg` 字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| header | std_msgs/Header | 时间戳与 frame_id（base_link） |
| target_id | string | 目标 ID |
| object_name | string | 物体类别名 |
| x, y, z | float64 | base_link 系下 3D 坐标 |
| confidence | float64 | 识别置信度 |
| is_stable | bool | 视觉侧稳定性标志 |
| u, v | float64 | 像素坐标 |
| depth | float64 | 深度值（米） |
| image_width | int32 | 图像宽度 |
| image_height | int32 | 图像高度 |

兼容原则：
- 方案一主要使用 x, y, z, confidence
- 方案二额外使用 u, v, depth, image_width, image_height, is_stable

### 2.5 robot_bringup

负责不同模式下的启动组合，不写具体业务逻辑。

目录结构：
```
robot_bringup/
├── launch/
│   ├── arm_bringup.launch.py            # 仅启动 arm_executor_node
│   ├── camera_target_demo.launch.py     # 旧版 camera_target_executor 演示
│   ├── open_loop_grasp.launch.py        # 方案一：开环抓取
│   └── visual_servo_grasp.launch.py     # 方案二：视觉伺服抓取
└── config/
    ├── open_loop_grasp.yaml             # 方案一参数
    └── visual_servo_grasp.yaml          # 方案二参数
```

两种方案通过 launch 切换，不需要改底层代码：

```bash
# 方案一：开环抓取
ros2 launch robot_bringup open_loop_grasp.launch.py

# 方案二：视觉伺服抓取
ros2 launch robot_bringup visual_servo_grasp.launch.py
```

可通过参数覆盖默认配置：

```bash
ros2 launch robot_bringup open_loop_grasp.launch.py config_file:=/path/to/my_config.yaml
```

---

## 3. 当前推荐运行模式

当前建议将机械臂侧分成两种模式：

### 3.1 直控模式（推荐当前开发使用）

这条链路用于当前重构中的机械臂侧开发。

特点：
- 直接连接 `airbot_server`
- 用统一执行节点接收命令
- 任务节点只发命令 / 读状态
- **两种抓取方案可长期共存，只通过 launch 切换**

推荐组成：
- `airbot_server -i can1 -p 50001`
- `arm_executor_node`
- `grasp_task_open_loop`（方案一）或 `grasp_task_visual_servo`（方案二）

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

| Topic | 类型 | 说明 |
|-------|------|------|
| `/robot_arm/joint_state` | `ArmJointState` | 关节状态 + 末端位姿（10 Hz） |

### 7.2 机械臂命令输入

| Topic | 类型 | 说明 |
|-------|------|------|
| `/robot_arm/cart_target` | `PointStamped` | Cartesian 小步命令（frame: base_link） |
| `/robot_arm/target_joint` | `Float64MultiArray` | 关节空间指令（6 个值） |
| `/robot_arm/gripper_cmd` | `String` | 夹爪指令（"open" / "close"） |

### 7.3 视觉目标输入

| Topic | 类型 | 说明 |
|-------|------|------|
| `/visual_target_base` | `VisualTarget` | 新版统一视觉目标（方案一 / 方案二共用） |
| `/camera_target_base` | `CameraTarget` | 旧版视觉目标（camera_target_executor 使用） |

---

## 8. 当前开发状态

当前机械臂工作区已经完成：

- 基础工作区搭建
- `AirbotWrapper` 的统一接口封装
- `arm_executor_node` 作为唯一硬件 owner（线程安全 SDK 访问）
- `/robot_arm/joint_state` 发布（10 Hz，含关节角、关节速度、末端位姿）
- `init_arm` / `sleep_arm` 基础动作脚本
- 旧版 `camera_target_executor` 保留兼容
- 新增 `VisualTarget.msg` 统一视觉目标消息
- 新增 `robot_tasks/shared/` 共用工具模块（target_filter, grasp_planner, servo_utils）
- 新增 `grasp_task_open_loop.py`（方案一：开环抓取，完整状态机）
- 新增 `grasp_task_visual_servo.py`（方案二：视觉伺服抓取，完整状态机）
- 新增 `open_loop_grasp.launch.py` / `visual_servo_grasp.launch.py`
- 新增 `open_loop_grasp.yaml` / `visual_servo_grasp.yaml` 参数文件

两种方案可以在同一台 X5 中长期共存，只通过 launch 切换，不需要改底层代码。

---

## 9. 后续重构计划

接下来计划继续做：

1. 完善 `AirbotWrapper`
   - 到位判断
   - 超时 / 失败反馈
   - 统一动作等待逻辑

2. 稳定 `arm_executor_node`
   - 作为唯一硬件 owner，持续优化线程安全
   - 统一发布状态与末端位姿

3. 继续增强任务层
   - 完善两个任务节点的状态机细节
   - 实机调试 joint6 补偿方向和时机
   - 视觉伺服闭环参数实机标定（gain_k, dead_zone 等）

4. 明确视觉侧目标接口
   - `/visual_target_base` 使用 `robot_msgs/VisualTarget`
   - 视觉侧发布节点对接

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
