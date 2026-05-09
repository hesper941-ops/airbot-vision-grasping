# airbot-vision-grasping

本仓库用于实现 AIRBOT Play 机械臂与奥比中光摄像头配合的视觉抓取任务。

当前仓库主要包含：

- `robot_ws/`：机械臂侧 ROS 2 工作区，负责接收视觉目标点并控制机械臂执行抓取。
- `Orbbec_ws/`：奥比中光摄像头与目标检测相关工作区。
- `hand_to_eye/`：眼在手上手眼标定相关文件。

详细说明请查看：

- [robot_ws 使用说明](robot_ws/README.md)

## 快速开始：命令行实现机械臂抓取

### 环境准备

1. **安装依赖**：
   ```bash
   # 安装 ROS 2 Humble
   # 安装 AIRBOT Python SDK
   pip install airbot-py

   # 安装其他 Python 依赖
   pip install numpy scipy
   ```

2. **构建工作区**：
   ```bash
   cd robot_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

### 启动抓取流程

#### 方式1：使用 Launch 文件（推荐）

```bash
# 启动完整的开环抓取系统
ros2 launch robot_bringup open_loop_grasp.launch.py

# 或者使用配置文件
ros2 launch robot_bringup open_loop_grasp.launch.py config_file:=path/to/config.yaml
```

#### 方式2：逐个启动节点（调试用）

1. **启动机械臂执行器**（硬件所有者）：
   ```bash
   ros2 run robot_arm_driver arm_executor_node
   ```

2. **启动视觉目标桥接**（坐标变换）：
   ```bash
   ros2 run robot_tasks visual_target_bridge
   ```

3. **启动抓取任务**（状态机控制）：
   ```bash
   ros2 run robot_tasks grasp_task_open_loop
   ```

4. **启动检测器**（在另一个终端）：
   ```bash
   # 假设检测器在 Orbbec_ws 中
   cd Orbbec_ws
   # 启动检测器节点（具体命令根据检测器实现）
   ```

### 监控和验证

1. **检查话题**：
   ```bash
   # 查看末端位姿
   ros2 topic echo /robot_arm/end_pose --once

   # 监听视觉目标
   ros2 topic echo /visual_target_base

   # 监听 Cartesian 指令
   ros2 topic echo /robot_arm/cart_target
   ```

2. **检查节点状态**：
   ```bash
   ros2 node list
   ros2 topic list
   ```

3. **查看日志**：
   ```bash
   # 在启动节点的终端查看实时日志
   # 或者使用 rqt_console 查看结构化日志
   ```

### 抓取流程说明

1. **初始化**：机械臂移动到安全位姿
2. **等待目标**：接收检测器的 `/duck_position` 消息
3. **坐标变换**：桥接节点转换为基座坐标系的 `/visual_target_base`
4. **规划路径**：任务节点生成 pre_grasp → grasp → lift → retreat 路径
5. **执行抓取**：分阶段下发 Cartesian 指令，夹爪闭合
6. **完成**：返回安全位姿，等待下一次抓取

### 故障排除

- **机械臂无响应**：检查 SDK 连接和初始化
- **无视觉目标**：检查检测器输出和桥接节点日志
- **抓取失败**：查看任务节点状态机日志，检查参数配置
- **抖动严重**：调整速度档位或到位判定参数

### 配置文件

默认配置文件位于 `robot_ws/src/robot_bringup/config/open_loop_grasp.yaml`，包含所有可调参数。
