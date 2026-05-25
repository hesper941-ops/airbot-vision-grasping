# airbot-vision-grasping

AIRBOT Play + Orbbec 视觉抓取项目。

当前唯一推荐的实机主链路：

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

不要把 `hand_to_eye/auto_pick_from_base.py` 当作主抓取入口；它只保留为 legacy/debug 调试脚本。当前主抓取节点是 `grasp_task_open_loop.py`，执行器节点是 `arm_executor_node.py`。

当前唯一推荐启动入口：

```bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

该 launch 只应启动：

```text
arm_executor_node.py
grasp_task_open_loop.py
```

## X5 实机路径

```text
仓库根目录：/home/sunrise/robot
ROS2 工作区：/home/sunrise/robot/robot_ws
Orbbec 工作区：/home/sunrise/robot/Orbbec_ws
转换脚本：/home/sunrise/robot/hand_to_eye/camera_to_base_transform.py
```

## 推荐启动顺序

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

终端 4：启动目标转换桥

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
python3 /home/sunrise/robot/hand_to_eye/camera_to_base_transform.py
```

## 最小验证

```bash
ros2 topic echo /duck_position --once
ros2 topic echo /visual_target_base --once
ros2 topic echo /robot_arm/end_pose --once
ros2 topic echo /robot_arm/joint_state --once
ros2 topic echo /robot_arm/executor_status
```
