# airbot-vision-grasping

AIRBOT Play + Orbbec 相机视觉抓取项目。

当前推荐且唯一的实机主链路是：

```text
Orbbec camera
-> detector/duck_detector_node 发布 /duck_position (camera_color_optical_frame)
-> hand_to_eye/camera_to_base_transform.py 发布 /visual_target_base (base_link)
-> robot_tasks/grasp_task_open_loop 消费 /visual_target_base
-> robot_arm_driver/arm_executor_node 作为唯一 AIRBOT SDK owner
-> AIRBOT Play 分阶段执行抓取
```

不要把 `hand_to_eye/auto_pick_from_base.py` 当作主抓取节点；它只保留为 legacy 调试脚本。主抓取节点是 `robot_tasks/grasp_task_open_loop`。

不要在主链路中启动 `hand_to_eye/end_position_publisher.py`；`arm_executor_node` 已经发布 `/robot_arm/end_pose`，并且它必须是唯一调用 AIRBOT SDK 的节点。

## 推荐实机启动顺序

终端 0：启动 AIRBOT 服务。

```bash
sudo airbot_server -i can1 -p 50001
```

终端 1：启动 Orbbec 相机。

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini2.launch.py
```

终端 2：启动检测节点。

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 run detector duck_detector_node
```

终端 3：启动机械臂执行器和开环抓取任务。

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/airbot-vision-grasping/robot_ws/install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

终端 4：启动相机坐标到 base_link 的转换桥。

注意：这里必须 source `robot_ws/install/setup.bash`，否则 `robot_msgs/msg/VisualTarget` 不可见。

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/airbot-vision-grasping/robot_ws/install/setup.bash
python3 /home/sunrise/robot/airbot-vision-grasping/hand_to_eye/camera_to_base_transform.py
```

## 最小验证清单

检测器输出，相机坐标系：

```bash
ros2 topic echo /duck_position --once
```

发布者：`duck_detector_node`

机械臂末端位姿：

```bash
ros2 topic echo /robot_arm/end_pose --once
```

发布者：`arm_executor_node`

转换后的视觉目标：

```bash
ros2 topic echo /visual_target_base --once
```

发布者：`hand_to_eye/camera_to_base_transform.py`

机械臂关节状态：

```bash
ros2 topic echo /robot_arm/joint_state --once
```

发布者：`arm_executor_node`

执行器状态：

```bash
ros2 topic echo /robot_arm/executor_status
```

发布者：`arm_executor_node`

## 故障定位

如果 `/duck_position` 有数据但 `/visual_target_base` 没数据，优先检查：

- `camera_to_base_transform.py` 是否已启动。
- 终端是否 source 了 `robot_ws/install/setup.bash`。
- `/robot_arm/end_pose` 是否存在且持续更新。
- `/duck_position.header.frame_id` 是否为 `camera_color_optical_frame` 或允许的相机 frame。

如果 `/visual_target_base` 有数据但机械臂不动，优先检查：

- `airbot_server` 是否正常运行。
- CAN 口是否正确，例如 `can1`。
- `arm_executor_node` 是否启动。
- `/robot_arm/executor_status` 是否为 `IDLE`、`BUSY`、`ERROR` 或 `REJECTED_*`。

## 深度和彩色图对齐检查

当前 detector 使用彩色图像像素 `u,v` 去深度图取深度，并使用 `/camera/color/camera_info` 反投影。必须确认 depth image 与 color image 已对齐，否则 `/duck_position` 会有系统偏差。

检查命令：

```bash
ros2 topic list | grep depth
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /duck_position --once
```

如果目标坐标明显漂移，请切换到 aligned depth topic，或在 Orbbec 驱动中开启 depth-to-color alignment。
