# Orbbec_ws 视觉侧说明

Orbbec 侧负责相机启动和目标检测。当前实机主链路中，检测节点发布相机坐标系目标点，坐标转换由 `hand_to_eye/camera_to_base_transform.py` 完成。

检测节点输出：

- `/duck_position` (`geometry_msgs/msg/PointStamped`)
- `/apple_position` (`geometry_msgs/msg/PointStamped`)
- `/box_position` (`geometry_msgs/msg/PointStamped`)

这些 topic 的 `header.frame_id` 应为 `camera_color_optical_frame`。

## 启动相机

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini2.launch.py
```

## 启动检测节点

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 run detector duck_detector_node
```

## 深度和彩色图对齐检查

当前 detector 使用彩色图像像素 `u,v` 去深度图取深度，并使用 `/camera/color/camera_info` 反投影。必须确认 depth image 与 color image 已对齐，否则 `/duck_position` 会有系统偏差。

检查命令：

```bash
ros2 topic list | grep depth
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /duck_position --once
```

如果目标坐标明显漂移，请切换到 aligned depth topic，或在 Orbbec 驱动中开启 depth-to-color alignment。

## 与机械臂侧对接

检测节点不直接发布 `/visual_target_base`。请同时 source `Orbbec_ws` 和 `robot_ws` 后运行：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/airbot-vision-grasping/robot_ws/install/setup.bash
python3 /home/sunrise/robot/airbot-vision-grasping/hand_to_eye/camera_to_base_transform.py
```
