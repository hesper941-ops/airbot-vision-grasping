# Orbbec_ws

视觉工作空间，包含 Orbbec Gemini2 相机驱动、YOLO BPU 检测、传统 detector 和情绪识别节点。

## 主要功能包

```text
Orbbec_ws/
└── src/
    ├── detect_yolo/          # YOLO BPU 通用目标检测
    ├── detector/             # 小黄鸭、绿色药盒、大樱桃等传统检测
    ├── emotion/              # 情绪识别旧版/实验节点
    ├── emotion_local/        # 当前推荐情绪识别节点
    └── emotion_landmark_cpp/ # C++ landmark + emotion 实验节点
```

## 编译

```bash
cd /home/sunrise/robot/Orbbec_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动相机

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini2.launch.py \
  enable_depth:=true \
  enable_ir:=false \
  enable_accel:=false \
  enable_gyro:=false \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  enable_d2c_viewer:=false \
  color_width:=640 \
  color_height:=480 \
  color_fps:=30
```

关键输入 topic：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
```

## YOLO 检测

YOLO 推荐常驻运行：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 run detect_yolo detect_yolo_node
```

调试显示画面：

```bash
ros2 run detect_yolo detect_yolo_node --ros-args -p show_image:=true
```

默认 `score_threshold` 为 `0.40`。节点发布：

```text
/yolo_detections
/detect_yolo/apple_position
/detect_yolo/banana_position
/detect_yolo/bottle_position
/detect_yolo/cake_position
```

## 传统 detector

传统 detector 主要用于 YOLO 之外的指定目标：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash

ros2 run detector duck_detector_node
ros2 run detector box_detector_node
ros2 run detector red_circle_detector_node
```

输出 topic：

```text
/duck_position
/box_position
/red_circle_position
```

其中 `red_circle` 在语音目标映射中对应“大樱桃”。

## 情绪识别

当前推荐节点：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
python3 /home/sunrise/robot/Orbbec_ws/src/emotion_local/emotion_local/emotion_fusion_node.py
```

输出：

```text
/emotion/result
```

`emotion_fusion_node.py` 默认 `conf_threshold` 为 `0.30`。低置信度时 `status` 为 `low_confidence`；无人脸时 `emotion` 为 `unknown`、`confidence` 为 `0.0`。

## 与机械臂侧对接

检测节点发布的是相机坐标系下的 `geometry_msgs/msg/PointStamped`。抓取前需要由：

```text
hand_to_eye/camera_to_base_transform.py
```

转换成：

```text
/visual_target_base  robot_msgs/msg/VisualTarget
```

该脚本需要同时 source `Orbbec_ws` 和 `robot_ws`。
