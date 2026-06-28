# vision_arm_x5

视觉感知、情绪识别与机械臂执行子系统，运行在视觉/机械臂侧 RDK X5 上。它是整机的“近距离感知与操作中心”，负责识别桌面物品、估计目标三维坐标、驱动 AIRBOT 六轴机械臂抓取，并把视觉和情绪上下文提供给语音/大模型侧。

## 子系统职责

- 启动 Orbbec Gemini2 RGB-D 相机并发布彩色图、深度图和相机内参；
- 使用 YOLO BPU 检测常见生活物品，输出二维框和三维坐标；
- 使用传统 detector 识别小黄鸭、绿色药盒、大樱桃等指定目标；
- 完成相机坐标到机械臂 `base_link` 的坐标转换；
- 控制 AIRBOT Play 六轴机械臂和 G2 夹爪执行抓取、抬升和复位；
- 运行五类情绪识别，输出 happy、neutral、surprise、low_mood、negative_distress；
- 将桌面物体和情绪上下文发布给语音侧，用于问答、陪伴和干预；
- 订阅语音侧 `/command`，根据任务自动启动识别和抓取链路。

## 目录结构

```text
vision_arm_x5/
├── README.md
├── Orbbec_ws/              # 相机、YOLO、detector、情绪识别工作空间
├── robot_ws/               # AIRBOT 机械臂驱动、消息、抓取任务工作空间
├── hand_to_eye/            # 手眼标定、坐标桥、语音抓取管理、视觉语音桥
├── docs/                   # 子系统调试说明
├── deploy/systemd/         # systemd 部署文件
├── start_airbot_can1.sh
└── start_auto_grasp.sh
```

## 功能链路

### 物品识别与桌面问答

YOLO 常驻运行，识别桌面上的 COCO 常见生活物品，并由 `vision_voice_bridge.py` 整理成语音侧可直接消费的中文文本和 JSON：

```text
Gemini2 RGB-D
  -> detect_yolo_node
  -> /yolo_detections
  -> vision_voice_bridge.py
  -> /vision/scene_text, /vision/dialogue_context
```

当用户问“桌子上有什么”时，语音侧可以直接读取 `/vision/scene_text` 或 `/vision/dialogue_context` 回答。

### 视觉定位与机械臂抓取

```text
目标 3D 坐标
  -> camera_to_base_transform.py
  -> /visual_target_base
  -> grasp_task_open_loop
  -> arm_executor_node
  -> AIRBOT SDK
```

支持目标：

```text
苹果、香蕉、瓶子、蛋糕、小黄鸭、绿色药盒、大樱桃
```

其中苹果、香蕉、瓶子、蛋糕来自 YOLO；小黄鸭、绿色药盒、大樱桃来自 detector。

### 情绪识别与主动陪护

```text
人脸图像
  -> emotion_fusion_node.py
  -> /emotion/result
  -> vision_voice_bridge.py
  -> /vision/emotion_context, /vision/dialogue_context
```

当情绪为 `low_mood` 或 `negative_distress` 时，视觉侧会标记：

```json
{"intervention_required": true}
```

语音侧可据此触发安抚、陪伴、询问是否需要帮助或进一步求助。

## 编译

```bash
cd /home/sunrise/robot/Orbbec_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

cd /home/sunrise/robot/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 推荐启动顺序

### 1. AIRBOT CAN server

AIRBOT 使用 `can1`：

```bash
sudo airbot_server -i can1 -p 50001
```

或：

```bash
bash /home/sunrise/robot/start_airbot_can1.sh
```

### 2. Orbbec 相机

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

### 3. YOLO 常驻

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 run detect_yolo detect_yolo_node
```

### 4. 视觉上下文桥接

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
python3 /home/sunrise/robot/hand_to_eye/vision_voice_bridge.py
```

### 5. 语音命令自动抓取管理

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
python3 /home/sunrise/robot/hand_to_eye/arm_task_manager.py
```

## 主要 Topic

| Topic | Type | 说明 |
| --- | --- | --- |
| `/command` | `std_msgs/msg/String` | 语音侧发布的任务命令 |
| `/yolo_detections` | `ai_msgs/msg/PerceptionTargets` | YOLO 原始识别结果 |
| `/vision/scene_text` | `std_msgs/msg/String` | 桌面物体中文播报文本 |
| `/vision/dialogue_context` | `std_msgs/msg/String` | 物体与情绪统一上下文 |
| `/emotion/result` | `std_msgs/msg/String` | 情绪识别原始结果 |
| `/detect_yolo/apple_position` | `geometry_msgs/msg/PointStamped` | 苹果相机坐标 |
| `/detect_yolo/banana_position` | `geometry_msgs/msg/PointStamped` | 香蕉相机坐标 |
| `/detect_yolo/bottle_position` | `geometry_msgs/msg/PointStamped` | 瓶子相机坐标 |
| `/detect_yolo/cake_position` | `geometry_msgs/msg/PointStamped` | 蛋糕相机坐标 |
| `/duck_position` | `geometry_msgs/msg/PointStamped` | 小黄鸭相机坐标 |
| `/box_position` | `geometry_msgs/msg/PointStamped` | 绿色药盒相机坐标 |
| `/red_circle_position` | `geometry_msgs/msg/PointStamped` | 大樱桃/红色圆相机坐标 |
| `/visual_target_base` | `robot_msgs/msg/VisualTarget` | 机械臂抓取目标 |
| `/robot_arm/executor_status` | `std_msgs/msg/String` | 机械臂执行状态 |

## 子目录说明

- `Orbbec_ws/README.md`：相机、YOLO、detector 与情绪识别节点。
- `robot_ws/README.md`：AIRBOT 抓取状态机和执行器接口。
- `hand_to_eye/README.md`：坐标转换、语音抓取管理、视觉语音桥接。
- `docs/grasp_startup_commands.md`：多终端调试启动顺序。
