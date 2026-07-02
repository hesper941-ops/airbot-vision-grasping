# hand_to_eye

视觉机械臂侧的桥接脚本目录，负责把相机检测结果、机械臂末端位姿、语音命令和语音侧上下文连接起来。

## 文件职责

| 文件 | 作用 |
| --- | --- |
| `camera_to_base_transform.py` | 将相机坐标系目标点转换成 `/visual_target_base` |
| `arm_task_manager.py` | 订阅语音侧 `/arm/grasp_command`，自动启动抓取链路 |
| `vision_voice_bridge.py` | 将 YOLO 和情绪识别结果发布给语音侧 |
| `solve_handeye.py` | 手眼标定求解 |
| `auto_pick_from_base.py` | 旧调试抓取脚本，主链路不推荐使用 |
| `end_position_publisher.py` | 旧调试脚本，主链路不要和 `arm_executor_node` 同时运行 |

## 坐标转换

主脚本：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
python3 /home/sunrise/robot/hand_to_eye/camera_to_base_transform.py
```

订阅：

```text
/robot_arm/end_pose
/detect_yolo/apple_position
/detect_yolo/banana_position
/detect_yolo/bottle_position
/detect_yolo/cake_position
/duck_position
/box_position
/red_circle_position
```

发布：

```text
/visual_target_base  robot_msgs/msg/VisualTarget
```

当前默认手眼参数在 `camera_to_base_transform.py` 中，含义是 `camera -> gripper`，即 `^gT_c`。实机重新标定后请优先通过 ROS 参数覆盖。

## 语音命令自动抓取

启动：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
python3 /home/sunrise/robot/hand_to_eye/arm_task_manager.py
```

语音侧按接口文档向 `/arm/grasp_command` 发布中文目标名：

```text
苹果
```

支持目标：

```text
苹果 / 香蕉 / 瓶子 / 蛋糕 -> YOLO
小黄鸭 / 绿色药盒 / 大樱桃 -> detector
```

默认策略：

- YOLO 常驻运行，`arm_task_manager.py` 不重复启动 YOLO。
- detector 目标会按需启动对应节点。
- 抓取过程中发布 `/arm_task/active_object`。
- `camera_to_base_transform.py` 只转发当前语音指定目标，避免多物体坐标干扰。

如果没有常驻 YOLO，可临时启用按需启动：

```bash
python3 /home/sunrise/robot/hand_to_eye/arm_task_manager.py --ros-args -p launch_yolo_for_grasp:=true
```

状态查看：

```bash
ros2 topic echo /arm_task/status
```

## 视觉信息给语音侧

启动：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
python3 /home/sunrise/robot/hand_to_eye/vision_voice_bridge.py
```

订阅：

```text
/yolo_detections
/emotion/result
```

发布给语音侧：

```text
/vision/scene_objects
/vision/scene_text
/vision/emotion_context
/vision/dialogue_context
```

`/vision/scene_text` 示例：

```text
我看到桌面上有苹果、香蕉、瓶子。
```

`/vision/dialogue_context` 场景事件示例：

```json
{
  "event": "scene_objects",
  "source": "/yolo_detections",
  "objects": [
    {
      "class_name": "apple",
      "name_zh": "苹果",
      "confidence": 0.9231,
      "graspable": true,
      "action": "grasp_allowed"
    }
  ],
  "text_zh": "我看到桌面上有苹果。"
}
```

情绪事件示例：

```json
{
  "event": "emotion",
  "source": "/emotion/result",
  "emotion": "low_mood",
  "emotion_zh": "情绪低落",
  "confidence": 0.8732,
  "status": "ok",
  "intervention_required": true
}
```

语音侧按接口文档订阅 `/vision/scene_objects` 和 `/vision/emotion_context`；`/vision/scene_text`、`/vision/dialogue_context` 仅作为本地调试/兼容输出。

## 调试命令

```bash
ros2 topic echo /visual_target_base
ros2 topic echo /arm_task/status
ros2 topic echo /vision/scene_objects
ros2 topic echo /vision/emotion_context
```
