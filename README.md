# airbot-vision-grasping

这是 AIRBOT Play 机械臂视觉抓取项目，包含两个主要工作区：

- `robot_ws/`：机械臂侧 ROS 2 包，包括执行器、任务状态机、消息和启动文件。
- `Orbbec_ws/`：Orbbec 相机和检测侧工作区。
- `hand_to_eye/`：手眼标定脚本和数据。

当前默认开环抓取流程要求视觉侧直接发布机器人 `base_link` 坐标系下的 `/visual_target_base`。旧的机器人侧 camera-to-base 桥接节点已经删除。

## 快速开始

编译机械臂工作区：

```bash
cd ~/robot/airbot-vision-grasping/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

启动 AIRBOT 服务：

```bash
sudo airbot_server -i can1 -p 50001
```

启动机械臂侧开环抓取流程：

```bash
cd ~/robot/airbot-vision-grasping/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

单独启动 Orbbec 视觉流程，并确保它发布：

```text
/visual_target_base  robot_msgs/msg/VisualTarget  frame_id=base_link
```

更多 topic 契约和测试命令见 [robot_ws/README.md](robot_ws/README.md) 与 [Orbbec_ws/README.md](Orbbec_ws/README.md)。
