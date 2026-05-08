# airbot-vision-grasping

本仓库用于实现 AIRBOT Play 机械臂与奥比中光摄像头配合的视觉抓取任务。

当前仓库主要包含：

- `robot_ws/`：机械臂侧 ROS 2 工作区，负责接收视觉目标点并控制机械臂执行抓取。
- `Orbbec_ws/`：奥比中光摄像头与目标检测相关工作区。
- `hand_to_eye/`：眼在手上手眼标定相关文件。

详细说明请查看：

- [robot_ws 使用说明](robot_ws/README.md)
