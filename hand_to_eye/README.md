# hand_to_eye 实机链路说明

主链路只使用：

```text
camera_to_base_transform.py
```

它订阅：

- `/robot_arm/end_pose`
- `/duck_position`
- `/apple_position`
- `/box_position`

它发布：

- `/visual_target_base`

`/visual_target_base` 类型是 `robot_msgs/msg/VisualTarget`，所以启动前必须 source 机械臂工作区：

```bash
source /home/sunrise/robot/airbot-vision-grasping/robot_ws/install/setup.bash
```

## 启动

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/airbot-vision-grasping/robot_ws/install/setup.bash
python3 /home/sunrise/robot/airbot-vision-grasping/hand_to_eye/camera_to_base_transform.py
```

## 可调参数

```bash
python3 /home/sunrise/robot/airbot-vision-grasping/hand_to_eye/camera_to_base_transform.py --ros-args \
  -p target_frame:=base_link \
  -p max_end_pose_age_sec:=0.5 \
  -p default_confidence:=0.85 \
  -p assume_target_stable:=true \
  -p republish_rate_hz:=10.0 \
  -p target_hold_sec:=0.8
```

手眼参数默认使用 PARK 结果：

```text
t_cam2gripper = [-0.0830395307186257, 0.008112286716840913, 0.08580828291231507]
q_cam2gripper_xyzw = [-0.49270434706957716, 0.5001884081237661, -0.49995706645552335, 0.5070472507158893]
```

该含义是 `camera -> gripper`，即 `^gT_c`，不要取反。

## legacy 脚本

`auto_pick_from_base.py` 只保留为旧调试脚本，不推荐作为主抓取节点。主抓取请使用：

```bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

`end_position_publisher.py` 会直接连接 AIRBOT SDK，只能用于旧调试链路。主链路中不要和 `arm_executor_node` 同时运行。
