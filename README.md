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

Default approach behavior:
- `blend_approach_enabled: true`
- The recommended open_loop main path is the AIRBOT official multi-waypoint
  blend through the internal `/robot_arm/cart_waypoints` path.
- Current main flow:
  `WAIT_PRE_TARGET -> PRE_OPEN_GRIPPER -> MOVE_APPROACH_BLEND -> CLOSE_GRIPPER -> MOVE_LIFT -> RETURN_INIT_POSE`.
- The sequential `MOVE_PRE_GRASP -> MOVE_GRASP` approach remains available
  only as fallback, and is also used when `blend_approach_enabled: false`.
- `CLOSE_GRIPPER` is not blended into the trajectory: the arm must stop at the
  grasp point, close the gripper, and only then lift.
- This does not use official `ArmControlOptions` or `blend_radius` parameters.
  It uses the existing SDK `PLANNING_WAYPOINTS / move_with_cart_waypoints`
  path through `AirbotWrapper.move_cart_waypoints()`.
- External callers and LLM modules should still publish only
  `/visual_target_base` in `base_link`. Do not publish `/robot_arm/cart_waypoints`
  directly; it is an internal execution topic between `grasp_task_open_loop.py`
  and `arm_executor_node.py`.

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

## Service robot one-command bringup / boot autostart

Recommended ROS2 entrypoint:

```bash
ros2 launch robot_bringup service_robot_grasp_bringup.launch.py
```

`airbot_server` is still started separately, either manually or with systemd.
It is not embedded in ROS2 launch.

The LLM interface remains `/visual_target_base`
(`robot_msgs/msg/VisualTarget`, `base_link`). LLM modules must not publish
directly to `/robot_arm/cart_waypoints`; that topic is still an internal
waypoint blend execution topic from `grasp_task_open_loop.py` to
`arm_executor_node.py`.

The waypoint blend approach is the recommended open_loop main path. The
existing `open_loop_grasp.launch.py` remains available as the lower-level
debug entrypoint.

Autostart templates and installation notes are in
`docs/auto_start_bringup.md`.

## TODO After Real-Machine Validation

After the `move_cart_waypoints` main path is verified on hardware, planned
cleanup:

1. Delete the `visual_servo` plan.
2. Delete the legacy J6 compensation path.
3. Delete unused `active_search`, `search_pose`, and old sequence modules.
4. Delete the sequential fallback or keep it as debug mode only.
5. Trim README and launch files.

These files stay in the repository until the real-machine path is stable.
