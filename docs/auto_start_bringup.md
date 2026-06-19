# Service Robot Grasp Bringup and Autostart

This project keeps AIRBOT server startup outside ROS2 launch. The AIRBOT CAN
server is managed manually or by systemd, while ROS2 nodes are managed by the
unified bringup launch file.

## Manual Startup

Terminal 0:

```bash
sudo airbot_server -i can1 -p 50001
```

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 launch robot_bringup service_robot_grasp_bringup.launch.py
```

By default, `service_robot_grasp_bringup.launch.py` starts only the arm main
chain:

```text
arm_executor_node
grasp_task_open_loop
```

Run the manual `ros2 launch` command successfully before enabling systemd
autostart for the first time.

## Camera, Detector, and Transform Options

The default `service-robot-grasp.service` sources only:

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
```

That is enough for the default arm main chain. If the site needs
`enable_camera:=true` or `enable_detector:=true`, also source the Orbbec
workspace before `robot_ws`:

```bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
```

If this line is added to `deploy/systemd/service-robot-grasp.service`, confirm
`/home/sunrise/robot/Orbbec_ws/install/setup.bash` exists on the robot. If the
site has no `Orbbec_ws`, delete that line or keep the default service template.

Before enabling camera-related launch options, confirm the actual site setup:

- `Orbbec_ws` is built and sourceable.
- The `detector` package provides `duck_detector_node`.
- The transform script path matches `hand_to_eye/camera_to_base_transform.py`
  on the robot.

## LLM or Mock Target Test

The LLM or upper task module only needs to publish:

```text
/visual_target_base
```

The message type is `robot_msgs/msg/VisualTarget`, and `header.frame_id` must
be `base_link`.

Do not publish directly to:

```text
/robot_arm/cart_target
/robot_arm/cart_waypoints
/robot_arm/target_joint
/robot_arm/gripper_cmd
```

`/robot_arm/cart_waypoints` is an internal topic from `grasp_task_open_loop` to
`arm_executor_node`.

## Inspect Parameters

```bash
ros2 param get /grasp_task_open_loop blend_approach_enabled
ros2 param get /grasp_task_open_loop front_grasp_x_offset
ros2 param get /grasp_task_open_loop official_reach_radius_m
```

## Inspect Internal Waypoints

```bash
ros2 topic echo /robot_arm/cart_waypoints
```

## Install systemd Templates

Review both files before installing. If `airbot_server` is not located at
`/usr/local/bin/airbot_server`, confirm the path with `which airbot_server` and
edit `deploy/systemd/airbot-server.service`.

If the robot user is not `sunrise`, edit the `User=` field in
`deploy/systemd/service-robot-grasp.service`.

```bash
sudo cp deploy/systemd/airbot-server.service /etc/systemd/system/
sudo cp deploy/systemd/service-robot-grasp.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable airbot-server.service
sudo systemctl enable service-robot-grasp.service
```

## Start, Stop, and Logs

```bash
sudo systemctl start airbot-server.service
sudo systemctl start service-robot-grasp.service

sudo systemctl status airbot-server.service
sudo systemctl status service-robot-grasp.service

journalctl -u airbot-server.service -f
journalctl -u service-robot-grasp.service -f
```

## Disable Autostart

```bash
sudo systemctl disable airbot-server.service
sudo systemctl disable service-robot-grasp.service
```

## Notes

- `airbot_server` is not launched from ROS2 launch.
- Confirm `can1` is available before enabling boot autostart.
- If the CAN interface name changes, edit `airbot-server.service`.
- If the Linux user is not `sunrise`, edit `service-robot-grasp.service`.
- The LLM must not publish `/robot_arm/cart_waypoints` directly.
- `/robot_arm/cart_waypoints` is an internal topic.
