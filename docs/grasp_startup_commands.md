# 机械臂视觉抓取话题汇总

## 主要话题汇总

| 话题 | 消息类型 | 方向 | 用途 |
| --- | --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 发布 | Orbbec 相机发布彩色图像，检测节点订阅后进行苹果、小黄鸭、绿色药盒识别 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 发布 | Orbbec 相机发布深度图像，检测节点用于计算目标深度 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 发布 | Orbbec 相机发布相机内参，检测节点用于像素坐标到相机三维坐标转换 |
| `/apple_position` | `geometry_msgs/msg/PointStamped` | 发布 | 苹果检测节点发布苹果在相机坐标系下的三维位置 |
| `/duck_position` | `geometry_msgs/msg/PointStamped` | 发布 | 小黄鸭检测节点发布小黄鸭在相机坐标系下的三维位置 |
| `/box_position` | `geometry_msgs/msg/PointStamped` | 发布 | 绿色药盒检测节点发布药盒在相机坐标系下的三维位置 |
| `/robot_arm/end_pose` | `geometry_msgs/msg/PoseStamped` | 发布 | 发布机械臂末端在 `base_link` 坐标系下的位姿，供相机到基座坐标变换使用 |
| `/visual_target_base` | `robot_msgs/msg/VisualTarget` | 发布 | 发布目标物体在机械臂基座坐标系 `base_link` 下的位置，供自动抓取节点使用 |
| `/robot_arm/joint_state` | `robot_msgs/msg/ArmJointState` | 发布 | 发布机械臂关节状态和末端位姿反馈，自动抓取节点用于判断运动状态 |
| `/robot_arm/cart_target` | `geometry_msgs/msg/PointStamped` | 发布 | 自动抓取节点发布末端笛卡尔目标点，控制机械臂移动到预抓取点、抓取点和抬升点 |
| `/robot_arm/target_joint` | `std_msgs/msg/Float64MultiArray` | 发布 | 自动抓取节点发布目标关节角，用于 joint6 旋转或回到 home 位姿 |
| `/robot_arm/gripper_cmd` | `std_msgs/msg/String` | 发布 | 自动抓取节点发布夹爪命令，常用值为 `open`、`close` |

## 各节点订阅与发布关系

| 节点 / 脚本 | 订阅话题 | 发布话题 | 作用 |
| --- | --- | --- | --- |
| Orbbec 相机节点 | 无 | `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info` | 提供 RGB-D 图像和相机内参 |
| `apple_detector_node` | `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info` | `/apple_position` | 检测苹果并输出相机坐标系三维点 |
| `duck_detector_node` | `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info` | `/duck_position` | 检测小黄鸭并输出相机坐标系三维点 |
| `box_detector_node` | `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info` | `/box_position` | 检测绿色药盒并输出相机坐标系三维点 |
| `end_position_publisher.py` | 无 | `/robot_arm/end_pose` | 从机械臂读取末端位姿并发布 |
| `camera_to_base_transform.py` | `/robot_arm/end_pose`、`/apple_position`、`/duck_position`、`/box_position` | `/visual_target_base` | 将相机坐标系目标点转换到 `base_link` 坐标系 |
| `auto_pick_from_base.py` | `/visual_target_base`、`/robot_arm/joint_state` | `/robot_arm/cart_target`、`/robot_arm/target_joint`、`/robot_arm/gripper_cmd` | 根据目标位置执行自动抓取流程 |
| `arm_executor_node.py` | `/robot_arm/cart_target`、`/robot_arm/target_joint`、`/robot_arm/gripper_cmd`、`/robot_arm/speed_profile`、`/robot_arm/reset_executor` | `/robot_arm/joint_state`、`/robot_arm/end_pose`、`/robot_arm/executor_status` | 机械臂统一执行层，接收目标并发布状态反馈 |

## 视觉抓取数据流

| 步骤 | 输入话题 | 处理节点 | 输出话题 | 数据含义 |
| --- | --- | --- | --- | --- |
| 1 | 相机硬件数据 | Orbbec 相机节点 | `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info` | 彩色图像、深度图像、相机内参 |
| 2 | `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info` | 检测节点 | `/apple_position` 或 `/duck_position` 或 `/box_position` | 目标在相机坐标系下的三维坐标 |
| 3 | `/robot_arm/end_pose`、目标位置话题 | `camera_to_base_transform.py` | `/visual_target_base` | 目标在机械臂基座坐标系下的位置 |
| 4 | `/visual_target_base`、`/robot_arm/joint_state` | `auto_pick_from_base.py` | `/robot_arm/cart_target`、`/robot_arm/target_joint`、`/robot_arm/gripper_cmd` | 机械臂运动目标和夹爪动作命令 |
| 5 | 抓取命令话题 | `arm_executor_node.py` 或机械臂执行节点 | `/robot_arm/joint_state`、`/robot_arm/end_pose`、`/robot_arm/executor_status` | 执行抓取并反馈机械臂状态 |

## 备注

| 项目 | 说明 |
| --- | --- |
| 坐标系 | `/apple_position`、`/duck_position`、`/box_position` 是相机坐标系；`/visual_target_base` 是 `base_link` 坐标系 |
| 检测节点选择 | 苹果、小黄鸭、绿色药盒三个检测节点通常只运行其中一个 |
| 自动抓取入口 | `auto_pick_from_base.py` 主要依赖 `/visual_target_base` 和 `/robot_arm/joint_state` |
| 夹爪命令 | `/robot_arm/gripper_cmd` 当前使用字符串命令，代码中发送 `open` 和 `close` |
