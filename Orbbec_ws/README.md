# Orbbec_ws 视觉侧说明

`Orbbec_ws` 负责相机启动、目标检测和坐标转换。

默认机械臂抓取流程要求视觉侧直接发布机器人基坐标系下的目标点。机械臂任务节点在默认启动链路中不再做 camera-to-base 坐标转换。

## 必须发布的结果

发布 topic：

```text
/visual_target_base
```

消息类型：

```text
robot_msgs/msg/VisualTarget
```

必要字段：

- `header.frame_id`：`base_link`
- `header.stamp`：当前视觉结果的时间戳
- `x`, `y`, `z`：`base_link` 坐标系下的目标中心点或抓取点，单位米
- `confidence`：检测置信度，通常范围为 `0.0` 到 `1.0`
- `depth`：如可用，填写目标深度，单位米
- `object_name` 和 `target_id`：可选，但建议填写，方便日志排查

机械臂侧会拒绝过期目标、错误坐标系目标、低置信度目标，以及超出工作空间的目标。

## 职责划分

视觉侧负责：

- 启动和配置 Orbbec 相机。
- 检测物体或抓取点。
- 将结果转换到机器人 `base_link` 坐标系。
- 目标可见时持续发布 `/visual_target_base`。

机械臂侧负责：

- 等待连续多帧稳定目标。
- 移动到预抓取位置。
- 再次等待新的稳定目标。
- 执行抓取、抬升和撤退。

旧的机器人侧 camera-to-base 桥接节点已经删除。请从视觉工作区直接发布 `/visual_target_base`。
