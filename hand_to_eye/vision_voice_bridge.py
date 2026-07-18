#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish visual context for the voice/LLM board without touching voice code.

Interface basis: ros_voice integration document v1.1.

Inputs:
  /yolo_detections      ai_msgs/msg/PerceptionTargets
  /emotion/result       std_msgs/msg/String, JSON from emotion_fusion_node.py

Required outputs for voice:
  /vision/scene_objects     std_msgs/msg/String JSON, published every 0.5 s when objects exist
  /vision/emotion_context   std_msgs/msg/String JSON, only after emotion_fusion_node publishes

Debug/compatibility outputs:
  /vision/scene_text        Short Chinese sentence for local debugging
  /vision/dialogue_context  Unified JSON events for older integration tests
"""

from collections import Counter, defaultdict
import json
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ai_msgs.msg import PerceptionTargets


GRASPABLE_CLASSES: Dict[str, str] = {
    "apple": "苹果",
    "banana": "香蕉",
    "bottle": "瓶子",
    "cake": "蛋糕",
}

DETECTOR_GRASPABLE_ZH = {
    "小黄鸭",
    "绿色药盒",
    "大樱桃",
}

CLASS_ZH: Dict[str, str] = {
    "person": "人",
    "bicycle": "自行车",
    "car": "汽车",
    "motorcycle": "摩托车",
    "airplane": "飞机",
    "bus": "公交车",
    "train": "火车",
    "truck": "卡车",
    "boat": "船",
    "traffic light": "红绿灯",
    "fire hydrant": "消防栓",
    "stop sign": "停止标志",
    "parking meter": "停车计时器",
    "bench": "长椅",
    "bird": "鸟",
    "cat": "猫",
    "dog": "狗",
    "horse": "马",
    "sheep": "羊",
    "cow": "牛",
    "elephant": "大象",
    "bear": "熊",
    "zebra": "斑马",
    "giraffe": "长颈鹿",
    "backpack": "背包",
    "umbrella": "雨伞",
    "handbag": "手提包",
    "tie": "领带",
    "suitcase": "行李箱",
    "frisbee": "飞盘",
    "skis": "滑雪板",
    "snowboard": "单板滑雪板",
    "sports ball": "球",
    "kite": "风筝",
    "baseball bat": "棒球棒",
    "baseball glove": "棒球手套",
    "skateboard": "滑板",
    "surfboard": "冲浪板",
    "tennis racket": "网球拍",
    "bottle": "瓶子",
    "wine glass": "酒杯",
    "cup": "杯子",
    "fork": "叉子",
    "knife": "刀",
    "spoon": "勺子",
    "bowl": "碗",
    "banana": "香蕉",
    "apple": "苹果",
    "sandwich": "三明治",
    "orange": "橙子",
    "broccoli": "西兰花",
    "carrot": "胡萝卜",
    "hot dog": "热狗",
    "pizza": "披萨",
    "donut": "甜甜圈",
    "cake": "蛋糕",
    "chair": "椅子",
    "couch": "沙发",
    "potted plant": "盆栽",
    "bed": "床",
    "dining table": "餐桌",
    "toilet": "马桶",
    "tv": "电视",
    "laptop": "笔记本电脑",
    "mouse": "鼠标",
    "remote": "遥控器",
    "keyboard": "键盘",
    "cell phone": "手机",
    "microwave": "微波炉",
    "oven": "烤箱",
    "toaster": "烤面包机",
    "sink": "水槽",
    "refrigerator": "冰箱",
    "book": "书",
    "clock": "时钟",
    "vase": "花瓶",
    "scissors": "剪刀",
    "teddy bear": "玩具熊",
    "hair drier": "吹风机",
    "toothbrush": "牙刷",
}

EMOTION_ZH = {
    "happy": "开心",
    "neutral": "平静",
    "surprise": "惊讶",
    "low_mood": "情绪低落",
    "negative_distress": "负面痛苦",
    "unknown": "未知",
}


class VisionVoiceBridge(Node):
    def __init__(self):
        super().__init__("vision_voice_bridge")

        self.declare_parameter("scene_publish_period_sec", 0.5)
        self.declare_parameter("scene_vote_window_sec", 0.5)
        self.declare_parameter("min_detection_confidence", 0.35)
        self.declare_parameter("emotion_intervention_classes", ["low_mood", "negative_distress"])

        self.scene_publish_period_sec = float(
            self.get_parameter("scene_publish_period_sec").value)
        self.scene_vote_window_sec = float(
            self.get_parameter("scene_vote_window_sec").value)
        self.min_detection_confidence = float(
            self.get_parameter("min_detection_confidence").value)
        self.emotion_intervention_classes = set(
            str(v) for v in self.get_parameter("emotion_intervention_classes").value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.scene_objects_pub = self.create_publisher(String, "/vision/scene_objects", qos)
        self.scene_text_pub = self.create_publisher(String, "/vision/scene_text", qos)
        self.emotion_context_pub = self.create_publisher(String, "/vision/emotion_context", qos)
        self.dialogue_context_pub = self.create_publisher(String, "/vision/dialogue_context", qos)

        self.create_subscription(PerceptionTargets, "/yolo_detections", self.yolo_callback, 10)
        self.create_subscription(String, "/emotion/result", self.emotion_callback, 10)
        self.timer = self.create_timer(self.scene_publish_period_sec, self.publish_scene_context)

        self.scene_frames: List[Tuple[float, List[dict]]] = []

        self.get_logger().info(
            "vision_voice_bridge ready. /vision/scene_objects uses a "
            f"{self.scene_vote_window_sec:.2f}s majority-vote window; "
            "emotion_context is published only when /emotion/result is received.")

    def yolo_callback(self, msg: PerceptionTargets):
        objects = []
        now = self.now_sec()
        for target in msg.targets:
            item = self.make_scene_object(target)
            if item is not None:
                objects.append(item)

        objects.sort(key=lambda item: item.get("confidence", 0.0), reverse=True)
        self.scene_frames.append((now, objects))
        self.prune_scene_frames(now)

    def make_scene_object(self, target) -> Optional[dict]:
        class_name = str(getattr(target, "type", "unknown") or "unknown")
        confidence = self.target_confidence(target)
        if confidence < self.min_detection_confidence:
            return None

        item = {
            "class_name": class_name,
            "name_zh": CLASS_ZH.get(class_name, class_name),
            "confidence": round(confidence, 4),
            "graspable": class_name in GRASPABLE_CLASSES,
            "action": "grasp_allowed" if class_name in GRASPABLE_CLASSES else "dialogue_only",
        }
        point = self.target_point(target)
        if point is not None:
            item["camera_xyz"] = {
                "x": round(point[0], 4),
                "y": round(point[1], 4),
                "z": round(point[2], 4),
            }
        return item

    def emotion_callback(self, msg: String):
        try:
            raw = json.loads(msg.data)
        except json.JSONDecodeError:
            raw = {"status": "parse_error", "raw": msg.data}

        if raw.get("status") == "no_face" or raw.get("has_face") is False:
            return

        emotion = str(raw.get("emotion") or raw.get("stable_emotion") or "unknown")
        context = {
            "event": "emotion",
            "source": "/emotion/result",
            "stamp_sec": round(self.now_sec(), 3),
            "emotion": emotion,
            "emotion_zh": EMOTION_ZH.get(emotion, emotion),
            "confidence": raw.get("confidence", 0.0),
            "status": raw.get("status", "unknown"),
            "intervention_required": emotion in self.emotion_intervention_classes,
            "raw": raw,
        }
        self.publish_json(self.emotion_context_pub, context)
        self.publish_json(self.dialogue_context_pub, context)

    def publish_scene_context(self):
        now = self.now_sec()
        frames = self.recent_scene_frames(now)
        if not frames:
            return

        objects = self.voted_scene_objects(frames)
        if not objects:
            return

        first_stamp = min(stamp for stamp, _ in frames)
        last_stamp = max(stamp for stamp, _ in frames)
        context = {
            "event": "scene_objects",
            "source": "/yolo_detections",
            "stamp_sec": round(last_stamp, 3),
            "age_sec": round(now - last_stamp, 3),
            "window_sec": round(max(last_stamp - first_stamp, self.scene_vote_window_sec), 3),
            "window_frames": len(frames),
            "count": len(objects),
            "objects": objects,
            "graspable_targets": [
                item for item in objects if item.get("graspable")
            ],
            "dialogue_only_objects": [
                item for item in objects if not item.get("graspable")
            ],
        }
        text = self.scene_text(objects)
        self.publish_json(self.scene_objects_pub, context)
        self.scene_text_pub.publish(String(data=text))
        dialogue_context = dict(context)
        dialogue_context["text_zh"] = text
        self.publish_json(self.dialogue_context_pub, dialogue_context)

    def recent_scene_frames(self, now: float) -> List[Tuple[float, List[dict]]]:
        self.prune_scene_frames(now)
        return [
            (stamp, objects)
            for stamp, objects in self.scene_frames
            if now - stamp <= self.scene_vote_window_sec
        ]

    def prune_scene_frames(self, now: float):
        keep_sec = max(self.scene_vote_window_sec * 2.0, self.scene_publish_period_sec * 2.0, 1.0)
        self.scene_frames = [
            (stamp, objects)
            for stamp, objects in self.scene_frames
            if now - stamp <= keep_sec
        ]

    def voted_scene_objects(self, frames: List[Tuple[float, List[dict]]]) -> List[dict]:
        class_frame_hits: Counter = Counter()
        class_count_votes = defaultdict(Counter)
        candidates_by_class = defaultdict(list)

        for _, objects in frames:
            per_frame_counts = Counter(item.get("class_name", "unknown") for item in objects)
            for class_name, count in per_frame_counts.items():
                class_frame_hits[class_name] += 1
                class_count_votes[class_name][count] += 1
            for item in objects:
                candidates_by_class[item.get("class_name", "unknown")].append(item)

        if not class_frame_hits:
            return []

        max_hits = max(class_frame_hits.values())
        selected_classes = [
            class_name for class_name, hits in class_frame_hits.items()
            if hits == max_hits
        ]

        voted_objects = []
        for class_name in sorted(selected_classes, key=lambda name: (-class_frame_hits[name], name)):
            count_vote = class_count_votes[class_name]
            modal_count = count_vote.most_common(1)[0][0] if count_vote else 1
            candidates = sorted(
                candidates_by_class[class_name],
                key=lambda item: item.get("confidence", 0.0),
                reverse=True,
            )
            if not candidates:
                continue
            for item in candidates[:max(1, modal_count)]:
                stable_item = dict(item)
                stable_item["window_hits"] = class_frame_hits[class_name]
                stable_item["window_frames"] = len(frames)
                voted_objects.append(stable_item)

        voted_objects.sort(
            key=lambda item: (item.get("window_hits", 0), item.get("confidence", 0.0)),
            reverse=True,
        )
        return voted_objects

    def scene_text(self, objects: List[dict]) -> str:
        counts: Dict[str, int] = {}
        for item in objects:
            name = item.get("name_zh") or item.get("class_name", "物品")
            counts[name] = counts.get(name, 0) + 1

        phrases = []
        for name, count in counts.items():
            if count <= 1:
                phrases.append(name)
            else:
                phrases.append(f"{count}个{name}")
        return "我看到桌面上有" + "、".join(phrases) + "。"

    def target_confidence(self, target) -> float:
        confidences = []
        for roi in getattr(target, "rois", []):
            confidences.append(float(getattr(roi, "confidence", 0.0)))
        for point_group in getattr(target, "points", []):
            for value in getattr(point_group, "confidence", []):
                confidences.append(float(value))
        return max(confidences) if confidences else 0.0

    def target_point(self, target):
        for point_group in getattr(target, "points", []):
            points = getattr(point_group, "point", [])
            if points:
                point = points[0]
                return (
                    float(getattr(point, "x", 0.0)),
                    float(getattr(point, "y", 0.0)),
                    float(getattr(point, "z", 0.0)),
                )
        return None

    def publish_json(self, publisher, payload: dict):
        publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = VisionVoiceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
