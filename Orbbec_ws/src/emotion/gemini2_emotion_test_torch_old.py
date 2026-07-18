#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import math
import numpy as np
from PIL import Image

import cv2
import torch
import torch.nn as nn
from torchvision import transforms, models

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String, Float32, Int32, Bool
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


# =========================
# 1. 路径 / 话题 / 参数
# =========================
PROJECT_DIR = os.path.expanduser("~/ros2_ws/src/emotion_project/face_emotion_demo")

# 固定使用 mid 平衡版模型
CKPT_PATH = os.path.join(
    PROJECT_DIR,
    "checkpoints/mobilenetv3_fer2013_mid/best_model.pth"
)

YUNET_PATH = os.path.join(
    PROJECT_DIR,
    "models/face_detection_yunet_2023mar.onnx"
)

COLOR_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/depth/image_raw"

# 调试图与详细结果
DEBUG_IMAGE_TOPIC = "/emotion/debug_image"
RESULT_TOPIC = "/emotion/result"
DEPTH_RESULT_TOPIC = "/emotion/face_depth_m"

# 给机械臂订阅的简单话题
RAW_LABEL_TOPIC = "/emotion/raw_label"
STABLE_LABEL_TOPIC = "/emotion/label"      # 建议机械臂订阅这个
STABLE_ID_TOPIC = "/emotion/id"
STABLE_SCORE_TOPIC = "/emotion/score"
HAS_FACE_TOPIC = "/emotion/has_face"

IMG_SIZE = 224
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# =========================
# 2. 显示窗口开关
# 本地调试默认开；部署到 RDK X5 可用：
# SHOW_EMOTION_WINDOW=0 python gemini2_emotion_test.py
# =========================
SHOW_LOCAL_WINDOW = os.environ.get("SHOW_EMOTION_WINDOW", "1") == "1"
WINDOW_NAME = "Gemini2 Emotion Debug"

# =========================
# 3. 人脸检测参数
# =========================
FACE_SCORE_THRESHOLD = 0.65
FACE_NMS_THRESHOLD = 0.30
FACE_TOPK = 5000
MIN_FACE_SIZE = 60

# 没检到脸时，最多沿用上一次 bbox 的帧数
KEEP_LAST_FACE_FRAMES = 6

# =========================
# 4. 稳定输出参数
# =========================
# 同一个原始情绪连续出现多少帧后，才更新“稳定情绪”
STABLE_EMOTION_FRAMES = 3

# 原始情绪最低置信度，低于它就不更新稳定情绪
MIN_STABLE_SCORE = 0.40

# 连续多少帧没脸，才把稳定情绪重置为 no_face
NO_FACE_RESET_FRAMES = 3


# =========================
# 5. 情绪模型
# =========================
def build_emotion_model(num_classes: int):
    model = models.mobilenet_v3_small(weights=None)
    if hasattr(model.classifier[2], "p"):
        model.classifier[2].p = 0.25
    in_features = model.classifier[-1].in_features
    model.classifier[-1] = nn.Linear(in_features, num_classes)
    return model


def load_emotion_model(ckpt_path: str):
    if not os.path.exists(ckpt_path):
        raise FileNotFoundError(f"找不到情绪模型文件：{ckpt_path}")

    checkpoint = torch.load(ckpt_path, map_location="cpu", weights_only=False)

    if isinstance(checkpoint, dict) and "class_names" in checkpoint:
        class_names = checkpoint["class_names"]
    else:
        class_names = ["angry", "disgust", "fear", "happy", "neutral", "sad", "surprise"]

    model = build_emotion_model(num_classes=len(class_names))

    if isinstance(checkpoint, dict) and "model_state_dict" in checkpoint:
        model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    else:
        model.load_state_dict(checkpoint, strict=True)

    model.to(DEVICE)
    model.eval()
    return model, class_names


EMOTION_TRANSFORM = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.Grayscale(num_output_channels=3),
    transforms.ToTensor(),
    transforms.Normalize(
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    ),
])


def preprocess_face(face_bgr: np.ndarray) -> torch.Tensor:
    rgb = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)
    x = EMOTION_TRANSFORM(pil).unsqueeze(0)
    return x.to(DEVICE)


# =========================
# 6. 人脸检测器
# =========================
def create_face_detector(model_path: str):
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"找不到 YuNet 模型文件：{model_path}")

    if hasattr(cv2, "FaceDetectorYN_create"):
        detector = cv2.FaceDetectorYN_create(
            model_path,
            "",
            (320, 320),
            FACE_SCORE_THRESHOLD,
            FACE_NMS_THRESHOLD,
            FACE_TOPK
        )
    else:
        detector = cv2.FaceDetectorYN.create(
            model_path,
            "",
            (320, 320),
            FACE_SCORE_THRESHOLD,
            FACE_NMS_THRESHOLD,
            FACE_TOPK
        )
    return detector


def enhance_for_face_detection(color_bgr: np.ndarray) -> np.ndarray:
    """
    给 YuNet 做轻量增强，提高低对比度下的人脸可检性。
    """
    gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_eq = clahe.apply(gray)
    bgr_eq = cv2.cvtColor(gray_eq, cv2.COLOR_GRAY2BGR)
    return bgr_eq


# =========================
# 7. 深度工具
# =========================
def robust_depth_from_roi(depth_img: np.ndarray, cx: int, cy: int, encoding: str, radius: int = 4):
    """
    在人脸中心附近取一个小窗口，使用中位数做鲁棒深度。
    0 值和 NaN 会被忽略。
    返回单位统一为 米。
    """
    h, w = depth_img.shape[:2]
    x1 = max(0, cx - radius)
    x2 = min(w, cx + radius + 1)
    y1 = max(0, cy - radius)
    y2 = min(h, cy + radius + 1)

    roi = depth_img[y1:y2, x1:x2]
    if roi.size == 0:
        return float("nan")

    valid = roi[np.isfinite(roi)]
    valid = valid[valid > 0]
    if valid.size == 0:
        return float("nan")

    z = float(np.median(valid))

    enc = encoding.upper()
    if "16U" in enc:
        z_m = z / 1000.0
    else:
        z_m = z

    return z_m


# =========================
# 8. 主节点
# =========================
class Gemini2EmotionNode(Node):
    def __init__(self):
        super().__init__("gemini2_emotion_test")

        self.bridge = CvBridge()

        # 详细结果与调试
        self.result_pub = self.create_publisher(String, RESULT_TOPIC, 10)
        self.depth_pub = self.create_publisher(Float32, DEPTH_RESULT_TOPIC, 10)
        self.debug_pub = self.create_publisher(RosImage, DEBUG_IMAGE_TOPIC, 10)

        # 机械臂简单订阅
        self.raw_label_pub = self.create_publisher(String, RAW_LABEL_TOPIC, 10)
        self.stable_label_pub = self.create_publisher(String, STABLE_LABEL_TOPIC, 10)
        self.stable_id_pub = self.create_publisher(Int32, STABLE_ID_TOPIC, 10)
        self.stable_score_pub = self.create_publisher(Float32, STABLE_SCORE_TOPIC, 10)
        self.has_face_pub = self.create_publisher(Bool, HAS_FACE_TOPIC, 10)

        self.detector = create_face_detector(YUNET_PATH)
        self.model, self.class_names = load_emotion_model(CKPT_PATH)

        # 类别名 -> 编号
        self.class_to_id = {name: idx for idx, name in enumerate(self.class_names)}

        # 原始概率 EMA 平滑
        self.ema_probs = None
        self.ema_alpha = 0.7

        # 上一次人脸框缓存，减少 no_face 闪烁
        self.last_bbox = None
        self.keep_face_frames = 0

        # 稳定情绪状态
        self.last_raw_emotion = None
        self.same_emotion_count = 0
        self.no_face_count = 0

        self.stable_emotion = "no_face"
        self.stable_score = 0.0
        self.stable_id = -1

        self.get_logger().info(f"加载情绪模型：{CKPT_PATH}")
        self.get_logger().info(f"加载人脸检测模型：{YUNET_PATH}")
        self.get_logger().info(f"订阅彩色话题：{COLOR_TOPIC}")
        self.get_logger().info(f"订阅深度话题：{DEPTH_TOPIC}")
        self.get_logger().info(f"本地窗口显示：{'ON' if SHOW_LOCAL_WINDOW else 'OFF'}")

        self.color_sub = Subscriber(self, RosImage, COLOR_TOPIC)
        self.depth_sub = Subscriber(self, RosImage, DEPTH_TOPIC)

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.12
        )
        self.sync.registerCallback(self.sync_callback)

        if SHOW_LOCAL_WINDOW:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    def pick_best_face(self, faces: np.ndarray):
        """
        选一个主脸：面积 * 分数 最大，但太小的人脸过滤掉
        """
        best = None
        best_score = -1.0

        for face in faces:
            x, y, w, h = face[0], face[1], face[2], face[3]
            conf = face[14]

            if w < MIN_FACE_SIZE or h < MIN_FACE_SIZE:
                continue

            score = float(w * h * conf)
            if score > best_score:
                best_score = score
                best = face

        return best

    def infer_emotion(self, face_bgr: np.ndarray):
        x = preprocess_face(face_bgr)
        with torch.no_grad():
            logits = self.model(x)
            probs = torch.softmax(logits, dim=1).cpu().numpy()[0]

        if self.ema_probs is None:
            self.ema_probs = probs
        else:
            self.ema_probs = self.ema_alpha * self.ema_probs + (1.0 - self.ema_alpha) * probs

        pred_idx = int(np.argmax(self.ema_probs))
        pred_name = self.class_names[pred_idx]
        pred_score = float(self.ema_probs[pred_idx])
        return pred_name, pred_score, self.ema_probs

    def update_stable_emotion_with_face(self, raw_emotion: str, raw_score: float):
        """
        有脸时更新稳定情绪。
        只有同一原始情绪连续出现若干帧，且分数足够，才更新稳定输出。
        """
        self.no_face_count = 0

        if raw_emotion == self.last_raw_emotion:
            self.same_emotion_count += 1
        else:
            self.last_raw_emotion = raw_emotion
            self.same_emotion_count = 1

        if raw_score >= MIN_STABLE_SCORE and self.same_emotion_count >= STABLE_EMOTION_FRAMES:
            self.stable_emotion = raw_emotion
            self.stable_score = raw_score
            self.stable_id = self.class_to_id.get(raw_emotion, -1)

    def update_stable_emotion_no_face(self):
        """
        没脸时更新稳定情绪。
        连续几帧没脸后，才真正重置成 no_face。
        """
        self.no_face_count += 1
        self.last_raw_emotion = None
        self.same_emotion_count = 0

        if self.no_face_count >= NO_FACE_RESET_FRAMES:
            self.stable_emotion = "no_face"
            self.stable_score = 0.0
            self.stable_id = -1
            self.ema_probs = None

    def show_local_window(self, vis):
        if SHOW_LOCAL_WINDOW:
            cv2.imshow(WINDOW_NAME, vis)
            cv2.waitKey(1)

    def publish_all_topics(
        self,
        raw_emotion: str,
        raw_score: float,
        has_face: bool,
        depth_m: float,
        bbox,
        debug_image,
        stamp_msg
    ):
        # 详细 JSON
        result = {
            "raw_emotion": raw_emotion,
            "raw_score": round(float(raw_score), 4),
            "stable_emotion": self.stable_emotion,
            "stable_id": int(self.stable_id),
            "stable_score": round(float(self.stable_score), 4),
            "has_face": bool(has_face),
            "depth_m": None if math.isnan(depth_m) else round(float(depth_m), 4),
            "bbox": bbox
        }

        result_msg = String()
        result_msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(result_msg)

        # 原始情绪
        raw_label_msg = String()
        raw_label_msg.data = raw_emotion
        self.raw_label_pub.publish(raw_label_msg)

        # 稳定情绪
        stable_label_msg = String()
        stable_label_msg.data = self.stable_emotion
        self.stable_label_pub.publish(stable_label_msg)

        # 稳定情绪编号
        id_msg = Int32()
        id_msg.data = int(self.stable_id)
        self.stable_id_pub.publish(id_msg)

        # 稳定情绪分数
        score_msg = Float32()
        score_msg.data = float(self.stable_score)
        self.stable_score_pub.publish(score_msg)

        # 是否有人脸
        has_face_msg = Bool()
        has_face_msg.data = bool(has_face)
        self.has_face_pub.publish(has_face_msg)

        # 深度
        depth_msg = Float32()
        depth_msg.data = float("nan") if math.isnan(depth_m) else float(depth_m)
        self.depth_pub.publish(depth_msg)

        # 调试图
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        debug_msg.header = stamp_msg.header
        self.debug_pub.publish(debug_msg)

    def sync_callback(self, color_msg: RosImage, depth_msg: RosImage):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        h, w = color.shape[:2]
        vis = color.copy()

        cv2.putText(
            vis, f"{w}x{h}", (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2
        )

        # 人脸检测前做增强
        detect_img = enhance_for_face_detection(color)

        self.detector.setInputSize((w, h))
        _, faces = self.detector.detect(detect_img)

        selected_bbox = None
        det_conf = 0.0
        used_cached_bbox = False

        if faces is not None and len(faces) > 0:
            face = self.pick_best_face(faces)

            if face is not None:
                x, y, fw, fh = map(int, face[:4])
                det_conf = float(face[14])

                margin_x = int(0.12 * fw)
                margin_y = int(0.18 * fh)

                x1 = max(0, x - margin_x)
                y1 = max(0, y - margin_y)
                x2 = min(w, x + fw + margin_x)
                y2 = min(h, y + fh + margin_y)

                selected_bbox = [x1, y1, x2, y2]
                self.last_bbox = selected_bbox
                self.keep_face_frames = KEEP_LAST_FACE_FRAMES

        # 当前帧没检测到脸，则短时间沿用上一帧 bbox
        if selected_bbox is None and self.last_bbox is not None and self.keep_face_frames > 0:
            selected_bbox = self.last_bbox
            self.keep_face_frames -= 1
            used_cached_bbox = True

        if selected_bbox is None:
            self.update_stable_emotion_no_face()

            cv2.putText(
                vis, "No face", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
            )
            cv2.putText(
                vis, f"stable: {self.stable_emotion} ({self.stable_score:.3f})",
                (20, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )

            self.publish_all_topics(
                raw_emotion="no_face",
                raw_score=0.0,
                has_face=False,
                depth_m=float("nan"),
                bbox=None,
                debug_image=vis,
                stamp_msg=color_msg
            )
            self.show_local_window(vis)
            return

        x1, y1, x2, y2 = selected_bbox
        face_crop = color[y1:y2, x1:x2]
        if face_crop.size == 0:
            self.update_stable_emotion_no_face()

            self.publish_all_topics(
                raw_emotion="no_face",
                raw_score=0.0,
                has_face=False,
                depth_m=float("nan"),
                bbox=None,
                debug_image=vis,
                stamp_msg=color_msg
            )
            self.show_local_window(vis)
            return

        raw_emotion, raw_score, probs = self.infer_emotion(face_crop)
        self.update_stable_emotion_with_face(raw_emotion, raw_score)

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        depth_m = robust_depth_from_roi(depth, cx, cy, depth_msg.encoding, radius=4)

        # 画框
        box_color = (0, 200, 255) if used_cached_bbox else (0, 255, 0)
        cv2.rectangle(vis, (x1, y1), (x2, y2), box_color, 2)
        cv2.circle(vis, (cx, cy), 3, (255, 0, 0), -1)

        text1 = f"raw: {raw_emotion} ({raw_score:.3f})"
        text2 = f"stable: {self.stable_emotion} ({self.stable_score:.3f})"
        text3 = f"face_det: {det_conf:.3f}" if not used_cached_bbox else "face_det: cached"
        text4 = f"depth: {depth_m:.3f} m" if not math.isnan(depth_m) else "depth: invalid"

        cv2.putText(vis, text1, (x1, max(25, y1 - 40)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2)
        cv2.putText(vis, text2, (x1, max(50, y1 - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
        cv2.putText(vis, text3, (x1, min(h - 35, y2 + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        cv2.putText(vis, text4, (x1, min(h - 10, y2 + 45)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2)

        # 右上角显示各类概率
        px = max(10, w - 250)
        py = 30
        for i, name in enumerate(self.class_names):
            s = f"{name}: {probs[i]:.3f}"
            cv2.putText(vis, s, (px, py + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)

        self.publish_all_topics(
            raw_emotion=raw_emotion,
            raw_score=raw_score,
            has_face=True,
            depth_m=depth_m,
            bbox=[int(x1), int(y1), int(x2), int(y2)],
            debug_image=vis,
            stamp_msg=color_msg
        )

        self.show_local_window(vis)


def main():
    rclpy.init()
    node = Gemini2EmotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if SHOW_LOCAL_WINDOW:
                cv2.destroyAllWindows()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
