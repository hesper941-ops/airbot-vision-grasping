#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
from collections import deque, Counter

import cv2
import numpy as np
from hobot_dnn import pyeasy_dnn as dnn

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String, Float32, Int32, Bool
from cv_bridge import CvBridge


PROJECT_DIR = "/home/sunrise/robot/Orbbec_ws/src/emotion"

BIN_PATH = os.path.join(PROJECT_DIR, "emotion_mobilenetv3_small_mid_featuremap.bin")
LABEL_PATH = os.path.join(PROJECT_DIR, "emotion_labels.txt")
YUNET_PATH = os.path.join(PROJECT_DIR, "face_detection_yunet_2023mar.onnx")

COLOR_TOPIC = "/camera/color/image_raw"

DEBUG_IMAGE_TOPIC = "/emotion/debug_image"
RESULT_TOPIC = "/emotion/result"
RAW_LABEL_TOPIC = "/emotion/raw_label"
STABLE_LABEL_TOPIC = "/emotion/label"
STABLE_ID_TOPIC = "/emotion/id"
STABLE_SCORE_TOPIC = "/emotion/score"
HAS_FACE_TOPIC = "/emotion/has_face"

IMG_SIZE = 224
SHOW_LOCAL_WINDOW = os.environ.get("SHOW_EMOTION_WINDOW", "1") == "1"
DEBUG_EMOTION = os.environ.get("DEBUG_EMOTION", "0") == "1"
EMOTION_PREPROCESS = os.environ.get("EMOTION_PREPROCESS", "imagenet").lower()
WINDOW_NAME = "Gemini2 Emotion Debug"

FACE_SCORE_THRESHOLD = 0.65
FACE_NMS_THRESHOLD = 0.30
FACE_TOPK = 5000
MIN_FACE_SIZE = 60

MIN_STABLE_SCORE = 0.45
STABLE_WINDOW_SIZE = 5


def softmax(x: np.ndarray) -> np.ndarray:
    x = x.astype(np.float32)
    x = x - np.max(x)
    e = np.exp(x)
    return e / np.sum(e)


def load_labels(label_path: str):
    if not os.path.exists(label_path):
        raise FileNotFoundError(f"Label file not found: {label_path}")
    with open(label_path, "r", encoding="utf-8") as f:
        labels = [line.strip() for line in f if line.strip()]
    if not labels:
        raise RuntimeError("Label file is empty")
    return labels


def create_face_detector(model_path: str):
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"YuNet model not found: {model_path}")

    if hasattr(cv2, "FaceDetectorYN_create"):
        detector = cv2.FaceDetectorYN_create(
            model_path, "", (320, 320),
            FACE_SCORE_THRESHOLD, FACE_NMS_THRESHOLD, FACE_TOPK
        )
    else:
        detector = cv2.FaceDetectorYN.create(
            model_path, "", (320, 320),
            FACE_SCORE_THRESHOLD, FACE_NMS_THRESHOLD, FACE_TOPK
        )
    return detector


def enhance_for_face_detection(color_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_eq = clahe.apply(gray)
    return cv2.cvtColor(gray_eq, cv2.COLOR_GRAY2BGR)


def read_attr(obj, *names, default=None):
    for name in names:
        if hasattr(obj, name):
            try:
                return getattr(obj, name)
            except Exception:
                pass
    return default


def shape_to_list(shape):
    if shape is None:
        return []
    if isinstance(shape, (list, tuple)):
        return [int(v) for v in shape]
    if hasattr(shape, "__iter__") and not isinstance(shape, str):
        try:
            return [int(v) for v in shape]
        except Exception:
            return []
    return []


class Gemini2EmotionNode(Node):
    def __init__(self):
        super().__init__("gemini2_emotion_x5")

        self.bridge = CvBridge()

        self.result_pub = self.create_publisher(String, RESULT_TOPIC, 10)
        self.debug_pub = self.create_publisher(RosImage, DEBUG_IMAGE_TOPIC, 10)
        self.raw_label_pub = self.create_publisher(String, RAW_LABEL_TOPIC, 10)
        self.stable_label_pub = self.create_publisher(String, STABLE_LABEL_TOPIC, 10)
        self.stable_id_pub = self.create_publisher(Int32, STABLE_ID_TOPIC, 10)
        self.stable_score_pub = self.create_publisher(Float32, STABLE_SCORE_TOPIC, 10)
        self.has_face_pub = self.create_publisher(Bool, HAS_FACE_TOPIC, 10)

        self.class_names = load_labels(LABEL_PATH)
        self.class_to_id = {name: idx for idx, name in enumerate(self.class_names)}

        self.detector = create_face_detector(YUNET_PATH)
        self.model = dnn.load(BIN_PATH)[0]
        self.input_layout = self.detect_input_layout()
        self._io_logged = False
        self._debug_count = 0

        self.recent_results = deque(maxlen=STABLE_WINDOW_SIZE)

        self.sub = self.create_subscription(
            RosImage,
            COLOR_TOPIC,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Loaded BIN: {BIN_PATH}")
        self.get_logger().info(f"Loaded labels: {LABEL_PATH}")
        self.get_logger().info(f"Loaded YuNet: {YUNET_PATH}")
        self.get_logger().info(f"Subscribed topic: {COLOR_TOPIC}")
        self.get_logger().info(f"Using emotion model input layout: {self.input_layout}")
        self.get_logger().info(f"Using emotion preprocess mode: {EMOTION_PREPROCESS}")
        self.log_model_io()

        if SHOW_LOCAL_WINDOW:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    def detect_input_layout(self) -> str:
        """Return NCHW or NHWC when the model metadata makes it clear."""
        try:
            inputs = read_attr(self.model, "inputs", "input_tensors", default=[])
            if not inputs:
                return "NCHW"
            item = inputs[0]
            layout = str(read_attr(item, "layout", "tensor_layout", default="")).upper()
            if "NHWC" in layout:
                return "NHWC"
            if "NCHW" in layout:
                return "NCHW"

            shape = shape_to_list(read_attr(item, "shape", "valid_shape", default=None))
            if len(shape) == 4:
                if shape[-1] == 3:
                    return "NHWC"
                if shape[1] == 3:
                    return "NCHW"
        except Exception:
            pass
        return "NCHW"

    def log_model_io(self):
        """Print model input/output metadata exposed by hobot_dnn."""
        try:
            inputs = read_attr(self.model, "inputs", "input_tensors", default=[])
            outputs = read_attr(self.model, "outputs", "output_tensors", default=[])
            self.get_logger().info(f"Model input count: {len(inputs)}")
            for i, item in enumerate(inputs):
                name = read_attr(item, "name", default="unknown")
                shape = read_attr(item, "shape", "valid_shape", "aligned_shape", default="unknown")
                layout = read_attr(item, "layout", "tensor_layout", default="unknown")
                dtype = read_attr(item, "dtype", "data_type", default="unknown")
                qtype = read_attr(item, "quanti_type", "quant_type", default="unknown")
                scale = read_attr(item, "scale_data", "scale", default="unknown")
                self.get_logger().info(
                    f"Input[{i}] name={name}, shape={shape}, layout={layout}, dtype={dtype}, "
                    f"quant={qtype}, scale={scale}")
            self.get_logger().info(f"Model output count: {len(outputs)}")
            for i, item in enumerate(outputs):
                name = read_attr(item, "name", default="unknown")
                shape = read_attr(item, "shape", "valid_shape", "aligned_shape", default="unknown")
                layout = read_attr(item, "layout", "tensor_layout", default="unknown")
                dtype = read_attr(item, "dtype", "data_type", default="unknown")
                qtype = read_attr(item, "quanti_type", "quant_type", default="unknown")
                scale = read_attr(item, "scale_data", "scale", default="unknown")
                self.get_logger().info(
                    f"Output[{i}] name={name}, shape={shape}, layout={layout}, dtype={dtype}, "
                    f"quant={qtype}, scale={scale}")
        except Exception as exc:
            self.get_logger().warning(f"Failed to inspect model IO metadata: {exc}")

    def preprocess_face(self, face_bgr: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        gray3 = np.stack([gray, gray, gray], axis=-1)

        img = cv2.resize(gray3, (IMG_SIZE, IMG_SIZE)).astype(np.float32)

        if EMOTION_PREPROCESS == "imagenet":
            img = img / 255.0
            mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
            std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
            img = (img - mean) / std
        elif EMOTION_PREPROCESS == "zero_one":
            img = img / 255.0
        elif EMOTION_PREPROCESS == "minus_one_one":
            img = img / 127.5 - 1.0
        elif EMOTION_PREPROCESS == "uint8":
            pass
        else:
            raise ValueError(
                "Unsupported EMOTION_PREPROCESS. Use imagenet, zero_one, minus_one_one, or uint8.")

        if self.input_layout == "NCHW":
            img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        return img.astype(np.float32)

    def infer_emotion(self, face_bgr: np.ndarray):
        x = self.preprocess_face(face_bgr)

        if DEBUG_EMOTION and self._debug_count < 20:
            self.get_logger().info(
                f"Input stats: shape={x.shape}, dtype={x.dtype}, "
                f"min={float(np.min(x)):.4f}, max={float(np.max(x)):.4f}, "
                f"mean={float(np.mean(x)):.4f}, std={float(np.std(x)):.4f}")

        try:
            outputs = self.model.forward(x)
        except TypeError:
            outputs = self.model.forward([x])

        out0 = outputs[0] if isinstance(outputs, (list, tuple)) else outputs
        logits = np.array(out0.buffer, dtype=np.float32).reshape(-1)
        if logits.size > len(self.class_names):
            if DEBUG_EMOTION and self._debug_count < 20:
                self.get_logger().warning(
                    f"Output buffer has {logits.size} values, using first {len(self.class_names)} classes")
            logits = logits[:len(self.class_names)]
        probs = softmax(logits)

        if DEBUG_EMOTION and self._debug_count < 20:
            pairs = ", ".join(
                f"{name}={float(probs[i]):.4f}"
                for i, name in enumerate(self.class_names)
                if i < len(probs)
            )
            self.get_logger().info(
                f"Output logits={np.array2string(logits, precision=4, separator=', ')}, "
                f"probs: {pairs}")
            self._debug_count += 1

        pred_idx = int(np.argmax(probs))
        pred_name = self.class_names[pred_idx]
        pred_score = float(probs[pred_idx])

        return pred_name, pred_score, probs

    def pick_best_face(self, faces: np.ndarray):
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

    def get_stable_result(self, raw_label: str, raw_score: float):
        if raw_score >= MIN_STABLE_SCORE:
            self.recent_results.append(raw_label)

        if len(self.recent_results) == 0:
            return "unknown", -1, 0.0

        counter = Counter(self.recent_results)
        stable_label, count = counter.most_common(1)[0]
        stable_score = count / len(self.recent_results)
        stable_id = self.class_to_id.get(stable_label, -1)
        return stable_label, stable_id, float(stable_score)

    def publish_no_face(self, vis, color_msg):
        result = {
            "raw_emotion": "no_face",
            "raw_score": 0.0,
            "stable_emotion": "no_face",
            "stable_id": -1,
            "stable_score": 0.0,
            "has_face": False,
            "bbox": None
        }

        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(msg)

        m1 = String(); m1.data = "no_face"; self.raw_label_pub.publish(m1)
        m2 = String(); m2.data = "no_face"; self.stable_label_pub.publish(m2)
        m3 = Int32(); m3.data = -1; self.stable_id_pub.publish(m3)
        m4 = Float32(); m4.data = 0.0; self.stable_score_pub.publish(m4)
        m5 = Bool(); m5.data = False; self.has_face_pub.publish(m5)

        debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        debug_msg.header = color_msg.header
        self.debug_pub.publish(debug_msg)

    def image_callback(self, color_msg: RosImage):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        h, w = color.shape[:2]
        vis = color.copy()

        detect_img = enhance_for_face_detection(color)
        self.detector.setInputSize((w, h))
        _, faces = self.detector.detect(detect_img)

        if faces is None or len(faces) == 0:
            cv2.putText(vis, "No face", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            self.publish_no_face(vis, color_msg)
            if SHOW_LOCAL_WINDOW:
                cv2.imshow(WINDOW_NAME, vis)
                cv2.waitKey(1)
            return

        face = self.pick_best_face(faces)
        if face is None:
            cv2.putText(vis, "No valid face", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            self.publish_no_face(vis, color_msg)
            if SHOW_LOCAL_WINDOW:
                cv2.imshow(WINDOW_NAME, vis)
                cv2.waitKey(1)
            return

        x, y, fw, fh = map(int, face[:4])
        conf = float(face[14])

        margin_x = int(0.12 * fw)
        margin_y = int(0.18 * fh)

        x1 = max(0, x - margin_x)
        y1 = max(0, y - margin_y)
        x2 = min(w, x + fw + margin_x)
        y2 = min(h, y + fh + margin_y)

        face_crop = color[y1:y2, x1:x2]
        if face_crop.size == 0:
            self.publish_no_face(vis, color_msg)
            if SHOW_LOCAL_WINDOW:
                cv2.imshow(WINDOW_NAME, vis)
                cv2.waitKey(1)
            return

        raw_label, raw_score, probs = self.infer_emotion(face_crop)
        stable_label, stable_id, stable_score = self.get_stable_result(raw_label, raw_score)

        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.putText(vis, f"raw: {raw_label} ({raw_score:.3f})", (x1, max(25, y1 - 40)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2)
        cv2.putText(vis, f"stable: {stable_label} ({stable_score:.3f})", (x1, max(50, y1 - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
        cv2.putText(vis, f"face_det: {conf:.3f}", (x1, min(h - 10, y2 + 25)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        px = max(10, w - 250)
        py = 30
        for i, name in enumerate(self.class_names):
            cv2.putText(vis, f"{name}: {probs[i]:.3f}", (px, py + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)

        result = {
            "raw_emotion": raw_label,
            "raw_score": round(float(raw_score), 4),
            "stable_emotion": stable_label,
            "stable_id": int(stable_id),
            "stable_score": round(float(stable_score), 4),
            "has_face": True,
            "bbox": [int(x1), int(y1), int(x2), int(y2)]
        }

        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(msg)

        m1 = String(); m1.data = raw_label; self.raw_label_pub.publish(m1)
        m2 = String(); m2.data = stable_label; self.stable_label_pub.publish(m2)
        m3 = Int32(); m3.data = stable_id; self.stable_id_pub.publish(m3)
        m4 = Float32(); m4.data = float(stable_score); self.stable_score_pub.publish(m4)
        m5 = Bool(); m5.data = True; self.has_face_pub.publish(m5)

        debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        debug_msg.header = color_msg.header
        self.debug_pub.publish(debug_msg)

        if SHOW_LOCAL_WINDOW:
            cv2.imshow(WINDOW_NAME, vis)
            cv2.waitKey(1)


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
