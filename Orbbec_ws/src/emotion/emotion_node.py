#!/usr/bin/env python3
import json
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from hobot_dnn import pyeasy_dnn as dnn


class EmotionNode(Node):
    def __init__(self):
        super().__init__("emotion_node")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("result_topic", "/emotion/result")
        self.declare_parameter("vis_topic", "/emotion/vis_image")
        self.declare_parameter("model_path", "/home/sunrise/robot/Orbbec_ws/src/emotion/emotion_resnet18_5cls_224.bin")
        self.declare_parameter("label_path", "/home/sunrise/robot/Orbbec_ws/src/emotion/emotion_labels.txt")
        self.declare_parameter("show_image", False)
        self.declare_parameter("process_every_n", 3)
        self.declare_parameter("min_face_size", 60)
        self.declare_parameter("face_margin", 0.25)
        self.declare_parameter("conf_threshold", 0.35)

        self.image_topic = self.get_parameter("image_topic").value
        self.result_topic = self.get_parameter("result_topic").value
        self.vis_topic = self.get_parameter("vis_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.label_path = self.get_parameter("label_path").value
        self.show_image = bool(self.get_parameter("show_image").value)
        self.process_every_n = int(self.get_parameter("process_every_n").value)
        self.min_face_size = int(self.get_parameter("min_face_size").value)
        self.face_margin = float(self.get_parameter("face_margin").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)

        self.input_size = 224
        self.frame_count = 0
        self.last_result_time = 0.0
        self.window_name = "RDK X5 Emotion Recognition"

        self.bridge = CvBridge()

        self.labels = self.load_labels(self.label_path)
        self.model = self.load_model(self.model_path)
        self.face_detector = self.load_face_detector()

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.vis_pub = self.create_publisher(Image, self.vis_topic, 10)

        if self.show_image:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 720)

        self.get_logger().info("Emotion node started.")
        self.get_logger().info(f"Subscribe image topic: {self.image_topic}")
        self.get_logger().info(f"Publish result topic: {self.result_topic}")
        self.get_logger().info(f"Publish visual topic: {self.vis_topic}")
        self.get_logger().info(f"Show image window: {self.show_image}")
        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"Labels: {self.labels}")

    def load_labels(self, label_path):
        p = Path(label_path)
        if not p.exists():
            raise FileNotFoundError(f"Label file not found: {label_path}")

        labels = []
        with open(p, "r", encoding="utf-8") as f:
            for line in f:
                name = line.strip()
                if name:
                    labels.append(name)

        if len(labels) == 0:
            raise RuntimeError("Label file is empty.")

        return labels

    def load_model(self, model_path):
        p = Path(model_path)
        if not p.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")

        models = dnn.load(str(p))
        if len(models) == 0:
            raise RuntimeError("No model loaded.")

        self.get_logger().info("BPU model loaded successfully.")
        return models[0]

    def load_face_detector(self):
        candidate_paths = []

        try:
            candidate_paths.append(str(Path(cv2.data.haarcascades) / "haarcascade_frontalface_default.xml"))
        except Exception:
            pass

        candidate_paths.extend([
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        ])

        for path in candidate_paths:
            if path and Path(path).exists():
                detector = cv2.CascadeClassifier(path)
                if not detector.empty():
                    self.get_logger().info(f"Loaded face detector: {path}")
                    return detector

        raise FileNotFoundError("Cannot find haarcascade_frontalface_default.xml.")

    def image_callback(self, msg):
        self.frame_count += 1

        if self.process_every_n > 1 and self.frame_count % self.process_every_n != 0:
            return

        t0 = time.time()

        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge convert failed: {e}")
            return

        if rgb is None or rgb.size == 0:
            return

        vis = rgb.copy()
        face_box = self.detect_largest_face(rgb)

        if face_box is None:
            result = {
                "status": "no_face",
                "emotion": "unknown",
                "confidence": 0.0,
                "box": None,
                "timestamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                }
            }

            now = time.time()
            if now - self.last_result_time > 1.0:
                self.publish_result(result)
                self.last_result_time = now

            self.draw_no_face(vis)
            self.publish_vis_and_show(vis, msg.header)
            return

        x1, y1, x2, y2 = face_box
        face_rgb = rgb[y1:y2, x1:x2]

        if face_rgb.size == 0:
            return

        face_resized = cv2.resize(
            face_rgb,
            (self.input_size, self.input_size),
            interpolation=cv2.INTER_AREA
        )

        nv12 = self.rgb_to_nv12(face_resized)

        try:
            outputs = self.model.forward(nv12)
            logits = np.array(outputs[0].buffer, dtype=np.float32).reshape(-1)
        except Exception as e:
            self.get_logger().error(f"BPU inference failed: {e}")
            return

        probs = self.softmax(logits)
        cls_id = int(np.argmax(probs))
        confidence = float(probs[cls_id])
        emotion = self.labels[cls_id] if cls_id < len(self.labels) else str(cls_id)

        t1 = time.time()
        infer_ms = (t1 - t0) * 1000.0

        status = "ok" if confidence >= self.conf_threshold else "low_confidence"

        result = {
            "status": status,
            "emotion": emotion,
            "confidence": round(confidence, 4),
            "class_id": cls_id,
            "box": [int(x1), int(y1), int(x2), int(y2)],
            "infer_ms": round(infer_ms, 2),
            "timestamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            }
        }

        self.publish_result(result)
        self.draw_result(vis, face_box, emotion, confidence, infer_ms, status)
        self.publish_vis_and_show(vis, msg.header)

    def detect_largest_face(self, rgb):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        gray = cv2.equalizeHist(gray)

        faces = self.face_detector.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(self.min_face_size, self.min_face_size),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        if faces is None or len(faces) == 0:
            return None

        h, w = rgb.shape[:2]
        faces = sorted(faces, key=lambda b: b[2] * b[3], reverse=True)

        x, y, fw, fh = faces[0]

        cx = x + fw / 2.0
        cy = y + fh / 2.0
        side = max(fw, fh) * (1.0 + self.face_margin)

        x1 = int(round(cx - side / 2.0))
        y1 = int(round(cy - side / 2.0))
        x2 = int(round(cx + side / 2.0))
        y2 = int(round(cy + side / 2.0))

        x1 = max(0, min(w - 1, x1))
        y1 = max(0, min(h - 1, y1))
        x2 = max(0, min(w, x2))
        y2 = max(0, min(h, y2))

        if x2 <= x1 or y2 <= y1:
            return None

        return x1, y1, x2, y2

    def rgb_to_nv12(self, rgb):
        h, w = rgb.shape[:2]

        if h % 2 != 0 or w % 2 != 0:
            rgb = cv2.resize(rgb, (w - w % 2, h - h % 2))
            h, w = rgb.shape[:2]

        area = h * w
        yuv420p = cv2.cvtColor(rgb, cv2.COLOR_RGB2YUV_I420).reshape((area * 3 // 2,))

        y = yuv420p[:area]
        uv_planar = yuv420p[area:].reshape((2, area // 4))
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))

        nv12 = np.zeros((area * 3 // 2,), dtype=np.uint8)
        nv12[:area] = y
        nv12[area:] = uv_packed

        return nv12

    def softmax(self, x):
        x = x.astype(np.float32)
        x = x - np.max(x)
        exp_x = np.exp(x)
        return exp_x / np.sum(exp_x)

    def publish_result(self, result):
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(msg)

    def publish_vis_and_show(self, rgb, header):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
            img_msg.header = header
            self.vis_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f"publish vis image failed: {e}")

        if self.show_image:
            try:
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                cv2.imshow(self.window_name, bgr)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f"show image failed: {e}")

    def draw_no_face(self, rgb):
        cv2.putText(
            rgb,
            "No face detected",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 0),
            2,
            cv2.LINE_AA
        )

    def draw_result(self, rgb, box, emotion, confidence, infer_ms, status):
        x1, y1, x2, y2 = box

        color = self.get_color(emotion)

        cv2.rectangle(rgb, (x1, y1), (x2, y2), color, 2)

        text1 = f"{emotion}  conf={confidence:.2f}"
        text2 = f"{status}  {infer_ms:.1f} ms"

        y_text1 = max(25, y1 - 35)
        y_text2 = max(50, y1 - 10)

        cv2.putText(
            rgb,
            text1,
            (x1, y_text1),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            rgb,
            text2,
            (x1, y_text2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            rgb,
            "Emotion Recognition on RDK X5",
            (20, rgb.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

    def get_color(self, emotion):
        if emotion == "happy":
            return (0, 255, 0)
        if emotion == "low_mood":
            return (255, 128, 0)
        if emotion == "negative_distress":
            return (255, 0, 0)
        if emotion == "neutral":
            return (0, 180, 255)
        if emotion == "surprise":
            return (255, 255, 0)
        return (255, 255, 255)


def main():
    rclpy.init()
    node = EmotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_image:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()