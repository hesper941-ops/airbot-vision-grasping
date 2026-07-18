#!/usr/bin/env python3
"""Emotion recognition node for the deployed ResNet18 plus-local BPU model.

The startup command keeps using emotion_fusion_node.py, but this node now runs
one local 5-class model:
  /home/sunrise/robot/Orbbec_ws/src/emotion_local/emotion_resnet18_5cls_plus_local_224.bin

Model IO checked on RDK X5:
  input:  NV12, shape=(1, 3, 224, 224), dtype=uint8
  output: logits, shape=(1, 5, 1, 1), dtype=float32
"""

from collections import Counter, deque
import json
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32, String
from cv_bridge import CvBridge
from hobot_dnn import pyeasy_dnn as dnn


IMG_SIZE = 224
STABLE_WINDOW_SIZE = 5


def softmax(x: np.ndarray) -> np.ndarray:
    x = x.astype(np.float32)
    x = x - np.max(x)
    e = np.exp(x)
    return e / np.sum(e)


class EmotionFusionNode(Node):
    def __init__(self):
        super().__init__('emotion_fusion_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('result_topic', '/emotion/result')
        self.declare_parameter('vis_topic', '/emotion/vis_image')
        self.declare_parameter(
            'model_path',
            '/home/sunrise/robot/Orbbec_ws/src/emotion_local/emotion_resnet18_5cls_plus_local_224.bin')
        self.declare_parameter(
            'label_path',
            '/home/sunrise/robot/Orbbec_ws/src/emotion_local/emotion_labels.txt')
        self.declare_parameter('show_image', False)
        self.declare_parameter('process_every_n', 3)
        self.declare_parameter('min_face_size', 60)
        self.declare_parameter('face_margin', 0.25)
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('min_stable_score', 0.40)

        self.image_topic = self.get_parameter('image_topic').value
        self.result_topic = self.get_parameter('result_topic').value
        self.vis_topic = self.get_parameter('vis_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.label_path = self.get_parameter('label_path').value
        self.show_image = bool(self.get_parameter('show_image').value)
        self.process_every_n = int(self.get_parameter('process_every_n').value)
        self.min_face_size = int(self.get_parameter('min_face_size').value)
        self.face_margin = float(self.get_parameter('face_margin').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.min_stable_score = float(self.get_parameter('min_stable_score').value)

        self.frame_count = 0
        self.recent_results = deque(maxlen=STABLE_WINDOW_SIZE)
        self.window_name = 'RDK X5 Emotion Recognition'

        self.bridge = CvBridge()
        self.labels = self.load_labels(self.label_path)
        self.class_to_id = {name: idx for idx, name in enumerate(self.labels)}
        self.model = self.load_model(self.model_path)
        self.face_detector = self.load_face_detector()

        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.vis_pub = self.create_publisher(Image, self.vis_topic, 10)
        self.raw_label_pub = self.create_publisher(String, '/emotion/raw_label', 10)
        self.stable_label_pub = self.create_publisher(String, '/emotion/label', 10)
        self.stable_id_pub = self.create_publisher(Int32, '/emotion/id', 10)
        self.stable_score_pub = self.create_publisher(Float32, '/emotion/score', 10)
        self.has_face_pub = self.create_publisher(Bool, '/emotion/has_face', 10)

        if self.show_image:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 720)

        self.get_logger().info('ResNet18 plus-local emotion node started')
        self.get_logger().info(f'Model: {self.model_path}')
        self.get_logger().info(f'Labels: {self.labels}')

    def load_labels(self, label_path: str):
        p = Path(label_path)
        if not p.exists():
            raise FileNotFoundError(f'Label file not found: {label_path}')
        labels = [line.strip() for line in p.read_text(encoding='utf-8').splitlines() if line.strip()]
        if not labels:
            raise RuntimeError('Label file is empty.')
        return labels

    def load_model(self, model_path: str):
        p = Path(model_path)
        if not p.exists():
            raise FileNotFoundError(f'Model file not found: {model_path}')
        models = dnn.load(str(p))
        if len(models) == 0:
            raise RuntimeError('No model loaded.')
        self.log_model_io(models[0])
        return models[0]

    def log_model_io(self, model):
        try:
            for section in ('inputs', 'outputs'):
                tensors = getattr(model, section, [])
                self.get_logger().info(f'Model {section} count: {len(tensors)}')
                for i, tensor in enumerate(tensors):
                    props = getattr(tensor, 'properties', None)
                    if props is None:
                        continue
                    self.get_logger().info(
                        f'{section}[{i}] name={getattr(tensor, "name", "unknown")}, '
                        f'shape={getattr(props, "shape", "unknown")}, '
                        f'layout={getattr(props, "layout", "unknown")}, '
                        f'dtype={getattr(props, "dtype", "unknown")}, '
                        f'tensor_type={getattr(props, "tensor_type", "unknown")}')
        except Exception as exc:
            self.get_logger().warning(f'Failed to inspect model IO: {exc}')

    def load_face_detector(self):
        candidates = []
        try:
            candidates.append(str(Path(cv2.data.haarcascades) / 'haarcascade_frontalface_default.xml'))
        except Exception:
            pass
        candidates.extend([
            '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
        ])
        for item in candidates:
            if item and Path(item).exists():
                detector = cv2.CascadeClassifier(item)
                if not detector.empty():
                    self.get_logger().info(f'Face detector: {item}')
                    return detector
        raise FileNotFoundError('Cannot find haarcascade_frontalface_default.xml')

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.process_every_n > 1 and self.frame_count % self.process_every_n != 0:
            return

        t0 = time.time()
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge convert failed: {exc}')
            return
        if rgb is None or rgb.size == 0:
            return

        vis = rgb.copy()
        face_box = self.detect_largest_face(rgb)
        if face_box is None:
            self.publish_no_face(vis, msg)
            return

        x1, y1, x2, y2 = face_box
        face_rgb = rgb[y1:y2, x1:x2]
        if face_rgb.size == 0:
            self.publish_no_face(vis, msg)
            return

        face_resized = cv2.resize(face_rgb, (IMG_SIZE, IMG_SIZE), interpolation=cv2.INTER_AREA)
        nv12 = self.rgb_to_nv12(face_resized)

        try:
            outputs = self.model.forward(nv12)
            logits = np.array(outputs[0].buffer, dtype=np.float32).reshape(-1)
        except Exception as exc:
            self.get_logger().error(f'BPU inference failed: {exc}')
            return

        if logits.size > len(self.labels):
            logits = logits[:len(self.labels)]
        probs = softmax(logits)
        raw_id = int(np.argmax(probs))
        raw_label = self.labels[raw_id] if raw_id < len(self.labels) else str(raw_id)
        raw_score = float(probs[raw_id])
        stable_label, stable_id, stable_score = self.get_stable_result(raw_label, raw_score)
        infer_ms = (time.time() - t0) * 1000.0
        status = 'ok' if raw_score >= self.conf_threshold else 'low_confidence'

        result = {
            'status': status,
            'emotion': stable_label,
            'confidence': round(raw_score, 4),
            'raw_emotion': raw_label,
            'raw_score': round(raw_score, 4),
            'stable_emotion': stable_label,
            'stable_id': int(stable_id),
            'stable_score': round(stable_score, 4),
            'has_face': True,
            'class_id': int(raw_id),
            'box': [int(x1), int(y1), int(x2), int(y2)],
            'infer_ms': round(infer_ms, 2),
            'model': 'emotion_resnet18_5cls_plus_local_224',
            'timestamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
        }
        self.publish_result(result)
        self.publish_simple_topics(raw_label, stable_label, stable_id, stable_score, True)
        self.draw_result(vis, face_box, raw_label, raw_score, stable_label, stable_score, infer_ms, status)
        self.publish_vis(vis, msg.header)

    def detect_largest_face(self, rgb):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        gray = cv2.equalizeHist(gray)
        faces = self.face_detector.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(self.min_face_size, self.min_face_size),
            flags=cv2.CASCADE_SCALE_IMAGE,
        )
        if faces is None or len(faces) == 0:
            return None
        h, w = rgb.shape[:2]
        x, y, fw, fh = sorted(faces, key=lambda b: b[2] * b[3], reverse=True)[0]
        cx = x + fw / 2.0
        cy = y + fh / 2.0
        side = max(fw, fh) * (1.0 + self.face_margin)
        x1 = max(0, int(cx - side / 2))
        y1 = max(0, int(cy - side / 2))
        x2 = min(w, int(cx + side / 2))
        y2 = min(h, int(cy + side / 2))
        return (x1, y1, x2, y2) if x2 > x1 and y2 > y1 else None

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

    def get_stable_result(self, raw_label: str, raw_score: float):
        if raw_score >= self.min_stable_score:
            self.recent_results.append(raw_label)
        if len(self.recent_results) == 0:
            return 'unknown', -1, 0.0
        counter = Counter(self.recent_results)
        stable_label, count = counter.most_common(1)[0]
        stable_score = count / len(self.recent_results)
        stable_id = self.class_to_id.get(stable_label, -1)
        return stable_label, stable_id, float(stable_score)

    def publish_no_face(self, vis, msg: Image):
        result = {
            'status': 'no_face',
            'emotion': 'unknown',
            'confidence': 0.0,
            'raw_emotion': 'no_face',
            'raw_score': 0.0,
            'stable_emotion': 'no_face',
            'stable_id': -1,
            'stable_score': 0.0,
            'has_face': False,
            'box': None,
            'model': 'emotion_resnet18_5cls_plus_local_224',
            'timestamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
        }
        self.publish_result(result)
        self.publish_simple_topics('no_face', 'no_face', -1, 0.0, False)
        self.draw_no_face(vis)
        self.publish_vis(vis, msg.header)

    def publish_result(self, result: dict):
        out = String()
        out.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(out)

    def publish_simple_topics(self, raw_label, stable_label, stable_id, stable_score, has_face):
        m1 = String(); m1.data = raw_label; self.raw_label_pub.publish(m1)
        m2 = String(); m2.data = stable_label; self.stable_label_pub.publish(m2)
        m3 = Int32(); m3.data = int(stable_id); self.stable_id_pub.publish(m3)
        m4 = Float32(); m4.data = float(stable_score); self.stable_score_pub.publish(m4)
        m5 = Bool(); m5.data = bool(has_face); self.has_face_pub.publish(m5)

    def publish_vis(self, rgb, header):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            img_msg.header = header
            self.vis_pub.publish(img_msg)
        except Exception as exc:
            self.get_logger().warn(f'publish vis image failed: {exc}')
        if self.show_image:
            try:
                cv2.imshow(self.window_name, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)
            except Exception:
                pass

    def draw_no_face(self, rgb):
        cv2.putText(rgb, 'No face detected', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)

    def draw_result(self, rgb, box, raw_label, raw_score, stable_label, stable_score, infer_ms, status):
        x1, y1, x2, y2 = box
        colors = {
            'happy': (0, 255, 0),
            'low_mood': (255, 128, 0),
            'negative_distress': (255, 0, 0),
            'neutral': (0, 180, 255),
            'surprise': (255, 255, 0),
        }
        color = colors.get(stable_label, (255, 255, 255))
        cv2.rectangle(rgb, (x1, y1), (x2, y2), color, 2)
        cv2.putText(rgb, f'raw: {raw_label} {raw_score:.2f}', (x1, max(25, y1 - 40)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)
        cv2.putText(rgb, f'stable: {stable_label} {stable_score:.2f}', (x1, max(50, y1 - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
        cv2.putText(rgb, f'{status} {infer_ms:.0f}ms', (x1, min(rgb.shape[0] - 10, y2 + 25)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)


def main():
    rclpy.init()
    node = EmotionFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_image:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
