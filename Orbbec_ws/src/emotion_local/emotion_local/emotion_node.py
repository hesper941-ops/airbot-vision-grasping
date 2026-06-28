#!/usr/bin/env python3
"""RDK X5 5-class face emotion recognition node (local model)."""
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
        super().__init__('emotion_local_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('result_topic', '/emotion/result')
        self.declare_parameter('vis_topic', '/emotion/vis_image')
        self.declare_parameter('model_path',
            '/home/sunrise/robot/Orbbec_ws/src/emotion_local/emotion_resnet18_5cls_plus_local_224.bin')
        self.declare_parameter('label_path',
            '/home/sunrise/robot/Orbbec_ws/src/emotion_local/emotion_labels.txt')
        self.declare_parameter('show_image', False)
        self.declare_parameter('process_every_n', 3)
        self.declare_parameter('min_face_size', 60)
        self.declare_parameter('face_margin', 0.25)
        self.declare_parameter('conf_threshold', 0.35)

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

        self.input_size = 224
        self.frame_count = 0
        self.last_result_time = 0.0
        self.window_name = 'RDK X5 Emotion Recognition (Local Model)'

        self.bridge = CvBridge()
        self.labels = self._load_labels()
        self.model = self._load_model()
        self.face_detector = self._load_face_detector()

        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.vis_pub = self.create_publisher(Image, self.vis_topic, 10)

        if self.show_image:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 720)

        self.get_logger().info(f'Emotion local node started | model={self.model_path}')
        self.get_logger().info(f'Labels: {self.labels}')

    def _load_labels(self):
        p = Path(self.label_path)
        if not p.exists():
            raise FileNotFoundError(f'Label file not found: {self.label_path}')
        labels = []
        with open(p, 'r', encoding='utf-8') as f:
            for line in f:
                name = line.strip()
                if name:
                    labels.append(name)
        if len(labels) == 0:
            raise RuntimeError('Label file is empty.')
        return labels

    def _load_model(self):
        p = Path(self.model_path)
        if not p.exists():
            raise FileNotFoundError(f'Model file not found: {self.model_path}')
        models = dnn.load(str(p))
        if len(models) == 0:
            raise RuntimeError('No model loaded.')
        self.get_logger().info('BPU model loaded successfully.')
        return models[0]

    def _load_face_detector(self):
        candidates = [
            '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
        ]
        for path in candidates:
            if Path(path).exists():
                d = cv2.CascadeClassifier(path)
                if not d.empty():
                    self.get_logger().info(f'Face detector: {path}')
                    return d
        raise FileNotFoundError('Cannot find haarcascade_frontalface_default.xml')

    def image_callback(self, msg):
        self.frame_count += 1
        if self.process_every_n > 1 and self.frame_count % self.process_every_n != 0:
            return

        t0 = time.time()
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return
        if rgb is None or rgb.size == 0:
            return

        vis = rgb.copy()
        face_box = self._detect_largest_face(rgb)

        if face_box is None:
            now = time.time()
            if now - self.last_result_time > 1.0:
                self._publish_result({'status': 'no_face', 'emotion': 'unknown',
                    'confidence': 0.0, 'box': None})
                self.last_result_time = now
            cv2.putText(vis, 'No face detected', (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            self._publish_vis(vis, msg.header)
            return

        x1, y1, x2, y2 = face_box
        face_rgb = rgb[y1:y2, x1:x2]
        if face_rgb.size == 0:
            return

        face_resized = cv2.resize(face_rgb, (self.input_size, self.input_size),
            interpolation=cv2.INTER_AREA)
        nv12 = self._rgb_to_nv12(face_resized)

        try:
            outputs = self.model.forward(nv12)
            logits = np.array(outputs[0].buffer, dtype=np.float32).reshape(-1)
        except Exception as e:
            self.get_logger().error(f'BPU inference failed: {e}')
            return

        probs = self._softmax(logits)
        cls_id = int(np.argmax(probs))
        confidence = float(probs[cls_id])
        emotion = self.labels[cls_id] if cls_id < len(self.labels) else str(cls_id)
        infer_ms = (time.time() - t0) * 1000.0
        status = 'ok' if confidence >= self.conf_threshold else 'low_confidence'

        self._publish_result({
            'status': status, 'emotion': emotion,
            'confidence': round(confidence, 4), 'class_id': cls_id,
            'box': [int(x1), int(y1), int(x2), int(y2)],
            'infer_ms': round(infer_ms, 2),
        })
        self._draw_result(vis, face_box, emotion, confidence, infer_ms, status)
        self._publish_vis(vis, msg.header)

    def _detect_largest_face(self, rgb):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        gray = cv2.equalizeHist(gray)
        faces = self.face_detector.detectMultiScale(gray, scaleFactor=1.1,
            minNeighbors=5, minSize=(self.min_face_size, self.min_face_size),
            flags=cv2.CASCADE_SCALE_IMAGE)
        if faces is None or len(faces) == 0:
            return None
        h, w = rgb.shape[:2]
        x, y, fw, fh = sorted(faces, key=lambda b: b[2] * b[3], reverse=True)[0]
        cx, cy = x + fw / 2.0, y + fh / 2.0
        side = max(fw, fh) * (1.0 + self.face_margin)
        x1 = max(0, int(cx - side / 2))
        y1 = max(0, int(cy - side / 2))
        x2 = min(w, int(cx + side / 2))
        y2 = min(h, int(cy + side / 2))
        return (x1, y1, x2, y2) if x2 > x1 and y2 > y1 else None

    def _rgb_to_nv12(self, rgb):
        h, w = rgb.shape[:2]
        if h % 2 or w % 2:
            rgb = cv2.resize(rgb, (w - w % 2, h - h % 2))
            h, w = rgb.shape[:2]
        area = h * w
        yuv = cv2.cvtColor(rgb, cv2.COLOR_RGB2YUV_I420).reshape((area * 3 // 2,))
        nv12 = np.zeros(area * 3 // 2, dtype=np.uint8)
        nv12[:area] = yuv[:area]
        uv = yuv[area:].reshape((2, area // 4)).transpose((1, 0)).reshape((area // 2,))
        nv12[area:] = uv
        return nv12

    def _softmax(self, x):
        x = x - np.max(x)
        e = np.exp(x)
        return e / np.sum(e)

    def _publish_result(self, result):
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(msg)

    def _publish_vis(self, rgb, header):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            img_msg.header = header
            self.vis_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'publish vis: {e}')
        if self.show_image:
            try:
                cv2.imshow(self.window_name, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)
            except Exception:
                pass

    def _draw_result(self, rgb, box, emotion, confidence, infer_ms, status):
        x1, y1, x2, y2 = box
        colors = {'happy': (0, 255, 0), 'low_mood': (255, 128, 0),
            'negative_distress': (255, 0, 0), 'neutral': (0, 180, 255),
            'surprise': (255, 255, 0)}
        color = colors.get(emotion, (255, 255, 255))
        cv2.rectangle(rgb, (x1, y1), (x2, y2), color, 2)
        yt = max(25, y1 - 35)
        cv2.putText(rgb, f'{emotion}  {confidence:.2f}', (x1, yt),
            cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
        cv2.putText(rgb, f'{status}  {infer_ms:.1f}ms', (x1, max(50, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(rgb, 'Emotion Recognition on RDK X5 (local model)',
            (20, rgb.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (255, 255, 255), 2)


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


if __name__ == '__main__':
    main()
