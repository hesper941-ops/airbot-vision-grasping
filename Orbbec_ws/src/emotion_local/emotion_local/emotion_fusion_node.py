#!/usr/bin/env python3
"""Dual-model emotion recognition with weighted fusion.

Old model strengths: happy, neutral, surprise
New model strengths: low_mood, negative_distress

Fusion: weighted average of both models' probability distributions.
"""
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


class EmotionFusionNode(Node):
    def __init__(self):
        super().__init__('emotion_fusion_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('result_topic', '/emotion/result')
        self.declare_parameter('vis_topic', '/emotion/vis_image')

        old_model = '/home/sunrise/robot/Orbbec_ws/src/emotion/emotion_resnet18_5cls_224.bin'
        new_model = '/home/sunrise/robot/Orbbec_ws/src/emotion_local/emotion_resnet18_5cls_plus_local_224.bin'
        labels_path = '/home/sunrise/robot/Orbbec_ws/src/emotion_local/emotion_labels.txt'

        self.declare_parameter('old_model_path', old_model)
        self.declare_parameter('new_model_path', new_model)
        self.declare_parameter('label_path', labels_path)
        self.declare_parameter('show_image', False)
        self.declare_parameter('process_every_n', 2)
        self.declare_parameter('min_face_size', 60)
        self.declare_parameter('face_margin', 0.25)
        self.declare_parameter('conf_threshold', 0.30)

        self.show_image = bool(self.get_parameter('show_image').value)
        self.process_every_n = int(self.get_parameter('process_every_n').value)
        self.min_face_size = int(self.get_parameter('min_face_size').value)
        self.face_margin = float(self.get_parameter('face_margin').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)

        self.input_size = 224
        self.frame_count = 0
        self.last_result_time = 0.0
        self.window_name = 'RDK X5 Dual-Model Emotion Fusion'

        # Per-class weights: [happy, low_mood, negative_distress, neutral, surprise]
        # >1 = trust more, <1 = trust less
        self.old_weight = np.array([1.5, 0.1, 0.1, 1.5, 1.0], dtype=np.float32)
        self.new_weight = np.array([0.1, 1.5, 1.5, 0.2, 0.5], dtype=np.float32)

        self.bridge = CvBridge()
        self.labels = self._load_labels(
            self.get_parameter('label_path').value)
        self.old_model = self._load_model(
            self.get_parameter('old_model_path').value, 'Old')
        self.new_model = self._load_model(
            self.get_parameter('new_model_path').value, 'New')
        self.face_detector = self._load_face_detector()

        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.image_callback, 10)
        self.result_pub = self.create_publisher(
            String, self.get_parameter('result_topic').value, 10)
        self.vis_pub = self.create_publisher(
            Image, self.get_parameter('vis_topic').value, 10)

        if self.show_image:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 720)

        self.get_logger().info('Dual-model fusion node started')
        self.get_logger().info(f'Labels: {self.labels}')
        self.get_logger().info(f'Old weights: {self.old_weight.tolist()}')
        self.get_logger().info(f'New weights: {self.new_weight.tolist()}')

    def _load_labels(self, path):
        p = Path(path)
        labels = []
        with open(p, 'r', encoding='utf-8') as f:
            for line in f:
                name = line.strip()
                if name:
                    labels.append(name)
        return labels

    def _load_model(self, path, tag):
        p = Path(path)
        models = dnn.load(str(p))
        self.get_logger().info(f'{tag} model loaded: {path}')
        return models[0]

    def _load_face_detector(self):
        for path in ['/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
                     '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml']:
            if Path(path).exists():
                d = cv2.CascadeClassifier(path)
                if not d.empty():
                    return d
        raise FileNotFoundError('Face detector not found')

    def image_callback(self, msg):
        self.frame_count += 1
        if self.process_every_n > 1 and self.frame_count % self.process_every_n != 0:
            return

        t0 = time.time()
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            return
        if rgb is None or rgb.size == 0:
            return

        vis = rgb.copy()
        face_box = self._detect_largest_face(rgb)

        if face_box is None:
            now = time.time()
            if now - self.last_result_time > 1.0:
                self._publish({'status': 'no_face', 'emotion': 'unknown',
                    'confidence': 0.0, 'old': None, 'new': None})
                self.last_result_time = now
            cv2.putText(vis, 'No face detected', (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            self._show(vis, msg.header)
            return

        x1, y1, x2, y2 = face_box
        face = rgb[y1:y2, x1:x2]
        if face.size == 0:
            return

        face_resized = cv2.resize(face, (self.input_size, self.input_size),
            interpolation=cv2.INTER_AREA)
        nv12 = self._rgb_to_nv12(face_resized)

        # Run both models
        try:
            old_out = self.old_model.forward(nv12)
            old_probs = self._softmax(
                np.array(old_out[0].buffer, dtype=np.float32).reshape(-1))
        except Exception:
            old_probs = None

        try:
            new_out = self.new_model.forward(nv12)
            new_probs = self._softmax(
                np.array(new_out[0].buffer, dtype=np.float32).reshape(-1))
        except Exception:
            new_probs = None

        if old_probs is None and new_probs is None:
            return

        # Weighted fusion
        if old_probs is not None and new_probs is not None:
            fused = (old_probs * self.old_weight + new_probs * self.new_weight) / \
                    (self.old_weight + self.new_weight)
        elif old_probs is not None:
            fused = old_probs
        else:
            fused = new_probs

        cls_id = int(np.argmax(fused))
        confidence = float(fused[cls_id])
        emotion = self.labels[cls_id]
        infer_ms = (time.time() - t0) * 1000.0
        status = 'ok' if confidence >= self.conf_threshold else 'low_confidence'

        old_info = None
        new_info = None
        if old_probs is not None:
            oid = int(np.argmax(old_probs))
            old_info = {'emotion': self.labels[oid], 'confidence': round(float(old_probs[oid]), 3)}
        if new_probs is not None:
            nid = int(np.argmax(new_probs))
            new_info = {'emotion': self.labels[nid], 'confidence': round(float(new_probs[nid]), 3)}

        self._publish({
            'status': status, 'emotion': emotion,
            'confidence': round(confidence, 4),
            'old': old_info, 'new': new_info,
            'infer_ms': round(infer_ms, 2),
        })

        self._draw_fused(vis, face_box, emotion, confidence, old_info, new_info, infer_ms)
        self._show(vis, msg.header)

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

    def _publish(self, result):
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(msg)

    def _show(self, rgb, header):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            img_msg.header = header
            self.vis_pub.publish(img_msg)
        except Exception:
            pass
        if self.show_image:
            try:
                cv2.imshow(self.window_name, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)
            except Exception:
                pass

    def _draw_fused(self, rgb, box, emotion, conf, old_info, new_info, infer_ms):
        x1, y1, x2, y2 = box
        colors = {'happy': (0, 255, 0), 'low_mood': (255, 128, 0),
            'negative_distress': (255, 0, 0), 'neutral': (0, 180, 255),
            'surprise': (255, 255, 0)}
        color = colors.get(emotion, (255, 255, 255))
        cv2.rectangle(rgb, (x1, y1), (x2, y2), color, 2)

        # Fused result
        cv2.putText(rgb, f'FUSED: {emotion} {conf:.2f}', (x1, max(25, y1 - 50)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        # Old model
        if old_info:
            oe, oc = old_info["emotion"], old_info["confidence"]
            ot = f'OLD: {oe} {oc:.2f}'
        else:
            ot = 'OLD: N/A'
        cv2.putText(rgb, ot, (x1, max(50, y1 - 25)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        # New model
        if new_info:
            ne, nc = new_info["emotion"], new_info["confidence"]
            nt = f'NEW: {ne} {nc:.2f}'
        else:
            nt = 'NEW: N/A'
        cv2.putText(rgb, nt, (x1, max(70, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        cv2.putText(rgb, f'{infer_ms:.0f}ms', (x1, max(90, y1 + 15)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


def main():
    rclpy.init()
    node = EmotionFusionNode()
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
