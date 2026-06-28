#!/usr/bin/env python3
import csv
import math
import time
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class EmotionAutoCollector(Node):
    def __init__(self):
        super().__init__("emotion_auto_collector")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("output_dir", "/home/sunrise/robot/Orbbec_ws/src/emotion/collected_dataset")
        self.declare_parameter("label", "")
        self.declare_parameter("show_image", True)
        self.declare_parameter("start_delay_sec", 3.0)
        self.declare_parameter("interval_sec", 0.5)
        self.declare_parameter("max_count", 60)
        self.declare_parameter("require_face", True)
        self.declare_parameter("min_face_size", 50)
        self.declare_parameter("face_margin", 0.30)

        self.image_topic = self.get_parameter("image_topic").value
        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.label = str(self.get_parameter("label").value)
        self.show_image = bool(self.get_parameter("show_image").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.interval_sec = float(self.get_parameter("interval_sec").value)
        self.max_count = int(self.get_parameter("max_count").value)
        self.require_face = bool(self.get_parameter("require_face").value)
        self.min_face_size = int(self.get_parameter("min_face_size").value)
        self.face_margin = float(self.get_parameter("face_margin").value)

        self.labels = ["happy", "low_mood", "negative_distress", "neutral", "surprise"]

        if self.label not in self.labels:
            raise ValueError(f"Invalid label: {self.label}. Must be one of {self.labels}")

        self.bridge = CvBridge()
        self.face_detector = self.load_face_detector()

        self.raw_dir = self.output_dir / "raw" / self.label
        self.crop_dir = self.output_dir / "face_crop" / self.label
        self.meta_path = self.output_dir / "metadata.csv"

        self.raw_dir.mkdir(parents=True, exist_ok=True)
        self.crop_dir.mkdir(parents=True, exist_ok=True)
        self.init_metadata()

        self.start_time = time.time()
        self.last_save_time = 0.0
        self.saved_count = 0
        self.stop_requested = False
        self.window_name = "Emotion Dataset Collector"

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        if self.show_image:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 720)

        self.get_logger().info("Emotion auto collector started.")
        self.get_logger().info(f"Label: {self.label}")
        self.get_logger().info(f"Start delay: {self.start_delay_sec} sec")
        self.get_logger().info(f"Interval: {self.interval_sec} sec")
        self.get_logger().info(f"Max count: {self.max_count}")
        self.get_logger().info(f"Output dir: {self.output_dir}")
        self.get_logger().info("Press q in image window or Ctrl+C in terminal to stop early.")

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

        raise FileNotFoundError("Cannot find haarcascade_frontalface_default.xml")

    def init_metadata(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)

        if not self.meta_path.exists():
            with open(self.meta_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "timestamp",
                    "label",
                    "raw_path",
                    "crop_path",
                    "face_found",
                    "x1",
                    "y1",
                    "x2",
                    "y2",
                    "width",
                    "height"
                ])

    def image_callback(self, msg):
        if self.stop_requested:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge convert failed: {e}")
            return

        if rgb is None or rgb.size == 0:
            return

        h, w = rgb.shape[:2]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        face_box = self.detect_largest_face(rgb)
        vis = bgr.copy()

        if face_box is not None:
            x1, y1, x2, y2 = face_box
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(
                vis,
                "Face detected",
                (x1, max(25, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )
        else:
            cv2.putText(
                vis,
                "No face detected",
                (20, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
                cv2.LINE_AA
            )

        now = time.time()
        elapsed = now - self.start_time

        if elapsed < self.start_delay_sec:
            remain = int(math.ceil(self.start_delay_sec - elapsed))
            self.draw_overlay(vis, f"Label: {self.label}", f"Start in {remain}", "Prepare your expression")
        else:
            if self.saved_count < self.max_count:
                if now - self.last_save_time >= self.interval_sec:
                    saved = self.save_sample(self.label, bgr, face_box, w, h)
                    if saved:
                        self.last_save_time = now

            self.draw_overlay(
                vis,
                f"Label: {self.label}",
                f"Collecting: {self.saved_count}/{self.max_count}",
                "Keep expression stable"
            )

        if self.saved_count >= self.max_count:
            self.draw_overlay(
                vis,
                f"Label: {self.label}",
                f"Finished: {self.saved_count}/{self.max_count}",
                "Auto closing..."
            )
            if self.show_image:
                cv2.imshow(self.window_name, vis)
                cv2.waitKey(800)

            self.get_logger().info(f"Finished collecting {self.label}: {self.saved_count} images.")
            self.stop_requested = True
            return

        if self.show_image:
            cv2.imshow(self.window_name, vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.get_logger().info("User pressed q. Stop collecting.")
                self.stop_requested = True

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

    def save_sample(self, label, bgr, face_box, width, height):
        if self.require_face and face_box is None:
            return False

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{label}_{timestamp}.jpg"

        raw_path = self.raw_dir / filename
        crop_path = self.crop_dir / filename

        ok_raw = cv2.imwrite(str(raw_path), bgr)

        face_found = face_box is not None
        x1 = y1 = x2 = y2 = -1
        crop_saved = False
        saved_crop_path = ""

        if face_found:
            x1, y1, x2, y2 = face_box
            crop = bgr[y1:y2, x1:x2]
            if crop.size > 0:
                crop_saved = cv2.imwrite(str(crop_path), crop)
                if crop_saved:
                    saved_crop_path = str(crop_path)

        if not ok_raw:
            self.get_logger().warn("Failed to save raw image.")
            return False

        if self.require_face and not crop_saved:
            return False

        with open(self.meta_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                label,
                str(raw_path),
                saved_crop_path,
                int(face_found),
                x1,
                y1,
                x2,
                y2,
                width,
                height
            ])

        self.saved_count += 1

        self.get_logger().info(
            f"Saved {label}: {self.saved_count}/{self.max_count}, raw={raw_path.name}, crop_saved={crop_saved}"
        )

        return True

    def draw_overlay(self, img, line1, line2, line3):
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (img.shape[1], 115), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, img, 0.55, 0, img)

        cv2.putText(
            img,
            line1,
            (20, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            img,
            line2,
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.95,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            img,
            line3,
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )


def main(args=None):
    rclpy.init(args=args)
    node = EmotionAutoCollector()

    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_image:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
