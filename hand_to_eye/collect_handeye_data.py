#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import time
import threading

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


class HandEyeCollectorHeadless(Node):
    def __init__(self):
        super().__init__('handeye_collector_headless')

        self.image_topic = '/camera/color/image_raw'
        self.camera_info_topic = '/camera/color/camera_info'
        self.pose_topic = '/robot_arm/end_pose'

        self.board_cols = 11
        self.board_rows = 8
        self.square_size = 0.03

        self.save_root = '/home/sunrise/robot/hand_to_eye/data'
        self.image_dir = os.path.join(self.save_root, 'images')
        self.preview_dir = os.path.join(self.save_root, 'preview')
        self.csv_path = os.path.join(self.save_root, 'poses.csv')

        self.process_period = 0.1
        self.status_print_period = 1.5
        self.min_save_interval_sec = 1.0

        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.preview_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_image_msg = None
        self.latest_image = None
        self.latest_camera_info = None
        self.latest_pose_msg = None

        self.pattern_found = False
        self.latest_corners = None
        self.last_detect_time = 0.0
        self.last_status_print_time = 0.0
        self.last_save_time = 0.0

        self.sample_index = self._get_next_index()

        self.command = None
        self.quit_requested = False

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.timer = self.create_timer(self.process_period, self.process_once)

        self._prepare_csv()

        self.input_thread = threading.Thread(target=self.console_input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info('=== Hand-eye headless collector started ===')
        self.get_logger().info(f'Image topic      : {self.image_topic}')
        self.get_logger().info(f'CameraInfo topic : {self.camera_info_topic}')
        self.get_logger().info(f'Pose topic       : {self.pose_topic}')
        self.get_logger().info(f'Pattern size     : ({self.board_cols}, {self.board_rows})')
        self.get_logger().info(f'Square size      : {self.square_size} m')
        self.get_logger().info(f'Save root        : {self.save_root}')
        self.get_logger().info('Commands:')
        self.get_logger().info('  s + Enter -> save current sample')
        self.get_logger().info('  p + Enter -> print current status')
        self.get_logger().info('  q + Enter -> quit')

    def _prepare_csv(self):
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'index',
                    'image_name',
                    'preview_name',
                    'image_stamp_sec',
                    'image_stamp_nanosec',
                    'pose_stamp_sec',
                    'pose_stamp_nanosec',
                    'pose_frame_id',
                    'px', 'py', 'pz',
                    'qx', 'qy', 'qz', 'qw',
                    'camera_frame_id',
                    'img_width', 'img_height',
                    'fx', 'fy', 'cx', 'cy',
                    'distortion_model',
                    'd0', 'd1', 'd2', 'd3', 'd4', 'd5', 'd6', 'd7'
                ])

    def _get_next_index(self):
        if not os.path.exists(self.csv_path):
            return 1

        max_idx = 0
        with open(self.csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    idx = int(row['index'])
                    if idx > max_idx:
                        max_idx = idx
                except Exception:
                    pass
        return max_idx + 1

    def console_input_loop(self):
        while not self.quit_requested:
            try:
                cmd = input().strip().lower()
                with self.lock:
                    self.command = cmd
            except EOFError:
                break
            except Exception:
                break

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_image_msg = msg
                self.latest_image = cv_img
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        with self.lock:
            self.latest_camera_info = msg

    def pose_callback(self, msg: PoseStamped):
        with self.lock:
            self.latest_pose_msg = msg

    def detect_chessboard(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern_size = (self.board_cols, self.board_rows)

        found = False
        corners = None

        try:
            found, corners = cv2.findChessboardCornersSB(gray, pattern_size)
        except Exception:
            found = False
            corners = None

        if not found:
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)

            if found:
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001
                )
                corners = cv2.cornerSubPix(
                    gray,
                    corners,
corners,
           (11, 11),
                    (-1, -1),
                    criteria
                )

        return found, corners

    def make_preview(self, image, found, corners):
        preview = image.copy()

        status_text = 'FOUND' if found else 'NOT_FOUND'
        color = (0, 255, 0) if found else (0, 0, 255)

        cv2.putText(preview, f'Chessboard: {status_text}', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        cv2.putText(preview, f'Pattern: ({self.board_cols}, {self.board_rows})', (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        cv2.putText(preview, f'Sample index: {self.sample_index}', (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        if found and corners is not None:
            cv2.drawChessboardCorners(
                preview,
                (self.board_cols, self.board_rows),
                corners,
                found
            )

        return preview

    def print_status(self):
        with self.lock:
            has_img = self.latest_image is not None
            has_info = self.latest_camera_info is not None
            has_pose = self.latest_pose_msg is not None
            found = self.pattern_found
            info = self.latest_camera_info
            pose_msg = self.latest_pose_msg

        self.get_logger().info('----------- Current Status -----------')
        self.get_logger().info(f'Image received      : {has_img}')
        self.get_logger().info(f'CameraInfo received : {has_info}')
        self.get_logger().info(f'Pose received       : {has_pose}')
        self.get_logger().info(f'Chessboard found    : {found}')
        self.get_logger().info(f'Next sample index   : {self.sample_index}')

        if info is not None:
            self.get_logger().info(
                f'Camera fx={info.k[0]:.6f}, fy={info.k[4]:.6f}, '
                f'cx={info.k[2]:.6f}, cy={info.k[5]:.6f}'
            )

        if pose_msg is not None:
            p = pose_msg.pose.position
            q = pose_msg.pose.orientation
            self.get_logger().info(
                f'Pose p=({p.x:.6f}, {p.y:.6f}, {p.z:.6f}), '
                f'q=({q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f})'
            )

        self.get_logger().info('--------------------------------------')

    def save_sample(self):
        with self.lock:
            image_msg = self.latest_image_msg
            image = None if self.latest_image is None else self.latest_image.copy()
            pose_msg = self.latest_pose_msg
            info = self.latest_camera_info
            found = self.pattern_found
            corners = None if self.latest_corners is None else self.latest_corners.copy()

        if image_msg is None or image is None:
            self.get_logger().warning('No image received yet, cannot save.')
            return

        if pose_msg is None:
            self.get_logger().warning('No /robot_arm/end_pose received yet, cannot save.')
            return

        if info is None:
            self.get_logger().warning('No /camera/color/camera_info received yet, cannot save.')
            return

        if not found or corners is None:
            self.get_logger().warning('Chessboard NOT found, sample not saved.')
            return

        now = time.time()
        if now - self.last_save_time < self.min_save_interval_sec:
            self.get_logger().warning('Saving too fast, please wait a moment.')
            return

        image_name = f'{self.sample_index:04d}.png'
        preview_name = f'{self.sample_index:04d}_preview.png'

        image_path = os.path.join(self.image_dir, image_name)
        preview_path = os.path.join(self.preview_dir, preview_name)

        preview = self.make_preview(image, found, corners)

        ok1 = cv2.imwrite(image_path, image)
        ok2 = cv2.imwrite(preview_path, preview)

        if not ok1:
            self.get_logger().error(f'Failed to save image: {image_path}')
            return
        if not ok2:
            self.get_logger().warning(f'Failed to save preview: {preview_path}')

        d = list(info.d)
        while len(d) < 8:
            d.append(0.0)

        p = pose_msg.pose.position
        q = pose_msg.pose.orientation

        with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.sample_index,
                image_name,
                preview_name,
                image_msg.header.stamp.sec,
                image_msg.header.stamp.nanosec,
                pose_msg.header.stamp.sec,
                pose_msg.header.stamp.nanosec,
                pose_msg.header.frame_id,
                p.x, p.y, p.z,
                q.x, q.y, q.z, q.w,
                info.header.frame_id,
                info.width, info.height,
                info.k[0], info.k[4], info.k[2], info.k[5],
                info.distortion_model,
                d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]
            ])

        self.get_logger().info(f'Saved sample {self.sample_index}')
        self.get_logger().info(f'Image   : {image_path}')
        self.get_logger().info(f'Preview : {preview_path}')

        self.sample_index += 1
        self.last_save_time = now

    def process_once(self):
        if self.quit_requested:
            rclpy.shutdown()
            return

        with self.lock:
            cmd = self.command
            self.command = None
            image = None if self.latest_image is None else self.latest_image.copy()
            has_info = self.latest_camera_info is not None
            has_pose = self.latest_pose_msg is not None

        if cmd:
            if cmd == 's':
                self.save_sample()
            elif cmd == 'p':
                self.print_status()
            elif cmd == 'q':
                self.get_logger().info('Quit requested by user.')
                self.quit_requested = True
                rclpy.shutdown()
                return
            else:
                self.get_logger().info('Unknown command. Use: s / p / q')

        if image is None:
            now = time.time()
            if now - self.last_status_print_time > self.status_print_period:
                self.get_logger().info('Waiting for /camera/color/image_raw ...')
                self.last_status_print_time = now
            return

        found, corners = self.detect_chessboard(image)

        with self.lock:
            self.pattern_found = found
            self.latest_corners = corners
            self.last_detect_time = time.time()

        now = time.time()
        if now - self.last_status_print_time > self.status_print_period:
            if found:
                self.get_logger().info(
                    f'[FOUND] Chessboard detected. Input "s" and press Enter to save sample {self.sample_index}.'
                )
            else:
                self.get_logger().info(
                    f'[NOT FOUND] Waiting for chessboard... (CameraInfo={has_info}, Pose={has_pose})'
                )
            self.last_status_print_time = now


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCollectorHeadless()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
