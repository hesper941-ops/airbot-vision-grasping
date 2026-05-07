#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import time
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


def quat_xyzw_to_rot(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q = q / np.linalg.norm(q)
    x, y, z, w = q

    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ], dtype=np.float64)
    return R


def make_transform(R, t_xyz):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.array(t_xyz, dtype=np.float64).reshape(3)
    return T


class HandEyeValidator(Node):
    def __init__(self):
        super().__init__('handeye_validator')

        # ====== Topics ======
        self.image_topic = '/camera/color/image_raw'
        self.camera_info_topic = '/camera/color/camera_info'
        self.pose_topic = '/robot_arm/end_pose'

        # ====== Chessboard params ======
        self.board_cols = 11
        self.board_rows = 8
        self.square_size = 0.03  # m

        # ====== Hand-eye result file ======
        self.handeye_json = '/home/sunrise/robot/hand_to_eye/data/handeye_output/handeye_park.json'

        # ====== State ======
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_image = None
        self.latest_image_msg = None
        self.latest_camera_info = None
        self.latest_pose_msg = None
        self.pattern_found = False
        self.latest_corners = None
        self.last_status_print_time = 0.0
        self.status_print_period = 1.5

        self.samples_base_xyz = []
        self.command = None
        self.quit_requested = False

        # ====== Load hand-eye ======
        self.T_gc = self.load_handeye(self.handeye_json)  # ^gT_c

        # ====== Subs ======
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, 10
        )

        self.timer = self.create_timer(0.1, self.process_once)

        self.input_thread = threading.Thread(target=self.console_input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info('=== Hand-eye validation started ===')
        self.get_logger().info(f'Using handeye file: {self.handeye_json}')
        self.get_logger().info('Chessboard can be at a NEW position.')
        self.get_logger().info('But during one validation round, it must remain fixed.')
        self.get_logger().info('Commands:')
        self.get_logger().info('  s + Enter -> save current validation sample')
        self.get_logger().info('  p + Enter -> print current status')
        self.get_logger().info('  q + Enter -> finish and print statistics')

    def load_handeye(self, json_path):
        if not os.path.exists(json_path):
            raise FileNotFoundError(f'Hand-eye json not found: {json_path}')

        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        t = data['t_cam2gripper']   # camera -> gripper
        q = data['q_cam2gripper_xyzw']

        R_gc = quat_xyzw_to_rot(q[0], q[1], q[2], q[3])
        T_gc = make_transform(R_gc, t)

        self.get_logger().info(
            f'Loaded PARK result: t_cam2gripper={t}, q_cam2gripper={q}'
        )
        return T_gc

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

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_image = cv_img
                self.latest_image_msg = msg
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def camera_info_callback(self, msg):
        with self.lock:
            self.latest_camera_info = msg

    def pose_callback(self, msg):
        with self.lock:
            self.latest_pose_msg = msg

    def detect_chessboard(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern_size = (self.board_cols, self.board_rows)

        found = False
        corners = None

        if hasattr(cv2, 'findChessboardCornersSB'):
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
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        return found, corners

    def make_object_points(self):
        objp = []
        for i in range(self.board_rows):
            for j in range(self.board_cols):
                objp.append([j * self.square_size, i * self.square_size, 0.0])
        return np.array(objp, dtype=np.float64)

    def current_board_center_in_base(self):
        with self.lock:
            image = None if self.latest_image is None else self.latest_image.copy()
            info = self.latest_camera_info
            pose_msg = self.latest_pose_msg

        if image is None or info is None or pose_msg is None:
            return False, None, 'Missing image/camera_info/pose.'

        found, corners = self.detect_chessboard(image)
        self.pattern_found = found
        self.latest_corners = corners

        if not found or corners is None:
            return False, None, 'Chessboard not found.'

        K = np.array([
            [info.k[0], 0.0, info.k[2]],
            [0.0, info.k[4], info.k[5]],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        D = np.array(list(info.d), dtype=np.float64).reshape(-1, 1)

        objp = self.make_object_points()

        ok, rvec, tvec = cv2.solvePnP(
            objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            return False, None, 'solvePnP failed.'

        R_ct, _ = cv2.Rodrigues(rvec)   # ^cR_t
        T_ct = make_transform(R_ct, tvec.reshape(3))  # ^cT_t

        # 当前末端位姿：^bT_g
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        R_bg = quat_xyzw_to_rot(q.x, q.y, q.z, q.w)
        T_bg = make_transform(R_bg, [p.x, p.y, p.z])

        # 取棋盘中心点，而不是原点
        center_t = np.array([
            (self.board_cols - 1) * self.square_size / 2.0,
            (self.board_rows - 1) * self.square_size / 2.0,
            0.0,
            1.0
        ], dtype=np.float64)

        # ^bP = ^bT_g * ^gT_c * ^cT_t * ^tP
        center_b = T_bg @ self.T_gc @ T_ct @ center_t
        xyz = center_b[:3].copy()

        return True, xyz, 'OK'

    def print_status(self):
        with self.lock:
            has_img = self.latest_image is not None
            has_info = self.latest_camera_info is not None
            has_pose = self.latest_pose_msg is not None

        self.get_logger().info('----------- Validation Status -----------')
        self.get_logger().info(f'Image received      : {has_img}')
        self.get_logger().info(f'CameraInfo received : {has_info}')
        self.get_logger().info(f'Pose received       : {has_pose}')
        self.get_logger().info(f'Current saved count : {len(self.samples_base_xyz)}')

        ok, xyz, msg = self.current_board_center_in_base()
        if ok:
            self.get_logger().info(
                f'Current board center in base_link = '
                f'({xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}) m'
            )
        else:
            self.get_logger().info(f'Current solve status: {msg}')

        self.get_logger().info('-----------------------------------------')

    def save_current_sample(self):
        ok, xyz, msg = self.current_board_center_in_base()
        if not ok:
            self.get_logger().warning(f'Cannot save: {msg}')
            return

        self.samples_base_xyz.append(xyz)
        idx = len(self.samples_base_xyz)

        self.get_logger().info(
            f'Saved validation sample {idx}: '
            f'({xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}) m'
        )

    def print_final_statistics(self):
        n = len(self.samples_base_xyz)
        if n == 0:
            self.get_logger().warning('No validation samples saved.')
            return

        pts = np.array(self.samples_base_xyz, dtype=np.float64)
        mean_xyz = pts.mean(axis=0)
        std_xyz = pts.std(axis=0)
        err = pts - mean_xyz
        err_norm = np.linalg.norm(err, axis=1)
        max_err = float(np.max(err_norm))
        mean_err = float(np.mean(err_norm))
        std_norm = float(np.linalg.norm(std_xyz))

        self.get_logger().info('============== FINAL VALIDATION ==============')
        self.get_logger().info(f'Sample count: {n}')
        for i, p in enumerate(pts, start=1):
            self.get_logger().info(
                f'#{i}: ({p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f}) m'
            )

        self.get_logger().info(
            f'MEAN  = ({mean_xyz[0]:.6f}, {mean_xyz[1]:.6f}, {mean_xyz[2]:.6f}) m'
        )
        self.get_logger().info(
            f'STD   = ({std_xyz[0]:.6f}, {std_xyz[1]:.6f}, {std_xyz[2]:.6f}) m'
        )
        self.get_logger().info(f'STD_NORM = {std_norm:.6f} m')
        self.get_logger().info(f'MEAN_ERR = {mean_err:.6f} m')
        self.get_logger().info(f'MAX_ERR  = {max_err:.6f} m')

        if std_norm < 0.005:
            self.get_logger().info('Result: VERY GOOD (< 5 mm)')
        elif std_norm < 0.01:
            self.get_logger().info('Result: GOOD / USABLE (5~10 mm)')
        elif std_norm < 0.02:
            self.get_logger().info('Result: MARGINAL (1~2 cm)')
        else:
            self.get_logger().info('Result: POOR (> 2 cm), please re-check.')

        self.get_logger().info('==============================================')

    def process_once(self):
        if self.quit_requested:
            rclpy.shutdown()
            return

        with self.lock:
            cmd = self.command
            self.command = None

        if cmd:
            cmd = cmd.strip().lower()
            key = cmd[0] if len(cmd) > 0 else ''

            if key == 's':
                self.save_current_sample()
            elif key == 'p':
                self.print_status()
            elif key == 'q':
                self.print_final_statistics()
                self.quit_requested = True
                return
            else:
                self.get_logger().info('Unknown command. Use: s / p / q')

        now = time.time()
        if now - self.last_status_print_time > self.status_print_period:
            ok, xyz, msg = self.current_board_center_in_base()
            if ok:
                self.get_logger().info(
                    f'[FOUND] Current board center in base_link = '
                    f'({xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}) m | '
                    f'input "s" to save'
                )
            else:
                self.get_logger().info(f'[WAIT] {msg}')
            self.last_status_print_time = now


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeValidator()
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