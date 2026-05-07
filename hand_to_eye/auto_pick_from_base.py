#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import tty
import termios
import select
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped

from airbot_py.arm import AIRBOTPlay, RobotMode, SpeedProfile


class KeyboardQuitListener(threading.Thread):
    def __init__(self, node):
        super().__init__(daemon=True)
        self.node = node

    def run(self):
        if not sys.stdin.isatty():
            self.node.get_logger().warning('stdin is not a TTY, Q-key stop is unavailable in this terminal.')
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self.node.program_exit:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not rlist:
                    continue

                ch = sys.stdin.read(1)
                if ch.lower() == 'q':
                    self.node.on_keyboard_quit()
                    break
        except Exception as e:
            self.node.get_logger().error(f'Keyboard listener failed: {e}')
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass


class AutoPickFromBase(Node):
    def __init__(self):
        super().__init__('auto_pick_from_base')

        self.robot_url = 'localhost'
        self.robot_port = 50001

        # ---------------------------
        # 抓取参数
        # ---------------------------
        self.gripper_open = 1.0
        self.apple_gripper_close = 0.0
        self.duck_gripper_close = 0.02
        self.box_gripper_close = 0.03

        self.pregrasp_z_offset = 0.08
        self.apple_grasp_z_offset = 0.00
        self.duck_grasp_z_offset = -0.03
        self.box_grasp_z_offset = -0.02
        self.lift_z_offset = 0.10

        # 抓取位置补偿：前后方向 / 左右方向微调
        self.apple_grasp_x_offset = 0.00
        self.duck_grasp_x_offset = 0.00
        self.box_grasp_x_offset = 0.00

        self.apple_grasp_y_offset = 0.00
        self.duck_grasp_y_offset = 0.00
        self.box_grasp_y_offset = 0.00

        # ---------------------------
        # 目标滤波参数
        # ---------------------------
        self.target_window_size = 8
        self.max_target_std = 0.01
        self.target_buffer = deque(maxlen=self.target_window_size)

        # ---------------------------
        # 工作空间
        # ---------------------------
        self.x_min = 0.10
        self.x_max = 1.00
        self.y_min = -0.45
        self.y_max = 0.50
        self.z_min = 0.02
        self.z_max = 0.70

        # 允许轻微越界的软裁剪余量（单位：米）
        self.soft_clip_margin = 0.06

        # ---------------------------
        # 运行状态
        # ---------------------------
        self.latest_pose_msg = None
        self.busy = False
        self.done_once = False
        self.current_target_name = None

        self.stop_requested = False
        self.shutdown_requested = False
        self.program_exit = False
        self.last_wait_log_time = 0.0

        # ---------------------------
        # 动作等待
        # ---------------------------
        self.motion_sleep = 0.35
        self.gripper_sleep = 0.8

        # ---------------------------
        # 初始状态（程序启动时记录）
        # ---------------------------
        self.home_joint = None
        self.home_pos = None
        self.home_quat = None

        # ---------------------------
        # 可选中间安全关节位，默认关闭
        # ---------------------------
        self.use_safe_joint_pose = False
        self.safe_joint_pose = [0.0, -0.60, 1.10, 0.0, -0.50, 0.0]

        # ---------------------------
        # Joint 6：所有物体都顺时针旋转 90°
        # 之前设定：顺时针 = -90°
        # ---------------------------
        self.joint6_clockwise_delta_deg = -90.0
        self.j6_min_deg = -170.0
        self.j6_max_deg = 170.0
        self.joint_limit_margin_deg = 12.0

        # ---------------------------
        # 订阅
        # ---------------------------
        self.apple_sub = self.create_subscription(
            PointStamped,
            '/apple_position_base',
            lambda msg: self.object_callback(msg, 'apple'),
            10
        )

        self.duck_sub = self.create_subscription(
            PointStamped,
            '/duck_position_base',
            lambda msg: self.object_callback(msg, 'duck'),
            10
        )

        self.box_sub = self.create_subscription(
            PointStamped,
            '/box_position_base',
            lambda msg: self.object_callback(msg, 'box'),
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_arm/end_pose',
            self.pose_callback,
            10
        )

        # 定时检查是否按 Q 请求停止并收尾
        self.stop_timer = self.create_timer(0.1, self.stop_watchdog_callback)

        self.get_logger().info('AutoPickFromBase started.')
        self.get_logger().info(
            'Subscribing: /apple_position_base, /duck_position_base, /box_position_base, /robot_arm/end_pose'
        )
        self.get_logger().info('Press Q in this terminal to request stop, return home, and exit.')

        self.try_capture_home_state_once()

        self.kbd_listener = KeyboardQuitListener(self)
        self.kbd_listener.start()

    # =========================================================
    # 回调
    # =========================================================
    def pose_callback(self, msg: PoseStamped):
        self.latest_pose_msg = msg

    def on_keyboard_quit(self):
        if self.stop_requested:
            return
        self.stop_requested = True
        self.get_logger().warning('Q pressed: stop requested. Robot will return home after current blocking step finishes.')

    def stop_watchdog_callback(self):
        if self.shutdown_requested and rclpy.ok():
            self.program_exit = True
            self.get_logger().warning('Shutting down ROS node.')
            rclpy.shutdown()
            return

        if not self.stop_requested:
            return

        # 忙时不打断阻塞动作线程，只提示并等待线程归还控制权
        if self.busy:
            now = time.time()
            if now - self.last_wait_log_time > 1.0:
                self.get_logger().warning('Stop requested. Waiting for current motion step to return...')
                self.last_wait_log_time = now
            return

        # 空闲时则直接回 home 并退出
        self.get_logger().warning('Stop requested while idle. Returning home and exiting.')
        self.try_return_home_with_new_connection()
        self.shutdown_requested = True

    # =========================================================
    # AIRBOT 状态/动作包装
    # =========================================================
    def set_speed(self, robot, speed_name='slow'):
        if speed_name == 'slow':
            robot.set_speed_profile(SpeedProfile.SLOW)
        elif speed_name == 'fast':
            robot.set_speed_profile(SpeedProfile.FAST)
        else:
            robot.set_speed_profile(SpeedProfile.DEFAULT)

    def get_joint_pos_checked(self, robot):
        joint = robot.get_joint_pos()
        if joint is None or len(joint) < 6:
            raise RuntimeError('get_joint_pos() failed')
        return list(joint)

    def get_end_pose_checked(self, robot):
        pose = robot.get_end_pose()
        if pose is None or len(pose) < 2:
            raise RuntimeError('get_end_pose() failed')
        pos = list(pose[0])
        quat = list(pose[1])
        return pos, quat

    def move_eef_pos_safe(self, robot, pos_value):
        robot.switch_mode(RobotMode.PLANNING_POS)
        robot.move_eef_pos([pos_value])
        time.sleep(self.gripper_sleep)

    def move_joint_waypoints_safe(self, robot, waypoints, speed_name='slow'):
        if len(waypoints) < 2:
            return
        self.set_speed(robot, speed_name)
        robot.switch_mode(RobotMode.PLANNING_WAYPOINTS)
        robot.move_with_joint_waypoints(waypoints)
        time.sleep(self.motion_sleep)

    def move_cart_waypoints_safe(self, robot, waypoints, speed_name='slow'):
        if len(waypoints) < 1:
            return
        self.set_speed(robot, speed_name)
        robot.switch_mode(RobotMode.PLANNING_WAYPOINTS)
        robot.move_with_cart_waypoints(waypoints)
        time.sleep(self.motion_sleep)

    def ensure_not_stopped(self, robot):
        if self.stop_requested:
            self.get_logger().warning('Stop flag detected. Returning home now.')
            self.safe_return_home(robot)
            self.shutdown_requested = True
            raise RuntimeError('Stopped by user (Q).')

    # =========================================================
    # 初始状态
    # =========================================================
    def try_capture_home_state_once(self):
        try:
            with AIRBOTPlay(url=self.robot_url, port=self.robot_port) as robot:
                self.set_speed(robot, 'default')
                self.home_joint = self.get_joint_pos_checked(robot)
                self.home_pos, self.home_quat = self.get_end_pose_checked(robot)
                self.get_logger().info(
                    f'Captured home state: '
                    f'joint={[round(v, 3) for v in self.home_joint]}, '
                    f'pose=({self.home_pos[0]:.3f}, {self.home_pos[1]:.3f}, {self.home_pos[2]:.3f})'
                )
        except Exception as e:
            self.get_logger().warning(f'Failed to capture home state at startup: {e}')

    def try_return_home_with_new_connection(self):
        if self.home_joint is None:
            self.get_logger().warning('Home joint is unknown, cannot return home.')
            return

        try:
            with AIRBOTPlay(url=self.robot_url, port=self.robot_port) as robot:
                self.safe_return_home(robot)
        except Exception as e:
            self.get_logger().error(f'Failed to return home with new connection: {e}')

    def safe_return_home(self, robot):
        if self.home_joint is None:
            self.get_logger().warning('Home joint is unknown, skip returning home.')
            return

        try:
            current_joint = self.get_joint_pos_checked(robot)
        except Exception as e:
            self.get_logger().warning(f'Cannot read current joint before homing: {e}')
            current_joint = None

        try:
            self.set_speed(robot, 'slow')

            if current_joint is not None:
                if self.use_safe_joint_pose:
                    self.move_joint_waypoints_safe(
                        robot,
                        [current_joint, self.safe_joint_pose, self.home_joint],
                        'slow'
                    )
                else:
                    self.move_joint_waypoints_safe(
                        robot,
                        [current_joint, self.home_joint],
                        'slow'
                    )
            else:
                robot.switch_mode(RobotMode.PLANNING_POS)
                robot.move_to_joint_pos(self.home_joint)
                time.sleep(self.motion_sleep)

            self.get_logger().info('Returned to home joint pose.')
        except Exception as e:
            self.get_logger().error(f'Return-home failed: {e}')

    # =========================================================
    # 处理目标
    # =========================================================
    def in_workspace(self, p: np.ndarray) -> bool:
        return (
            self.x_min <= p[0] <= self.x_max and
            self.y_min <= p[1] <= self.y_max and
            self.z_min <= p[2] <= self.z_max
        )

    def soft_clip_target(self, p: np.ndarray):
        """
        轻微越界时，裁剪回工作空间边界。
        如果越界过大，则仍判为无效目标。
        返回：
            clipped_target, accepted, was_clipped
        """
        lower = np.array([self.x_min, self.y_min, self.z_min], dtype=np.float64)
        upper = np.array([self.x_max, self.y_max, self.z_max], dtype=np.float64)

        too_low = p < (lower - self.soft_clip_margin)
        too_high = p > (upper + self.soft_clip_margin)

        if np.any(too_low) or np.any(too_high):
            return p.copy(), False, False

        clipped = np.clip(p, lower, upper)
        was_clipped = not np.allclose(clipped, p, atol=1e-9)
        return clipped, True, was_clipped

    def deg_to_rad(self, deg: float) -> float:
        return deg * np.pi / 180.0

    def rad_to_deg(self, rad: float) -> float:
        return rad * 180.0 / np.pi

    def get_grasp_z_offset(self, object_name: str) -> float:
        if object_name == 'apple':
            return self.apple_grasp_z_offset
        if object_name == 'duck':
            return self.duck_grasp_z_offset
        if object_name == 'box':
            return self.box_grasp_z_offset
        return 0.0

    def get_gripper_close(self, object_name: str) -> float:
        if object_name == 'apple':
            return self.apple_gripper_close
        if object_name == 'duck':
            return self.duck_gripper_close
        if object_name == 'box':
            return self.box_gripper_close
        return 0.0

    def get_grasp_x_offset(self, object_name: str) -> float:
        if object_name == 'apple':
            return self.apple_grasp_x_offset
        if object_name == 'duck':
            return self.duck_grasp_x_offset
        if object_name == 'box':
            return self.box_grasp_x_offset
        return 0.0

    def get_grasp_y_offset(self, object_name: str) -> float:
        if object_name == 'apple':
            return self.apple_grasp_y_offset
        if object_name == 'duck':
            return self.duck_grasp_y_offset
        if object_name == 'box':
            return self.box_grasp_y_offset
        return 0.0

    def build_grasp_points(self, target_base: np.ndarray, object_name: str):
        grasp_x_offset = self.get_grasp_x_offset(object_name)
        grasp_y_offset = self.get_grasp_y_offset(object_name)
        grasp_z_offset = self.get_grasp_z_offset(object_name)

        pregrasp = target_base.copy()
        pregrasp[0] += grasp_x_offset
        pregrasp[1] += grasp_y_offset
        pregrasp[2] += self.pregrasp_z_offset

        grasp = target_base.copy()
        grasp[0] += grasp_x_offset
        grasp[1] += grasp_y_offset
        grasp[2] += grasp_z_offset

        mid = grasp.copy()
        mid[2] = (pregrasp[2] + grasp[2]) * 0.5

        lift = target_base.copy()
        lift[0] += grasp_x_offset
        lift[1] += grasp_y_offset
        lift[2] += self.lift_z_offset

        return pregrasp, mid, grasp, lift

    def choose_joint6_clockwise_target(self, current_joint):
        current_j6_deg = self.rad_to_deg(current_joint[5])
        target_deg = current_j6_deg + self.joint6_clockwise_delta_deg

        # 优先保留安全 margin
        if self.j6_min_deg + self.joint_limit_margin_deg <= target_deg <= self.j6_max_deg - self.joint_limit_margin_deg:
            target_joint = list(current_joint)
            target_joint[5] = self.deg_to_rad(target_deg)
            return target_joint, target_deg, 'within_margin'

        # 退一步：只要没到硬限位也允许
        if self.j6_min_deg <= target_deg <= self.j6_max_deg:
            target_joint = list(current_joint)
            target_joint[5] = self.deg_to_rad(target_deg)
            return target_joint, target_deg, 'within_hard_limit'

        raise RuntimeError(
            f'Clockwise -90 deg on joint6 exceeds limits. '
            f'current={current_j6_deg:.3f} deg, target={target_deg:.3f} deg, '
            f'limit=[{self.j6_min_deg:.1f}, {self.j6_max_deg:.1f}] deg'
        )

    # =========================================================
    # 目标回调
    # =========================================================
    def object_callback(self, msg: PointStamped, object_name: str):
        if self.done_once or self.busy:
            return

        if self.latest_pose_msg is None:
            self.get_logger().warning(f'No /robot_arm/end_pose yet, cannot pick {object_name}.')
            return

        if self.stop_requested:
            return

        raw_target = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)

        # 先做轻微越界软裁剪
        target, accepted, was_clipped = self.soft_clip_target(raw_target)

        if not accepted:
            self.get_logger().warning(
                f'{object_name} target out of workspace: '
                f'({raw_target[0]:.3f}, {raw_target[1]:.3f}, {raw_target[2]:.3f})'
            )
            return

        if was_clipped:
            self.get_logger().warning(
                f'{object_name} target softly clipped: '
                f'raw=({raw_target[0]:.3f}, {raw_target[1]:.3f}, {raw_target[2]:.3f}) -> '
                f'clipped=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})'
            )

        if self.current_target_name != object_name:
            self.current_target_name = object_name
            self.target_buffer.clear()
            self.get_logger().info(f'Switch target to {object_name}, reset target buffer.')

        self.target_buffer.append(target)

        self.get_logger().info(
            f'Received {object_name}_position_base: '
            f'({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}), '
            f'buffer={len(self.target_buffer)}/{self.target_window_size}'
        )

        if len(self.target_buffer) < self.target_window_size:
            return

        pts = np.array(self.target_buffer, dtype=np.float64)
        mean_pt = pts.mean(axis=0)
        std_pt = pts.std(axis=0)
        std_norm = float(np.linalg.norm(std_pt))

        self.get_logger().info(
            f'{object_name} target filter: '
            f'mean=({mean_pt[0]:.3f}, {mean_pt[1]:.3f}, {mean_pt[2]:.3f}), '
            f'std_norm={std_norm:.4f}'
        )

        if std_norm > self.max_target_std:
            self.get_logger().info(
                f'{object_name} target not stable enough yet, wait for more frames.'
            )
            return

        self.busy = True
        grasp_thread = threading.Thread(
            target=self.execute_grasp_cycle,
            args=(mean_pt, object_name),
            daemon=True
        )
        grasp_thread.start()

    # =========================================================
    # 执行抓取流程
    # =========================================================
    def execute_grasp_cycle(self, target_base: np.ndarray, object_name: str):
        robot = None
        try:
            self.get_logger().info(
                f'Executing grasp cycle for {object_name}, '
                f'target_base=({target_base[0]:.3f}, {target_base[1]:.3f}, {target_base[2]:.3f})'
            )

            gripper_close = self.get_gripper_close(object_name)
            pregrasp, mid, grasp, lift = self.build_grasp_points(target_base, object_name)

            self.get_logger().info(
                f'{object_name} params: '
                f'pregrasp_z_offset={self.pregrasp_z_offset:.3f}, '
                f'grasp_z_offset={self.get_grasp_z_offset(object_name):.3f}, '
                f'lift_z_offset={self.lift_z_offset:.3f}, '
                f'gripper_close={gripper_close:.3f}, '
                f'grasp_x_offset={self.get_grasp_x_offset(object_name):.3f}, '
                f'grasp_y_offset={self.get_grasp_y_offset(object_name):.3f}'
            )

            self.get_logger().info(
                f'pregrasp=({pregrasp[0]:.3f}, {pregrasp[1]:.3f}, {pregrasp[2]:.3f})'
            )
            self.get_logger().info(
                f'mid=({mid[0]:.3f}, {mid[1]:.3f}, {mid[2]:.3f})'
            )
            self.get_logger().info(
                f'grasp=({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f})'
            )
            self.get_logger().info(
                f'lift=({lift[0]:.3f}, {lift[1]:.3f}, {lift[2]:.3f})'
            )

            with AIRBOTPlay(url=self.robot_url, port=self.robot_port) as robot:
                self.set_speed(robot, 'slow')

                start_joint = self.get_joint_pos_checked(robot)
                start_pos, start_quat = self.get_end_pose_checked(robot)

                # 如果启动时没记到 home，就在抓取开始时补记
                if self.home_joint is None:
                    self.home_joint = list(start_joint)
                    self.home_pos = list(start_pos)
                    self.home_quat = list(start_quat)
                    self.get_logger().info('Home state was missing, captured from current start pose.')

                # 预抓取姿态固定使用“初始姿态”
                approach_quat = list(self.home_quat) if self.home_quat is not None else list(start_quat)

                self.get_logger().info(
                    f'start_pose=({start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f})'
                )

                self.ensure_not_stopped(robot)

                # Step 1: 张开夹爪
                self.get_logger().info('Step 1: open gripper')
                self.move_eef_pos_safe(robot, self.gripper_open)
                self.ensure_not_stopped(robot)

                # Step 2: 如果启用安全关节位，先过渡
                if self.use_safe_joint_pose:
                    self.get_logger().info('Step 2: move to safe joint pose')
                    current_joint = self.get_joint_pos_checked(robot)
                    self.move_joint_waypoints_safe(
                        robot,
                        [current_joint, self.safe_joint_pose],
                        'slow'
                    )
                    self.ensure_not_stopped(robot)

                # Step 3: 走到预抓取点，姿态固定用初始姿态
                self.get_logger().info('Step 3: move to pregrasp with initial orientation')
                self.move_cart_waypoints_safe(
                    robot,
                    [
                        [pregrasp.tolist(), approach_quat],
                    ],
                    'slow'
                )
                self.ensure_not_stopped(robot)

                # Step 4: 手腕整体顺时针旋转 90°
                self.get_logger().info('Step 4: rotate joint6 clockwise by 90 degrees')
                current_joint = self.get_joint_pos_checked(robot)
                target_joint_after_rotate, target_deg, reason = self.choose_joint6_clockwise_target(current_joint)
                self.get_logger().info(
                    f'Joint6 rotate clockwise: current={self.rad_to_deg(current_joint[5]):.3f} deg, '
                    f'target={target_deg:.3f} deg, mode={reason}'
                )
                self.move_joint_waypoints_safe(
                    robot,
                    [current_joint, target_joint_after_rotate],
                    'slow'
                )
                self.ensure_not_stopped(robot)

                # 旋转后重新读取末端姿态四元数
                _, rotated_quat = self.get_end_pose_checked(robot)

                # Step 5: 直线下探
                self.get_logger().info('Step 5: descend by cart waypoints')
                self.move_cart_waypoints_safe(
                    robot,
                    [
                        [mid.tolist(), rotated_quat],
                        [grasp.tolist(), rotated_quat],
                    ],
                    'slow'
                )
                self.ensure_not_stopped(robot)

                # Step 6: 闭合夹爪
                self.get_logger().info('Step 6: close gripper')
                self.move_eef_pos_safe(robot, gripper_close)
                self.ensure_not_stopped(robot)

                # Step 7: 抬起
                self.get_logger().info('Step 7: lift by cart waypoints')
                self.move_cart_waypoints_safe(
                    robot,
                    [
                        [mid.tolist(), rotated_quat],
                        [lift.tolist(), rotated_quat],
                    ],
                    'slow'
                )
                self.ensure_not_stopped(robot)

                # Step 8: 回到 home joint pose
                self.get_logger().info('Step 8: return to initial home joint pose')
                current_joint = self.get_joint_pos_checked(robot)

                if self.use_safe_joint_pose:
                    self.move_joint_waypoints_safe(
                        robot,
                        [current_joint, self.safe_joint_pose, self.home_joint],
                        'slow'
                    )
                else:
                    self.move_joint_waypoints_safe(
                        robot,
                        [current_joint, self.home_joint],
                        'slow'
                    )

                self.set_speed(robot, 'default')

            self.get_logger().info(f'Grasp cycle finished for {object_name}.')
            self.done_once = True

        except Exception as e:
            self.get_logger().error(f'Grasp cycle failed for {object_name}: {e}')

            # 异常时也尽量回初始位
            if robot is not None:
                try:
                    self.safe_return_home(robot)
                except Exception as e2:
                    self.get_logger().error(f'Failed to return home after error: {e2}')

        finally:
            self.busy = False
            if self.stop_requested:
                self.shutdown_requested = True

    def destroy_node(self):
        self.program_exit = True
        self.get_logger().info('AutoPickFromBase node shutdown.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoPickFromBase()
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