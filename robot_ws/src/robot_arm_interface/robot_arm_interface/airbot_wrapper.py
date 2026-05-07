# -*- coding: utf-8 -*-
import time
from airbot_py.arm import AIRBOTPlay, RobotMode, SpeedProfile


class AirbotWrapper:
    def __init__(self, url="localhost", port=50001):
        self.url = url
        self.port = port
        self.robot = None

    def connect(self, speed_profile="default"):
        if self.robot is None:
            self.robot = AIRBOTPlay(url=self.url, port=self.port)
            self.robot.connect()
        self.set_speed_profile(speed_profile)

    def disconnect(self):
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None

    def set_speed_profile(self, speed_profile="default"):
        if self.robot is None:
            raise RuntimeError("Robot is not connected.")

        if speed_profile == "slow":
            self.robot.set_speed_profile(SpeedProfile.SLOW)
        elif speed_profile == "fast":
            self.robot.set_speed_profile(SpeedProfile.FAST)
        else:
            self.robot.set_speed_profile(SpeedProfile.DEFAULT)

    def get_state(self):
        return self.robot.get_state()

    def get_joint_pos(self):
        return self.robot.get_joint_pos()

    def get_joint_vel(self):
        return self.robot.get_joint_vel()

    def get_end_pose(self):
        return self.robot.get_end_pose()

    def move_joints(self, joint_target):
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.robot.move_to_joint_pos(joint_target)

    def move_cart_waypoints(self, waypoints):
        self.robot.switch_mode(RobotMode.PLANNING_WAYPOINTS)
        self.robot.move_with_cart_waypoints(waypoints)

    def move_to_cart_target_with_current_orientation(self, target_xyz):
        pose = self.get_end_pose()
        if pose is None or len(pose) < 2:
            raise RuntimeError("Failed to read current end pose.")
        current_quat = list(pose[1])
        self.move_cart_waypoints([
            [list(target_xyz), current_quat],
        ])

    def servo_joints(self, joint_target):
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        self.robot.servo_joint_pos(joint_target)

    def go_home(self):
        home_joint = [0.0, -0.785398, 0.785398, 0.0, 0.0, 0.0]
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.robot.move_to_joint_pos(home_joint)

    def open_gripper(self):
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        for _ in range(50):
            self.robot.servo_eef_pos([0.07])
            time.sleep(0.02)

    def close_gripper(self):
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        for _ in range(50):
            self.robot.servo_eef_pos([0.0])
            time.sleep(0.02)
