# -*- coding: utf-8 -*-
import time
from airbot_py.arm import AIRBOTPlay, RobotMode, SpeedProfile


class AirbotWrapper:
    def __init__(self, url="localhost", port=50001):
        self.url = url
        self.port = port
        self.robot = None

    def connect(self):
        if self.robot is None:
            self.robot = AIRBOTPlay(url=self.url, port=self.port)
            self.robot.connect()
            self.robot.set_speed_profile(SpeedProfile.DEFAULT)

    def disconnect(self):
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None

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