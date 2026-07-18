#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from robot_arm_interface.airbot_wrapper import AirbotWrapper


def deg2rad(x):
    return x * math.pi / 180.0


def main():
    print("Initializing the robot arm...")

    initial_joint_deg = [0, -45, 120, -90, 90, 90]
    initial_joint_pos = [deg2rad(v) for v in initial_joint_deg]

    print(f"Target initial joint degree: {initial_joint_deg}")
    print(f"Target initial joint rad: {initial_joint_pos}")

    robot = AirbotWrapper(url="localhost", port=50001)
    try:
        robot.connect(speed_profile='default')
        robot.move_joints(initial_joint_pos)
        time.sleep(1.0)
        print("Robot arm has moved to the initial position.")
    finally:
        robot.disconnect()


if __name__ == '__main__':
    main()
