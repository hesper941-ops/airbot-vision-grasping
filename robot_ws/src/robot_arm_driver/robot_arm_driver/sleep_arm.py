#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from robot_arm_interface.airbot_wrapper import AirbotWrapper


def deg2rad(x):
    return x * math.pi / 180.0


def main():
    print("Moving robot arm to sleep position...")

    sleep_joint_deg = [0.0, -45.0, 105.0, -75.0, 75.0, 90.0]
    sleep_joint_pos = [deg2rad(v) for v in sleep_joint_deg]

    print(f"Target sleep joint degree: {sleep_joint_deg}")
    print(f"Target sleep joint rad: {sleep_joint_pos}")

    robot = AirbotWrapper(url="localhost", port=50001)
    try:
        robot.connect(speed_profile='slow')
        robot.move_joints(sleep_joint_pos)
        time.sleep(1.0)
        print("Robot arm has moved to the sleep position.")
    finally:
        robot.disconnect()


if __name__ == '__main__':
    main()
