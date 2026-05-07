# -*- coding: utf-8 -*-
from airbot_py.arm import AIRBOTPlay, SpeedProfile
import math
import time


def deg_list_to_rad_list(deg_list):
    return [math.radians(x) for x in deg_list]


def initialize_arm():
    print("Initializing the robot arm...")

    with AIRBOTPlay(url="localhost", port=50001) as robot:
        robot.set_speed_profile(SpeedProfile.DEFAULT)

        initial_joint_deg = [0, -45, 120, -90, 90, 90]

        initial_joint_pos = deg_list_to_rad_list(initial_joint_deg)

        print("Target initial joint degree:", initial_joint_deg)
        print("Target initial joint rad:", initial_joint_pos)

        robot.move_to_joint_pos(initial_joint_pos)
        time.sleep(1.0)

        print("Robot arm has moved to the initial position.")


def main():
    initialize_arm()


if __name__ == '__main__':
    main()