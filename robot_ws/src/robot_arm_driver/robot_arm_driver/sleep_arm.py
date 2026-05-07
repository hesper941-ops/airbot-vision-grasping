# -*- coding: utf-8 -*-
from airbot_py.arm import AIRBOTPlay, SpeedProfile
import time


def sleep_arm():
    print("Start moving arm to sleep pose...")

    with AIRBOTPlay(url="localhost", port=50001) as robot:
        robot.set_speed_profile(SpeedProfile.DEFAULT)

        sleep_joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print("Target sleep joint position:", sleep_joint_pos)

        robot.move_to_joint_pos(sleep_joint_pos)
        time.sleep(2.0)

        print("Robot arm has moved to the sleep pose.")


def main():
    sleep_arm()


if __name__ == '__main__':
    main()