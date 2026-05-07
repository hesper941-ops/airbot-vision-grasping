#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from airbot_py.arm import AIRBOTPlay, RobotMode, SpeedProfile


def deg2rad(x):
    return x * math.pi / 180.0


def rad2deg(x):
    return x * 180.0 / math.pi


def interpolate_joint_waypoints(start_joint, end_joint, max_step_deg=10.0):
    diff_deg = [abs(rad2deg(e - s)) for s, e in zip(start_joint, end_joint)]
    max_diff_deg = max(diff_deg)
    n_seg = max(1, int(math.ceil(max_diff_deg / max_step_deg)))

    waypoints = []
    for i in range(1, n_seg + 1):
        alpha = i / n_seg
        joint_i = [(1.0 - alpha) * s + alpha * e for s, e in zip(start_joint, end_joint)]
        waypoints.append(joint_i)
    return waypoints


def main():
    # 更低一点、更松一点的推荐初始位（单位：度）
    target_joint_deg = [0.0, -45.0, 105.0, -75.0, 75.0, 90.0]
    target_joint = [deg2rad(v) for v in target_joint_deg]

    print("Target joint pose (deg):", target_joint_deg)

    with AIRBOTPlay(url='localhost', port=50001) as robot:
        robot.set_speed_profile(SpeedProfile.SLOW)

        current_joint = robot.get_joint_pos()
        if current_joint is None or len(current_joint) < 6:
            raise RuntimeError("Failed to read current joint position.")

        print("Current joint pose (deg):", [round(rad2deg(v), 2) for v in current_joint])

        # 小步插值移动，减少超时风险
        waypoints = interpolate_joint_waypoints(current_joint, target_joint, max_step_deg=10.0)

        robot.switch_mode(RobotMode.PLANNING_WAYPOINTS)
        robot.move_with_joint_waypoints(waypoints)

        time.sleep(1.0)

        final_joint = robot.get_joint_pos()
        if final_joint is not None:
            print("Final joint pose (deg):", [round(rad2deg(v), 2) for v in final_joint])

        print("Moved to lower, more tolerant home pose.")


if __name__ == '__main__':
    main()
