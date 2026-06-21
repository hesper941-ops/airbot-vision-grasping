#!/usr/bin/env python3
"""Static runtime-surface check for grasp_task_open_loop.py."""

from pathlib import Path
import math
import sys


REQUIRED_DEFINITIONS = [
    "def _handle_pending_speed_profile",
    "def _publish_joint_target",
    "def _publish_cart_target",
    "def _publish_cart_waypoints",
    "def _publish_gripper_command",
    "def _publish_speed_profile",
    "def _publish_reset_executor",
    "def _set_speed_profile",
    "def _handle_cartesian_motion",
]


def _distance(a, b) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def _interpolate_segment(start, end, max_step: float):
    distance = _distance(start, end)
    if distance <= 1e-9:
        return []
    steps = max(1, int(math.ceil(distance / max_step)))
    return [
        [
            float(start[0]) + (float(end[0]) - float(start[0])) * (index / steps),
            float(start[1]) + (float(end[1]) - float(start[1])) * (index / steps),
            float(start[2]) + (float(end[2]) - float(start[2])) * (index / steps),
        ]
        for index in range(1, steps + 1)
    ]


def _build_test_waypoints(current, pre_grasp, grasp, max_step: float):
    waypoints = []
    waypoints.extend(_interpolate_segment(current, pre_grasp, max_step))
    waypoints.extend(_interpolate_segment(pre_grasp, grasp, max_step))
    return waypoints


def _max_segment(current, waypoints) -> float:
    previous = current
    maximum = 0.0
    for point in waypoints:
        maximum = max(maximum, _distance(previous, point))
        previous = point
    return maximum


def main() -> int:
    repo_root = Path(__file__).resolve().parents[4]
    source = repo_root / "robot_ws" / "src" / "robot_tasks" / "robot_tasks" / "grasp_task_open_loop.py"
    text = source.read_text(encoding="utf-8")

    missing = [definition for definition in REQUIRED_DEFINITIONS if definition not in text]
    if missing:
        print("FAIL: missing open_loop runtime surface definitions:")
        for definition in missing:
            print(f"  - {definition}")
        return 1

    current = [0.241, 0.000, 0.520]
    pre_grasp = [0.380, 0.089, 0.548]
    grasp = [0.380, 0.089, 0.468]
    waypoints = _build_test_waypoints(current, pre_grasp, grasp, 0.09)
    max_segment = _max_segment(current, waypoints)
    if not (len(waypoints) > 2 and len(waypoints) <= 6):
        print(f"FAIL: test waypoint_count={len(waypoints)} outside expected range")
        return 1
    if max_segment > 0.09 + 1e-9:
        print(f"FAIL: test max_segment={max_segment:.6f} exceeds 0.09")
        return 1
    if waypoints[-1] != grasp:
        print(f"FAIL: final waypoint {waypoints[-1]} != grasp {grasp}")
        return 1

    print("PASS: open_loop runtime surface definitions present")
    print(
        "PASS: cart_waypoints interpolation scenario "
        f"waypoint_count={len(waypoints)}, max_segment={max_segment:.6f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
