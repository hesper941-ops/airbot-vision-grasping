#!/usr/bin/env python3
"""Static runtime-surface check for grasp_task_open_loop.py."""

from pathlib import Path
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

    print("PASS: open_loop runtime surface definitions present")
    return 0


if __name__ == "__main__":
    sys.exit(main())
