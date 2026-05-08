# -*- coding: utf-8 -*-
from dataclasses import dataclass, field
import math
import time
from airbot_py.arm import AIRBOTPlay, RobotMode, SpeedProfile


@dataclass
class ArmSafetyConfig:
    """Conservative software guardrails before commands reach the SDK."""

    workspace_min: list = field(default_factory=lambda: [0.10, -0.45, 0.02])
    workspace_max: list = field(default_factory=lambda: [1.00, 0.50, 0.70])
    max_cartesian_step: float = 0.10
    max_joint_speed: float = 1.20
    joint_limit_margin: float = 0.08
    joint_limits: list = field(default_factory=lambda: [
        [-3.1416, 3.1416],
        [-2.6000, 2.6000],
        [-2.6000, 2.6000],
        [-3.1416, 3.1416],
        [-2.6000, 2.6000],
        [-3.1416, 3.1416],
    ])


class AirbotWrapper:
    """Light wrapper around the AIRBOT Python SDK.

    This layer is the only place in robot_ws that directly touches the SDK.
    It should expose simple connect/disconnect, state read, and motion helpers.
    """

    def __init__(self, url="localhost", port=50001, safety_config=None):
        self.url = url
        self.port = port
        self.robot = None
        self.safety = safety_config or ArmSafetyConfig()

    def connect(self, speed_profile="default"):
        """Connect to the AIRBOT server and set the initial speed profile."""
        if self.robot is None:
            self.robot = AIRBOTPlay(url=self.url, port=self.port)
            self.robot.connect()
        self.set_speed_profile(speed_profile)

    def disconnect(self):
        """Disconnect cleanly from the robot service."""
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None

    def set_speed_profile(self, speed_profile="default"):
        """Translate a simple speed label into SDK speed profile values."""
        if self.robot is None:
            raise RuntimeError("Robot is not connected.")

        if speed_profile == "slow":
            self.robot.set_speed_profile(SpeedProfile.SLOW)
        elif speed_profile == "fast":
            self.robot.set_speed_profile(SpeedProfile.FAST)
        else:
            self.robot.set_speed_profile(SpeedProfile.DEFAULT)

    def get_state(self):
        """Return the API state string or object from the SDK."""
        return self.robot.get_state()

    def get_joint_pos(self):
        """Read the current joint positions from the robot."""
        return self.robot.get_joint_pos()

    def get_joint_vel(self):
        """Read the current joint velocities from the robot."""
        return self.robot.get_joint_vel()

    def get_end_pose(self):
        """Read the current end-effector pose from the robot."""
        return self.robot.get_end_pose()

    def move_joints(self, joint_target):
        """Send a joint-space motion command in planning mode after safety checks."""
        self.validate_joint_target(joint_target)
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.robot.move_to_joint_pos(joint_target)

    def move_cart_waypoints(self, waypoints):
        """Send a Cartesian waypoint plan to the robot after safety checks."""
        self.validate_cart_waypoints(waypoints)
        self.robot.switch_mode(RobotMode.PLANNING_WAYPOINTS)
        self.robot.move_with_cart_waypoints(waypoints)

    def move_to_cart_target_with_current_orientation(self, target_xyz):
        """Move the end-effector to a Cartesian target while preserving orientation."""
        pose = self.get_end_pose()
        if pose is None or len(pose) < 2:
            raise RuntimeError("Failed to read current end pose.")
        current_quat = list(pose[1])
        self.move_cart_waypoints([
            [list(target_xyz), current_quat],
        ])

    def servo_joints(self, joint_target):
        """Send a direct joint servo command after safety checks."""
        self.validate_joint_target(joint_target)
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        self.robot.servo_joint_pos(joint_target)

    def go_home(self):
        """Move the robot to a predefined home joint pose."""
        home_joint = [0.0, -0.785398, 0.785398, 0.0, 0.0, 0.0]
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.robot.move_to_joint_pos(home_joint)

    def validate_joint_target(self, joint_target):
        """Reject joint commands outside configured limits or near dangerous states."""
        if joint_target is None or len(joint_target) != len(self.safety.joint_limits):
            raise ValueError("Joint target must contain 6 values.")

        self._validate_current_joint_state()

        for index, value in enumerate(joint_target):
            low, high = self.safety.joint_limits[index]
            if not low <= float(value) <= high:
                raise ValueError(
                    f"Joint {index + 1} target {value:.3f} is outside "
                    f"limit [{low:.3f}, {high:.3f}]."
                )

    def validate_cart_target(self, target_xyz):
        """Reject Cartesian targets outside the workspace or too far from current pose."""
        if target_xyz is None or len(target_xyz) != 3:
            raise ValueError("Cartesian target must contain x, y, z.")

        self._validate_current_joint_state()
        self._validate_workspace(target_xyz)

        pose = self.get_end_pose()
        if pose is None or len(pose) < 2:
            raise RuntimeError("Failed to read current end pose.")

        current_xyz = list(pose[0])
        step = self._distance(current_xyz, target_xyz)
        if step > self.safety.max_cartesian_step:
            raise ValueError(
                f"Cartesian step {step:.3f} m exceeds safe limit "
                f"{self.safety.max_cartesian_step:.3f} m."
            )

        quat = list(pose[1])
        self._validate_quaternion(quat)

    def validate_cart_waypoints(self, waypoints):
        """Validate every Cartesian waypoint before forwarding it to the SDK."""
        if not waypoints:
            raise ValueError("Cartesian waypoint list is empty.")

        previous_xyz = list(self.get_end_pose()[0])
        self._validate_current_joint_state()

        for waypoint in waypoints:
            if waypoint is None or len(waypoint) < 2:
                raise ValueError("Each waypoint must contain position and orientation.")
            target_xyz = list(waypoint[0])
            target_quat = list(waypoint[1])

            self._validate_workspace(target_xyz)
            self._validate_quaternion(target_quat)

            step = self._distance(previous_xyz, target_xyz)
            if step > self.safety.max_cartesian_step:
                raise ValueError(
                    f"Cartesian waypoint step {step:.3f} m exceeds safe limit "
                    f"{self.safety.max_cartesian_step:.3f} m."
                )
            previous_xyz = target_xyz

    def _validate_current_joint_state(self):
        """Check current joint pose and velocity before accepting a new command."""
        joint_pos = list(self.get_joint_pos())
        joint_vel = list(self.get_joint_vel())

        if len(joint_pos) != len(self.safety.joint_limits):
            raise RuntimeError("Current joint position does not contain 6 values.")

        for index, value in enumerate(joint_pos):
            low, high = self.safety.joint_limits[index]
            margin = self.safety.joint_limit_margin
            if value < low + margin or value > high - margin:
                raise RuntimeError(
                    f"Current joint {index + 1} is too close to its limit: {value:.3f}."
                )

        if joint_vel:
            max_speed = max(abs(float(v)) for v in joint_vel)
            if max_speed > self.safety.max_joint_speed:
                raise RuntimeError(
                    f"Current joint speed {max_speed:.3f} rad/s exceeds safe limit "
                    f"{self.safety.max_joint_speed:.3f} rad/s."
                )

    def _validate_workspace(self, target_xyz):
        """Check target position against the configured Cartesian workspace."""
        for index, value in enumerate(target_xyz):
            low = self.safety.workspace_min[index]
            high = self.safety.workspace_max[index]
            if not low <= float(value) <= high:
                axis = "xyz"[index]
                raise ValueError(
                    f"Target {axis}={value:.3f} is outside workspace "
                    f"[{low:.3f}, {high:.3f}]."
                )

    def _validate_quaternion(self, quat):
        """Reject malformed orientations before they reach waypoint planning."""
        if len(quat) != 4:
            raise ValueError("Orientation quaternion must contain 4 values.")

        norm = math.sqrt(sum(float(value) ** 2 for value in quat))
        if norm < 0.95 or norm > 1.05:
            raise ValueError(f"Orientation quaternion norm {norm:.3f} is invalid.")

    def _distance(self, a, b):
        """Compute Euclidean distance between two 3D positions."""
        return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))

    def open_gripper(self):
        """Open the gripper by commanding end-effector position."""
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        for _ in range(50):
            self.robot.servo_eef_pos([0.07])
            time.sleep(0.02)

    def close_gripper(self):
        """Close the gripper by commanding end-effector position."""
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        for _ in range(50):
            self.robot.servo_eef_pos([0.0])
            time.sleep(0.02)
