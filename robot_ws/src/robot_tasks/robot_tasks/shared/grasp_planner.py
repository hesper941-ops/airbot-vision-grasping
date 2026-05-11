"""Shared grasp waypoint planner.

The planner converts a stable 3D target into simple Cartesian waypoints used by
the open-loop and visual-servo task nodes. It also computes the pre-grasp joint6
orientation compensation while respecting configured joint limits.
"""

import math
from typing import Optional


class GraspPlanner:
    """Generate staged Cartesian waypoints from a target in base_link."""

    def __init__(self, config: dict):
        self.pre_grasp_z_offset = config.get('pre_grasp_z_offset', 0.12)
        self.grasp_z_offset = config.get('grasp_z_offset', 0.0)
        self.lift_z_offset = config.get('lift_z_offset', 0.10)
        self.safe_pose = config.get('safe_pose', [0.35, 0.00, 0.35])
        self.approach_mode = str(config.get('approach_mode', 'front')).strip().lower()
        self.table_z = float(config.get('table_z', 0.0))
        self.table_clearance = float(config.get('table_clearance', 0.04))
        self.final_grasp_clearance = float(config.get('final_grasp_clearance', 0.015))
        self.front_approach_x_offset = float(config.get('front_approach_x_offset', -0.10))
        self.front_approach_z_offset = float(config.get('front_approach_z_offset', 0.05))
        self.min_safe_motion_z = float(config.get('min_safe_motion_z', 0.08))
        self.reject_target_below_table = bool(config.get('reject_target_below_table', True))
        self.joint6_compensation_deg = config.get('joint6_compensation_deg', 90.0)
        self.j6_home_deg = float(config.get('j6_home_deg', 90.0))
        self.j6_allowed_delta_deg = float(config.get('j6_allowed_delta_deg', 90.0))
        self.j6_preferred_offsets_deg = list(config.get(
            'j6_preferred_offsets_deg', [0.0, 90.0, -90.0]))
        self.forbid_camera_upside_down = bool(config.get('forbid_camera_upside_down', True))
        self.last_j6_debug = []
        # Conservative J6 range based on AIRBOT Play official specs.
        # Confirm exact hardware model before widening these defaults.
        self.joint6_min_rad = float(config.get('joint6_min_rad', -2.9671))
        self.joint6_max_rad = float(config.get('joint6_max_rad', 2.9671))

        limits = config.get('workspace_limits', {})
        self.x_min = limits.get('x_min', 0.10)
        self.x_max = limits.get('x_max', 1.00)
        self.y_min = limits.get('y_min', -0.45)
        self.y_max = limits.get('y_max', 0.50)
        self.z_min = limits.get('z_min', 0.02)
        self.z_max = limits.get('z_max', 0.70)

    def set_approach_mode(self, mode: str):
        mode = str(mode).strip().lower()
        if mode not in ('front', 'top_down'):
            raise ValueError(f"Unsupported approach_mode={mode!r}; expected front or top_down.")
        self.approach_mode = mode

    def compute_pre_grasp(self, target_xyz: list) -> list:
        """Return the pre-grasp point above the target."""
        return self.compute_safe_pre_grasp(target_xyz)

    def compute_grasp(self, target_xyz: list) -> list:
        """Return the final grasp point near the target."""
        return self.compute_safe_grasp(target_xyz)

    def compute_lift(self, current_xyz: list) -> list:
        """Return a vertical lift target based on the current end-effector pose."""
        return self.compute_safe_lift(current_xyz)

    def compute_safe_pre_grasp(self, target_xyz: list) -> list:
        """Return a safe pre-grasp waypoint for top-down or front approach."""
        target = self._validate_target(target_xyz)
        safe_z = self.safe_motion_z

        if self.approach_mode == 'top_down':
            waypoint = [
                target[0],
                target[1],
                max(target[2] + self.pre_grasp_z_offset, safe_z),
            ]
        elif self.approach_mode == 'front':
            waypoint = [
                target[0] + self.front_approach_x_offset,
                target[1],
                max(target[2] + self.front_approach_z_offset, safe_z),
            ]
        else:
            raise ValueError(
                f"Unsupported approach_mode={self.approach_mode!r}; expected top_down or front.")

        return self.validate_waypoint(waypoint, is_final_grasp=False)

    def compute_safe_grasp(self, target_xyz: list) -> list:
        """Return a final grasp waypoint that never goes below table clearance."""
        target = self._validate_target(target_xyz)
        min_grasp_z = self.safe_motion_z if self.approach_mode == 'front' else self.final_grasp_z
        waypoint = [
            target[0],
            target[1],
            max(target[2] + self.grasp_z_offset, min_grasp_z),
        ]
        return self.validate_waypoint(waypoint, is_final_grasp=True)

    def compute_safe_lift(self, current_xyz: list) -> list:
        """Return a vertical lift target without dragging sideways."""
        current = self._validate_xyz(current_xyz, name='current_xyz')
        return self.validate_waypoint([
            current_xyz[0],
            current_xyz[1],
            max(current[2] + self.lift_z_offset, self.safe_motion_z),
        ], is_final_grasp=False)

    def validate_waypoint(self, xyz: list, is_final_grasp: bool = False) -> list:
        """Validate and clamp a waypoint while enforcing table clearance."""
        point = self._validate_xyz(xyz, name='waypoint')
        min_z = self.final_grasp_z if is_final_grasp else self.safe_motion_z
        if point[2] < min_z:
            point[2] = min_z
        self._validate_workspace(point)
        return point

    def validate_approach_direction(self, current_xyz: list, target_xyz: list, mode=None):
        """Reject approach moves that would come from under the target."""
        current = self._validate_xyz(current_xyz, name='current_xyz')
        target = self._validate_target(target_xyz)
        selected_mode = (mode or self.approach_mode).strip().lower()

        if selected_mode == 'top_down':
            pre_grasp = self.compute_safe_pre_grasp(target)
            if current[2] + 1e-6 < target[2]:
                raise ValueError(
                    f"Unsafe top-down approach: current z={current[2]:.3f} is below target z={target[2]:.3f}.")
            if pre_grasp[2] + 1e-6 < target[2]:
                raise ValueError(
                    f"Unsafe top-down pre-grasp z={pre_grasp[2]:.3f} is below target z={target[2]:.3f}.")
        elif selected_mode == 'front':
            if current[2] + 1e-6 < self.safe_motion_z:
                raise ValueError(
                    f"Unsafe front approach: current z={current[2]:.3f} below safe z={self.safe_motion_z:.3f}.")
        else:
            raise ValueError(
                f"Unsupported approach_mode={selected_mode!r}; expected top_down or front.")

        return True

    def get_safe_pose(self) -> list:
        """Return the configured safe retreat pose."""
        return self.validate_waypoint(self.safe_pose, is_final_grasp=False)

    @property
    def safe_motion_z(self) -> float:
        return max(self.table_z + self.table_clearance, self.min_safe_motion_z)

    @property
    def final_grasp_z(self) -> float:
        return self.table_z + self.final_grasp_clearance

    @staticmethod
    def distance(a: list, b: list) -> float:
        """Return Euclidean distance between two 3D points."""
        return math.sqrt(
            (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
        )

    @staticmethod
    def limit_step(current: list, target: list, max_step: float) -> list:
        """Limit a Cartesian step so a single command does not jump too far."""
        distance = GraspPlanner.distance(current, target)
        if distance <= max_step:
            return target
        ratio = max_step / distance
        return [
            current[0] + (target[0] - current[0]) * ratio,
            current[1] + (target[1] - current[1]) * ratio,
            current[2] + (target[2] - current[2]) * ratio,
        ]

    @staticmethod
    def reached(current: list, target: list, threshold: float = 0.03) -> bool:
        """Return True when current position is close enough to target."""
        return GraspPlanner.distance(current, target) < threshold

    @property
    def joint6_compensation_rad(self) -> float:
        """Return the configured joint6 compensation in radians."""
        return math.radians(self.joint6_compensation_deg)

    def compute_joint6_target(self, current_joint_pos: list) -> Optional[list]:
        """Choose a safe J6 target without allowing camera-upside-down poses."""
        self.last_j6_debug = []
        if len(current_joint_pos) < 6:
            self.last_j6_debug.append('Reject J6: current_joint_pos has fewer than 6 values.')
            return None

        current_j6 = float(current_joint_pos[5])
        home = self.j6_home_rad
        allowed_low, allowed_high = self.j6_allowed_range_rad
        if self.forbid_camera_upside_down and not allowed_low <= current_j6 <= allowed_high:
            self.last_j6_debug.append(
                f'Current J6 {math.degrees(current_j6):.1f} deg is outside safe range '
                f'[{math.degrees(allowed_low):.1f}, {math.degrees(allowed_high):.1f}] deg; '
                f'prefer returning to home {self.j6_home_deg:.1f} deg.')

        candidates = []
        for offset_deg in self.j6_preferred_offsets_deg:
            candidate = home + math.radians(float(offset_deg))
            candidates.append((offset_deg, candidate))

        valid = []
        for offset_deg, value in candidates:
            if not self.joint6_min_rad <= value <= self.joint6_max_rad:
                self.last_j6_debug.append(
                    f'Reject J6 candidate home{offset_deg:+.1f} deg -> {math.degrees(value):.1f} deg: '
                    f'outside hardware range [{math.degrees(self.joint6_min_rad):.1f}, '
                    f'{math.degrees(self.joint6_max_rad):.1f}] deg.')
                continue
            if self.forbid_camera_upside_down and not allowed_low <= value <= allowed_high:
                self.last_j6_debug.append(
                    f'Reject J6 candidate home{offset_deg:+.1f} deg -> {math.degrees(value):.1f} deg: '
                    f'outside safe camera range [{math.degrees(allowed_low):.1f}, '
                    f'{math.degrees(allowed_high):.1f}] deg.')
                continue
            valid.append(value)

        if not valid:
            self.last_j6_debug.append('Reject J6: no valid home/+-90 candidate remains.')
            return None

        if self.forbid_camera_upside_down and not allowed_low <= current_j6 <= allowed_high and home in valid:
            selected = home
        else:
            selected = min(valid, key=lambda value: abs(value - current_j6))
        self.last_j6_debug.append(
            f'J6 selection: current={math.degrees(current_j6):.1f} deg, '
            f'selected={math.degrees(selected):.1f} deg, home={self.j6_home_deg:.1f} deg, '
            f'allowed=[{math.degrees(allowed_low):.1f}, {math.degrees(allowed_high):.1f}] deg.')
        target = list(current_joint_pos[:])
        target[5] = selected
        return target

    def compute_j6_home_target(self, current_joint_pos: list) -> Optional[list]:
        if len(current_joint_pos) < 6:
            return None
        home = self.j6_home_rad
        if not self.joint6_min_rad <= home <= self.joint6_max_rad:
            return None
        allowed_low, allowed_high = self.j6_allowed_range_rad
        if self.forbid_camera_upside_down and not allowed_low <= home <= allowed_high:
            return None
        target = list(current_joint_pos[:])
        target[5] = home
        return target

    @property
    def j6_home_rad(self) -> float:
        return math.radians(self.j6_home_deg)

    @property
    def j6_allowed_range_rad(self):
        delta = math.radians(self.j6_allowed_delta_deg)
        return self.j6_home_rad - delta, self.j6_home_rad + delta

    def _joint6_limit_margin(self, value: float) -> float:
        """Return distance from the closest joint6 limit."""
        return min(value - self.joint6_min_rad, self.joint6_max_rad - value)

    def _clamp(self, xyz: list) -> list:
        """Clamp a Cartesian point into the configured workspace."""
        return [
            min(max(float(xyz[0]), self.x_min), self.x_max),
            min(max(float(xyz[1]), self.y_min), self.y_max),
            min(max(float(xyz[2]), self.z_min), self.z_max),
        ]

    def _validate_workspace(self, xyz: list):
        for index, value in enumerate(xyz):
            low = [self.x_min, self.y_min, self.z_min][index]
            high = [self.x_max, self.y_max, self.z_max][index]
            if not low <= float(value) <= high:
                axis = 'xyz'[index]
                raise ValueError(
                    f"Waypoint {axis}={value:.3f} is outside workspace [{low:.3f}, {high:.3f}].")

    def _validate_target(self, target_xyz: list) -> list:
        target = self._validate_xyz(target_xyz, name='target_xyz')
        if self.reject_target_below_table and target[2] < self.table_z:
            raise ValueError(
                f"Target z={target[2]:.3f} is below table_z={self.table_z:.3f}.")
        return target

    @staticmethod
    def _validate_xyz(xyz: list, name: str = 'xyz') -> list:
        if xyz is None or len(xyz) != 3:
            raise ValueError(f"{name} must contain x, y, z.")
        point = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
        if not all(math.isfinite(v) for v in point):
            raise ValueError(f"{name} contains non-finite coordinates.")
        return point
