from __future__ import annotations

import math
from dataclasses import dataclass

from robot.robot import Robot

# =============================================================================
# GPS / ArUco navigation-only mission
# =============================================================================
# This file is intentionally isolated from main.py and from scripted_navigation.py.
#
# Behavior:
#   - Visit 3 configured target locations.
#   - Visit the configured drop-off location.
#   - Move X first, then Y, to avoid diagonal travel near the table.
#   - Do NOT move the claw, close the claw, lift, or release anything.
# =============================================================================


# -----------------------------------------------------------------------------
# GPS / ArUco navigation tuning parameters

TARGET_TOLERANCE_MM = 100.0
AXIS_TARGET_TOLERANCE_MM = 80.0

GPS_APPROACH_SPEED_MM_S = 80.0
GPS_TURN_GAIN = 1.5
GPS_MAX_TURN_RATE_DEG_S = 40.0
GPS_HEADING_DEADBAND_DEG = 5.0
GPS_TURN_IN_PLACE_THRESHOLD_DEG = 45.0
GPS_ALIGN_TOLERANCE_DEG = 6.0

# If False, the rover body/reference point drives to each target coordinate.
# If True, the rover stops behind each target so the claw point would be over the
# target, but it still will NOT move the claw. Leave False for pure navigation
# testing. Set True if you want to verify final claw-over-object positioning.
USE_CLAW_OFFSET_FOR_VISITS = False

# Distance from rover body reference point / wheelbase center to the claw point.
# Only used when USE_CLAW_OFFSET_FOR_VISITS = True.
CLAW_FORWARD_OFFSET_MM = 180.0

# If True and USE_CLAW_OFFSET_FOR_VISITS is also True, the drop-off coordinate is
# treated as a claw/release point instead of a rover-center point.
GPS_DROPOFF_IS_CLAW_COORDINATE = True


@dataclass(frozen=True)
class GpsTarget:
    name: str
    x_mm: float
    y_mm: float
    tolerance_mm: float = TARGET_TOLERANCE_MM


@dataclass
class AxisAlignedGoal:
    name: str
    target_x_mm: float
    target_y_mm: float
    center_x_mm: float
    center_y_mm: float
    final_heading_deg: float
    tolerance_mm: float


# Replace these with your measured world-frame course coordinates.
# These are the locations the rover should visit during the navigation-only test.
# Mission order is intentionally patty first, even if the patty is physically
# between the two bun locations. Change the order of this list to change the
# visit sequence.
GPS_OBJECT_TARGETS: list[GpsTarget] = [
    GpsTarget("patty_location", 900.0, 300.0),
    GpsTarget("bottom_bun_location", 500.0, 300.0),
    GpsTarget("top_bun_location", 1300.0, 300.0),
]

GPS_DROPOFF_TARGET = GpsTarget("dropoff_location", 1700.0, 300.0)

# Backward-compatible alias in case older test code imports GpsPickupTarget.
GpsPickupTarget = GpsTarget


# =============================================================================
# Basic helpers
# =============================================================================


def wrap_angle_deg(angle: float) -> float:
    """Wrap an angle to [-180, 180] degrees."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def get_navigation_pose(robot: Robot) -> tuple[float, float, float] | None:
    """
    Return the rover world pose from the Robot GPS / fused-pose system.

    Returns (x_mm, y_mm, theta_deg), or None if the ArUco/GPS pose is not fresh.
    """
    if not robot.is_gps_active():
        return None

    if robot.has_fused_pose():
        pose = robot.get_fused_pose()
        if pose is not None:
            return pose

    return None


def robot_center_target_for_claw(
    target_x_mm: float,
    target_y_mm: float,
    approach_heading_deg: float,
    claw_forward_offset_mm: float = CLAW_FORWARD_OFFSET_MM,
) -> tuple[float, float]:
    """
    Convert a target coordinate into the rover-center coordinate that would put
    the claw point over the target. Only used when USE_CLAW_OFFSET_FOR_VISITS is True.
    """
    heading_rad = math.radians(approach_heading_deg)
    center_x = target_x_mm - claw_forward_offset_mm * math.cos(heading_rad)
    center_y = target_y_mm - claw_forward_offset_mm * math.sin(heading_rad)
    return center_x, center_y


def choose_y_approach_heading_deg(current_y_mm: float, target_y_mm: float) -> float:
    """
    Choose final heading for the Y-only approach.

    +90 means the rover faces +Y.
    -90 means the rover faces -Y.
    """
    return 90.0 if target_y_mm >= current_y_mm else -90.0


def make_axis_aligned_goal(
    robot: Robot,
    target: GpsTarget,
    use_claw_offset: bool = USE_CLAW_OFFSET_FOR_VISITS,
    claw_forward_offset_mm: float = CLAW_FORWARD_OFFSET_MM,
) -> AxisAlignedGoal | None:
    """
    Build an X-then-Y route target from the current pose.

    This should be called once per target, then the returned goal is reused by
    the mission phases. That prevents the rover from continuously recomputing a
    changing path and accidentally drifting into diagonal behavior.
    """
    pose = get_navigation_pose(robot)
    if pose is None:
        robot.stop()
        print(f"[GPS NAV] waiting for pose before preparing route to {target.name}...")
        return None

    _x_mm, y_mm, _theta_deg = pose
    final_heading_deg = choose_y_approach_heading_deg(y_mm, target.y_mm)

    if use_claw_offset:
        center_x_mm, center_y_mm = robot_center_target_for_claw(
            target.x_mm,
            target.y_mm,
            final_heading_deg,
            claw_forward_offset_mm,
        )
    else:
        center_x_mm, center_y_mm = target.x_mm, target.y_mm

    print(
        f"[GPS NAV] prepared X-then-Y route to {target.name}: "
        f"target=({target.x_mm:.0f}, {target.y_mm:.0f}) mm, "
        f"rover_center_goal=({center_x_mm:.0f}, {center_y_mm:.0f}) mm, "
        f"final_heading={final_heading_deg:.0f} deg, "
        f"claw_offset={'ON' if use_claw_offset else 'OFF'}"
    )

    return AxisAlignedGoal(
        name=target.name,
        target_x_mm=target.x_mm,
        target_y_mm=target.y_mm,
        center_x_mm=center_x_mm,
        center_y_mm=center_y_mm,
        final_heading_deg=final_heading_deg,
        tolerance_mm=target.tolerance_mm,
    )


def command_turn_toward_heading(robot: Robot, theta_deg: float, desired_heading_deg: float) -> float:
    """Command heading correction and return heading error."""
    heading_error_deg = wrap_angle_deg(desired_heading_deg - theta_deg)

    if abs(heading_error_deg) < GPS_HEADING_DEADBAND_DEG:
        turn_rate_deg_s = 0.0
    else:
        turn_rate_deg_s = GPS_TURN_GAIN * heading_error_deg

    turn_rate_deg_s = clamp(
        turn_rate_deg_s,
        -GPS_MAX_TURN_RATE_DEG_S,
        GPS_MAX_TURN_RATE_DEG_S,
    )

    robot.set_velocity(0.0, turn_rate_deg_s)
    return heading_error_deg


def align_to_heading_gps(
    robot: Robot,
    desired_heading_deg: float,
    tolerance_deg: float = GPS_ALIGN_TOLERANCE_DEG,
) -> bool:
    """Rotate in place until the rover is facing desired_heading_deg."""
    pose = get_navigation_pose(robot)
    if pose is None:
        robot.stop()
        print("[GPS NAV] waiting for valid pose before heading alignment...")
        return False

    _x_mm, _y_mm, theta_deg = pose
    heading_error_deg = wrap_angle_deg(desired_heading_deg - theta_deg)

    if abs(heading_error_deg) <= tolerance_deg:
        robot.stop()
        print(f"[GPS NAV] heading aligned to {desired_heading_deg:.0f} deg")
        return True

    command_turn_toward_heading(robot, theta_deg, desired_heading_deg)
    print(
        f"[GPS NAV] aligning heading: target={desired_heading_deg:.0f} deg, "
        f"theta={theta_deg:.1f} deg, error={heading_error_deg:.1f} deg"
    )
    return False


def drive_axis_gps(
    robot: Robot,
    axis: str,
    target_value_mm: float,
    tolerance_mm: float = AXIS_TARGET_TOLERANCE_MM,
) -> bool:
    """
    Drive along one world-frame axis only.

    axis="X" drives east/west until x ~= target_value_mm.
    axis="Y" drives north/south until y ~= target_value_mm.
    """
    pose = get_navigation_pose(robot)
    if pose is None:
        robot.stop()
        print(f"[GPS NAV] waiting for valid pose before {axis}-axis move...")
        return False

    x_mm, y_mm, theta_deg = pose
    axis_upper = axis.upper()

    if axis_upper == "X":
        current_value_mm = x_mm
        positive_heading_deg = 0.0
        negative_heading_deg = 180.0
    elif axis_upper == "Y":
        current_value_mm = y_mm
        positive_heading_deg = 90.0
        negative_heading_deg = -90.0
    else:
        raise ValueError(f"axis must be 'X' or 'Y', got {axis!r}")

    remaining_mm = target_value_mm - current_value_mm

    if abs(remaining_mm) <= tolerance_mm:
        robot.stop()
        print(f"[GPS NAV] {axis_upper}-axis target reached")
        return True

    desired_heading_deg = positive_heading_deg if remaining_mm >= 0.0 else negative_heading_deg
    heading_error_deg = wrap_angle_deg(desired_heading_deg - theta_deg)

    # If the rover is facing far away from the desired axis direction, turn in
    # place first instead of arcing toward the table.
    if abs(heading_error_deg) > GPS_TURN_IN_PLACE_THRESHOLD_DEG:
        forward_speed_mm_s = 0.0
    else:
        forward_speed_mm_s = GPS_APPROACH_SPEED_MM_S

    if abs(heading_error_deg) < GPS_HEADING_DEADBAND_DEG:
        turn_rate_deg_s = 0.0
    else:
        turn_rate_deg_s = GPS_TURN_GAIN * heading_error_deg

    turn_rate_deg_s = clamp(
        turn_rate_deg_s,
        -GPS_MAX_TURN_RATE_DEG_S,
        GPS_MAX_TURN_RATE_DEG_S,
    )

    robot.set_velocity(forward_speed_mm_s, turn_rate_deg_s)

    print(
        f"[GPS NAV] {axis_upper}-move target={target_value_mm:.0f} mm "
        f"pose=({x_mm:.0f}, {y_mm:.0f}, {theta_deg:.1f}) "
        f"remaining={remaining_mm:.0f} mm "
        f"heading_error={heading_error_deg:.1f} deg "
        f"cmd=({forward_speed_mm_s:.0f} mm/s, {turn_rate_deg_s:.1f} deg/s)"
    )
    return False


def drive_to_world_point_gps(
    robot: Robot,
    target_x_mm: float,
    target_y_mm: float,
    tolerance_mm: float = TARGET_TOLERANCE_MM,
) -> bool:
    """
    Direct point-to-point GPS drive.

    Kept for quick testing only. The full navigation-only mission below does not
    use this because it can create diagonal paths.
    """
    pose = get_navigation_pose(robot)

    if pose is None:
        robot.stop()
        print("[GPS NAV] waiting for valid ArUco/GPS pose...")
        return False

    x_mm, y_mm, theta_deg = pose
    dx = target_x_mm - x_mm
    dy = target_y_mm - y_mm
    distance_mm = math.hypot(dx, dy)

    if distance_mm <= tolerance_mm:
        robot.stop()
        print("[GPS NAV] direct target reached")
        return True

    target_angle_deg = math.degrees(math.atan2(dy, dx))
    heading_error_deg = wrap_angle_deg(target_angle_deg - theta_deg)

    if abs(heading_error_deg) > GPS_TURN_IN_PLACE_THRESHOLD_DEG:
        forward_speed_mm_s = 0.0
    else:
        forward_speed_mm_s = GPS_APPROACH_SPEED_MM_S

    if abs(heading_error_deg) < GPS_HEADING_DEADBAND_DEG:
        turn_rate_deg_s = 0.0
    else:
        turn_rate_deg_s = GPS_TURN_GAIN * heading_error_deg

    turn_rate_deg_s = clamp(
        turn_rate_deg_s,
        -GPS_MAX_TURN_RATE_DEG_S,
        GPS_MAX_TURN_RATE_DEG_S,
    )

    robot.set_velocity(forward_speed_mm_s, turn_rate_deg_s)

    print(
        f"[GPS NAV] DIRECT target=({target_x_mm:.0f}, {target_y_mm:.0f}) "
        f"pose=({x_mm:.0f}, {y_mm:.0f}, {theta_deg:.1f}) "
        f"dist={distance_mm:.0f} mm "
        f"heading_error={heading_error_deg:.1f} deg "
        f"cmd=({forward_speed_mm_s:.0f} mm/s, {turn_rate_deg_s:.1f} deg/s)"
    )
    return False


# =============================================================================
# GPS 3-target navigation-only mission state machine
# =============================================================================


class GpsNavigationMission:
    """
    Non-blocking GPS mission: visit 3 target locations, then visit dropoff.

    Default visit order is patty first, bottom bun second, top bun third.

    No claw commands are sent. The mission intentionally uses X-first, then Y
    movement for every target to avoid diagonal travel into the table.
    """

    def __init__(
        self,
        targets: list[GpsTarget] | None = None,
        dropoff: GpsTarget = GPS_DROPOFF_TARGET,
    ) -> None:
        self.targets = targets if targets is not None else GPS_OBJECT_TARGETS
        self.dropoff = dropoff
        self.index = 0
        self.phase = "PREPARE_TARGET_ROUTE"
        self.done = False
        self.active_goal: AxisAlignedGoal | None = None

    def reset(self) -> None:
        self.index = 0
        self.phase = "PREPARE_TARGET_ROUTE"
        self.done = False
        self.active_goal = None

    def _current_target(self) -> GpsTarget:
        return self.targets[self.index]

    def _prepare_visit_goal(self, robot: Robot, target: GpsTarget) -> bool:
        self.active_goal = make_axis_aligned_goal(
            robot,
            target,
            use_claw_offset=USE_CLAW_OFFSET_FOR_VISITS,
        )
        return self.active_goal is not None

    def _prepare_dropoff_goal(self, robot: Robot) -> bool:
        use_claw_offset = USE_CLAW_OFFSET_FOR_VISITS and GPS_DROPOFF_IS_CLAW_COORDINATE
        self.active_goal = make_axis_aligned_goal(
            robot,
            self.dropoff,
            use_claw_offset=use_claw_offset,
        )
        return self.active_goal is not None

    def update(self, robot: Robot) -> bool:
        if self.done:
            robot.stop()
            return True

        if self.phase == "PREPARE_TARGET_ROUTE":
            target = self._current_target()
            if self._prepare_visit_goal(robot, target):
                print(f"[GPS MISSION] navigating to target {self.index + 1}/{len(self.targets)}: {target.name}")
                self.phase = "NAV_TARGET_X"
            return False

        if self.phase == "NAV_TARGET_X":
            assert self.active_goal is not None
            if drive_axis_gps(robot, "X", self.active_goal.center_x_mm, AXIS_TARGET_TOLERANCE_MM):
                self.phase = "NAV_TARGET_Y"
            return False

        if self.phase == "NAV_TARGET_Y":
            assert self.active_goal is not None
            if drive_axis_gps(robot, "Y", self.active_goal.center_y_mm, self.active_goal.tolerance_mm):
                self.phase = "ALIGN_AT_TARGET"
            return False

        if self.phase == "ALIGN_AT_TARGET":
            assert self.active_goal is not None
            if align_to_heading_gps(robot, self.active_goal.final_heading_deg):
                print(f"[GPS MISSION] visited {self.active_goal.name}; no pickup command sent")
                self.index += 1
                self.active_goal = None
                self.phase = "PREPARE_DROPOFF_ROUTE" if self.index >= len(self.targets) else "PREPARE_TARGET_ROUTE"
            return False

        if self.phase == "PREPARE_DROPOFF_ROUTE":
            if self._prepare_dropoff_goal(robot):
                print(f"[GPS MISSION] navigating to dropoff: {self.dropoff.name}")
                self.phase = "NAV_DROPOFF_X"
            return False

        if self.phase == "NAV_DROPOFF_X":
            assert self.active_goal is not None
            if drive_axis_gps(robot, "X", self.active_goal.center_x_mm, AXIS_TARGET_TOLERANCE_MM):
                self.phase = "NAV_DROPOFF_Y"
            return False

        if self.phase == "NAV_DROPOFF_Y":
            assert self.active_goal is not None
            if drive_axis_gps(robot, "Y", self.active_goal.center_y_mm, self.active_goal.tolerance_mm):
                self.phase = "ALIGN_AT_DROPOFF"
            return False

        if self.phase == "ALIGN_AT_DROPOFF":
            assert self.active_goal is not None
            if align_to_heading_gps(robot, self.active_goal.final_heading_deg):
                robot.stop()
                self.done = True
                print("[GPS MISSION] navigation-only mission complete; no dropoff command sent")
                return True
            return False

        robot.stop()
        raise RuntimeError(f"Unknown GPS mission phase: {self.phase}")


# Backward-compatible alias in case an older test imports GpsPickupMission.
GpsPickupMission = GpsNavigationMission
