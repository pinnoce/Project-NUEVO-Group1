from __future__ import annotations

import math

from robot.robot import Robot


# ---------------------------------------------------------------------------
# GPS / ArUco navigation tuning parameters
# ---------------------------------------------------------------------------

# How close the CLAW must get to the target before saying "reached"
TARGET_TOLERANCE_MM = 100.0

# Forward speed while driving toward a target
GPS_APPROACH_SPEED_MM_S = 80.0

# Turning correction gain
GPS_TURN_GAIN = 1.5

# Maximum allowed turning speed
GPS_MAX_TURN_RATE_DEG_S = 40.0

# Ignore tiny heading errors to reduce jitter
GPS_HEADING_DEADBAND_DEG = 5.0

# If heading error is larger than this, rotate in place before driving forward
GPS_TURN_IN_PLACE_THRESHOLD_DEG = 45.0


# ---------------------------------------------------------------------------
# Claw geometry
# ---------------------------------------------------------------------------
# IMPORTANT:
# These are measured from the midpoint between the two drive wheels.
#
# +x = forward from wheel midpoint
# +y = left from wheel midpoint
#
# Example:
# If the claw is 180 mm in front of the wheel midpoint and centered:
#   CLAW_BODY_OFFSET_X_MM = 180.0
#   CLAW_BODY_OFFSET_Y_MM = 0.0
#
# If the claw is 180 mm forward and 30 mm to the RIGHT:
#   CLAW_BODY_OFFSET_X_MM = 180.0
#   CLAW_BODY_OFFSET_Y_MM = -30.0
CLAW_BODY_OFFSET_X_MM = 180.0
CLAW_BODY_OFFSET_Y_MM = 0.0


def wrap_angle_deg(angle: float) -> float:
    """
    Wrap an angle to the range [-180, 180] degrees.
    """
    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


def rotate_body_to_world(
    body_x_mm: float,
    body_y_mm: float,
    theta_deg: float,
) -> tuple[float, float]:
    """
    Rotate a robot-body-frame point into the world frame.

    Body frame:
        +x = robot forward
        +y = robot left

    World frame:
        theta is counter-clockwise from world +X.
    """
    theta_rad = math.radians(theta_deg)

    world_x_mm = (
        body_x_mm * math.cos(theta_rad)
        - body_y_mm * math.sin(theta_rad)
    )

    world_y_mm = (
        body_x_mm * math.sin(theta_rad)
        + body_y_mm * math.cos(theta_rad)
    )

    return world_x_mm, world_y_mm


def get_navigation_pose(robot: Robot) -> tuple[float, float, float] | None:
    """
    Return the rover body-origin pose from the existing Robot GPS/fused-pose system.

    Returns:
        (x_mm, y_mm, theta_deg)

    Returns None if GPS/ArUco data is not fresh yet.
    """
    if not robot.is_gps_active():
        return None

    if robot.has_fused_pose():
        pose = robot.get_fused_pose()
        if pose is not None:
            return pose

    return None


def get_claw_world_position(
    robot_x_mm: float,
    robot_y_mm: float,
    robot_theta_deg: float,
) -> tuple[float, float]:
    """
    Calculate where the claw is in world coordinates.

    The robot pose is the midpoint between the drive wheels.
    The claw offset is measured from that midpoint.
    """
    claw_dx_world_mm, claw_dy_world_mm = rotate_body_to_world(
        CLAW_BODY_OFFSET_X_MM,
        CLAW_BODY_OFFSET_Y_MM,
        robot_theta_deg,
    )

    claw_x_world_mm = robot_x_mm + claw_dx_world_mm
    claw_y_world_mm = robot_y_mm + claw_dy_world_mm

    return claw_x_world_mm, claw_y_world_mm


def get_body_target_for_claw_target(
    target_x_mm: float,
    target_y_mm: float,
    robot_theta_deg: float,
) -> tuple[float, float]:
    """
    Convert a desired claw world target into the body-origin world target.

    In plain English:
    If the claw needs to end at (target_x, target_y), this function calculates
    where the midpoint between the wheels should go.
    """
    claw_dx_world_mm, claw_dy_world_mm = rotate_body_to_world(
        CLAW_BODY_OFFSET_X_MM,
        CLAW_BODY_OFFSET_Y_MM,
        robot_theta_deg,
    )

    body_target_x_mm = target_x_mm - claw_dx_world_mm
    body_target_y_mm = target_y_mm - claw_dy_world_mm

    return body_target_x_mm, body_target_y_mm


def drive_to_world_point_gps(
    robot: Robot,
    target_x_mm: float,
    target_y_mm: float,
) -> bool:
    """
    Drive so the CLAW reaches a world-frame target.

    Args:
        robot:
            Robot API object.

        target_x_mm:
            Target X coordinate in the arena/world frame.

        target_y_mm:
            Target Y coordinate in the arena/world frame.

    Returns:
        True if the claw has reached the target.
        False if still driving or waiting for GPS.
    """
    pose = get_navigation_pose(robot)

    if pose is None:
        robot.stop()
        print("[GPS NAV] waiting for valid ArUco/GPS pose...")
        return False

    robot_x_mm, robot_y_mm, robot_theta_deg = pose

    claw_x_mm, claw_y_mm = get_claw_world_position(
        robot_x_mm,
        robot_y_mm,
        robot_theta_deg,
    )

    claw_dx_mm = target_x_mm - claw_x_mm
    claw_dy_mm = target_y_mm - claw_y_mm
    claw_distance_mm = math.hypot(claw_dx_mm, claw_dy_mm)

    if claw_distance_mm <= TARGET_TOLERANCE_MM:
        robot.stop()
        print("[GPS NAV] claw target reached")
        return True

    body_target_x_mm, body_target_y_mm = get_body_target_for_claw_target(
        target_x_mm,
        target_y_mm,
        robot_theta_deg,
    )

    body_dx_mm = body_target_x_mm - robot_x_mm
    body_dy_mm = body_target_y_mm - robot_y_mm

    target_angle_deg = math.degrees(math.atan2(body_dy_mm, body_dx_mm))
    heading_error_deg = wrap_angle_deg(target_angle_deg - robot_theta_deg)

    if abs(heading_error_deg) > GPS_TURN_IN_PLACE_THRESHOLD_DEG:
        forward_speed_mm_s = 0.0
    else:
        forward_speed_mm_s = GPS_APPROACH_SPEED_MM_S

    if abs(heading_error_deg) < GPS_HEADING_DEADBAND_DEG:
        turn_rate_deg_s = 0.0
    else:
        turn_rate_deg_s = GPS_TURN_GAIN * heading_error_deg

    turn_rate_deg_s = max(
        -GPS_MAX_TURN_RATE_DEG_S,
        min(GPS_MAX_TURN_RATE_DEG_S, turn_rate_deg_s),
    )

    robot.set_velocity(forward_speed_mm_s, turn_rate_deg_s)

    print(
        f"[GPS NAV] target=({target_x_mm:.0f}, {target_y_mm:.0f}) "
        f"robot=({robot_x_mm:.0f}, {robot_y_mm:.0f}, {robot_theta_deg:.1f}) "
        f"claw=({claw_x_mm:.0f}, {claw_y_mm:.0f}) "
        f"claw_dist={claw_distance_mm:.0f} mm "
        f"heading_error={heading_error_deg:.1f} deg "
        f"cmd=({forward_speed_mm_s:.0f} mm/s, {turn_rate_deg_s:.1f} deg/s)"
    )

    return False
