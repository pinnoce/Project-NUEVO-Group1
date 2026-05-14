from __future__ import annotations

import math

from robot.robot import Robot


# ---------------------------------------------------------------------------
# GPS/ArUco navigation tuning parameters
# ---------------------------------------------------------------------------

# How close the rover must get to the target before saying "reached"
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


def wrap_angle_deg(angle: float) -> float:
    """
    Wrap an angle to the range [-180, 180] degrees.
    """
    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


def get_navigation_pose(robot: Robot) -> tuple[float, float, float] | None:
    """
    Return the rover pose from the existing Robot GPS/fused-pose system.

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


def drive_to_world_point_gps(
    robot: Robot,
    target_x_mm: float,
    target_y_mm: float,
) -> bool:
    """
    Drive toward a world-frame target using ArUco/GPS pose.

    Args:
        robot: Robot API object.
        target_x_mm: Target X coordinate in millimeters.
        target_y_mm: Target Y coordinate in millimeters.

    Returns:
        True if the target is reached.
        False if still driving or waiting for GPS.
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

    if distance_mm <= TARGET_TOLERANCE_MM:
        robot.stop()
        print("[GPS NAV] target reached")
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

    turn_rate_deg_s = max(
        -GPS_MAX_TURN_RATE_DEG_S,
        min(GPS_MAX_TURN_RATE_DEG_S, turn_rate_deg_s),
    )

    robot.set_velocity(forward_speed_mm_s, turn_rate_deg_s)

    print(
        f"[GPS NAV] target=({target_x_mm:.0f}, {target_y_mm:.0f}) "
        f"pose=({x_mm:.0f}, {y_mm:.0f}, {theta_deg:.1f}) "
        f"dist={distance_mm:.0f} mm "
        f"heading_error={heading_error_deg:.1f} deg "
        f"cmd=({forward_speed_mm_s:.0f} mm/s, {turn_rate_deg_s:.1f} deg/s)"
    )

    return False
