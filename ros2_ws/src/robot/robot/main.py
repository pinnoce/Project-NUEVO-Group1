"""
main.py — Look-left → watch-for-green → pure-pursuit path following
==================================================================
On BTN_1 the robot:
  1. Turns LOOK_LEFT_OFFSET_DEG (30°) CCW from its initial heading.
  2. Watches the camera for a green traffic light.
  3. Turns back to the initial heading.
  4. Follows PATH_CONTROL_POINTS with pure pursuit.

BTN_2 cancels at any stage and returns to IDLE.
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline  # noqa: F401 - optional helper for students


# ---------------------------------------------------------------------------
# Sensor toggles — set True if the corresponding node is running
# Hardware calibration (wheel geometry, lidar mount, tag offset) lives in
# robot/hardware_map.py.
# ---------------------------------------------------------------------------

ENABLE_LIDAR = False
ENABLE_GPS   = False

TAG_ID = 15  # IMPORTANT: set to the ArUco marker ID on your robot


# ---------------------------------------------------------------------------
# Look-for-green configuration
# ---------------------------------------------------------------------------

LOOK_LEFT_OFFSET_DEG          = 35.0   # CCW offset from INITIAL_THETA_DEG
TURN_TOLERANCE_DEG            = 3.0
LOOK_TURN_MAX_ANGULAR_RAD_S   = 0.4    # slow look-left turn (~23°/s)
RETURN_TURN_MAX_ANGULAR_RAD_S = 1.0    # default fast cap for turn-back
VISION_STALE_SEC              = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE  = 0.50


# ---------------------------------------------------------------------------
# GPS tuning (only used when ENABLE_GPS = True)
#
# GPS_POSITION_ALPHA     — how strongly each GPS fix pulls the fused position.
#                          0.05 = smooth/slow, 0.10 = default, 0.30 = aggressive
#
# ENABLE_GPS_TANGENT_HEADING — derive heading from GPS trajectory direction.
#                          False = pure odometry heading (default).
#
# GPS_TANGENT_ALPHA      — how strongly GPS tangent corrects odometry heading.
#                          0.05 = gentle, 0.15 = default, 0.30 = aggressive
#
# GPS_TANGENT_MIN_DISPLACEMENT_MM — travel required before accepting a new
#                          heading sample. 100 = responsive, 200 = default,
#                          400 = noise-robust (for jittery GPS)
#
# To tune: watch θ_odom vs θ_fused in the status output while running.
# ---------------------------------------------------------------------------

GPS_POSITION_ALPHA           = 0.30
ENABLE_GPS_TANGENT_HEADING   = True
GPS_TANGENT_ALPHA            = 0.25
GPS_TANGENT_MIN_DISPLACEMENT_MM = 50.0


# ---------------------------------------------------------------------------
# Pure pursuit configuration
# ---------------------------------------------------------------------------

PATH_CONTROL_POINTS = [
    (0.0, 3350.0), # starting from cross (straight then turn on to ramp lane)
    (410.0, 3350.0),
    (410.0, 630.0),
    (1210.0, 630.0),
    (1210.0, 3350.0),
    (1710.0, 3350.0),
    (1710.0, 600.0),
]

# Optional: densify long segments for smoother tracking.
PATH_CONTROL_POINTS = densify_polyline(PATH_CONTROL_POINTS, spacing=50.0)

VELOCITY_MM_S      = 100.0
LOOKAHEAD_MM       = 120.0
TOLERANCE_MM       = 25.0
ADVANCE_RADIUS_MM  = 80.0
MAX_ANGULAR_RAD_S  = 0.8

STATUS_PRINT_INTERVAL_S = 0.5

LOOK_HEADING_DEG = INITIAL_THETA_DEG + LOOK_LEFT_OFFSET_DEG


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(
            x_mm=LIDAR_MOUNT_X_MM,
            y_mm=LIDAR_MOUNT_Y_MM,
            theta_deg=LIDAR_MOUNT_THETA_DEG,
        )
        robot.set_lidar_filter(
            range_min_mm=LIDAR_RANGE_MIN_MM,
            range_max_mm=LIDAR_RANGE_MAX_MM,
            fov_deg=LIDAR_FOV_DEG,
        )
        robot.start_lidar_world_publisher()
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")
        if ENABLE_GPS_TANGENT_HEADING:
            robot.enable_gps_tangent_heading(
                alpha=GPS_TANGENT_ALPHA,
                min_displacement_mm=GPS_TANGENT_MIN_DISPLACEMENT_MM,
            )


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def show_watching_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.BLUE, 200)


def clear_watching_leds(robot: Robot) -> None:
    robot.set_led(LED.BLUE, 0)


def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    if ENABLE_GPS and robot.has_fused_pose():
        fx, fy, ftheta = robot.get_fused_pose()
        print(
            f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ_odom={otheta:5.1f}°  |  "
            f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ_fused={ftheta:5.1f}°  "
            f"gps={'fresh' if robot.is_gps_active() else 'stale'}"
        )
    else:
        print(f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")


def see_green_light(robot: Robot) -> bool:
    """True if a green traffic-light detection passes the confidence floor."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False
    for detection in robot.get_detections("traffic light"):
        if float(detection["confidence"]) < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue
        color = detection.get("attributes", {}).get("color", {}).get("value")
        if color == "green":
            return True
    return False


def start_turn_to(robot: Robot, angle_deg: float, max_angular_rad_s: float = 1.0):
    return robot.turn_to(
        angle_deg,
        blocking=False,
        tolerance_deg=TURN_TOLERANCE_DEG,
        max_angular_rad_s=max_angular_rad_s,
    )


def start_path(robot: Robot):
    return robot.purepursuit_follow_path(
        waypoints=PATH_CONTROL_POINTS,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        blocking=False,
    )


def cancel_handle(handle) -> None:
    if handle is None:
        return
    handle.cancel()
    handle.wait(timeout=1.0)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start (look-left → watch → path), BTN_2 to cancel")
            print(
                f"[CFG] velocity={VELOCITY_MM_S:.0f} mm/s  lookahead={LOOKAHEAD_MM:.0f} mm  "
                f"tolerance={TOLERANCE_MM:.0f} mm  advance_radius={ADVANCE_RADIUS_MM:.0f} mm"
            )
            print(
                f"[CFG] look_heading={LOOK_HEADING_DEG:.1f}°  "
                f"initial_heading={INITIAL_THETA_DEG:.1f}°  "
                f"min_green_conf={MIN_TRAFFIC_LIGHT_CONFIDENCE:.2f}"
            )
            if ENABLE_LIDAR:
                print(
                    f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f}, {LIDAR_MOUNT_Y_MM:.0f}) mm "
                    f"theta={LIDAR_MOUNT_THETA_DEG:.1f}° filter={LIDAR_RANGE_MIN_MM:.0f}-"
                    f"{LIDAR_RANGE_MAX_MM:.0f} mm fov={LIDAR_FOV_DEG}"
                )
            if ENABLE_GPS:
                print(
                    f"[CFG] gps tag_id={TAG_ID}  "
                    f"tag_body=({TAG_BODY_OFFSET_X_MM:.0f}, {TAG_BODY_OFFSET_Y_MM:.0f}) mm  "
                    f"position_alpha={GPS_POSITION_ALPHA:.2f}"
                )
                if ENABLE_GPS_TANGENT_HEADING:
                    print(
                        f"[CFG] heading=gps_tangent  "
                        f"alpha={GPS_TANGENT_ALPHA:.2f}  "
                        f"min_displacement={GPS_TANGENT_MIN_DISPLACEMENT_MM:.0f} mm"
                    )
                else:
                    print("[CFG] heading=imu")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_watching_leds(robot)
                print(f"[FSM] LOOK_LEFT — turning to {LOOK_HEADING_DEG:.1f}°")
                motion_handle = start_turn_to(
                    robot, LOOK_HEADING_DEG,
                    max_angular_rad_s=LOOK_TURN_MAX_ANGULAR_RAD_S,
                )
                state = "LOOK_LEFT"

        elif state == "LOOK_LEFT":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_handle(motion_handle)
                motion_handle = None
                robot.stop()
                clear_watching_leds(robot)
                show_idle_leds(robot)
                print("[FSM] IDLE — cancelled during look-left turn")
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print("[FSM] WATCHING — waiting for green traffic light")
                state = "WATCHING"

        elif state == "WATCHING":
            if robot.was_button_pressed(Button.BTN_2):
                clear_watching_leds(robot)
                show_idle_leds(robot)
                print("[FSM] IDLE — cancelled while watching")
                state = "IDLE"
            elif see_green_light(robot):
                print(f"[VISION] green light detected — turning back to {INITIAL_THETA_DEG:.1f}°")
                clear_watching_leds(robot)
                show_moving_leds(robot)
                motion_handle = start_turn_to(
                    robot, INITIAL_THETA_DEG,
                    max_angular_rad_s=RETURN_TURN_MAX_ANGULAR_RAD_S,
                )
                state = "TURN_BACK"

        elif state == "TURN_BACK":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_handle(motion_handle)
                motion_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — cancelled during turn-back")
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print(f"[FSM] MOVING — {len(PATH_CONTROL_POINTS)} waypoints")
                motion_handle = start_path(robot)
                last_status_print_at = now
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_handle(motion_handle)
                motion_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — path cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if motion_handle is not None and motion_handle.is_finished():
                    print("[FSM] DONE — path complete")
                    print_status(robot)
                    motion_handle = None
                    robot.stop()
                    show_idle_leds(robot)
                    print("[FSM] IDLE — press BTN_1 to run again")
                    state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
