"""
traffic_light_leds.py - vision-triggered LED control
====================================================
This example reads traffic-light detections from the vision node and uses the
Robot API to light the matching onboard LED.

HOW TO RUN
----------
Start the vision node in another terminal:

    ros2 run vision vision_node

Then copy this file over main.py and restart the robot node:

    cp examples/traffic_light_leds.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
If a red traffic light is detected, the red LED turns on.
If a green traffic light is detected, the green LED turns on.
If no new red/green traffic light is seen for 2 seconds, all LEDs turn off.

WHAT THIS TEACHES
-----------------
1. Reading vision results through the Robot API
2. Finding a specific detected class and reading its attributes
3. Holding an output for a short time without blocking the FSM loop
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


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

ENABLE_LIDAR = False
ENABLE_GPS = False
TAG_ID = 19

TURN_DEGREES = 90.0
FORWARD_DISTANCE_MM = 500.0
DRIVE_VELOCITY_MM_S = 200.0
DRIVE_TOLERANCE_MM = 20.0
TURN_TOLERANCE_DEG = 3.0

LED_BRIGHTNESS = 255
LIGHT_HOLD_SEC = 2.0
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50
GREEN_HOLD_SEC = 10.0  # keep driving this many seconds after last green detection


# ---------------------------------------------------------------------------
# Main Helpers
# ---------------------------------------------------------------------------

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
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

# ---------------------------------------------------------------------------
# Specific Helpers
# ---------------------------------------------------------------------------

def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)

def show_traffic_light_color(robot: Robot, color: str) -> None:
    if color == "red":
        robot.set_led(LED.RED, LED_BRIGHTNESS)
        robot.set_led(LED.GREEN, 0)
    elif color == "green":
        robot.set_led(LED.RED, 0)
        robot.set_led(LED.GREEN, LED_BRIGHTNESS)

def find_traffic_light_color(robot: Robot) -> str | None:
    """Return the best recent red/green traffic-light result, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color = None
    best_confidence = -1.0

    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue

        attributes = detection.get("attributes", {})
        color_attribute = attributes.get("color", {})
        color = color_attribute.get("value")
        if color not in ("red", "green"):
            continue

        if confidence > best_confidence:
            best_confidence = confidence
            best_color = str(color)

    return best_color
def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)

def cancel_motion(handle) -> None:
    if handle is None:
        return
    handle.cancel()
    handle.wait(timeout=1.0)

def print_status(robot: Robot, label: str) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    if ENABLE_GPS and robot.has_fused_pose():
        fused_pose = robot.get_fused_pose()
        if fused_pose is not None:
            fx, fy, ftheta = fused_pose
            print(
                f"  [{label}] odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°  |  "
                f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ={ftheta:5.1f}°  "
                f"(gps_fresh={robot.is_gps_active()})"
            )
            return
    print(f"  [{label}] odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")

# ---------------------------------------------------------------------------
# run() - entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    lights_off_at = 0.0
    last_shown_color = None
    green_last_seen_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # -- INIT -----------------------------------------------------------
        if state == "INIT":
            start_robot(robot)
            dim_all_leds(robot)
            print("[FSM] WATCHING - show a red or green traffic light")
            state = "WATCHING"

        # -- WATCHING -------------------------------------------------------
        elif state == "WATCHING":
            now = time.monotonic()

            traffic_light_color = find_traffic_light_color(robot)

            if traffic_light_color == "green":
                if last_shown_color != "green":
                    print("[VISION] traffic light: green — driving forward")
                last_shown_color = "green"
                green_last_seen_at = now
                lights_off_at = now + LIGHT_HOLD_SEC
                show_traffic_light_color(robot, "green")
                robot.set_velocity(DRIVE_VELOCITY_MM_S, 0.0)

            elif traffic_light_color == "red":
                if last_shown_color != "red":
                    print("[VISION] traffic light: red — stopped")
                last_shown_color = "red"
                green_last_seen_at = 0.0
                lights_off_at = now + LIGHT_HOLD_SEC
                show_traffic_light_color(robot, "red")
                robot.stop()

            elif green_last_seen_at > 0.0 and (now - green_last_seen_at) < GREEN_HOLD_SEC:
                # Green was seen recently — keep driving even if it just left the frame
                lights_off_at = now + LIGHT_HOLD_SEC
                robot.set_velocity(DRIVE_VELOCITY_MM_S, 0.0)

            else:
                # No green seen for GREEN_HOLD_SEC (camera covered, or never triggered)
                robot.stop()
                if lights_off_at > 0.0 and now >= lights_off_at:
                    dim_all_leds(robot)
                    lights_off_at = 0.0
                    if last_shown_color is not None:
                        print("[VISION] no recent red/green light - LEDs off")
                    last_shown_color = None

        # -- Tick-rate control ---------------------------------------------
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()