import time

from robot.hardware_map import (
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
    Button,
    LED,
)
from robot.robot import FirmwareState, Robot
from robot.gps_navigation import drive_to_world_point_gps


# ---------------------------------------------------------------------------
# GPS / ArUco setup
# ---------------------------------------------------------------------------

# Change this to the ArUco tag ID mounted on your rover.
TAG_ID = 19


# ---------------------------------------------------------------------------
# Course target coordinates
# ---------------------------------------------------------------------------
# These are WORLD / ARENA coordinates in millimeters.
#
# The rover will go to the patty first, then the buns.
# It will NOT pick anything up. It only drives the claw to each target location.
#
# Replace these with your measured course coordinates.
PATTY_TARGET_MM = (500.0, 300.0)
BOTTOM_BUN_TARGET_MM = (300.0, 300.0)
TOP_BUN_TARGET_MM = (700.0, 300.0)
DROPOFF_TARGET_MM = (1000.0, 600.0)


TARGET_SEQUENCE = [
    ("PATTY", PATTY_TARGET_MM),
    ("BOTTOM_BUN", BOTTOM_BUN_TARGET_MM),
    ("TOP_BUN", TOP_BUN_TARGET_MM),
    ("DROPOFF", DROPOFF_TARGET_MM),
]


# Pause at each target so you can see that it reached the location.
TARGET_PAUSE_S = 1.0


def configure_robot(robot: Robot) -> None:
    """
    Configure the robot before motion.

    GPS offset stays zero.
    The tag offset is from the wheel midpoint.
    The claw offset is handled separately in gps_navigation.py.
    """
    robot.set_unit(POSITION_UNIT)

    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    robot.enable_gps()

    # Track the ArUco tag on your rover.
    robot.set_tracked_tag_id(TAG_ID)

    # Leave GPS/world offset at zero unless you measured a real global shift.
    robot.set_gps_offset(0.0, 0.0)

    # Tag location relative to midpoint between the two drive wheels.
    # These values come from hardware_map.py.
    robot.set_tag_body_offset(
        TAG_BODY_OFFSET_X_MM,
        TAG_BODY_OFFSET_Y_MM,
    )

    print(f"[TEST GPS] tracking ArUco tag {TAG_ID}")
    print(
        "[TEST GPS] tag body offset = "
        f"({TAG_BODY_OFFSET_X_MM:.0f}, {TAG_BODY_OFFSET_Y_MM:.0f}) mm"
    )


def start_robot(robot: Robot) -> None:
    """
    Put firmware into RUNNING mode.
    """
    current = robot.get_state()

    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()

    robot.set_state(FirmwareState.RUNNING)


def stop_robot(robot: Robot) -> None:
    """
    Stop the rover and put firmware back into IDLE.
    """
    robot.stop()
    robot.set_state(FirmwareState.IDLE)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "WAIT_TO_START"
    target_index = 0
    pause_started_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("[TEST GPS] Press BTN_1 to start.")
    print("[TEST GPS] Press BTN_1 again while running to stop/end.")

    while True:
        if state == "WAIT_TO_START":
            robot.stop()
            robot.set_led(LED.ORANGE, 200)

            if robot.was_button_pressed(Button.BTN_1):
                start_robot(robot)
                target_index = 0
                state = "DRIVING_TO_TARGET"

                name, target = TARGET_SEQUENCE[target_index]
                print(
                    f"[TEST GPS] starting sequence. First target: "
                    f"{name} at ({target[0]:.0f}, {target[1]:.0f}) mm"
                )

        elif state == "DRIVING_TO_TARGET":
            robot.set_led(LED.GREEN, 200)

            if robot.was_button_pressed(Button.BTN_1):
                print("[TEST GPS] BTN_1 pressed. Stopping sequence.")
                stop_robot(robot)
                state = "DONE"

            else:
                name, target = TARGET_SEQUENCE[target_index]
                target_x_mm, target_y_mm = target

                reached = drive_to_world_point_gps(
                    robot,
                    target_x_mm,
                    target_y_mm,
                )

                if reached:
                    robot.stop()
                    pause_started_at = time.monotonic()
                    print(f"[TEST GPS] reached {name}. Pausing briefly.")
                    state = "PAUSE_AT_TARGET"

        elif state == "PAUSE_AT_TARGET":
            robot.set_led(LED.BLUE, 200)

            if robot.was_button_pressed(Button.BTN_1):
                print("[TEST GPS] BTN_1 pressed. Stopping sequence.")
                stop_robot(robot)
                state = "DONE"

            elif time.monotonic() - pause_started_at >= TARGET_PAUSE_S:
                target_index += 1

                if target_index >= len(TARGET_SEQUENCE):
                    print("[TEST GPS] full target sequence complete.")
                    stop_robot(robot)
                    state = "DONE"

                else:
                    name, target = TARGET_SEQUENCE[target_index]
                    print(
                        f"[TEST GPS] next target: "
                        f"{name} at ({target[0]:.0f}, {target[1]:.0f}) mm"
                    )
                    state = "DRIVING_TO_TARGET"

        elif state == "DONE":
            robot.stop()
            robot.set_led(LED.RED, 200)
            print("[TEST GPS] done. Press Ctrl+C to exit, or restart the node.")
            break

        next_tick += period
        sleep_s = next_tick - time.monotonic()

        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
