import time

from robot.hardware_map import (
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    WHEEL_BASE,
    WHEEL_DIAMETER,
    Button,
    LED,
)
from robot.robot import FirmwareState, Robot
from robot.scripted_navigation import drive_scripted_motion, reset_scripted_motion


# Change this to "TEST_FORWARD" if you just want a simple first test.
TEST_SEGMENT = "BURGER_NO_PICKUP"


def configure_robot(robot: Robot) -> None:
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


def start_robot(robot: Robot) -> None:
    current = robot.get_state()

    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()

    robot.set_state(FirmwareState.RUNNING)


def stop_robot(robot: Robot) -> None:
    robot.stop()
    robot.set_state(FirmwareState.IDLE)


def run(robot: Robot) -> None:
    configure_robot(robot)
    reset_scripted_motion()

    state = "WAIT_TO_START"

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("[TEST SCRIPTED] Press BTN_1 to start.")
    print("[TEST SCRIPTED] Press BTN_1 again while running to stop/end.")
    print(f"[TEST SCRIPTED] selected segment: {TEST_SEGMENT}")

    while True:
        if state == "WAIT_TO_START":
            robot.stop()
            robot.set_led(LED.ORANGE, 200)

            if robot.was_button_pressed(Button.BTN_1):
                start_robot(robot)
                reset_scripted_motion()
                print(f"[TEST SCRIPTED] running segment: {TEST_SEGMENT}")
                state = "RUNNING"

        elif state == "RUNNING":
            robot.set_led(LED.GREEN, 200)

            if robot.was_button_pressed(Button.BTN_1):
                print("[TEST SCRIPTED] BTN_1 pressed. Stopping sequence.")
                stop_robot(robot)
                reset_scripted_motion()
                state = "DONE"

            else:
                reached = drive_scripted_motion(robot, TEST_SEGMENT)

                if reached:
                    robot.stop()
                    print("[TEST SCRIPTED] segment complete")
                    stop_robot(robot)
                    state = "DONE"

        elif state == "DONE":
            robot.stop()
            robot.set_led(LED.RED, 200)
            print("[TEST SCRIPTED] done. Press Ctrl+C to exit, or restart the node.")
            break

        next_tick += period
        sleep_s = next_tick - time.monotonic()

        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
