from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

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
)

from robot.robot import FirmwareState, Robot
from robot.scripted_navigation import drive_scripted_motion, reset_scripted_motion


# This must exist inside SCRIPTED_PATHS in scripted_navigation.py
TEST_SEGMENT = "TEST_FORWARD"


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


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)
    reset_scripted_motion()

    print(f"[TEST SCRIPTED] Running scripted segment: {TEST_SEGMENT}")
    print("[TEST SCRIPTED] Press Ctrl+C to stop early.")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        reached = drive_scripted_motion(robot, TEST_SEGMENT)

        if reached:
            robot.stop()
            print("[TEST SCRIPTED] Segment complete.")
            break

        next_tick += period
        sleep_s = next_tick - time.monotonic()

        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = Node("test_scripted_navigation")
    robot = Robot(node)

    try:
        run(robot)

    except KeyboardInterrupt:
        print("[TEST SCRIPTED] Interrupted. Stopping robot.")

    finally:
        try:
            robot.stop()
            robot.shutdown()
        except Exception as exc:
            print(f"[TEST SCRIPTED] Shutdown error: {exc}")

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
