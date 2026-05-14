from __future__ import annotations

import threading
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
from robot.scripted_navigation import (
    ScriptedNavigationMission,
    drive_scripted_motion,
    reset_all_scripted_state,
)

# =============================================================================
# Standalone scripted-navigation-only test
# =============================================================================
# Run from inside the ROS2 container after starting the bridge node:
#   cd /ros2_ws/src/robot
#   python3 -m robot.test_scripted_navigation
#
# This file does NOT use main.py and does NOT command the claw.
# =============================================================================

# Options:
#   "FULL_NAVIGATION_MISSION" -> visit 3 scripted targets, then dropoff; no claw
#   "SEGMENT_ONLY"            -> run TEST_SEGMENT only
TEST_MODE = "FULL_NAVIGATION_MISSION"
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


def start_spin_thread(node: Node) -> threading.Thread:
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    return spin_thread


def run_segment_test(robot: Robot) -> None:
    print(f"[TEST SCRIPTED] Running scripted segment: {TEST_SEGMENT}")
    print("[TEST SCRIPTED] No claw commands will be sent. Press Ctrl+C to stop early.")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        reached = drive_scripted_motion(robot, TEST_SEGMENT)
        if reached:
            robot.stop()
            print("[TEST SCRIPTED] Segment complete; no pickup command sent.")
            break

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


def run_full_navigation_mission(robot: Robot) -> None:
    print("[TEST SCRIPTED] Running scripted navigation-only mission.")
    print("[TEST SCRIPTED] Make sure the rover is physically at the scripted start pose.")
    print("[TEST SCRIPTED] No claw commands will be sent. Press Ctrl+C to stop early.")

    mission = ScriptedNavigationMission()
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if mission.update(robot):
            robot.stop()
            print("[TEST SCRIPTED] Scripted navigation-only mission complete.")
            break

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)
    reset_all_scripted_state()

    if TEST_MODE == "SEGMENT_ONLY":
        run_segment_test(robot)
    elif TEST_MODE == "FULL_NAVIGATION_MISSION":
        run_full_navigation_mission(robot)
    else:
        raise ValueError(f"Unknown TEST_MODE: {TEST_MODE}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("test_scripted_navigation")
    spin_thread = start_spin_thread(node)
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
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
