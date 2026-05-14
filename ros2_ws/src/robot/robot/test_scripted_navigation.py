from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node

from robot.hardware_map import (
    Button,
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

START_STOP_BUTTON_ID = int(Button.BTN_1)
BUTTON_RELEASE_TIMEOUT_S = 10.0

# Options:
#   "FULL_NAVIGATION_MISSION" -> visit patty first, then buns, then dropoff; no claw
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




def wait_until_button_released(robot: Robot, button_id: int = START_STOP_BUTTON_ID, timeout_s: float = BUTTON_RELEASE_TIMEOUT_S) -> None:
    """Wait for the start/stop button to be released so one press is not counted twice."""
    deadline = time.monotonic() + timeout_s
    while robot.get_button(button_id):
        if time.monotonic() > deadline:
            print(f"[TEST SCRIPTED] BTN_{button_id} still held; continuing anyway")
            break
        time.sleep(0.02)

    # Clear any edge created by the start press so the next press becomes stop.
    robot.was_button_pressed(button_id, consume=True)


def wait_for_start_button(robot: Robot, button_id: int = START_STOP_BUTTON_ID) -> None:
    """Block here until BTN_1 is pressed. The rover will not move before this."""
    robot.stop()
    print(f"[TEST SCRIPTED] Ready. Press BTN_{button_id} to START the scripted navigation test.")
    robot.wait_for_button(button_id)
    wait_until_button_released(robot, button_id)
    print(f"[TEST SCRIPTED] BTN_{button_id} pressed. Starting mission. Press BTN_{button_id} again to STOP.")


def stop_button_pressed(robot: Robot, button_id: int = START_STOP_BUTTON_ID) -> bool:
    """Return True once when BTN_1 is pressed during the mission."""
    if robot.was_button_pressed(button_id, consume=True):
        robot.stop()
        print(f"[TEST SCRIPTED] BTN_{button_id} pressed. Stopping mission.")
        return True
    return False


def run_segment_test(robot: Robot) -> None:
    print(f"[TEST SCRIPTED] Running scripted segment: {TEST_SEGMENT}")
    print("[TEST SCRIPTED] No claw commands will be sent. Press BTN_1 to stop early, or Ctrl+C as a backup.")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if stop_button_pressed(robot):
            break

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
    print("[TEST SCRIPTED] Running scripted navigation-only mission: patty first, then buns.")
    print("[TEST SCRIPTED] Make sure the rover is physically at the scripted start pose.")
    print("[TEST SCRIPTED] No claw commands will be sent. Press BTN_1 to stop early, or Ctrl+C as a backup.")

    mission = ScriptedNavigationMission()
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if stop_button_pressed(robot):
            break

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
    wait_for_start_button(robot)

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
