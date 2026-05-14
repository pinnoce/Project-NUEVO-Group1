from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node

from robot.gps_navigation import (
    AXIS_TARGET_TOLERANCE_MM,
    GPS_DROPOFF_TARGET,
    GPS_OBJECT_TARGETS,
    GpsNavigationMission,
    GpsTarget,
    align_to_heading_gps,
    drive_axis_gps,
    drive_to_world_point_gps,
    get_navigation_pose,
    make_axis_aligned_goal,
)
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

# =============================================================================
# Standalone GPS-navigation-only test
# =============================================================================
# Run from inside the ROS2 container after starting the bridge node:
#   cd /ros2_ws/src/robot
#   python3 -m robot.test_gps_navigation
#
# This file does NOT use main.py and does NOT command the claw.
# =============================================================================

START_STOP_BUTTON_ID = int(Button.BTN_1)
BUTTON_RELEASE_TIMEOUT_S = 10.0

# Options:
#   "FULL_NAVIGATION_MISSION" -> visit patty first, then buns, then dropoff; no claw motion
#   "SINGLE_TARGET"           -> drive to TEST_TARGET only; no claw motion
TEST_MODE = "FULL_NAVIGATION_MISSION"

# Options for SINGLE_TARGET:
#   "XY_FIRST" -> move X first, then Y
#   "DIRECT"   -> direct/diagonal drive; useful only for quick debugging
SINGLE_TARGET_ROUTE_MODE = "XY_FIRST"
TEST_TARGET_NAME = "single_test_target"
TEST_TARGET_X_MM = 500.0
TEST_TARGET_Y_MM = 300.0


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
            print(f"[TEST GPS] BTN_{button_id} still held; continuing anyway")
            break
        time.sleep(0.02)

    # Clear any edge created by the start press so the next press becomes stop.
    robot.was_button_pressed(button_id, consume=True)


def wait_for_start_button(robot: Robot, button_id: int = START_STOP_BUTTON_ID) -> None:
    """Block here until BTN_1 is pressed. The rover will not move before this."""
    robot.stop()
    print(f"[TEST GPS] Ready. Press BTN_{button_id} to START the GPS navigation test.")
    robot.wait_for_button(button_id)
    wait_until_button_released(robot, button_id)
    print(f"[TEST GPS] BTN_{button_id} pressed. Starting mission. Press BTN_{button_id} again to STOP.")


def stop_button_pressed(robot: Robot, button_id: int = START_STOP_BUTTON_ID) -> bool:
    """Return True once when BTN_1 is pressed during the mission."""
    if robot.was_button_pressed(button_id, consume=True):
        robot.stop()
        print(f"[TEST GPS] BTN_{button_id} pressed. Stopping mission.")
        return True
    return False


def print_pose_status(robot: Robot) -> None:
    pose = get_navigation_pose(robot)
    if pose is None:
        print("[TEST GPS] pose unavailable; make sure camera/GPS/ArUco pose is active")
        return

    x_mm, y_mm, theta_deg = pose
    print(f"[TEST GPS] pose x={x_mm:.0f} mm, y={y_mm:.0f} mm, theta={theta_deg:.1f} deg")


def run_single_target_direct_test(robot: Robot) -> None:
    print(f"[TEST GPS] DIRECT driving to target ({TEST_TARGET_X_MM:.0f}, {TEST_TARGET_Y_MM:.0f}) mm")
    print("[TEST GPS] No claw commands will be sent. Press BTN_1 to stop early, or Ctrl+C as a backup.")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    last_status_print = 0.0

    while True:
        if stop_button_pressed(robot):
            break

        now = time.monotonic()
        if now - last_status_print >= 1.0:
            print_pose_status(robot)
            last_status_print = now

        reached = drive_to_world_point_gps(robot, TEST_TARGET_X_MM, TEST_TARGET_Y_MM)
        if reached:
            robot.stop()
            print("[TEST GPS] direct target reached; no pickup command sent")
            break

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


def run_single_target_xy_first_test(robot: Robot) -> None:
    print(f"[TEST GPS] X-first/Y-second driving to target ({TEST_TARGET_X_MM:.0f}, {TEST_TARGET_Y_MM:.0f}) mm")
    print("[TEST GPS] No claw commands will be sent. Press BTN_1 to stop early, or Ctrl+C as a backup.")

    target = GpsTarget(TEST_TARGET_NAME, TEST_TARGET_X_MM, TEST_TARGET_Y_MM)
    goal = None
    phase = "PREPARE"

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    last_status_print = 0.0

    while True:
        if stop_button_pressed(robot):
            break

        now = time.monotonic()
        if now - last_status_print >= 1.0:
            print_pose_status(robot)
            last_status_print = now

        if phase == "PREPARE":
            goal = make_axis_aligned_goal(robot, target)
            if goal is not None:
                phase = "X"

        elif phase == "X":
            assert goal is not None
            if drive_axis_gps(robot, "X", goal.center_x_mm, AXIS_TARGET_TOLERANCE_MM):
                phase = "Y"

        elif phase == "Y":
            assert goal is not None
            if drive_axis_gps(robot, "Y", goal.center_y_mm, goal.tolerance_mm):
                phase = "ALIGN"

        elif phase == "ALIGN":
            assert goal is not None
            if align_to_heading_gps(robot, goal.final_heading_deg):
                robot.stop()
                print("[TEST GPS] X-first/Y-second target reached and aligned; no pickup command sent")
                break

        else:
            raise RuntimeError(f"Unknown single-target phase: {phase}")

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


def run_single_target_test(robot: Robot) -> None:
    if SINGLE_TARGET_ROUTE_MODE == "XY_FIRST":
        run_single_target_xy_first_test(robot)
    elif SINGLE_TARGET_ROUTE_MODE == "DIRECT":
        run_single_target_direct_test(robot)
    else:
        raise ValueError(f"Unknown SINGLE_TARGET_ROUTE_MODE: {SINGLE_TARGET_ROUTE_MODE}")


def run_full_navigation_mission(robot: Robot) -> None:
    print("[TEST GPS] Running GPS X-first/Y-second navigation-only mission: patty first, then buns.")
    print("[TEST GPS] No claw commands will be sent.")
    print("[TEST GPS] Current target coordinates:")
    for target in GPS_OBJECT_TARGETS:
        print(f"  - {target.name}: ({target.x_mm:.0f}, {target.y_mm:.0f}) mm")
    print(f"  - dropoff: ({GPS_DROPOFF_TARGET.x_mm:.0f}, {GPS_DROPOFF_TARGET.y_mm:.0f}) mm")
    print("[TEST GPS] Press BTN_1 to stop early, or Ctrl+C as a backup.")

    if get_navigation_pose(robot) is None:
        print("[TEST GPS] No fused pose yet. Make sure the camera sees the rover ArUco tag.")

    mission = GpsNavigationMission()
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    last_status_print = 0.0

    while True:
        if stop_button_pressed(robot):
            break

        now = time.monotonic()
        if now - last_status_print >= 1.0:
            print_pose_status(robot)
            last_status_print = now

        if mission.update(robot):
            robot.stop()
            print("[TEST GPS] GPS navigation-only mission complete.")
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
    wait_for_start_button(robot)

    if TEST_MODE == "SINGLE_TARGET":
        run_single_target_test(robot)
    elif TEST_MODE == "FULL_NAVIGATION_MISSION":
        run_full_navigation_mission(robot)
    else:
        raise ValueError(f"Unknown TEST_MODE: {TEST_MODE}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("test_gps_navigation")
    spin_thread = start_spin_thread(node)
    robot = Robot(node)

    try:
        run(robot)
    except KeyboardInterrupt:
        print("[TEST GPS] interrupted; stopping robot")
    finally:
        try:
            robot.stop()
            robot.shutdown()
        except Exception as exc:
            print(f"[TEST GPS] shutdown error: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
