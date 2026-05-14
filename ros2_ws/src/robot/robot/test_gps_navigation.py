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
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)

from robot.robot import FirmwareState, Robot
from robot.gps_navigation import drive_to_world_point_gps


# ---------------------------------------------------------------------------
# Change these for your actual course/test
# ---------------------------------------------------------------------------

# Replace this with the ArUco marker ID mounted on your rover.
TAG_ID = 19

# Pick a safe nearby world-frame test target.
# Units: mm.
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

    robot.enable_gps()
    robot.set_tracked_tag_id(TAG_ID)
    robot.set_tag_body_offset(
        TAG_BODY_OFFSET_X_MM,
        TAG_BODY_OFFSET_Y_MM,
    )

    print(f"[TEST GPS] tracking ArUco tag ID: {TAG_ID}")


def start_robot(robot: Robot) -> None:
    current = robot.get_state()

    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()

    robot.set_state(FirmwareState.RUNNING)


def print_pose_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()

    if robot.has_fused_pose():
        fused_pose = robot.get_fused_pose()

        if fused_pose is not None:
            fx, fy, ftheta = fused_pose

            print(
                f"[TEST GPS] "
                f"odom=({ox:6.0f}, {oy:6.0f}) mm theta={otheta:5.1f} deg | "
                f"fused=({fx:6.0f}, {fy:6.0f}) mm theta={ftheta:5.1f} deg | "
                f"gps_fresh={robot.is_gps_active()}"
            )
            return

    print(
        f"[TEST GPS] "
        f"odom=({ox:6.0f}, {oy:6.0f}) mm theta={otheta:5.1f} deg | "
        f"gps_fresh={robot.is_gps_active()}"
    )


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    print(
        f"[TEST GPS] driving to target "
        f"({TEST_TARGET_X_MM:.0f}, {TEST_TARGET_Y_MM:.0f}) mm"
    )
    print("[TEST GPS] waiting for valid ArUco/GPS pose")
    print("[TEST GPS] press Ctrl+C to stop early")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    last_status_print = 0.0

    while True:
        now = time.monotonic()

        if now - last_status_print >= 1.0:
            print_pose_status(robot)
            last_status_print = now

        reached = drive_to_world_point_gps(
            robot,
            TEST_TARGET_X_MM,
            TEST_TARGET_Y_MM,
        )

        if reached:
            robot.stop()
            print("[TEST GPS] target reached")
            break

        next_tick += period
        sleep_s = next_tick - time.monotonic()

        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = Node("test_gps_navigation")
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


if __name__ == "__main__":
    main()
