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
)

from robot.robot import FirmwareState, Robot
from robot.gps_navigation import drive_to_world_point_gps


TAG_ID = 19

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
    robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)

    print(f"[TEST GPS] tracking ArUco tag {TAG_ID}")


def start_robot(robot: Robot) -> None:
    current = robot.get_state()

    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()

    robot.set_state(FirmwareState.RUNNING)


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    print(
        f"[TEST GPS] driving to "
        f"({TEST_TARGET_X_MM:.0f}, {TEST_TARGET_Y_MM:.0f}) mm"
    )

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
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
