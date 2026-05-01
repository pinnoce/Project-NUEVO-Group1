"""
sensor_verification.py — Lidar + GPS + pure-pursuit UI verification run.

Purpose
-------
Drive a 2-grid-cell square path while printing live sensor diagnostics so you
can simultaneously verify:
  1. Lidar config    — obstacle count printed each tick; cloud appears on UI.
  2. GPS setup       — tag detection, arena-frame fix, and gps_active flag.
  3. UI plots        — fused trail (blue), odometry trail (green), GPS cross
                       (yellow), and lidar cloud all update during the run.

HOW TO USE
----------
1. Copy to main.py:
       cp examples/sensor_verification.py main.py

2. Tune the constants below (robot hardware + GPS calibration).

3. Restart the robot node:
       ros2 run robot robot

4. Open the UI → RPi Sensors → World Map.

5. Press BTN_1 to start the path.  Watch:
     - Console: GPS/lidar diagnostics print every second.
     - World Map canvas: fused trail (blue) and odometry trail (green) diverge;
       GPS cross (yellow) appears when the tag is visible; lidar cloud updates.

6. Press BTN_2 at any time to cancel and return to IDLE.
   Press BTN_1 again from IDLE to run the path again.

GPS CALIBRATION NOTE
--------------------
`GPS_OFFSET_X_MM` and `GPS_OFFSET_Y_MM` translate raw ArUco detections
(metres, camera frame) into arena-frame millimetres.  Measure once:
  - Drive the robot to a point with known arena coordinates (x_known, y_known).
  - Read the raw GPS fix: (gps_x * 1000, gps_y * 1000).
  - offset = (x_known − gps_x*1000, y_known − gps_y*1000).
The defaults (GRID_MM/2, GRID_MM*2.5) are a reasonable first guess based on
the venue layout — update them once you have a measured fix.
"""

from __future__ import annotations
import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit


# ── Robot hardware ─────────────────────────────────────────────────────────────
POSITION_UNIT            = Unit.MM
WHEEL_DIAMETER           = 74.0    # mm
WHEEL_BASE               = 333.0   # mm
INITIAL_THETA            = 90.0    # degrees — robot faces +Y at start

LEFT_WHEEL_MOTOR         = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED  = False
RIGHT_WHEEL_MOTOR        = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ── GPS / ArUco tag ────────────────────────────────────────────────────────────
# Venue: 5×6 grid, 610 mm cells, origin at centre of bottom edge of bottom-left cell.
GRID_MM          = 610.0
GPS_TAG_ID       = -1       # -1 = accept any tag; set to your actual tag ID
GPS_OFFSET_X_MM  = GRID_MM / 2       # 305 mm — adjust after first measured fix
GPS_OFFSET_Y_MM  = GRID_MM * 2.5     # 1525 mm — adjust after first measured fix

# ── Pure-pursuit path ──────────────────────────────────────────────────────────
# A 2-grid-cell square, centred on the odometry origin (wherever you place the robot).
# The robot starts at (0, 0) after reset and returns there at the end.
_S = GRID_MM * 2   # 1220 mm side length
PATH = [
    (0.0,  0.0),
    (_S,   0.0),
    (_S,   _S),
    (0.0,  _S),
    (0.0,  0.0),
]

VELOCITY           = 150.0   # mm/s
LOOKAHEAD          = 120.0   # mm
TOLERANCE          = 30.0    # mm  — final waypoint stop threshold
ADVANCE_RADIUS     = 80.0    # mm  — intermediate waypoint drop radius
MAX_ANGULAR_RAD_S  = 1.5     # rad/s

# ── Diagnostics ────────────────────────────────────────────────────────────────
DIAG_INTERVAL_S = 1.0   # how often to print the sensor status line


# ──────────────────────────────────────────────────────────────────────────────

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.set_tracked_tag_id(GPS_TAG_ID)
    robot.set_gps_offset(GPS_OFFSET_X_MM, GPS_OFFSET_Y_MM)


def _print_diagnostics(robot: Robot, label: str) -> None:
    """Print a one-line sensor status for GPS, lidar, and fused pose."""
    fx, fy, ftheta = robot.get_fused_pose()
    rx, ry, rtheta = robot.get_pose()
    gps_ok  = robot.is_gps_active()
    n_obs   = len(robot.get_obstacles())

    print(
        f"[{label}] "
        f"fused=({fx:.0f}, {fy:.0f}, {ftheta:.1f}°)  "
        f"odom=({rx:.0f}, {ry:.0f})  "
        f"GPS={'OK' if gps_ok else '--'}  "
        f"lidar_pts={n_obs}"
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state       = "INIT"
    drive_handle = None
    last_diag_t  = 0.0

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        now = time.monotonic()

        # ── INIT ──────────────────────────────────────────────────────────────
        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.reset_odometry()
            robot.wait_for_pose_update(timeout=0.5)
            robot.set_led(LED.ORANGE, 200)
            print("[INIT] Ready — press BTN_1 to start path, BTN_2 to cancel")
            print(f"       GPS tag: {'any' if GPS_TAG_ID == -1 else GPS_TAG_ID}  "
                  f"offset=({GPS_OFFSET_X_MM:.0f}, {GPS_OFFSET_Y_MM:.0f}) mm")
            print(f"       Path: {len(PATH)} waypoints, side={_S:.0f} mm")
            state = "IDLE"

        # ── IDLE ──────────────────────────────────────────────────────────────
        elif state == "IDLE":
            if now - last_diag_t >= DIAG_INTERVAL_S:
                _print_diagnostics(robot, "IDLE")
                last_diag_t = now

            if robot.was_button_pressed(Button.BTN_1):
                robot.set_led(LED.GREEN, 200)
                robot.set_led(LED.ORANGE, 0)
                print(f"[IDLE→MOVING] Starting {len(PATH)}-waypoint path")
                drive_handle = robot.purepursuit_follow_path(
                    waypoints=PATH,
                    velocity=VELOCITY,
                    lookahead=LOOKAHEAD,
                    tolerance=TOLERANCE,
                    advance_radius=ADVANCE_RADIUS,
                    max_angular_rad_s=MAX_ANGULAR_RAD_S,
                    blocking=False,
                )
                state = "MOVING"

        # ── MOVING ────────────────────────────────────────────────────────────
        elif state == "MOVING":
            if now - last_diag_t >= DIAG_INTERVAL_S:
                _print_diagnostics(robot, "MOVING")
                last_diag_t = now

            if robot.get_button(Button.BTN_2):
                drive_handle.cancel()
                drive_handle.wait(timeout=1.0)
                drive_handle = None
                robot.stop()
                robot.set_led(LED.ORANGE, 200)
                robot.set_led(LED.GREEN, 0)
                print("[MOVING→IDLE] Cancelled — press BTN_1 to restart")
                state = "IDLE"

            elif drive_handle is not None and drive_handle.is_finished():
                drive_handle = None
                robot.stop()
                robot.set_led(LED.ORANGE, 200)
                robot.set_led(LED.GREEN, 0)
                _print_diagnostics(robot, "DONE")
                print("[MOVING→IDLE] Path complete — press BTN_1 to run again")
                state = "IDLE"

        # ── Tick-rate control (do not modify) ─────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
