"""
main.py — Look-left → watch-for-green → pure-pursuit → burger assembly
======================================================================
On BTN_1 the robot:
  1. Turns LOOK_LEFT_OFFSET_DEG CCW from its initial heading.
  2. Watches the camera for a green traffic light.
  3. Turns back to the initial heading.
  4. Follows the INITIAL pure-pursuit path to the patty pickup location.
  5. Runs the burger-assembly sub-FSM (4 stops on the shelf to the left):
       patty pick → left-bun place → right-bun pick → stack place + pick.
     Each stop = turn left → lidar approach → manipulate → retreat → turn back.
  6. Follows the CONTINUATION pure-pursuit path with the finished burger.

BTN_2 cancels at any stage and returns to IDLE.

Body-frame convention used here: +x = forward, +y = left (matches the
docstring on robot.set_lidar_mount and the lidar transform pipeline).
"""

from __future__ import annotations

import math
import time

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    Motor,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    ServoChannel,
    StepMoveType,
    Stepper,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline  # noqa: F401 - optional helper for students


# ===========================================================================
# CONFIGURATION — tune the mission here.
# Everything you'd normally edit lives in this top section, grouped by feature.
# The implementation (helpers + FSM) is below the IMPLEMENTATION banner; you
# rarely need to touch it.
# ===========================================================================


# ---------------------------------------------------------------------------
# Sensor toggles — set True if the corresponding node is running
# Hardware calibration (wheel geometry, lidar mount, tag offset) lives in
# robot/hardware_map.py.
# ---------------------------------------------------------------------------

ENABLE_LIDAR = True   # required for shelf approach during burger assembly
ENABLE_GPS   = False

TAG_ID = 15  # IMPORTANT: set to the ArUco marker ID on your robot


# ---------------------------------------------------------------------------
# Look-for-green configuration
# ---------------------------------------------------------------------------

LOOK_LEFT_OFFSET_DEG          = 35.0   # CCW offset from INITIAL_THETA_DEG
TURN_TOLERANCE_DEG            = 3.0
LOOK_TURN_MAX_ANGULAR_RAD_S   = 0.4    # slow look-left turn (~23°/s)
RETURN_TURN_MAX_ANGULAR_RAD_S = 1.0    # default fast cap for turn-back
VISION_STALE_SEC              = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE  = 0.50   # YOLO box confidence on the traffic-light detection itself
MIN_GREEN_COLOR_CONFIDENCE    = 0.10   # green-blob-area / crop-area required to call it "green" — raise to reject ambient/reflected green


# ---------------------------------------------------------------------------
# GPS tuning (only used when ENABLE_GPS = True)
#
# GPS_POSITION_ALPHA     — how strongly each GPS fix pulls the fused position.
#                          0.05 = smooth/slow, 0.10 = default, 0.30 = aggressive
#
# ENABLE_GPS_TANGENT_HEADING — derive heading from GPS trajectory direction.
#                          False = pure odometry heading (default).
#
# GPS_TANGENT_ALPHA      — how strongly GPS tangent corrects odometry heading.
#                          0.05 = gentle, 0.15 = default, 0.30 = aggressive
#
# GPS_TANGENT_MIN_DISPLACEMENT_MM — travel required before accepting a new
#                          heading sample. 100 = responsive, 200 = default,
#                          400 = noise-robust (for jittery GPS)
#
# To tune: watch θ_odom vs θ_fused in the status output while running.
# ---------------------------------------------------------------------------

GPS_POSITION_ALPHA           = 0.30
ENABLE_GPS_TANGENT_HEADING   = True
GPS_TANGENT_ALPHA            = 0.25
GPS_TANGENT_MIN_DISPLACEMENT_MM = 50.0


# ---------------------------------------------------------------------------
# Pure pursuit configuration
# ---------------------------------------------------------------------------

PATH_CONTROL_POINTS = [
    (0.0, 1130.0), # go traight next to patty
    (0.0, 3400.0), # go straight to near end of lane
    (430.0, 3400.0), # turn and go to next lane
    (430.0, 500.0), # go straight (over ramp) to end of lane
    (1210.0, 500.0), # turn and go to next lane
    (1210.0, 3400.0), # go straight to near end of lane
    (1710.0, 3400.0),
    (1710.0, 500.0),
]

# Split index: waypoints[:PATTY_PICKUP_WAYPOINT_INDEX+1] = initial path to patty
#              waypoints[PATTY_PICKUP_WAYPOINT_INDEX+1:] = continuation after burger
PATTY_PICKUP_WAYPOINT_INDEX = 0

# Obstacle-avoidance segment (cones live here). The continuation after the
# burger is driven in three legs:
#   leg A  — pure pursuit from just past the patty up to AVOID_SEGMENT_START_INDEX
#   LAPF   — leashed APF from there to AVOID_SEGMENT_GOAL_INDEX, steering around cones
#   leg C  — pure pursuit through every remaining waypoint
# Both indices are into PATH_CONTROL_POINTS, which stays the single source of truth.
AVOID_SEGMENT_START_INDEX = 4   # (1210, 500)  — last waypoint pure-pursued before the cones
AVOID_SEGMENT_GOAL_INDEX  = 5   # (1210, 3400) — LAPF goal at the far end of the cones

INITIAL_PATH_CONTROL_POINTS = PATH_CONTROL_POINTS[: PATTY_PICKUP_WAYPOINT_INDEX + 1]
CONTINUATION_LEG_A_CONTROL_POINTS = PATH_CONTROL_POINTS[
    PATTY_PICKUP_WAYPOINT_INDEX + 1 : AVOID_SEGMENT_START_INDEX + 1
]
AVOID_GOAL_POINT = PATH_CONTROL_POINTS[AVOID_SEGMENT_GOAL_INDEX]
CONTINUATION_LEG_C_CONTROL_POINTS = PATH_CONTROL_POINTS[AVOID_SEGMENT_GOAL_INDEX + 1 :]

# Optional: densify long segments for smoother tracking. The LAPF goal is a
# single point, so it is never densified.
INITIAL_PATH_CONTROL_POINTS = densify_polyline(INITIAL_PATH_CONTROL_POINTS, spacing=50.0)
if CONTINUATION_LEG_A_CONTROL_POINTS:
    CONTINUATION_LEG_A_CONTROL_POINTS = densify_polyline(
        CONTINUATION_LEG_A_CONTROL_POINTS, spacing=50.0
    )
if CONTINUATION_LEG_C_CONTROL_POINTS:
    CONTINUATION_LEG_C_CONTROL_POINTS = densify_polyline(
        CONTINUATION_LEG_C_CONTROL_POINTS, spacing=50.0
    )

VELOCITY_MM_S      = 100.0
LOOKAHEAD_MM       = 120.0
TOLERANCE_MM       = 25.0
ADVANCE_RADIUS_MM  = 80.0
MAX_ANGULAR_RAD_S  = 0.8

STATUS_PRINT_INTERVAL_S = 0.5

LOOK_HEADING_DEG = INITIAL_THETA_DEG + LOOK_LEFT_OFFSET_DEG


# ---------------------------------------------------------------------------
# Obstacle avoidance (LAPF) configuration
#
# Used on the continuation leg from PATH_CONTROL_POINTS[AVOID_SEGMENT_START_INDEX]
# to [AVOID_SEGMENT_GOAL_INDEX]. LAPF drives toward AVOID_GOAL_POINT while a
# "virtual target" on a forward leash is pushed sideways by repulsion from
# tracked obstacle disks. Those disks are built automatically from the lidar
# scan (ENABLE_LIDAR must be True), so no extra sensor setup is needed.
#
# LAPF_LEASH_LENGTH_MM is drive-geometry dependent: ~400 mm for a front-wheel
# drive base, ~50 mm for a rear-wheel drive base. This robot is REAR-wheel
# drive, so the leash is short — retune if the chassis changes.
# ---------------------------------------------------------------------------

AVOID_VELOCITY_MM_S       = 60.0    # cautious forward speed through the cone field
AVOID_TOLERANCE_MM        = 50.0    # goal-reached tolerance at the LAPF goal
AVOID_MAX_ANGULAR_RAD_S   = 1.0

LAPF_LEASH_LENGTH_MM      = 50.0    # rear-wheel drive → short leash
LAPF_LEASH_HALF_ANGLE_DEG = 25.0    # forward cone the virtual target may steer within
LAPF_REPULSION_RANGE_MM   = 300.0   # surface air-gap at which a cone starts repelling
LAPF_REPULSION_GAIN       = 550.0
LAPF_ATTRACTION_GAIN      = 1.0
LAPF_TARGET_SPEED_MM_S    = 200.0   # virtual-target sim speed (NOT the robot's drive speed)
LAPF_FORCE_EMA_ALPHA      = 0.35
LAPF_INFLATION_MARGIN_MM  = 150.0   # extra radius added to each cone before repulsion


# ---------------------------------------------------------------------------
# Burger assembly configuration
#
# Layout assumption: the robot travels along its initial heading; the shelf is
# on its LEFT (CCW 90° from the travel direction). On the shelf, from the
# robot's perspective when facing it: left = bun #1, middle = patty,
# right = bun #2. Final stack lives on the LEFT bun's spot.
#
# All distances are in mm. All lift heights are in stepper steps measured
# RELATIVE to the lift's position at boot — which is taken to be the shelf
# surface (step 0 = gripper aligned with the shelf top).
#
# IMPORTANT: there is no limit switch on the lift shaft, so the stepper is
# NOT homed. Before powering on, park the lift so the gripper sits at the
# shelf surface — that physical position becomes step 0 for the run.
# Positive step values raise the lift; negative values lower it below the
# shelf (rarely needed).
# ---------------------------------------------------------------------------

ENABLE_BURGER_ASSEMBLY = True

# === Gripper servo ===
# Two close angles because the bun and patty have different diameters.
# Use the patty angle when gripping only a patty; use the bun angle for a bun
# alone or for any combined stack (bun+patty, bun+patty+bun).
GRIPPER_SERVO              = ServoChannel.CH_1
GRIPPER_OPEN_DEG           = 0.0
GRIPPER_CLOSE_PATTY_DEG    = 95.0    # tighter — patty has smaller diameter
GRIPPER_CLOSE_BUN_DEG      = 85.0    # looser — bun (or bun+patty stack) is wider
GRIPPER_SETTLE_S           = 0.8

# === Vertical lift (stepper on shaft) ===
LIFT_STEPPER             = Stepper.STEPPER_1
LIFT_DIR_INVERTED        = True   # firmware step counts positive = down; flip so positive LIFT_*_STEPS raise the lift
LIFT_RAISE_TO_CARRY_AT_STARTUP = True  # if True, lift moves to LIFT_CARRY_STEPS at start_robot
LIFT_MAX_VELOCITY        = 2000
LIFT_ACCELERATION        = 800
LIFT_MOVE_TIMEOUT_S      = 20.0

# Edit these to match your bun + patty geometry. Step 0 = shelf surface.
BUN_HEIGHT_STEPS         = 11000    # height of one bun, in stepper steps
PATTY_HEIGHT_STEPS       = 11000    # height of one patty, in stepper steps
LIFT_CARRY_STEPS         = 20000   # safe travel height above the shelf — clears the tallest item

# Per-action lift targets (override directly if the derived defaults don't fit).
# 0 = gripper at shelf surface; positive = raised above the shelf.
LIFT_PATTY_PICK_STEPS         = 0
LIFT_LEFT_BUN_TOP_PLACE_STEPS = BUN_HEIGHT_STEPS
LIFT_RIGHT_BUN_PICK_STEPS     = 0
LIFT_STACK_TOP_PLACE_STEPS    = BUN_HEIGHT_STEPS + PATTY_HEIGHT_STEPS
LIFT_STACK_PICK_STEPS         = 0

# === Lidar plot snapshot (debug viz) ===
# Writes a body-frame matplotlib PNG to LIDAR_PLOT_PATH at LIDAR_PLOT_HZ. Open
# the file with an auto-refreshing viewer (feh --reload, Eye of GNOME, VS Code
# preview) on the host to watch the lidar live. The plot shows obstacle points,
# the forward cone, and the standoff threshold so you can verify the shelf
# approach is reading what you expect.
ENABLE_LIDAR_PLOT       = True
LIDAR_PLOT_HZ           = 2.0
LIDAR_PLOT_PATH         = "/runtime_output/lidar/latest.png"
LIDAR_PLOT_XLIM_MM      = (-1500.0, 1500.0)
LIDAR_PLOT_YLIM_MM      = (-500.0, 2500.0)

# === Shelf approach (used after turning to face the shelf) ===
# Per-stop shelf-turn offset from travel_heading_deg (CCW positive). Tune each
# entry independently — odometry drift can make a single offset misalign by
# the 3rd or 4th shelf turn. Indices line up with BURGER_STOPS.
SHELF_TURN_OFFSETS_DEG = [
    98.0,   # 0: patty
    97.0,   # 1: left_bun
    97.0,   # 2: right_bun
    93.0,   # 3: stack
]
SHELF_TURN_MAX_ANGULAR_RAD_S = 0.6
SHELF_TURN_TOLERANCE_DEG     = 3.0
SHELF_APPROACH_VEL_MM_S      = 40.0    # slow forward speed while reading lidar
#
# SHELF_STANDOFF_MM is the body-frame forward distance (wheel midpoint to the
# nearest forward lidar return) at which the approach stops. The lidar lives
# LIDAR_MOUNT_X_MM forward of the wheel midpoint and only reports returns
# beyond LIDAR_FILTER_MIN_MM from itself, so the closest distance you can
# actually measure in body frame is:
#
#     min_measurable_body_mm = LIDAR_MOUNT_X_MM + LIDAR_FILTER_MIN_MM
#
# Keep SHELF_STANDOFF_MM above that floor. To get closer to the shelf, lower
# LIDAR_FILTER_MIN_MM (the RPLidar C1 reads down to ~50 mm).
#
LIDAR_FILTER_MIN_MM          = 25.0    # overrides hardware_map.LIDAR_RANGE_MIN_MM for this run
SHELF_STANDOFF_MM            = 300.0   # stop when nearest forward point ≤ this
SHELF_APPROACH_FOV_HALF_DEG  = 10.0    # half-angle of the forward cone to consider
SHELF_APPROACH_MAX_DIST_MM   = 300.0   # safety cap on forward advance per approach
SHELF_APPROACH_TIMEOUT_S     = 20.0
RETREAT_FROM_SHELF_MM        = 150.0   # back away after manipulation

# === Straight-line travel between stops (signed: + = forward, - = backward) ===
TRAVEL_PATTY_TO_LEFT_BUN_MM     = -152.0   # back along travel heading to left bun
TRAVEL_LEFT_BUN_TO_RIGHT_BUN_MM =  304.0   # forward to right bun
TRAVEL_RIGHT_BUN_TO_STACK_MM    = -304.0   # back to the stack spot (= left bun spot)

BURGER_TRAVEL_VEL_MM_S    = 80.0
BURGER_TRAVEL_TOL_MM      = 15.0
BURGER_TRAVEL_TIMEOUT_S   = 20.0

# === Stops list ===
# Each entry: (label, signed travel from previous stop in mm, [actions])
# Action: ("pick",  lift_steps, gripper_close_deg)  → lower-close-raise
#         ("place", lift_steps)                     → lower-open-raise  (always uses GRIPPER_OPEN_DEG)
BURGER_STOPS = [
    ("patty",     0.0,                              [("pick",  LIFT_PATTY_PICK_STEPS,      GRIPPER_CLOSE_PATTY_DEG)]),
    ("left_bun",  TRAVEL_PATTY_TO_LEFT_BUN_MM,      [("place", LIFT_LEFT_BUN_TOP_PLACE_STEPS)]),
    ("right_bun", TRAVEL_LEFT_BUN_TO_RIGHT_BUN_MM,  [("pick",  LIFT_RIGHT_BUN_PICK_STEPS,  GRIPPER_CLOSE_BUN_DEG)]),
    ("stack",     TRAVEL_RIGHT_BUN_TO_STACK_MM,     [
        ("place", LIFT_STACK_TOP_PLACE_STEPS),
        ("pick",  LIFT_STACK_PICK_STEPS, GRIPPER_CLOSE_BUN_DEG),   # full bun+patty+bun stack
    ]),
]


# ===========================================================================
# IMPLEMENTATION — logic below. Tune behavior via the constants above; you
# normally don't need to edit past this banner.
# ===========================================================================


# --- Lift state model (no limit switch — see the burger config header) ---
# Logical lift position in Python-tracked steps. Boot/park = 0 = shelf surface.
# Every move_lift_to() sends a RELATIVE delta keyed off this value, so the
# firmware's absolute step count (which has no homing reference) never matters.
_LIFT_LOGICAL_STEPS: int = 0


def lift_steps_signed(value: int) -> int:
    """Apply LIFT_DIR_INVERTED so positive LIFT_*_STEPS always means above the shelf."""
    return -int(value) if LIFT_DIR_INVERTED else int(value)


def move_lift_to(robot: Robot, target_steps: int, timeout: float = LIFT_MOVE_TIMEOUT_S) -> bool:
    """Drive the lift to a logical step target using a RELATIVE move.

    On success the tracked logical position is updated; on timeout it is left
    as-is so the caller can detect the failure and abort.
    """
    global _LIFT_LOGICAL_STEPS
    delta = int(target_steps) - _LIFT_LOGICAL_STEPS
    signed = lift_steps_signed(delta)
    expected_dir = "UP" if delta > 0 else ("DOWN" if delta < 0 else "no-op")
    print(
        f"[LIFT] move target={target_steps} current={_LIFT_LOGICAL_STEPS} "
        f"delta={delta:+d} firmware_steps={signed:+d} expected={expected_dir}"
    )
    if delta == 0:
        return True
    ok = robot.step_move(
        LIFT_STEPPER,
        steps=signed,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=timeout,
    )
    if ok:
        _LIFT_LOGICAL_STEPS = int(target_steps)
    return ok


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
            range_min_mm=LIDAR_FILTER_MIN_MM,
            range_max_mm=LIDAR_RANGE_MAX_MM,
            fov_deg=LIDAR_FOV_DEG,
        )
        robot.start_lidar_world_publisher()
        print("[sensor] lidar enabled — subscribing to /scan")
    elif ENABLE_BURGER_ASSEMBLY:
        print("[warn] burger assembly enabled but lidar disabled — "
              f"shelf approach will fall back to a fixed {SHELF_APPROACH_MAX_DIST_MM:.0f} mm forward move")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")
        if ENABLE_GPS_TANGENT_HEADING:
            robot.enable_gps_tangent_heading(
                alpha=GPS_TANGENT_ALPHA,
                min_displacement_mm=GPS_TANGENT_MIN_DISPLACEMENT_MM,
            )

    if ENABLE_BURGER_ASSEMBLY:
        robot.enable_servo(GRIPPER_SERVO)
        robot.step_set_config(
            LIFT_STEPPER,
            max_velocity=LIFT_MAX_VELOCITY,
            acceleration=LIFT_ACCELERATION,
        )


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

    if ENABLE_BURGER_ASSEMBLY and LIFT_RAISE_TO_CARRY_AT_STARTUP:
        prime_lift(robot)


def prime_lift(robot: Robot) -> bool:
    """
    Treat the lift's current physical position as step 0, raise it to the
    carry height, and set the gripper open. Assumes the lift was parked at
    the bottom of the shaft before power-on.
    """
    print("[BURGER] enabling lift stepper (no homing — using boot position as logical step 0)")
    robot.step_enable(LIFT_STEPPER)
    if not move_lift_to(robot, LIFT_CARRY_STEPS):
        print("[warn] lift failed to reach carry height at startup")
        robot.step_disable(LIFT_STEPPER)
        return False
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)
    return True


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    if ENABLE_BURGER_ASSEMBLY:
        # Re-enable the servo defensively — firmware disables all actuators on
        # any drop to IDLE/ESTOP, which would silently swallow set_servo.
        robot.enable_servo(GRIPPER_SERVO)
        robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def show_watching_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.BLUE, 200)


def clear_watching_leds(robot: Robot) -> None:
    robot.set_led(LED.BLUE, 0)


def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    if ENABLE_GPS and robot.has_fused_pose():
        fx, fy, ftheta = robot.get_fused_pose()
        print(
            f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ_odom={otheta:5.1f}°  |  "
            f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ_fused={ftheta:5.1f}°  "
            f"gps={'fresh' if robot.is_gps_active() else 'stale'}"
        )
    else:
        print(f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")


def see_green_light(robot: Robot) -> bool:
    """True if a green traffic-light detection passes both the YOLO and color floors."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False
    for detection in robot.get_detections("traffic light"):
        if float(detection["confidence"]) < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue
        color_attr = detection.get("attributes", {}).get("color", {})
        if color_attr.get("value") != "green":
            continue
        color_score = color_attr.get("score")
        if color_score is None or float(color_score) < MIN_GREEN_COLOR_CONFIDENCE:
            continue
        return True
    return False


def start_turn_to(robot: Robot, angle_deg: float, max_angular_rad_s: float = 1.0):
    return robot.turn_to(
        angle_deg,
        blocking=False,
        tolerance_deg=TURN_TOLERANCE_DEG,
        max_angular_rad_s=max_angular_rad_s,
    )


def start_path(robot: Robot, waypoints):
    return robot.purepursuit_follow_path(
        waypoints=waypoints,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        blocking=False,
    )


def cancel_handle(handle) -> None:
    if handle is None:
        return
    handle.cancel()
    handle.wait(timeout=1.0)


def start_lapf_goal(robot: Robot, goal_xy):
    """Start a non-blocking leashed-APF run toward goal_xy (world frame)."""
    return robot.lapf_to_goal(
        goal_xy[0],
        goal_xy[1],
        velocity=AVOID_VELOCITY_MM_S,
        tolerance=AVOID_TOLERANCE_MM,
        leash_length_mm=LAPF_LEASH_LENGTH_MM,
        repulsion_range_mm=LAPF_REPULSION_RANGE_MM,
        target_speed_mm_s=LAPF_TARGET_SPEED_MM_S,
        max_angular_rad_s=AVOID_MAX_ANGULAR_RAD_S,
        repulsion_gain=LAPF_REPULSION_GAIN,
        attraction_gain=LAPF_ATTRACTION_GAIN,
        force_ema_alpha=LAPF_FORCE_EMA_ALPHA,
        inflation_margin_mm=LAPF_INFLATION_MARGIN_MM,
        leash_half_angle_deg=LAPF_LEASH_HALF_ANGLE_DEG,
        blocking=False,
    )


def print_avoid_status(robot: Robot) -> None:
    """Status line for the LAPF leg: pose, leashed virtual target, tracked cones."""
    x, y, theta = robot.get_pose()
    vt = robot.get_virtual_target()
    tracks = robot.get_obstacle_tracks()
    vt_str = f"vt=({vt[0]:6.0f}, {vt[1]:6.0f}) mm" if vt is not None else "vt=(none)"
    print(
        f"  pos=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°  {vt_str}  "
        f"tracked_cones={len(tracks)}"
    )


# ---------------------------------------------------------------------------
# Burger assembly helpers
# ---------------------------------------------------------------------------

def closest_forward_obstacle_mm(robot: Robot) -> float:
    """
    Return the minimum forward distance to a lidar point in body frame,
    restricted to a narrow forward cone. Body frame: +x = forward, +y = left.

    Returns float('inf') when no obstacle is in range.
    """
    fov_half_rad = math.radians(SHELF_APPROACH_FOV_HALF_DEG)
    nearest = float("inf")
    for x_mm, y_mm in robot.get_obstacles():
        if x_mm <= 0.0:
            continue  # behind the robot
        angle = math.atan2(y_mm, x_mm)  # 0 = straight ahead, positive = left
        if abs(angle) > fov_half_rad:
            continue
        if x_mm < nearest:
            nearest = x_mm
    return nearest


# Reused figure/axes so we don't leak memory across snapshots.
_LIDAR_PLOT_FIG = None
_LIDAR_PLOT_AX = None


def save_lidar_plot(robot: Robot, path: str = LIDAR_PLOT_PATH, lapf_overlay: bool = False) -> bool:
    """
    Render the current lidar obstacle cloud in body frame to a PNG.
    Body frame: +x = forward, +y = left. The plot maps body +y onto matplotlib
    X (inverted) and body +x onto matplotlib Y, so the figure reads like a
    top-down view with the robot facing UP and its LEFT side on the LEFT.

    When lapf_overlay is True (the AVOID_LAPF leg), the shelf-approach
    annotations are replaced with the things LAPF actually reasons about:
    the tracked obstacle disks with their inflation/repulsion rings, the
    leashed virtual target, and the goal — all transformed world → body frame.
    """
    global _LIDAR_PLOT_FIG, _LIDAR_PLOT_AX
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
    except ImportError:
        return False

    if _LIDAR_PLOT_FIG is None:
        _LIDAR_PLOT_FIG, _LIDAR_PLOT_AX = plt.subplots(figsize=(6, 6))
    ax = _LIDAR_PLOT_AX
    ax.clear()

    obstacles = robot.get_obstacles()
    if obstacles:
        plot_xs = [p[1] for p in obstacles]  # matplotlib X = body y
        plot_ys = [p[0] for p in obstacles]  # matplotlib Y = body x
        ax.scatter(plot_xs, plot_ys, s=4, c="red", label=f"lidar ({len(obstacles)} pts)")

    # Robot body origin (wheel midpoint) and lidar mount.
    ax.scatter([0.0], [0.0], s=90, c="black", marker="s", label="robot body")
    ax.scatter([LIDAR_MOUNT_Y_MM], [LIDAR_MOUNT_X_MM], s=50, c="blue",
               marker="^", label="lidar mount")

    if lapf_overlay:
        # LAPF view: what the planner actually reasons about. Tracked disks,
        # the virtual target, and the goal live in WORLD frame, so transform
        # each into body frame before mapping to plot axes (plot X = body_y,
        # plot Y = body_x). body = R(-theta) · (world - pose).
        px, py, ptheta_deg = robot.get_pose()
        ptheta = math.radians(ptheta_deg)
        cos_t = math.cos(ptheta)
        sin_t = math.sin(ptheta)

        def world_to_body(wx, wy):
            dx = float(wx) - px
            dy = float(wy) - py
            body_x = cos_t * dx + sin_t * dy   # forward
            body_y = -sin_t * dx + cos_t * dy  # left
            return body_x, body_y

        first = True
        for track in robot.get_obstacle_tracks():
            bx, by = world_to_body(track["x"], track["y"])
            radius = float(track.get("radius", 0.0))
            # Disk centre at (plot X = by, plot Y = bx).
            ax.add_patch(Circle((by, bx), max(radius, 20.0), fill=True,
                                color="red", alpha=0.30,
                                label="tracked cone" if first else None))
            ax.add_patch(Circle((by, bx), radius + LAPF_INFLATION_MARGIN_MM,
                                fill=False, color="orange", ls="--", alpha=0.6,
                                label=f"inflation +{LAPF_INFLATION_MARGIN_MM:.0f}" if first else None))
            ax.add_patch(Circle((by, bx), radius + LAPF_INFLATION_MARGIN_MM + LAPF_REPULSION_RANGE_MM,
                                fill=False, color="gray", ls=":", alpha=0.4,
                                label=f"repulsion +{LAPF_REPULSION_RANGE_MM:.0f}" if first else None))
            first = False

        vt = robot.get_virtual_target()
        if vt is not None:
            vbx, vby = world_to_body(vt[0], vt[1])
            ax.scatter([vby], [vbx], s=80, c="magenta", marker="*",
                       zorder=6, label="virtual target")

        gbx, gby = world_to_body(AVOID_GOAL_POINT[0], AVOID_GOAL_POINT[1])
        ax.scatter([gby], [gbx], s=90, c="green", marker="X",
                   zorder=6, label="goal")

        title = "LAPF avoidance (body frame)"
    else:
        # Forward cone (the same arc closest_forward_obstacle_mm filters by).
        # Endpoint in body frame: (cos(±h), sin(±h)) · cone_len. Mapped to
        # matplotlib: X = body_y = cone_len * sin(±h), Y = body_x = cone_len * cos(±h).
        cone_len = max(LIDAR_PLOT_YLIM_MM[1], 1500.0)
        half_rad = math.radians(SHELF_APPROACH_FOV_HALF_DEG)
        ax.plot([0.0, cone_len * math.sin(-half_rad)],
                [0.0, cone_len * math.cos(-half_rad)],
                "g--", alpha=0.5)
        ax.plot([0.0, cone_len * math.sin(+half_rad)],
                [0.0, cone_len * math.cos(+half_rad)],
                "g--", alpha=0.5, label=f"forward cone ±{SHELF_APPROACH_FOV_HALF_DEG:.0f}°")

        # Standoff threshold — close_enough triggers when min forward body-x ≤ this.
        ax.axhline(y=SHELF_STANDOFF_MM, color="orange", linestyle=":", alpha=0.8,
                   label=f"standoff {SHELF_STANDOFF_MM:.0f} mm")

        # Lidar near-range floor in body frame (= LIDAR_MOUNT_X_MM + LIDAR_FILTER_MIN_MM).
        near_floor = LIDAR_MOUNT_X_MM + LIDAR_FILTER_MIN_MM
        ax.axhline(y=near_floor, color="gray", linestyle=":", alpha=0.5,
                   label=f"lidar near floor {near_floor:.0f} mm")

        # Annotate nearest forward distance.
        nearest_mm = closest_forward_obstacle_mm(robot)
        if math.isfinite(nearest_mm):
            ax.annotate(f"nearest forward = {nearest_mm:.0f} mm",
                        xy=(0.02, 0.98), xycoords="axes fraction",
                        ha="left", va="top", fontsize=9,
                        bbox=dict(boxstyle="round", fc="white", alpha=0.8))

        title = "Lidar (body frame)"

    ax.set_xlim(*LIDAR_PLOT_XLIM_MM)
    ax.set_ylim(*LIDAR_PLOT_YLIM_MM)
    ax.invert_xaxis()  # body +y = robot LEFT → render on left side of figure
    ax.set_xlabel("body y  (+y = robot LEFT)  [mm]")
    ax.set_ylabel("body x  (+x = robot FORWARD)  [mm]")
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    try:
        _LIDAR_PLOT_FIG.savefig(path, dpi=80)
    except OSError as exc:
        print(f"[warn] failed to write lidar plot to {path}: {exc}")
        return False
    return True


def start_shelf_turn(robot: Robot, target_heading_deg: float):
    return robot.turn_to(
        target_heading_deg,
        blocking=False,
        tolerance_deg=SHELF_TURN_TOLERANCE_DEG,
        max_angular_rad_s=SHELF_TURN_MAX_ANGULAR_RAD_S,
    )


def start_straight_move(robot: Robot, distance_mm: float):
    """Drive forward (positive) or backward (negative) by distance_mm, non-blocking."""
    if distance_mm >= 0.0:
        return robot.move_forward(
            distance_mm,
            velocity=BURGER_TRAVEL_VEL_MM_S,
            tolerance=BURGER_TRAVEL_TOL_MM,
            blocking=False,
            timeout=BURGER_TRAVEL_TIMEOUT_S,
        )
    return robot.move_backward(
        -distance_mm,
        velocity=BURGER_TRAVEL_VEL_MM_S,
        tolerance=BURGER_TRAVEL_TOL_MM,
        blocking=False,
        timeout=BURGER_TRAVEL_TIMEOUT_S,
    )


def run_manipulation_actions(robot: Robot, actions) -> bool:
    """
    Blocking: for each action, lower the lift, set the gripper, settle, then
    raise to carry.

      ("pick",  lift_steps, close_deg) → lower → set_servo(close_deg) → raise
      ("place", lift_steps)            → lower → set_servo(GRIPPER_OPEN_DEG) → raise
    """
    robot.step_enable(LIFT_STEPPER)
    for action in actions:
        kind = action[0]
        lift_steps = action[1]

        if kind == "pick":
            gripper_target_deg = action[2] if len(action) >= 3 else GRIPPER_CLOSE_BUN_DEG
            descriptor = f"pick (close {gripper_target_deg:.0f}°)"
        elif kind == "place":
            gripper_target_deg = GRIPPER_OPEN_DEG
            descriptor = f"place (open {gripper_target_deg:.0f}°)"
        else:
            print(f"[warn] unknown manipulation kind {kind!r}; skipping action")
            continue

        print(f"[BURGER] {descriptor} @ lift={lift_steps} steps")

        if not move_lift_to(robot, lift_steps):
            print(f"[warn] lift failed to reach {lift_steps} steps")
            return False

        robot.set_servo(GRIPPER_SERVO, gripper_target_deg)
        time.sleep(GRIPPER_SETTLE_S)

        if not move_lift_to(robot, LIFT_CARRY_STEPS):
            print("[warn] lift failed to return to carry height")
            return False

    return True


def abort_to_idle(robot: Robot, motion_handle, reason: str) -> None:
    cancel_handle(motion_handle)
    robot.stop()
    show_idle_leds(robot)
    print(f"[FSM] IDLE — {reason}")


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    last_status_print_at = 0.0

    # Burger sub-FSM state
    burger_idx = 0          # current stop index in BURGER_STOPS
    approach_started_at = 0.0
    approach_start_pose = None  # (x_mm, y_mm) at the moment forward velocity began
    travel_heading_deg = INITIAL_THETA_DEG  # robot's "forward" heading during burger sequence

    last_lidar_plot_at = 0.0
    lidar_plot_period_s = (1.0 / LIDAR_PLOT_HZ) if LIDAR_PLOT_HZ > 0 else float("inf")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if (ENABLE_LIDAR_PLOT and ENABLE_LIDAR
                and (now - last_lidar_plot_at) >= lidar_plot_period_s):
            save_lidar_plot(robot, lapf_overlay=(state == "AVOID_LAPF"))
            last_lidar_plot_at = now

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start (look-left → watch → path → burger), BTN_2 to cancel")
            print(
                f"[CFG] velocity={VELOCITY_MM_S:.0f} mm/s  lookahead={LOOKAHEAD_MM:.0f} mm  "
                f"tolerance={TOLERANCE_MM:.0f} mm  advance_radius={ADVANCE_RADIUS_MM:.0f} mm"
            )
            print(
                f"[CFG] look_heading={LOOK_HEADING_DEG:.1f}°  "
                f"initial_heading={INITIAL_THETA_DEG:.1f}°  "
                f"min_traffic_light_conf={MIN_TRAFFIC_LIGHT_CONFIDENCE:.2f}  "
                f"min_green_color_conf={MIN_GREEN_COLOR_CONFIDENCE:.2f}"
            )
            if ENABLE_LIDAR:
                print(
                    f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f}, {LIDAR_MOUNT_Y_MM:.0f}) mm "
                    f"theta={LIDAR_MOUNT_THETA_DEG:.1f}° filter={LIDAR_FILTER_MIN_MM:.0f}-"
                    f"{LIDAR_RANGE_MAX_MM:.0f} mm fov={LIDAR_FOV_DEG}"
                )
                if ENABLE_LIDAR_PLOT:
                    print(
                        f"[CFG] lidar plot → {LIDAR_PLOT_PATH} @ {LIDAR_PLOT_HZ:.1f} Hz "
                        f"(open with feh --reload 1, or any auto-refreshing viewer)"
                    )
                print(
                    f"[CFG] avoid: LAPF leg idx {AVOID_SEGMENT_START_INDEX}→{AVOID_SEGMENT_GOAL_INDEX} "
                    f"goal={AVOID_GOAL_POINT}  vel={AVOID_VELOCITY_MM_S:.0f} mm/s  "
                    f"leash={LAPF_LEASH_LENGTH_MM:.0f} mm  rep_range={LAPF_REPULSION_RANGE_MM:.0f} mm  "
                    f"inflation={LAPF_INFLATION_MARGIN_MM:.0f} mm"
                )
            else:
                print("[warn] obstacle avoidance leg needs lidar — ENABLE_LIDAR is False; "
                      "LAPF will see no cones")
            if ENABLE_GPS:
                print(
                    f"[CFG] gps tag_id={TAG_ID}  "
                    f"tag_body=({TAG_BODY_OFFSET_X_MM:.0f}, {TAG_BODY_OFFSET_Y_MM:.0f}) mm  "
                    f"position_alpha={GPS_POSITION_ALPHA:.2f}"
                )
                if ENABLE_GPS_TANGENT_HEADING:
                    print(
                        f"[CFG] heading=gps_tangent  "
                        f"alpha={GPS_TANGENT_ALPHA:.2f}  "
                        f"min_displacement={GPS_TANGENT_MIN_DISPLACEMENT_MM:.0f} mm"
                    )
                else:
                    print("[CFG] heading=imu")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_watching_leds(robot)
                print(f"[FSM] LOOK_LEFT — turning to {LOOK_HEADING_DEG:.1f}°")
                motion_handle = start_turn_to(
                    robot, LOOK_HEADING_DEG,
                    max_angular_rad_s=LOOK_TURN_MAX_ANGULAR_RAD_S,
                )
                state = "LOOK_LEFT"

        elif state == "LOOK_LEFT":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_handle(motion_handle)
                motion_handle = None
                robot.stop()
                clear_watching_leds(robot)
                show_idle_leds(robot)
                print("[FSM] IDLE — cancelled during look-left turn")
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print("[FSM] WATCHING — waiting for green traffic light")
                state = "WATCHING"

        elif state == "WATCHING":
            if robot.was_button_pressed(Button.BTN_2):
                clear_watching_leds(robot)
                show_idle_leds(robot)
                print("[FSM] IDLE — cancelled while watching")
                state = "IDLE"
            elif see_green_light(robot):
                print(f"[VISION] green light detected — turning back to {INITIAL_THETA_DEG:.1f}°")
                clear_watching_leds(robot)
                show_moving_leds(robot)
                motion_handle = start_turn_to(
                    robot, INITIAL_THETA_DEG,
                    max_angular_rad_s=RETURN_TURN_MAX_ANGULAR_RAD_S,
                )
                state = "TURN_BACK"

        elif state == "TURN_BACK":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_handle(motion_handle)
                motion_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — cancelled during turn-back")
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print(f"[FSM] MOVING — {len(INITIAL_PATH_CONTROL_POINTS)} waypoints to patty position")
                motion_handle = start_path(robot, INITIAL_PATH_CONTROL_POINTS)
                last_status_print_at = now
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, "initial path cancelled")
                motion_handle = None
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if motion_handle is not None and motion_handle.is_finished():
                    print("[FSM] initial path complete")
                    print_status(robot)
                    motion_handle = None
                    robot.stop()
                    if not ENABLE_BURGER_ASSEMBLY:
                        show_idle_leds(robot)
                        print("[FSM] IDLE — press BTN_1 to run again")
                        state = "IDLE"
                    else:
                        _, _, travel_heading_deg = robot.get_pose()
                        burger_idx = 0
                        print(f"[BURGER] starting at stop {burger_idx} ({BURGER_STOPS[burger_idx][0]})")
                        print(
                            f"[BURGER] travel_heading={travel_heading_deg:.1f}°  "
                            f"shelf_offset={SHELF_TURN_OFFSETS_DEG[burger_idx]:+.1f}°"
                        )
                        if not move_lift_to(robot, LIFT_CARRY_STEPS):
                            abort_to_idle(robot, None, "lift failed to reach carry before shelf turn")
                            state = "IDLE"
                            continue
                        motion_handle = start_shelf_turn(
                            robot,
                            travel_heading_deg + SHELF_TURN_OFFSETS_DEG[burger_idx],
                        )
                        state = "B_TURN_TO_SHELF"

        # ---------------------------------------------------------------
        # Burger assembly sub-FSM
        # ---------------------------------------------------------------

        elif state == "B_TURN_TO_SHELF":
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, "burger cancelled during turn-to-shelf")
                motion_handle = None
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                approach_started_at = now
                approach_start_pose = robot.get_pose()[:2]
                if ENABLE_LIDAR:
                    print(
                        f"[BURGER] APPROACH_SHELF (stop={BURGER_STOPS[burger_idx][0]}) "
                        f"— standoff={SHELF_STANDOFF_MM:.0f} mm, vel={SHELF_APPROACH_VEL_MM_S:.0f} mm/s"
                    )
                    state = "B_APPROACH_SHELF"
                else:
                    print(
                        f"[BURGER] APPROACH_SHELF fallback (no lidar) — "
                        f"forward {SHELF_APPROACH_MAX_DIST_MM:.0f} mm"
                    )
                    motion_handle = robot.move_forward(
                        SHELF_APPROACH_MAX_DIST_MM,
                        velocity=SHELF_APPROACH_VEL_MM_S,
                        tolerance=BURGER_TRAVEL_TOL_MM,
                        blocking=False,
                        timeout=SHELF_APPROACH_TIMEOUT_S,
                    )
                    state = "B_APPROACH_FALLBACK"

        elif state == "B_APPROACH_SHELF":
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — burger cancelled during shelf approach")
                state = "IDLE"
                continue

            dist_mm = closest_forward_obstacle_mm(robot)
            cur_x, cur_y, _ = robot.get_pose()
            sx, sy = approach_start_pose
            advanced_mm = math.hypot(cur_x - sx, cur_y - sy)
            timed_out = (now - approach_started_at) > SHELF_APPROACH_TIMEOUT_S
            capped = advanced_mm >= SHELF_APPROACH_MAX_DIST_MM
            close_enough = dist_mm <= SHELF_STANDOFF_MM

            if close_enough or capped or timed_out:
                robot.stop()
                reason = ("standoff" if close_enough
                          else "max-distance cap" if capped
                          else "timeout")
                print(
                    f"[BURGER] approach done ({reason}) — "
                    f"nearest={dist_mm if math.isfinite(dist_mm) else float('inf'):.0f} mm  "
                    f"advanced={advanced_mm:.0f} mm"
                )
                state = "B_MANIPULATE"
            else:
                robot.set_velocity(SHELF_APPROACH_VEL_MM_S, 0.0)

        elif state == "B_APPROACH_FALLBACK":
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, "burger cancelled during fallback approach")
                motion_handle = None
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                state = "B_MANIPULATE"

        elif state == "B_MANIPULATE":
            label, _, actions = BURGER_STOPS[burger_idx]
            print(f"[BURGER] MANIPULATE @ {label} ({len(actions)} action(s))")
            ok = run_manipulation_actions(robot, actions)
            if not ok:
                abort_to_idle(robot, None, f"manipulation failed at stop {label}")
                state = "IDLE"
            else:
                print(f"[BURGER] retreating {RETREAT_FROM_SHELF_MM:.0f} mm from shelf")
                motion_handle = robot.move_backward(
                    RETREAT_FROM_SHELF_MM,
                    velocity=BURGER_TRAVEL_VEL_MM_S,
                    tolerance=BURGER_TRAVEL_TOL_MM,
                    blocking=False,
                    timeout=BURGER_TRAVEL_TIMEOUT_S,
                )
                state = "B_RETREAT"

        elif state == "B_RETREAT":
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, "burger cancelled during retreat")
                motion_handle = None
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                print(f"[BURGER] turning back to forward heading {travel_heading_deg:.1f}°")
                motion_handle = start_shelf_turn(robot, travel_heading_deg)
                state = "B_TURN_TO_FORWARD"

        elif state == "B_TURN_TO_FORWARD":
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, "burger cancelled during turn-to-forward")
                motion_handle = None
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                burger_idx += 1
                if burger_idx >= len(BURGER_STOPS):
                    # Burger built — start the continuation: pure-pursuit leg A,
                    # then LAPF through the cones, then pure-pursuit leg C.
                    if CONTINUATION_LEG_A_CONTROL_POINTS:
                        print(
                            f"[BURGER] complete — continuation leg A "
                            f"({len(CONTINUATION_LEG_A_CONTROL_POINTS)} waypoints) to avoidance start"
                        )
                        motion_handle = start_path(robot, CONTINUATION_LEG_A_CONTROL_POINTS)
                        last_status_print_at = now
                        state = "CONTINUATION_A"
                    else:
                        print(f"[BURGER] complete — LAPF avoidance to goal {AVOID_GOAL_POINT}")
                        motion_handle = start_lapf_goal(robot, AVOID_GOAL_POINT)
                        last_status_print_at = now
                        state = "AVOID_LAPF"
                else:
                    next_label, travel_mm, _ = BURGER_STOPS[burger_idx]
                    direction = "forward" if travel_mm >= 0.0 else "backward"
                    print(
                        f"[BURGER] traveling {direction} {abs(travel_mm):.0f} mm "
                        f"to stop {burger_idx} ({next_label})"
                    )
                    if travel_mm == 0.0:
                        # No travel needed — raise to carry, then turn-to-shelf.
                        if not move_lift_to(robot, LIFT_CARRY_STEPS):
                            abort_to_idle(robot, None, "lift failed to reach carry before shelf turn")
                            state = "IDLE"
                            continue
                        motion_handle = start_shelf_turn(
                            robot,
                            travel_heading_deg + SHELF_TURN_OFFSETS_DEG[burger_idx],
                        )
                        state = "B_TURN_TO_SHELF"
                    else:
                        motion_handle = start_straight_move(robot, travel_mm)
                        state = "B_TRAVEL"

        elif state == "B_TRAVEL":
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, "burger cancelled during inter-stop travel")
                motion_handle = None
                state = "IDLE"
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                if not move_lift_to(robot, LIFT_CARRY_STEPS):
                    abort_to_idle(robot, None, "lift failed to reach carry before shelf turn")
                    state = "IDLE"
                    continue
                motion_handle = start_shelf_turn(
                    robot,
                    travel_heading_deg + SHELF_TURN_OFFSETS_DEG[burger_idx],
                )
                state = "B_TURN_TO_SHELF"

        elif state == "CONTINUATION_A":
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, "continuation leg A cancelled")
                motion_handle = None
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if motion_handle is not None and motion_handle.is_finished():
                    print("[FSM] leg A complete — starting LAPF obstacle avoidance")
                    print_status(robot)
                    motion_handle = None
                    robot.stop()
                    print(
                        f"[AVOID] LAPF to goal {AVOID_GOAL_POINT}  "
                        f"vel={AVOID_VELOCITY_MM_S:.0f} mm/s  leash={LAPF_LEASH_LENGTH_MM:.0f} mm  "
                        f"rep_range={LAPF_REPULSION_RANGE_MM:.0f} mm  inflation={LAPF_INFLATION_MARGIN_MM:.0f} mm"
                    )
                    motion_handle = start_lapf_goal(robot, AVOID_GOAL_POINT)
                    last_status_print_at = now
                    state = "AVOID_LAPF"

        elif state == "AVOID_LAPF":
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, "LAPF avoidance cancelled")
                motion_handle = None
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_avoid_status(robot)
                    last_status_print_at = now
                if motion_handle is not None and motion_handle.is_finished():
                    print("[FSM] LAPF avoidance complete")
                    print_avoid_status(robot)
                    motion_handle = None
                    robot.stop()
                    if CONTINUATION_LEG_C_CONTROL_POINTS:
                        print(
                            f"[FSM] CONTINUATION_C — {len(CONTINUATION_LEG_C_CONTROL_POINTS)} "
                            f"waypoints to finish"
                        )
                        motion_handle = start_path(robot, CONTINUATION_LEG_C_CONTROL_POINTS)
                        last_status_print_at = now
                        state = "CONTINUATION_C"
                    else:
                        show_idle_leds(robot)
                        print("[FSM] DONE — no leg-C waypoints")
                        print("[FSM] IDLE — press BTN_1 to run again")
                        state = "IDLE"

        elif state == "CONTINUATION_C":
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, "continuation leg C cancelled")
                motion_handle = None
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if motion_handle is not None and motion_handle.is_finished():
                    print("[FSM] DONE — continuation complete")
                    print_status(robot)
                    motion_handle = None
                    robot.stop()
                    show_idle_leds(robot)
                    print("[FSM] IDLE — press BTN_1 to run again")
                    state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
