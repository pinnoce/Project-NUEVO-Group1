"""
main.py — Full competition FSM
====================================
Mission sequence (BTN_2 cancels at any stage):

  IDLE               → wait for BTN_1, raise lift to carry and open gripper
  TRAFFIC_LIGHT      → turn left 30°, wait for green light, turn back
  MOVING_1           → drive forward to the left-bun position
  BURGER_ASSEMBLY    → 3 shelf stops: left-bun pick, patty stack, right-bun stack+pick
  MOVING_2           → scripted navigation with lidar-parallel corrections
  OBSTACLE_AVOIDANCE → LAPF through cone field
  MOVING_3           → perpendicular wall approach, turn, parallel, drive
  GENDER_ID          → identify person gender from vision
  MOVING_4           → navigate to drop-off position (gender-dependent distance)
  BURGER_DROPOFF     → release burger on shelf
  STOP_SIGN          → drive until stop sign fills frame, wait 3s, final drive
  DONE

Body frame: +x = forward, +y = left.
Angles in degrees everywhere in this file (CCW positive).
"""

from __future__ import annotations

import math
import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    ServoChannel,
    StepMoveType,
    Stepper,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot


# ===========================================================================
# CONFIGURATION — tune everything here; do not edit below the IMPLEMENTATION
# banner unless you know what you're doing.
# ===========================================================================

ENABLE_LIDAR = True

# ---------------------------------------------------------------------------
# Lidar plot (live body-frame debug view; open LIDAR_PLOT_PATH in auto-refresh
# viewer such as `feh --reload 1 <path>`)
# ---------------------------------------------------------------------------
ENABLE_LIDAR_PLOT  = True
LIDAR_PLOT_HZ      = 2.0
LIDAR_PLOT_PATH    = "/runtime_output/lidar/latest.png"
LIDAR_PLOT_XLIM_MM = (-1500.0, 1500.0)
LIDAR_PLOT_YLIM_MM = (-500.0, 2500.0)

# ---------------------------------------------------------------------------
# Common wall-alignment parameters (used by both forward and side alignment)
# ---------------------------------------------------------------------------
WALL_ALIGN_MIN_POINTS    = 8      # minimum lidar returns to attempt a fit
WALL_ALIGN_MAX_THICKNESS = 0.40   # max minor/major PCA ratio to accept a wall
WALL_ALIGN_TOLERANCE_DEG = 2.0    # heading error below which alignment is done
WALL_ALIGN_MAX_ITERS     = 5      # max correction turns before giving up
WALL_ALIGN_TIMEOUT_S     = 8.0    # give up waiting for a fit after this
WALL_ALIGN_MAX_RANGE_MM  = 4000.0 # ignore points farther than this
WALL_ALIGN_MAX_ANGULAR_RAD_S = 0.4
WALL_ALIGN_TURN_TOLERANCE_DEG = 2.0
WALL_ALIGN_DEBUG         = True   # throttled diagnostics

# ---------------------------------------------------------------------------
# Traffic light
# ---------------------------------------------------------------------------
LOOK_LEFT_DEG              = 30.0
TL_TURN_MAX_ANGULAR_RAD_S  = 0.4
TL_TURN_TOLERANCE_DEG      = 3.0
TL_POST_TURN_PAUSE_S       = 0.5
VISION_STALE_SEC           = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50
MIN_GREEN_COLOR_CONFIDENCE = 0.07

# ---------------------------------------------------------------------------
# Moving 1 — drive to the LEFT BUN (first burger stop)
# ---------------------------------------------------------------------------
MOV1_DISTANCE_MM   = 1065.0    # forward distance to reach the left-bun position (RE-TUNE)
MOV1_VELOCITY_MM_S = 120.0
MOV1_TOLERANCE_MM  = 20.0
MOV1_PAUSE_S       = 0.5

# ---------------------------------------------------------------------------
# TEST HARNESS (this file only) — segment test
# ---------------------------------------------------------------------------
# Behaviour-trimmed copy of main.py: on BTN_1 the FSM jumps straight to a
# specific mission segment instead of the real traffic-light start, so that
# segment can be tuned in isolation. Odometry is reset to (0,0,90deg) on BTN_1
# (reset_mission), matching the real pre-LAPF reset, so the robot-relative
# LAPF_GOAL is reckoned from wherever the robot is placed.
TEST_SIMULATE_CARRY = True   # on BTN_1, raise lift to carry + close gripper to mimic holding the burger

# ---------------------------------------------------------------------------
# Gripper servo
# ---------------------------------------------------------------------------
GRIPPER_SERVO           = ServoChannel.CH_1
GRIPPER_OPEN_DEG        = 0.0
GRIPPER_CLOSE_PATTY_DEG = 97.0   # tighter for patty
GRIPPER_CLOSE_BUN_DEG   = 85.0   # looser for bun or full stack
GRIPPER_SETTLE_S        = 0.8

# ---------------------------------------------------------------------------
# Lift stepper
# ---------------------------------------------------------------------------
LIFT_STEPPER          = Stepper.STEPPER_1
LIFT_DIR_INVERTED     = True    # positive logical steps = upward
LIFT_MAX_VELOCITY     = 3500
LIFT_ACCELERATION     = 2500
LIFT_MOVE_TIMEOUT_S   = 30.0    # headroom for a slow lift if boot config was dropped
LIFT_CONFIG_SETTLE_S  = 0.15    # let step_set_config land before the first move

# Logical heights in steps.  Step 0 = shelf surface (boot position).
BUN_HEIGHT_STEPS      = 11000
PATTY_HEIGHT_STEPS    = 11000
LIFT_CARRY_STEPS      = 20000   # safe travel height

# New assembly order: LEFT BUN → PATTY (dropped onto bun) → RIGHT BUN (stacked onto it).
LIFT_SHELF_STEPS       = 0                   # shelf surface — pick the bottom layer here
LIFT_PATTY_PLACE_STEPS = PATTY_HEIGHT_STEPS  # drop the held bun so its base sits on the patty top
LIFT_RB_PLACE_STEPS    = BUN_HEIGHT_STEPS    # drop the held patty+bun so it sits on the right bun

# ---------------------------------------------------------------------------
# Burger assembly — shelf approach
# ---------------------------------------------------------------------------
SHELF_APPROACH_VEL_MM_S     = 40.0
SHELF_STANDOFF_MM           = 295.0   # stop when nearest forward point ≤ this
SHELF_APPROACH_FOV_HALF_DEG = 10.0    # narrow forward cone for approach distance
SHELF_APPROACH_MAX_DIST_MM  = 300.0   # safety cap on approach distance
SHELF_APPROACH_TIMEOUT_S    = 20.0
LIDAR_FILTER_MIN_MM         = 25.0    # overrides hardware_map for this run
RETREAT_FROM_SHELF_MM       = 200.0

# ---------------------------------------------------------------------------
# Burger assembly — 3-stop sequence (LEFT BUN → PATTY → RIGHT BUN), indexed [0,1,2]
#
# Per stop the robot:
#   1. turns LEFT  (BUR_FACE_TURN_DEG[i]) to face the shelf
#   2. squares perpendicular to the shelf with the lidar (forward-wall fit,
#      FOV half-angle BUR_PERP_FOV_DEG[i])
#   3. drives forward to SHELF_STANDOFF_MM
#   4. manipulates (pick bun / stack patty / stack right bun)
#   5. retreats BUR_RETREAT_MM[i] mm straight back
#   6. turns RIGHT (BUR_UNFACE_TURN_DEG[i]) back to the travel heading
#   7. re-squares PARALLEL to the shelf (left wall, FOV BUR_PARA_FOV_DEG[i])
#   8. drives forward BUR_TRAVEL_MM[i] mm to line up beside the next item
#      (the last stop has no travel — it hands off to MOV2)
#
# Turns are RELATIVE (drift-free); the lidar squares remove any residual.
# ---------------------------------------------------------------------------
BUR_FACE_TURN_DEG   = [90.0, 90.0, 90.0]     # CCW+ turn to face the shelf at each stop
BUR_UNFACE_TURN_DEG = [-90.0, -90.0, -90.0]  # CW turn back to travel heading after each stop
BUR_PERP_FOV_DEG    = [20.0, 20.0, 20.0]     # forward-wall square FOV half-angle per stop
BUR_PARA_FOV_DEG    = [70.0, 70.0, 70.0]     # parallel left-wall square FOV half-angle per stop
BUR_RETREAT_MM      = [200.0, 200.0, 200.0]  # straight-back retreat after each stop
BUR_TRAVEL_MM       = [275.0, 325.0]         # forward travel LB→patty, patty→RB (last stop: none)

SHELF_TURN_MAX_ANGULAR_RAD_S = 1.4
SHELF_TURN_TOLERANCE_DEG     = 2.0

INTER_PAUSE_S = 0.25

BURGER_TRAVEL_VEL_MM_S  = 80.0
BURGER_TRAVEL_TOL_MM    = 15.0
BURGER_TRAVEL_TIMEOUT_S = 20.0
BURGER_POST_STACK_PAUSE_S = 0.5

# ---------------------------------------------------------------------------
# Moving 2 — scripted route with lidar-parallel corrections
# Turns are RELATIVE (CCW = positive = left; negative = right).
# Drives are FORWARD (always positive distance here; robot should be aligned).
# Side for each align step: 'left' or 'right'.
# FOV for each align step is independently tunable.
# ---------------------------------------------------------------------------
# Turns: 1-3 are right (negative), 4-5 are left (positive)
MOV2_TURN_1_DEG    = 0.0   # right: exit burger area → start nav corridor
MOV2_DRIVE_1_MM    = 400.0    # drive after T1 align (spec line 2)
MOV2_DRIVE_2_MM    = 2500.0    # drive before T2       (spec line 3)
MOV2_TURN_2_DEG    = -90.0    # right turn 1
MOV2_DRIVE_3_MM    = 1000   # drive before T3       (spec line 4)
MOV2_TURN_3_DEG    = -98.0    # right turn 2
MOV2_DRIVE_4_MM    = 3500.0    # drive before T4       (spec line 5)
MOV2_TURN_4_DEG    = 93.0     # left: switch to right wall
MOV2_DRIVE_5_MM    = 700.0    # final drive           (spec line 6)
MOV2_TURN_5_DEG    = 91.0     # left: final heading correction before LAPF
MOV2_ALIGN_LEFT_FOV  = 70.0   # FOV for all left-wall parallel aligns
MOV2_ALIGN_RIGHT_FOV = 70.0   # FOV for all right-wall parallel aligns
MOV2_VELOCITY_MM_S   = 120.0
MOV2_TOLERANCE_MM    = 20.0
MOV2_DRIVE_TIMEOUT_S = 30.0
MOV2_APPROACH_VEL_MM_S = 100.0        # slow speed for the DRIVE_2 wall-standoff approach
MOV2_APPROACH_2_STANDOFF_MM = 165.0  # stop this far from the forward wall (replaces fixed MOV2_DRIVE_2_MM)
MOV2_APPROACH_3_STANDOFF_MM = 180.0  # stop this far from the forward wall (replaces fixed MOV2_DRIVE_3_MM)
MOV2_APPROACH_4_STANDOFF_MM = 150.0  # stop this far from the forward wall (replaces fixed MOV2_DRIVE_4_MM)
MOV2_PAUSE_SHORT_S   = 0.25
MOV2_PAUSE_LONG_S    = 0.5
MOV2_TURN_MAX_ANGULAR_RAD_S = 1.2
MOV2_TURN_TOLERANCE_DEG     = 2.0

# ---------------------------------------------------------------------------
# Obstacle avoidance (LAPF)
# ---------------------------------------------------------------------------
LAPF_GOAL = (0.0, 2700.0)   # robot-relative goal: odom is reset to (0,0,90°) right before LAPF, so this is straight ahead
LAPF_VELOCITY_MM_S      = 60.0
LAPF_TOLERANCE_MM       = 50.0
LAPF_MAX_ANGULAR_RAD_S  = 1.0
LAPF_LEASH_LENGTH_MM    = 50.0
LAPF_LEASH_HALF_ANGLE_DEG = 25.0
LAPF_REPULSION_RANGE_MM = 335.0
LAPF_REPULSION_GAIN     = 550.0
LAPF_ATTRACTION_GAIN    = 1.0
LAPF_TARGET_SPEED_MM_S  = 200.0
LAPF_FORCE_EMA_ALPHA    = 0.35
LAPF_INFLATION_MARGIN_MM = 150.0
LAPF_PAUSE_S            = 0.5

# ---------------------------------------------------------------------------
# Moving 3
# ---------------------------------------------------------------------------
MOV3_ALIGN_PERP_FOV     = 10.0   # FOV for perpendicular wall alignment
MOV3_WALL_STANDOFF_MM   = 150.0
MOV3_APPROACH_VEL_MM_S  = 60.0
MOV3_APPROACH_MAX_MM    = 2500.0
MOV3_APPROACH_TIMEOUT_S = 30.0
MOV3_TURN_DEG           = -92.0  # right turn after wall approach
MOV3_TURN_MAX_ANGULAR_RAD_S = 1.0
MOV3_TURN_TOLERANCE_DEG = 3.0
MOV3_ALIGN_PARA_FOV     = 60.0   # FOV for parallel left-wall alignment
MOV3_DRIVE_MM           = 175.0
MOV3_VELOCITY_MM_S      = 120.0
MOV3_TOLERANCE_MM       = 20.0
MOV3_DRIVE_TIMEOUT_S    = 30.0
MOV3_POST_DRIVE_PAUSE_S = 8.0

# ---------------------------------------------------------------------------
# Gender identification
# ---------------------------------------------------------------------------
MIN_PERSON_CONFIDENCE = 0.20

# ---------------------------------------------------------------------------
# Moving 4
# ---------------------------------------------------------------------------
MOV4_ALIGN_PERP_FOV     = 10.0
MOV4_WALL_STANDOFF_MM   = 195.0
MOV4_APPROACH_VEL_MM_S  = 50.0
MOV4_APPROACH_MAX_MM    = 2500.0
MOV4_APPROACH_TIMEOUT_S = 30.0
MOV4_TURN_DEG           = -90.0  # right turn
MOV4_TURN_MAX_ANGULAR_RAD_S = 1.0
MOV4_TURN_TOLERANCE_DEG = 2.0
MOV4_ALIGN_PARA_FOV     = 60.0
MOV4_DRIVE_1_MM         = 500.0  # first parallel-corrected drive
MOV4_VELOCITY_MM_S      = 120.0
MOV4_TOLERANCE_MM       = 20.0
MOV4_DRIVE_TIMEOUT_S    = 30.0
MOV4_PAUSE_S            = 0.5
# Gender-dependent final drive distance
MOV4_DRIVE_FEMALE_MM    = 1700.0
MOV4_DRIVE_MALE_MM      = 2050.0

# ---------------------------------------------------------------------------
# Burger drop-off
# ---------------------------------------------------------------------------
DROP_SHELF_TURN_DEG     = 93.0    # CCW (left) from current heading to face shelf
DROP_SHELF_ALIGN_FOV    = 40.0
DROP_SHELF_STANDOFF_MM  = 290.0
DROP_APPROACH_VEL_MM_S  = 40.0
DROP_APPROACH_MAX_MM    = 300.0
DROP_APPROACH_TIMEOUT_S = 20.0
DROP_LIFT_PLACE_STEPS   = 0       # lower to shelf surface for release
DROP_RETREAT_MM         = 180.0
DROP_TURN_BACK_DEG      = -92.0   # CW (right) to return to travel heading
DROP_TURN_BACK_ALIGN_FOV = 60.0   # FOV half-angle for the lidar parallel-left heading fix after turn-back
DROP_TRAVEL_VEL_MM_S    = 80.0
DROP_TRAVEL_TOL_MM      = 15.0
DROP_TRAVEL_TIMEOUT_S   = 20.0
DROP_POST_RELEASE_PAUSE_S = 0.5

# ---------------------------------------------------------------------------
# Stop sign
# ---------------------------------------------------------------------------
MIN_STOP_SIGN_CONFIDENCE = 0.91   # YOLO detection confidence threshold (adjustable)
STOP_SIGN_VEL_MM_S      = 80.0
STOP_SIGN_APPROACH_MM   = 300.0    # after first seeing the sign, drive this far, THEN stop
STOP_SIGN_APPROACH_FEMALE_EXTRA_MM = 250.0   # female: drive this much further than the male/base STOP_SIGN_APPROACH_MM before stopping
STOP_SIGN_WAIT_S        = 3.0
STOP_SIGN_FINAL_MM      = 750.0   # drive this far after the stop
STOP_SIGN_VEL_FINAL_MM_S = 100.0
STOP_SIGN_TOLERANCE_MM  = 20.0
STOP_SIGN_TIMEOUT_S     = 30.0


# ===========================================================================
# IMPLEMENTATION — do not change parameters above, edit logic only carefully
# ===========================================================================

# ---------------------------------------------------------------------------
# Module-level mutable state
# ---------------------------------------------------------------------------

_LIFT_LOGICAL_STEPS: int = 0      # tracks lift position relative to boot=0
_lift_primed: bool = False        # True once lift has been raised in current IDLE visit

_plot_ctx: dict = {               # shared between all alignment functions and plot
    'fov_half_deg': 20.0,
    'fov_center_deg': 0.0,        # 0=forward, 90=left, -90=right
    'standoff_mm': SHELF_STANDOFF_MM,
    'fit_pts': [],
    'fit_phi_deg': None,          # principal axis angle of last fit (deg)
    'fit_centroid': None,         # (cx, cy) centroid of last fit
}

_align_dbg_t: float = 0.0        # throttle for alignment debug prints
_detected_gender: tuple | None = None  # (label, score) from vision, set during GENDER_ID


# ---------------------------------------------------------------------------
# Lift helpers
# ---------------------------------------------------------------------------

def _lift_signed(steps: int) -> int:
    return -int(steps) if LIFT_DIR_INVERTED else int(steps)


def apply_lift_config(robot: Robot) -> None:
    """Re-send the lift step config while the firmware is live.

    configure_robot sends step_set_config in the boot window, before the
    bridge<->firmware link is up (the same window that drops set_odometry_parameters),
    so it is silently lost and the lift runs at the slow firmware default — a
    LIFT_CARRY move then takes longer than LIFT_MOVE_TIMEOUT_S and move_lift_to
    returns False even though the lift is moving fine. Re-apply after step_enable
    (post-RUNNING) so the config actually takes. See [[lift_config_boot_race]].
    """
    robot.step_set_config(
        LIFT_STEPPER, max_velocity=LIFT_MAX_VELOCITY, acceleration=LIFT_ACCELERATION,
    )
    time.sleep(LIFT_CONFIG_SETTLE_S)


def move_lift_to(robot: Robot, target_steps: int, timeout: float = LIFT_MOVE_TIMEOUT_S) -> bool:
    global _LIFT_LOGICAL_STEPS
    delta = int(target_steps) - _LIFT_LOGICAL_STEPS
    signed = _lift_signed(delta)
    print(
        f"[LIFT] target={target_steps}  current={_LIFT_LOGICAL_STEPS}  "
        f"delta={delta:+d}  firmware={signed:+d}"
    )
    if delta == 0:
        return True
    ok = robot.step_move(
        LIFT_STEPPER, steps=signed,
        move_type=StepMoveType.RELATIVE, blocking=True, timeout=timeout,
    )
    if ok:
        _LIFT_LOGICAL_STEPS = int(target_steps)
    return ok


# ---------------------------------------------------------------------------
# Wall line fitting (shared by forward and side alignment)
# ---------------------------------------------------------------------------

def _fit_line(pts: list[tuple[float, float]]) -> tuple[float | None, float | None, str]:
    """Fit a line to pts via PCA.

    Returns (phi_deg, thickness, message).
    phi_deg = principal axis angle in [-90, 90].
    thickness = minor/major PCA ratio (smaller = more wall-like).
    Returns (None, None, reason) when fit is not valid.
    """
    n = len(pts)
    if n < WALL_ALIGN_MIN_POINTS:
        return None, None, f"only {n} pts (need ≥{WALL_ALIGN_MIN_POINTS})"
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts) / n
    syy = sum((p[1] - my) ** 2 for p in pts) / n
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts) / n
    tr = sxx + syy
    disc = math.sqrt(max(0.0, (tr / 2.0) ** 2 - (sxx * syy - sxy * sxy)))
    major = tr / 2.0 + disc
    minor = tr / 2.0 - disc
    if major <= 0.0:
        return None, None, "degenerate covariance"
    thickness = math.sqrt(max(0.0, minor)) / math.sqrt(major)
    if thickness > WALL_ALIGN_MAX_THICKNESS:
        return None, None, f"blobby (thickness={thickness:.2f} > {WALL_ALIGN_MAX_THICKNESS})"
    phi = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    phi = (phi + math.pi / 2.0) % math.pi - math.pi / 2.0  # wrap to (-90°, 90°]
    return math.degrees(phi), thickness, f"ok  phi={math.degrees(phi):+.1f}°  n={n}  thick={thickness:.2f}"


def _dbg_wall(msg: str) -> None:
    global _align_dbg_t
    if not WALL_ALIGN_DEBUG:
        return
    t = time.monotonic()
    if t - _align_dbg_t >= 1.0:
        _align_dbg_t = t
        print(f"[WALL] {msg}")


# ---------------------------------------------------------------------------
# Forward (perpendicular) wall correction
# ---------------------------------------------------------------------------

def forward_wall_correction_deg(robot: Robot, fov_half_deg: float) -> float | None:
    """Return rotation (deg, CCW+) to make the robot face the forward wall head-on.

    Returns None when there are not enough points or the fit failed.
    Updates _plot_ctx with the current FOV and fit.
    """
    fov = math.radians(fov_half_deg)
    in_cone = [
        (x, y, math.hypot(x, y))
        for x, y in robot.get_obstacles()
        if x > 0.0 and abs(math.atan2(y, x)) <= fov
    ]
    pts = [(x, y) for x, y, d in in_cone if d <= WALL_ALIGN_MAX_RANGE_MM]

    _plot_ctx['fov_half_deg'] = fov_half_deg
    _plot_ctx['fov_center_deg'] = 0.0
    _plot_ctx['fit_pts'] = pts

    phi_deg, thickness, msg = _fit_line(pts)
    if phi_deg is None:
        nearest = min((d for _, _, d in in_cone), default=float("inf"))
        _dbg_wall(
            f"forward: {msg}  in_cone={len(in_cone)}  in_range={len(pts)}  "
            f"nearest={nearest:.0f} mm  fov±{fov_half_deg:.0f}°"
        )
        _plot_ctx['fit_phi_deg'] = None
        _plot_ctx['fit_centroid'] = None
        return None

    # For a forward wall, the wall surface is ⊥ to robot +x.
    # Principal axis of wall points ≈ +y (along wall surface).
    # psi = phi + 90° = angle of wall normal in body frame.
    # Heading correction = psi (how much to turn so +x aligns with wall normal).
    psi = phi_deg + 90.0
    psi = (psi + 90.0) % 180.0 - 90.0  # wrap to (-90°, 90°]
    _plot_ctx['fit_phi_deg'] = phi_deg
    n = len(pts)
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    _plot_ctx['fit_centroid'] = (cx, cy)
    _dbg_wall(f"forward: {msg}  correction={psi:+.1f}°")
    return psi


def forward_wall_gap_mm(robot: Robot) -> float:
    """Body-frame gap between lidar mount and nearest forward point (mm)."""
    fov = math.radians(SHELF_APPROACH_FOV_HALF_DEG)
    nearest = float("inf")
    for x, y in robot.get_obstacles():
        if x <= 0.0:
            continue
        if abs(math.atan2(y, x)) > fov:
            continue
        if x < nearest:
            nearest = x
    if not math.isfinite(nearest):
        return float("inf")
    return nearest - LIDAR_MOUNT_X_MM


def closest_forward_obstacle_mm(robot: Robot) -> float:
    """Body-frame forward distance to nearest point in the approach cone."""
    fov = math.radians(SHELF_APPROACH_FOV_HALF_DEG)
    nearest = float("inf")
    for x, y in robot.get_obstacles():
        if x <= 0.0:
            continue
        if abs(math.atan2(y, x)) > fov:
            continue
        if x < nearest:
            nearest = x
    return nearest


# ---------------------------------------------------------------------------
# Side (parallel) wall correction
# ---------------------------------------------------------------------------

def side_wall_correction_deg(robot: Robot, fov_half_deg: float, side: str) -> float | None:
    """Return rotation (deg, CCW+) to make the robot PARALLEL to the side wall.

    side = 'left' (wall at +y) or 'right' (wall at -y).
    Returns None when fit failed.
    Updates _plot_ctx.
    """
    fov = math.radians(fov_half_deg)
    center = math.pi / 2.0 if side == 'left' else -math.pi / 2.0
    pts_with_dist = [
        (x, y, math.hypot(x, y))
        for x, y in robot.get_obstacles()
        if math.hypot(x, y) > 1e-6
        and abs(math.atan2(y, x) - center) <= fov
    ]
    pts = [(x, y) for x, y, d in pts_with_dist if d <= WALL_ALIGN_MAX_RANGE_MM]

    _plot_ctx['fov_half_deg'] = fov_half_deg
    _plot_ctx['fov_center_deg'] = 90.0 if side == 'left' else -90.0
    _plot_ctx['fit_pts'] = pts

    phi_deg, thickness, msg = _fit_line(pts)
    if phi_deg is None:
        nearest = min((d for _, _, d in pts_with_dist), default=float("inf"))
        _dbg_wall(
            f"{side}: {msg}  in_cone={len(pts_with_dist)}  in_range={len(pts)}  "
            f"nearest={nearest:.0f} mm  fov±{fov_half_deg:.0f}°"
        )
        _plot_ctx['fit_phi_deg'] = None
        _plot_ctx['fit_centroid'] = None
        return None

    # For a side wall, robot should travel parallel to it → wall principal axis ≈ +x (0°).
    # phi_deg = deviation of principal axis from +x = correction to apply.
    _plot_ctx['fit_phi_deg'] = phi_deg
    n = len(pts)
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    _plot_ctx['fit_centroid'] = (cx, cy)
    _dbg_wall(f"{side}: {msg}  correction={phi_deg:+.1f}°")
    return phi_deg


# ---------------------------------------------------------------------------
# Lidar debug plot
# ---------------------------------------------------------------------------

_plot_fig = None
_plot_ax  = None


def save_lidar_plot(robot: Robot) -> bool:
    """Render body-frame lidar view with current FOV cone, fit line, and standoff."""
    global _plot_fig, _plot_ax
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    if _plot_fig is None:
        _plot_fig, _plot_ax = plt.subplots(figsize=(6, 6))
    ax = _plot_ax
    ax.clear()

    # --- Obstacle scatter ---
    obstacles = robot.get_obstacles()
    if obstacles:
        xs = [p[1] for p in obstacles]   # plot X = body y  (left side = left in fig)
        ys = [p[0] for p in obstacles]   # plot Y = body x  (forward = up in fig)
        ax.scatter(xs, ys, s=4, c='red', label=f'lidar ({len(obstacles)} pts)')

    # --- Robot body and lidar mount ---
    ax.scatter([0.0], [0.0], s=90, c='black', marker='s', label='robot body')
    ax.scatter([LIDAR_MOUNT_Y_MM], [LIDAR_MOUNT_X_MM], s=50, c='blue',
               marker='^', label='lidar mount')

    # --- FOV cone ---
    fov_half = _plot_ctx['fov_half_deg']
    center_deg = _plot_ctx['fov_center_deg']
    center_rad = math.radians(center_deg)
    cone_len = max(LIDAR_PLOT_YLIM_MM[1], 1500.0)
    for sign in (-1, 1):
        edge_rad = center_rad + sign * math.radians(fov_half)
        # body (x=forward, y=left) → plot (X=y, Y=x)
        px = cone_len * math.sin(edge_rad)   # body y component
        py = cone_len * math.cos(edge_rad)   # body x component
        ax.plot([0.0, px], [0.0, py], 'g--', alpha=0.5,
                label=f'FOV ±{fov_half:.0f}° @ {center_deg:.0f}°' if sign == 1 else None)

    # --- Fit line ---
    phi_deg = _plot_ctx.get('fit_phi_deg')
    centroid = _plot_ctx.get('fit_centroid')
    if phi_deg is not None and centroid is not None:
        cx, cy = centroid
        phi_rad = math.radians(phi_deg)
        line_len = 400.0
        # line direction in body frame: (cos phi, sin phi)
        # plot: X = body_y, Y = body_x
        dx_plot = math.sin(phi_rad) * line_len
        dy_plot = math.cos(phi_rad) * line_len
        ax.plot(
            [cy - dx_plot, cy + dx_plot],
            [cx - dy_plot, cx + dy_plot],
            'm-', linewidth=2, label=f'fit line φ={phi_deg:+.1f}°',
        )
        ax.scatter([cy], [cx], s=40, c='magenta', zorder=5)
        ax.annotate(
            f'φ={phi_deg:+.1f}°',
            xy=(cy, cx), xytext=(cy + 50, cx + 50),
            fontsize=9, color='magenta',
            arrowprops=dict(arrowstyle='->', color='magenta'),
        )

    # --- Standoff / approach cone for forward mode ---
    if abs(center_deg) < 1.0:  # forward-facing analysis
        standoff = _plot_ctx['standoff_mm']
        ax.axhline(y=standoff, color='orange', linestyle=':', alpha=0.8,
                   label=f'standoff {standoff:.0f} mm')
        near_floor = LIDAR_MOUNT_X_MM + LIDAR_FILTER_MIN_MM
        ax.axhline(y=near_floor, color='gray', linestyle=':', alpha=0.5,
                   label=f'near floor {near_floor:.0f} mm')
        nearest = closest_forward_obstacle_mm(robot)
        if math.isfinite(nearest):
            ax.annotate(
                f'nearest fwd = {nearest:.0f} mm',
                xy=(0.02, 0.98), xycoords='axes fraction',
                ha='left', va='top', fontsize=9,
                bbox=dict(boxstyle='round', fc='white', alpha=0.8),
            )
    else:
        # Side-wall parallel analysis: draw a distance line on the correct side
        side_dist = _plot_ctx.get('standoff_mm', 300.0)
        sign = 1.0 if center_deg > 0 else -1.0
        ax.axvline(x=sign * side_dist, color='orange', linestyle=':', alpha=0.8,
                   label=f'side wall ≈ {side_dist:.0f} mm')

    ax.set_xlim(*LIDAR_PLOT_XLIM_MM)
    ax.set_ylim(*LIDAR_PLOT_YLIM_MM)
    ax.invert_xaxis()
    ax.set_xlabel('body y  (+y = LEFT)  [mm]')
    ax.set_ylabel('body x  (+x = FORWARD)  [mm]')
    ax.set_title(f'Lidar  fov={_plot_ctx["fov_half_deg"]:.0f}°  center={_plot_ctx["fov_center_deg"]:.0f}°')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=7)
    try:
        _plot_fig.savefig(LIDAR_PLOT_PATH, dpi=80)
    except OSError as e:
        print(f'[PLOT] failed to write: {e}')
        return False
    return True


# ---------------------------------------------------------------------------
# Motion helpers
# ---------------------------------------------------------------------------

def _cancel(handle) -> None:
    if handle is not None:
        handle.cancel()
        handle.wait(timeout=1.0)


def _start_turn_to(robot: Robot, heading_deg: float, max_angular: float = WALL_ALIGN_MAX_ANGULAR_RAD_S) -> object:
    return robot.turn_to(heading_deg, blocking=False,
                         tolerance_deg=WALL_ALIGN_TURN_TOLERANCE_DEG,
                         max_angular_rad_s=max_angular)


def _start_shelf_turn(robot: Robot, heading_deg: float,
                      max_angular: float = SHELF_TURN_MAX_ANGULAR_RAD_S) -> object:
    return robot.turn_to(heading_deg, blocking=False,
                         tolerance_deg=SHELF_TURN_TOLERANCE_DEG,
                         max_angular_rad_s=max_angular)


def _start_drive(robot: Robot, dist_mm: float, vel: float, tol: float, timeout: float) -> object:
    if dist_mm >= 0.0:
        return robot.move_forward(dist_mm, velocity=vel, tolerance=tol, blocking=False, timeout=timeout)
    return robot.move_backward(-dist_mm, velocity=vel, tolerance=tol, blocking=False, timeout=timeout)


# ---------------------------------------------------------------------------
# Vision helpers
# ---------------------------------------------------------------------------

def _see_green_light(robot: Robot) -> bool:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False
    for det in robot.get_detections('traffic light'):
        if float(det['confidence']) < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue
        col = det.get('attributes', {}).get('color', {})
        if col.get('value') != 'green':
            continue
        score = col.get('score')
        if score is None or float(score) < MIN_GREEN_COLOR_CONFIDENCE:
            continue
        return True
    return False


def _detect_gender(robot: Robot) -> tuple | None:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None
    best = None
    for det in robot.get_detections('person'):
        if float(det['confidence']) < MIN_PERSON_CONFIDENCE:
            continue
        g = det.get('attributes', {}).get('gender', {})
        label = g.get('value')
        if not label:
            continue
        score = float(g.get('score') or 0.0)
        if best is None or score > best[1]:
            best = (label, score)
    return best


def _see_stop_sign(robot: Robot) -> bool:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False
    for det in robot.get_detections('stop sign'):
        if float(det['confidence']) >= MIN_STOP_SIGN_CONFIDENCE:
            return True
    return False


# ---------------------------------------------------------------------------
# Setup / start
# ---------------------------------------------------------------------------

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
        print('[sensor] lidar enabled')
    robot.enable_servo(GRIPPER_SERVO)
    robot.step_set_config(LIFT_STEPPER, max_velocity=LIFT_MAX_VELOCITY, acceleration=LIFT_ACCELERATION)


def start_robot(robot: Robot) -> None:
    cur = robot.get_state()
    if cur in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_mission(robot: Robot) -> None:
    # NOTE: do NOT reset _LIFT_LOGICAL_STEPS here. The lift's physical position
    # is owned by prime_lift (and the power-on park=0 contract), not by mission
    # reset. reset_mission is called both at INIT and again on BTN_1 — zeroing
    # the tracker on the BTN_1 call (after prime_lift already raised the lift to
    # carry) made the tracker read 0 while the lift was physically at carry, so
    # the patty _pick lowered by 0 (no-op) and then raised again. See
    # [[burger_fsm_design]] / [[lift_config_boot_race]].
    global _detected_gender, _lift_primed
    _detected_gender = None
    _lift_primed = False
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print('[warn] odometry reset not confirmed within 2s')
        robot.wait_for_pose_update(timeout=0.5)


def prime_lift(robot: Robot) -> None:
    """Raise lift to carry height and open gripper (call at mission start)."""
    global _LIFT_LOGICAL_STEPS
    _LIFT_LOGICAL_STEPS = 0
    # Re-enable the servo here (not just in configure_robot). configure_robot
    # runs before the firmware is put into RUNNING, so if the firmware is still
    # ESTOP/faulted at that point (e.g. after a Ctrl-C heartbeat trip on the
    # previous run), the enable_servo there is dropped — canEnableServoActuator()
    # only allows IDLE/RUNNING. prime_lift runs in IDLE, after start_robot has
    # set RUNNING, so this enable always takes effect. See [[lift_config_boot_race]].
    robot.enable_servo(GRIPPER_SERVO)
    robot.step_enable(LIFT_STEPPER)
    apply_lift_config(robot)
    if not move_lift_to(robot, LIFT_CARRY_STEPS):
        print('[warn] lift failed to reach carry height at start')
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)


# ---------------------------------------------------------------------------
# Manipulation helper (blocking)
# ---------------------------------------------------------------------------

def _pick(robot: Robot, lift_steps: int, gripper_deg: float) -> bool:
    """Lower → close → raise (blocking)."""
    if not move_lift_to(robot, lift_steps):
        return False
    robot.set_servo(GRIPPER_SERVO, gripper_deg)
    time.sleep(GRIPPER_SETTLE_S)
    return move_lift_to(robot, LIFT_CARRY_STEPS)


def _place(robot: Robot, lift_steps: int) -> bool:
    """Lower → open → raise (blocking)."""
    if not move_lift_to(robot, lift_steps):
        return False
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)
    return move_lift_to(robot, LIFT_CARRY_STEPS)


def _place_layer_then_pick(robot: Robot, place_steps: int, grip_deg: float) -> bool:
    """Add the currently held layer to the shelf item, then pick up the combined stack.

    Used at the patty and right-bun stops, where the gripper is already holding
    something. Sequence (no intermediate raise):
      raise to place_steps → open (the held layer settles onto the shelf item
      below it) → lower to the shelf surface → close at grip_deg (now gripping
      the whole stack) → raise to carry.
    """
    if not move_lift_to(robot, place_steps):
        return False
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)
    # go straight down to shelf — no intermediate raise
    if not move_lift_to(robot, LIFT_SHELF_STEPS):
        return False
    robot.set_servo(GRIPPER_SERVO, grip_deg)
    time.sleep(GRIPPER_SETTLE_S)
    return move_lift_to(robot, LIFT_CARRY_STEPS)


def simulate_carry(robot: Robot) -> None:
    """TEST harness: mimic holding the assembled burger.

    prime_lift already raised the lift to carry and opened the gripper; here we
    just close the gripper on the (hand-loaded) burger so the later drop-off
    release is a real open. Keeps the lift at carry height.
    """
    global _LIFT_LOGICAL_STEPS
    robot.enable_servo(GRIPPER_SERVO)
    robot.step_enable(LIFT_STEPPER)
    if _LIFT_LOGICAL_STEPS != LIFT_CARRY_STEPS:
        move_lift_to(robot, LIFT_CARRY_STEPS)
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_BUN_DEG)
    time.sleep(GRIPPER_SETTLE_S)
    print(f'[TEST] simulate_carry - lift at carry ({LIFT_CARRY_STEPS}), gripper closed on burger')


def _retreat(robot: Robot, dist_mm: float) -> None:
    """Blocking backward move away from shelf."""
    robot.move_backward(
        dist_mm,
        velocity=BURGER_TRAVEL_VEL_MM_S,
        tolerance=BURGER_TRAVEL_TOL_MM,
        blocking=True,
        timeout=BURGER_TRAVEL_TIMEOUT_S,
    )


def abort_to_idle(robot: Robot, handle, reason: str) -> None:
    global _lift_primed
    _cancel(handle)
    robot.stop()
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.BLUE, 0)
    _lift_primed = False   # re-prime on next IDLE entry
    print(f'[FSM] IDLE — {reason}')


# ===========================================================================
# Main FSM
# ===========================================================================

def run(robot: Robot) -> None:  # noqa: C901 (complexity)
    configure_robot(robot)

    state = 'INIT'
    motion_handle = None

    # --- Alignment sub-state ---
    _align_iters = 0
    _align_started_at = 0.0

    # --- Approach sub-state ---
    _approach_started_at = 0.0
    _approach_start_pose = (0.0, 0.0)

    # --- Pause sub-state ---
    _pause_until = 0.0
    _after_pause = ''

    last_plot_at = 0.0
    plot_period = (1.0 / LIDAR_PLOT_HZ) if LIDAR_PLOT_HZ > 0 else float('inf')

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        # --- Background lidar plot ---
        if ENABLE_LIDAR_PLOT and ENABLE_LIDAR and now - last_plot_at >= plot_period:
            save_lidar_plot(robot)
            last_plot_at = now

        # ===================================================================
        # INIT
        # ===================================================================
        if state == 'INIT':
            start_robot(robot)
            reset_mission(robot)
            robot.set_led(LED.ORANGE, 200)
            robot.set_led(LED.GREEN, 0)
            print('[FSM] IDLE — press BTN_1 to start')
            state = 'IDLE'

        # ===================================================================
        # IDLE — raise lift and open gripper on entry, wait for BTN_1
        # ===================================================================
        elif state == 'IDLE':
            global _lift_primed
            if not _lift_primed:
                prime_lift(robot)
                _lift_primed = True
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission(robot)
                robot.set_led(LED.ORANGE, 0)
                robot.set_led(LED.GREEN, 200)
                if TEST_SIMULATE_CARRY:
                    simulate_carry(robot)
                print('[TEST] jumping to MOV2_DRIVE_5 (pre-LAPF final drive + turn + odom reset, then LAPF)')
                state = 'MOV2_DRIVE_5'

        # ===================================================================
        # TRAFFIC LIGHT
        # ===================================================================
        elif state == 'TL_TURN_LEFT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled TL turn-left')
                motion_handle = None
                state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _pause_until = now + TL_POST_TURN_PAUSE_S
                _after_pause = 'TL_WATCH'
                state = 'PAUSING'

        elif state == 'TL_WATCH':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, None, 'cancelled TL watch')
                state = 'IDLE'
            elif _see_green_light(robot):
                print('[FSM] GREEN LIGHT detected')
                robot.set_led(LED.BLUE, 0)
                print(f'[FSM] TL_TURN_BACK — turning {-LOOK_LEFT_DEG:+.0f}° back to forward (relative)')
                motion_handle = _start_turn_to(
                    robot,
                    robot.get_pose()[2] - LOOK_LEFT_DEG,
                    max_angular=TL_TURN_MAX_ANGULAR_RAD_S,
                )
                state = 'TL_TURN_BACK'
            else:
                robot.set_led(LED.BLUE, 200)

        elif state == 'TL_TURN_BACK':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled TL turn-back')
                motion_handle = None
                state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _pause_until = now + TL_POST_TURN_PAUSE_S
                _after_pause = 'MOV1_DRIVE'
                state = 'PAUSING'

        # ===================================================================
        # MOVING 1 — drive forward to patty
        # ===================================================================
        elif state == 'MOV1_DRIVE':
            print(f'[FSM] MOV1 — drive {MOV1_DISTANCE_MM:.0f} mm forward')
            motion_handle = _start_drive(
                robot, MOV1_DISTANCE_MM, MOV1_VELOCITY_MM_S, MOV1_TOLERANCE_MM,
                timeout=30.0,
            )
            state = 'MOV1_WAIT'

        elif state == 'MOV1_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV1')
                motion_handle = None
                state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                robot.stop()
                _pause_until = now + MOV1_PAUSE_S
                _after_pause = 'BUR_0_TURN'
                state = 'PAUSING'

        # ===================================================================
        # BURGER ASSEMBLY — Stop 0: LEFT BUN (pick from shelf)
        #   turn left → square perp → approach → pick bun → retreat →
        #   turn right → square parallel → drive beside the patty
        # ===================================================================
        elif state == 'BUR_0_TURN':
            _rel = BUR_FACE_TURN_DEG[0]
            print(f'[FSM] BUR_0 (left bun) — turn {_rel:+.0f}° to face shelf (relative)')
            motion_handle = _start_shelf_turn(robot, robot.get_pose()[2] + _rel)
            state = 'BUR_0_TURN_WAIT'

        elif state == 'BUR_0_TURN_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_0 turn')
                motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_iters = 0; _align_started_at = now
                _pause_until = now + 0.5
                _after_pause = 'BUR_0_ALIGN'
                state = 'PAUSING'

        elif state == 'BUR_0_ALIGN':
            _plot_ctx['standoff_mm'] = SHELF_STANDOFF_MM
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_0 align')
                motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished():
                    motion_handle = None
            else:
                corr = forward_wall_correction_deg(robot, BUR_PERP_FOV_DEG[0])
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        print('[ALIGN] BUR_0: no fit, proceeding')
                        _align_started_at = 0.0
                        _approach_started_at = now
                        _approach_start_pose = robot.get_pose()[:2]
                        state = 'BUR_0_APPROACH'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] BUR_0: aligned ({corr:+.1f}°, {_align_iters} iter)')
                    _align_started_at = 0.0
                    _approach_started_at = now
                    _approach_start_pose = robot.get_pose()[:2]
                    state = 'BUR_0_APPROACH'
                else:
                    _align_iters += 1
                    motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'BUR_0_APPROACH':
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop(); abort_to_idle(robot, None, 'cancelled BUR_0 approach'); state = 'IDLE'
            else:
                dist = closest_forward_obstacle_mm(robot)
                cur = robot.get_pose()[:2]
                adv = math.hypot(cur[0] - _approach_start_pose[0], cur[1] - _approach_start_pose[1])
                if dist <= SHELF_STANDOFF_MM or adv >= SHELF_APPROACH_MAX_DIST_MM or \
                        now - _approach_started_at > SHELF_APPROACH_TIMEOUT_S:
                    robot.stop()
                    print(f'[FSM] BUR_0 approach done — dist={dist:.0f} adv={adv:.0f}')
                    # BLOCKING: pick the left bun (empty gripper → shelf surface → grip → carry)
                    robot.step_enable(LIFT_STEPPER)
                    if not _pick(robot, LIFT_SHELF_STEPS, GRIPPER_CLOSE_BUN_DEG):
                        abort_to_idle(robot, None, 'lift failed at BUR_0'); state = 'IDLE'
                    else:
                        _retreat(robot, BUR_RETREAT_MM[0])
                        state = 'BUR_0_TURN_BACK'
                else:
                    robot.set_velocity(SHELF_APPROACH_VEL_MM_S, 0.0)

        elif state == 'BUR_0_TURN_BACK':
            _rel = BUR_UNFACE_TURN_DEG[0]
            print(f'[FSM] BUR_0 — turn back {_rel:+.0f}° to travel heading (relative)')
            motion_handle = _start_shelf_turn(robot, robot.get_pose()[2] + _rel)
            state = 'BUR_0_TURN_BACK_WAIT'

        elif state == 'BUR_0_TURN_BACK_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_0 turn-back'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_started_at = 0.0
                _pause_until = now + INTER_PAUSE_S; _after_pause = 'BUR_0_PARA'; state = 'PAUSING'

        elif state == 'BUR_0_PARA':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_0 parallel'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, BUR_PARA_FOV_DEG[0], 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        print('[ALIGN] BUR_0 parallel: no fit, proceeding')
                        _align_started_at = 0.0; state = 'BUR_0_DRIVE'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] BUR_0 parallel done ({corr:+.1f}°)')
                    _align_started_at = 0.0; state = 'BUR_0_DRIVE'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'BUR_0_DRIVE':
            motion_handle = _start_drive(robot, BUR_TRAVEL_MM[0], BURGER_TRAVEL_VEL_MM_S,
                                         BURGER_TRAVEL_TOL_MM, BURGER_TRAVEL_TIMEOUT_S)
            state = 'BUR_0_DRIVE_WAIT'

        elif state == 'BUR_0_DRIVE_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_0 drive'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None; state = 'BUR_1_TURN'

        # ===================================================================
        # BURGER ASSEMBLY — Stop 1: PATTY (drop held bun onto patty, pick both)
        #   turn left → square perp → approach → lift to patty height, open
        #   (release bun onto patty), lower to shelf, close (grip patty+bun),
        #   raise → retreat → turn right → square parallel → drive beside RB
        # ===================================================================
        elif state == 'BUR_1_TURN':
            _rel = BUR_FACE_TURN_DEG[1]
            print(f'[FSM] BUR_1 (patty) — turn {_rel:+.0f}° to face shelf (relative)')
            motion_handle = _start_shelf_turn(robot, robot.get_pose()[2] + _rel)
            state = 'BUR_1_TURN_WAIT'

        elif state == 'BUR_1_TURN_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_1 turn'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_iters = 0; _align_started_at = now
                _pause_until = now + 0.5; _after_pause = 'BUR_1_ALIGN'; state = 'PAUSING'

        elif state == 'BUR_1_ALIGN':
            _plot_ctx['standoff_mm'] = SHELF_STANDOFF_MM
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_1 align'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                corr = forward_wall_correction_deg(robot, BUR_PERP_FOV_DEG[1])
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0; _approach_started_at = now
                        _approach_start_pose = robot.get_pose()[:2]; state = 'BUR_1_APPROACH'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] BUR_1 aligned ({corr:+.1f}°)')
                    _align_started_at = 0.0; _approach_started_at = now
                    _approach_start_pose = robot.get_pose()[:2]; state = 'BUR_1_APPROACH'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'BUR_1_APPROACH':
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop(); abort_to_idle(robot, None, 'cancelled BUR_1 approach'); state = 'IDLE'
            else:
                dist = closest_forward_obstacle_mm(robot)
                cur = robot.get_pose()[:2]
                adv = math.hypot(cur[0] - _approach_start_pose[0], cur[1] - _approach_start_pose[1])
                if dist <= SHELF_STANDOFF_MM or adv >= SHELF_APPROACH_MAX_DIST_MM or \
                        now - _approach_started_at > SHELF_APPROACH_TIMEOUT_S:
                    robot.stop()
                    print(f'[FSM] BUR_1 approach done — dist={dist:.0f} adv={adv:.0f}')
                    # BLOCKING: drop held bun onto the patty, then pick patty+bun together
                    robot.step_enable(LIFT_STEPPER)
                    if not _place_layer_then_pick(robot, LIFT_PATTY_PLACE_STEPS, GRIPPER_CLOSE_PATTY_DEG):
                        abort_to_idle(robot, None, 'lift failed at BUR_1'); state = 'IDLE'
                    else:
                        _retreat(robot, BUR_RETREAT_MM[1])
                        state = 'BUR_1_TURN_BACK'
                else:
                    robot.set_velocity(SHELF_APPROACH_VEL_MM_S, 0.0)

        elif state == 'BUR_1_TURN_BACK':
            _rel = BUR_UNFACE_TURN_DEG[1]
            print(f'[FSM] BUR_1 — turn back {_rel:+.0f}° to travel heading (relative)')
            motion_handle = _start_shelf_turn(robot, robot.get_pose()[2] + _rel)
            state = 'BUR_1_TURN_BACK_WAIT'

        elif state == 'BUR_1_TURN_BACK_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_1 turn-back'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_started_at = 0.0
                _pause_until = now + INTER_PAUSE_S; _after_pause = 'BUR_1_PARA'; state = 'PAUSING'

        elif state == 'BUR_1_PARA':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_1 parallel'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, BUR_PARA_FOV_DEG[1], 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        print('[ALIGN] BUR_1 parallel: no fit, proceeding')
                        _align_started_at = 0.0; state = 'BUR_1_DRIVE'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] BUR_1 parallel done ({corr:+.1f}°)')
                    _align_started_at = 0.0; state = 'BUR_1_DRIVE'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'BUR_1_DRIVE':
            motion_handle = _start_drive(robot, BUR_TRAVEL_MM[1], BURGER_TRAVEL_VEL_MM_S,
                                         BURGER_TRAVEL_TOL_MM, BURGER_TRAVEL_TIMEOUT_S)
            state = 'BUR_1_DRIVE_WAIT'

        elif state == 'BUR_1_DRIVE_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_1 drive'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None; state = 'BUR_2_TURN'

        # ===================================================================
        # BURGER ASSEMBLY — Stop 2: RIGHT BUN (drop patty+bun onto RB, pick burger)
        #   turn left → square perp → approach → lift to bun height, open
        #   (release patty+bun onto the right bun), lower to shelf, close
        #   (grip whole burger), raise → retreat → turn right → square
        #   parallel → hand off to MOV2 (no travel drive on the last stop)
        # ===================================================================
        elif state == 'BUR_2_TURN':
            _rel = BUR_FACE_TURN_DEG[2]
            print(f'[FSM] BUR_2 (right bun) — turn {_rel:+.0f}° to face shelf (relative)')
            motion_handle = _start_shelf_turn(robot, robot.get_pose()[2] + _rel)
            state = 'BUR_2_TURN_WAIT'

        elif state == 'BUR_2_TURN_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_2 turn'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_iters = 0; _align_started_at = now
                _pause_until = now + 0.5; _after_pause = 'BUR_2_ALIGN'; state = 'PAUSING'

        elif state == 'BUR_2_ALIGN':
            _plot_ctx['standoff_mm'] = SHELF_STANDOFF_MM
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_2 align'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                corr = forward_wall_correction_deg(robot, BUR_PERP_FOV_DEG[2])
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0; _approach_started_at = now
                        _approach_start_pose = robot.get_pose()[:2]; state = 'BUR_2_APPROACH'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] BUR_2 aligned ({corr:+.1f}°)')
                    _align_started_at = 0.0; _approach_started_at = now
                    _approach_start_pose = robot.get_pose()[:2]; state = 'BUR_2_APPROACH'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'BUR_2_APPROACH':
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop(); abort_to_idle(robot, None, 'cancelled BUR_2 approach'); state = 'IDLE'
            else:
                dist = closest_forward_obstacle_mm(robot)
                cur = robot.get_pose()[:2]
                adv = math.hypot(cur[0] - _approach_start_pose[0], cur[1] - _approach_start_pose[1])
                if dist <= SHELF_STANDOFF_MM or adv >= SHELF_APPROACH_MAX_DIST_MM or \
                        now - _approach_started_at > SHELF_APPROACH_TIMEOUT_S:
                    robot.stop()
                    print(f'[FSM] BUR_2 approach done — dist={dist:.0f} adv={adv:.0f}')
                    # BLOCKING: drop patty+bun onto the right bun, then pick the whole burger
                    robot.step_enable(LIFT_STEPPER)
                    if not _place_layer_then_pick(robot, LIFT_RB_PLACE_STEPS, GRIPPER_CLOSE_BUN_DEG):
                        abort_to_idle(robot, None, 'lift failed at BUR_2'); state = 'IDLE'
                    else:
                        _retreat(robot, BUR_RETREAT_MM[2])
                        state = 'BUR_2_TURN_BACK'
                else:
                    robot.set_velocity(SHELF_APPROACH_VEL_MM_S, 0.0)

        elif state == 'BUR_2_TURN_BACK':
            _rel = BUR_UNFACE_TURN_DEG[2]
            print(f'[FSM] BUR_2 — turn back {_rel:+.0f}° to travel heading (relative)')
            motion_handle = _start_shelf_turn(robot, robot.get_pose()[2] + _rel)
            state = 'BUR_2_TURN_BACK_WAIT'

        elif state == 'BUR_2_TURN_BACK_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_2 turn-back'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_started_at = 0.0
                _pause_until = now + INTER_PAUSE_S; _after_pause = 'BUR_2_PARA'; state = 'PAUSING'

        elif state == 'BUR_2_PARA':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled BUR_2 parallel'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, BUR_PARA_FOV_DEG[2], 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        print('[ALIGN] BUR_2 parallel: no fit, proceeding')
                        _align_started_at = 0.0
                        # NOTE: the burger now ends PARALLEL to the shelf (shelf on
                        # left), not facing it as the old stack stop did, so the
                        # downstream MOV2_TURN_1_DEG geometry needs re-tuning.
                        _pause_until = now + BURGER_POST_STACK_PAUSE_S
                        _after_pause = 'MOV2_TURN_1'; state = 'PAUSING'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] BUR_2 parallel done ({corr:+.1f}°)')
                    _align_started_at = 0.0
                    _pause_until = now + BURGER_POST_STACK_PAUSE_S
                    _after_pause = 'MOV2_TURN_1'; state = 'PAUSING'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        # ===================================================================
        # MOVING 2 — scripted route with lidar-parallel corrections
        # ===================================================================

        elif state == 'MOV2_TURN_1':
            heading = robot.get_pose()[2] + MOV2_TURN_1_DEG
            print(f'[FSM] MOV2 TURN 1 — {MOV2_TURN_1_DEG:+.0f}° → {heading:.0f}°')
            motion_handle = _start_turn_to(robot, heading, max_angular=MOV2_TURN_MAX_ANGULAR_RAD_S)
            state = 'MOV2_TURN_1_WAIT'

        elif state == 'MOV2_TURN_1_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 T1'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _pause_until = now + MOV2_PAUSE_LONG_S; _after_pause = 'MOV2_ALIGN_LEFT_1'; state = 'PAUSING'

        elif state == 'MOV2_ALIGN_LEFT_1':
            _plot_ctx['standoff_mm'] = 400.0
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 AL1'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, MOV2_ALIGN_LEFT_FOV, 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        print('[ALIGN] MOV2 AL1: no fit, proceeding')
                        _align_started_at = 0.0; state = 'MOV2_DRIVE_1'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] MOV2 AL1 done ({corr:+.1f}°)')
                    _align_started_at = 0.0; state = 'MOV2_DRIVE_1'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'MOV2_DRIVE_1':
            _align_started_at = 0.0
            motion_handle = _start_drive(robot, MOV2_DRIVE_1_MM, MOV2_VELOCITY_MM_S,
                                         MOV2_TOLERANCE_MM, MOV2_DRIVE_TIMEOUT_S)
            state = 'MOV2_DRIVE_1_WAIT'

        elif state == 'MOV2_DRIVE_1_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 D1'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _pause_until = now + MOV2_PAUSE_LONG_S; _after_pause = 'MOV2_ALIGN_LEFT_2'; state = 'PAUSING'

        elif state == 'MOV2_ALIGN_LEFT_2':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 AL2'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, MOV2_ALIGN_LEFT_FOV, 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0
                        _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                        state = 'MOV2_APPROACH_2'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    _align_started_at = 0.0
                    _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                    state = 'MOV2_APPROACH_2'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'MOV2_APPROACH_2':
            # Replaces the old fixed-distance MOV2_DRIVE_2: drive forward until the
            # forward wall is at standoff (same pattern as the post-LAPF MOV3
            # approach), capped at MOV2_DRIVE_2_MM so it never overruns.
            _plot_ctx['standoff_mm'] = MOV2_APPROACH_2_STANDOFF_MM
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop(); abort_to_idle(robot, None, 'cancelled MOV2 approach 2'); state = 'IDLE'
            else:
                gap = forward_wall_gap_mm(robot)
                cur = robot.get_pose()[:2]
                adv = math.hypot(cur[0] - _approach_start_pose[0], cur[1] - _approach_start_pose[1])
                if gap <= MOV2_APPROACH_2_STANDOFF_MM or adv >= MOV2_DRIVE_2_MM or \
                        now - _approach_started_at > MOV2_DRIVE_TIMEOUT_S:
                    robot.stop()
                    print(f'[FSM] MOV2 approach 2 — wall reached gap={gap:.0f} adv={adv:.0f}')
                    _pause_until = now + MOV2_PAUSE_SHORT_S; _after_pause = 'MOV2_TURN_2'; state = 'PAUSING'
                else:
                    robot.set_velocity(MOV2_APPROACH_VEL_MM_S, 0.0)

        elif state == 'MOV2_TURN_2':
            heading = robot.get_pose()[2] + MOV2_TURN_2_DEG
            print(f'[FSM] MOV2 TURN 2 — {MOV2_TURN_2_DEG:+.0f}°')
            motion_handle = _start_turn_to(robot, heading, max_angular=MOV2_TURN_MAX_ANGULAR_RAD_S)
            state = 'MOV2_TURN_2_WAIT'

        elif state == 'MOV2_TURN_2_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 T2'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_started_at = 0.0; _align_iters = 0
                _pause_until = now + MOV2_PAUSE_LONG_S; _after_pause = 'MOV2_ALIGN_LEFT_3'; state = 'PAUSING'

        elif state == 'MOV2_ALIGN_LEFT_3':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 AL3'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, MOV2_ALIGN_LEFT_FOV, 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0
                        _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                        state = 'MOV2_APPROACH_3'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    _align_started_at = 0.0
                    _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                    state = 'MOV2_APPROACH_3'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'MOV2_APPROACH_3':
            # Wall-standoff approach replacing fixed MOV2_DRIVE_3 (see MOV2_APPROACH_2).
            _plot_ctx['standoff_mm'] = MOV2_APPROACH_3_STANDOFF_MM
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop(); abort_to_idle(robot, None, 'cancelled MOV2 approach 3'); state = 'IDLE'
            else:
                gap = forward_wall_gap_mm(robot)
                cur = robot.get_pose()[:2]
                adv = math.hypot(cur[0] - _approach_start_pose[0], cur[1] - _approach_start_pose[1])
                if gap <= MOV2_APPROACH_3_STANDOFF_MM or adv >= MOV2_DRIVE_3_MM or \
                        now - _approach_started_at > MOV2_DRIVE_TIMEOUT_S:
                    robot.stop()
                    print(f'[FSM] MOV2 approach 3 — wall reached gap={gap:.0f} adv={adv:.0f}')
                    _pause_until = now + MOV2_PAUSE_SHORT_S; _after_pause = 'MOV2_TURN_3'; state = 'PAUSING'
                else:
                    robot.set_velocity(MOV2_APPROACH_VEL_MM_S, 0.0)

        elif state == 'MOV2_TURN_3':
            heading = robot.get_pose()[2] + MOV2_TURN_3_DEG
            print(f'[FSM] MOV2 TURN 3 — {MOV2_TURN_3_DEG:+.0f}°')
            motion_handle = _start_turn_to(robot, heading, max_angular=MOV2_TURN_MAX_ANGULAR_RAD_S)
            state = 'MOV2_TURN_3_WAIT'

        elif state == 'MOV2_TURN_3_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 T3'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_started_at = 0.0; _align_iters = 0
                _pause_until = now + MOV2_PAUSE_LONG_S; _after_pause = 'MOV2_ALIGN_LEFT_4'; state = 'PAUSING'

        elif state == 'MOV2_ALIGN_LEFT_4':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 AL4'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, MOV2_ALIGN_LEFT_FOV, 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0
                        _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                        state = 'MOV2_APPROACH_4'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    _align_started_at = 0.0
                    _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                    state = 'MOV2_APPROACH_4'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'MOV2_APPROACH_4':
            # Wall-standoff approach replacing fixed MOV2_DRIVE_4 (see MOV2_APPROACH_2).
            _plot_ctx['standoff_mm'] = MOV2_APPROACH_4_STANDOFF_MM
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop(); abort_to_idle(robot, None, 'cancelled MOV2 approach 4'); state = 'IDLE'
            else:
                gap = forward_wall_gap_mm(robot)
                cur = robot.get_pose()[:2]
                adv = math.hypot(cur[0] - _approach_start_pose[0], cur[1] - _approach_start_pose[1])
                if gap <= MOV2_APPROACH_4_STANDOFF_MM or adv >= MOV2_DRIVE_4_MM or \
                        now - _approach_started_at > MOV2_DRIVE_TIMEOUT_S:
                    robot.stop()
                    print(f'[FSM] MOV2 approach 4 — wall reached gap={gap:.0f} adv={adv:.0f}')
                    _pause_until = now + MOV2_PAUSE_SHORT_S; _after_pause = 'MOV2_TURN_4'; state = 'PAUSING'
                else:
                    robot.set_velocity(MOV2_APPROACH_VEL_MM_S, 0.0)

        elif state == 'MOV2_TURN_4':
            heading = robot.get_pose()[2] + MOV2_TURN_4_DEG
            print(f'[FSM] MOV2 TURN 4 — {MOV2_TURN_4_DEG:+.0f}° (switch to right wall)')
            motion_handle = _start_turn_to(robot, heading, max_angular=MOV2_TURN_MAX_ANGULAR_RAD_S)
            state = 'MOV2_TURN_4_WAIT'

        elif state == 'MOV2_TURN_4_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 T4'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_started_at = 0.0; _align_iters = 0
                _pause_until = now + MOV2_PAUSE_LONG_S; _after_pause = 'MOV2_ALIGN_RIGHT_1'; state = 'PAUSING'

        elif state == 'MOV2_ALIGN_RIGHT_1':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 AR1'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, MOV2_ALIGN_RIGHT_FOV, 'right')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0; state = 'MOV2_DRIVE_5'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    _align_started_at = 0.0; state = 'MOV2_DRIVE_5'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'MOV2_DRIVE_5':
            _align_started_at = 0.0
            motion_handle = _start_drive(robot, MOV2_DRIVE_5_MM, MOV2_VELOCITY_MM_S,
                                         MOV2_TOLERANCE_MM, MOV2_DRIVE_TIMEOUT_S)
            state = 'MOV2_DRIVE_5_WAIT'

        elif state == 'MOV2_DRIVE_5_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 D5'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                # MOV2_ALIGN_RIGHT_2 removed (redundant) — it re-squared to the
                # same right wall as MOV2_ALIGN_RIGHT_1 after a short straight
                # MOV2_DRIVE_5, so go straight to the final turn.
                _pause_until = now + MOV2_PAUSE_LONG_S; _after_pause = 'MOV2_TURN_5'; state = 'PAUSING'

        elif state == 'MOV2_TURN_5':
            # Explicit final left turn before LAPF. LAPF's own opening pivot was
            # too sluggish here (rear-wheel drive + short leash), so pre-orient
            # the robot up the lane before handing off to LAPF.
            _align_started_at = 0.0
            heading = robot.get_pose()[2] + MOV2_TURN_5_DEG
            print(f'[FSM] MOV2 TURN 5 — {MOV2_TURN_5_DEG:+.0f}°')
            motion_handle = _start_turn_to(robot, heading, max_angular=MOV2_TURN_MAX_ANGULAR_RAD_S)
            state = 'MOV2_TURN_5_WAIT'

        elif state == 'MOV2_TURN_5_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV2 T5'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                # Reset odometry now that the robot is physically pointed up the
                # lane. This zeroes accumulated drift so LAPF's goal is reckoned
                # from a fresh (0,0,90°) frame — LAPF_GOAL=(0,2700) then sits
                # straight ahead. Everything downstream of LAPF is lidar-relative,
                # so a mid-mission reset is safe here.
                robot.reset_odometry()
                if not robot.wait_for_odometry_reset(timeout=2.0):
                    print('[warn] odometry reset before LAPF not confirmed within 2s')
                _pause_until = now + MOV2_PAUSE_LONG_S; _after_pause = 'LAPF_RUN'; state = 'PAUSING'

        # ===================================================================
        # OBSTACLE AVOIDANCE — LAPF
        # ===================================================================
        elif state == 'LAPF_RUN':
            # --- diagnostic: why does LAPF steer where it does at onset? ---
            _px, _py, _pth = robot.get_pose()
            _bearing = math.degrees(math.atan2(LAPF_GOAL[1] - _py, LAPF_GOAL[0] - _px))
            _herr = (_bearing - _pth + 180.0) % 360.0 - 180.0   # +left / -right of heading
            _obs = robot.get_obstacles()
            _left = min((o for o in _obs if o[1] > 0), key=lambda o: o[0]**2 + o[1]**2, default=None)
            _right = min((o for o in _obs if o[1] < 0), key=lambda o: o[0]**2 + o[1]**2, default=None)
            print(f'[FSM] LAPF — goal {LAPF_GOAL} pose=({_px:.0f},{_py:.0f},{_pth:.0f}°) '
                  f'goal_bearing={_bearing:.0f}° heading_err={_herr:+.0f}° '
                  f'(>0 goal-left / <0 goal-right)  nearest_left={_left} nearest_right={_right}')
            motion_handle = robot.lapf_to_goal(
                LAPF_GOAL[0], LAPF_GOAL[1],
                velocity=LAPF_VELOCITY_MM_S,
                tolerance=LAPF_TOLERANCE_MM,
                leash_length_mm=LAPF_LEASH_LENGTH_MM,
                repulsion_range_mm=LAPF_REPULSION_RANGE_MM,
                target_speed_mm_s=LAPF_TARGET_SPEED_MM_S,
                max_angular_rad_s=LAPF_MAX_ANGULAR_RAD_S,
                repulsion_gain=LAPF_REPULSION_GAIN,
                attraction_gain=LAPF_ATTRACTION_GAIN,
                force_ema_alpha=LAPF_FORCE_EMA_ALPHA,
                inflation_margin_mm=LAPF_INFLATION_MARGIN_MM,
                leash_half_angle_deg=LAPF_LEASH_HALF_ANGLE_DEG,
                blocking=False,
            )
            state = 'LAPF_WAIT'

        elif state == 'LAPF_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled LAPF'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                robot.stop()
                _pause_until = now + LAPF_PAUSE_S; _after_pause = 'DONE'; state = 'PAUSING'  # SEG2: stop at LAPF goal

        # ===================================================================
        # MOVING 3 — perp approach, turn, parallel drive
        # ===================================================================
        elif state == 'MOV3_ALIGN_PERP':
            _plot_ctx['standoff_mm'] = MOV3_WALL_STANDOFF_MM
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV3 align perp'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = forward_wall_correction_deg(robot, MOV3_ALIGN_PERP_FOV)
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        print('[ALIGN] MOV3 perp: no fit, proceeding')
                        _align_started_at = 0.0
                        _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                        state = 'MOV3_APPROACH'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] MOV3 perp done ({corr:+.1f}°)')
                    _align_started_at = 0.0
                    _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                    state = 'MOV3_APPROACH'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'MOV3_APPROACH':
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop(); abort_to_idle(robot, None, 'cancelled MOV3 approach'); state = 'IDLE'
            else:
                gap = forward_wall_gap_mm(robot)
                cur = robot.get_pose()[:2]
                adv = math.hypot(cur[0] - _approach_start_pose[0], cur[1] - _approach_start_pose[1])
                if gap <= MOV3_WALL_STANDOFF_MM or adv >= MOV3_APPROACH_MAX_MM or \
                        now - _approach_started_at > MOV3_APPROACH_TIMEOUT_S:
                    robot.stop()
                    print(f'[FSM] MOV3 wall reached — gap={gap:.0f}  adv={adv:.0f}')
                    _pause_until = now + 0.25; _after_pause = 'MOV3_TURN'; state = 'PAUSING'
                else:
                    robot.set_velocity(MOV3_APPROACH_VEL_MM_S, 0.0)

        elif state == 'MOV3_TURN':
            heading = robot.get_pose()[2] + MOV3_TURN_DEG
            print(f'[FSM] MOV3 turn {MOV3_TURN_DEG:+.0f}° → {heading:.0f}°')
            motion_handle = _start_turn_to(robot, heading, max_angular=MOV3_TURN_MAX_ANGULAR_RAD_S)
            state = 'MOV3_TURN_WAIT'

        elif state == 'MOV3_TURN_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV3 turn'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_started_at = 0.0; _align_iters = 0
                _pause_until = now + 0.5; _after_pause = 'MOV3_ALIGN_PARA'; state = 'PAUSING'

        elif state == 'MOV3_ALIGN_PARA':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV3 align para'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, MOV3_ALIGN_PARA_FOV, 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0
                        state = 'MOV3_DRIVE'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    _align_started_at = 0.0
                    state = 'MOV3_DRIVE'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'MOV3_DRIVE':
            motion_handle = _start_drive(robot, MOV3_DRIVE_MM, MOV3_VELOCITY_MM_S,
                                         MOV3_TOLERANCE_MM, MOV3_DRIVE_TIMEOUT_S)
            state = 'MOV3_DRIVE_WAIT'

        elif state == 'MOV3_DRIVE_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV3 drive'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                robot.stop()
                _pause_until = now + MOV3_POST_DRIVE_PAUSE_S; _after_pause = 'GENDER_ID'; state = 'PAUSING'

        # ===================================================================
        # GENDER IDENTIFICATION
        # ===================================================================
        elif state == 'GENDER_ID':
            global _detected_gender
            found = _detect_gender(robot)
            if found is not None:
                _detected_gender = found
                print(f'[VISION] gender = {found[0]}  score={found[1]:.2f}')
            else:
                print('[VISION] no person detected — defaulting to female')
                _detected_gender = ('female', 0.0)
            state = 'MOV4_ALIGN_PERP'

        # ===================================================================
        # MOVING 4 — navigate to drop-off position
        # ===================================================================
        elif state == 'MOV4_ALIGN_PERP':
            _plot_ctx['standoff_mm'] = MOV4_WALL_STANDOFF_MM
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV4 align perp'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = forward_wall_correction_deg(robot, MOV4_ALIGN_PERP_FOV)
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0
                        _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                        state = 'MOV4_APPROACH'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] MOV4 perp done ({corr:+.1f}°)')
                    _align_started_at = 0.0
                    _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                    state = 'MOV4_APPROACH'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'MOV4_APPROACH':
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop(); abort_to_idle(robot, None, 'cancelled MOV4 approach'); state = 'IDLE'
            else:
                gap = forward_wall_gap_mm(robot)
                cur = robot.get_pose()[:2]
                adv = math.hypot(cur[0] - _approach_start_pose[0], cur[1] - _approach_start_pose[1])
                if gap <= MOV4_WALL_STANDOFF_MM or adv >= MOV4_APPROACH_MAX_MM or \
                        now - _approach_started_at > MOV4_APPROACH_TIMEOUT_S:
                    robot.stop()
                    _pause_until = now + 0.25; _after_pause = 'MOV4_TURN'; state = 'PAUSING'
                else:
                    robot.set_velocity(MOV4_APPROACH_VEL_MM_S, 0.0)

        elif state == 'MOV4_TURN':
            heading = robot.get_pose()[2] + MOV4_TURN_DEG
            print(f'[FSM] MOV4 turn {MOV4_TURN_DEG:+.0f}° → {heading:.0f}°')
            motion_handle = _start_turn_to(robot, heading, max_angular=MOV4_TURN_MAX_ANGULAR_RAD_S)
            state = 'MOV4_TURN_WAIT'

        elif state == 'MOV4_TURN_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV4 turn'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_started_at = 0.0; _align_iters = 0
                _pause_until = now + 0.5; _after_pause = 'MOV4_ALIGN_PARA_1'; state = 'PAUSING'

        elif state == 'MOV4_ALIGN_PARA_1':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV4 AP1'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, MOV4_ALIGN_PARA_FOV, 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0
                        state = 'MOV4_DRIVE_1'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    _align_started_at = 0.0
                    state = 'MOV4_DRIVE_1'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'MOV4_DRIVE_1':
            motion_handle = _start_drive(robot, MOV4_DRIVE_1_MM, MOV4_VELOCITY_MM_S,
                                         MOV4_TOLERANCE_MM, MOV4_DRIVE_TIMEOUT_S)
            state = 'MOV4_DRIVE_1_WAIT'

        elif state == 'MOV4_DRIVE_1_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV4 D1'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_started_at = 0.0; _align_iters = 0
                _pause_until = now + MOV4_PAUSE_S; _after_pause = 'MOV4_ALIGN_PARA_2'; state = 'PAUSING'

        elif state == 'MOV4_ALIGN_PARA_2':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV4 AP2'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, MOV4_ALIGN_PARA_FOV, 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0
                        state = 'MOV4_DRIVE_2'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    _align_started_at = 0.0
                    state = 'MOV4_DRIVE_2'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'MOV4_DRIVE_2':
            label = _detected_gender[0] if _detected_gender else 'female'
            dist = MOV4_DRIVE_MALE_MM if label == 'male' else MOV4_DRIVE_FEMALE_MM
            print(f'[FSM] MOV4 D2 — gender={label}  driving {dist:.0f} mm')
            motion_handle = _start_drive(robot, dist, MOV4_VELOCITY_MM_S,
                                         MOV4_TOLERANCE_MM, MOV4_DRIVE_TIMEOUT_S)
            state = 'MOV4_DRIVE_2_WAIT'

        elif state == 'MOV4_DRIVE_2_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled MOV4 D2'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _pause_until = now + 0.25; _after_pause = 'DROP_TURN'; state = 'PAUSING'

        # ===================================================================
        # BURGER DROP-OFF
        # ===================================================================
        elif state == 'DROP_TURN':
            heading = robot.get_pose()[2] + DROP_SHELF_TURN_DEG
            print(f'[FSM] DROPOFF — turn {DROP_SHELF_TURN_DEG:+.0f}° to face shelf → {heading:.0f}°')
            motion_handle = _start_shelf_turn(robot, heading)
            state = 'DROP_TURN_WAIT'

        elif state == 'DROP_TURN_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled DROP turn'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                _align_started_at = 0.0; _align_iters = 0
                _pause_until = now + 0.5; _after_pause = 'DROP_ALIGN'; state = 'PAUSING'

        elif state == 'DROP_ALIGN':
            _plot_ctx['standoff_mm'] = DROP_SHELF_STANDOFF_MM
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled DROP align'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = forward_wall_correction_deg(robot, DROP_SHELF_ALIGN_FOV)
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        _align_started_at = 0.0
                        _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                        state = 'DROP_APPROACH'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] DROP aligned ({corr:+.1f}°)')
                    _align_started_at = 0.0
                    _approach_started_at = now; _approach_start_pose = robot.get_pose()[:2]
                    state = 'DROP_APPROACH'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        elif state == 'DROP_APPROACH':
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop(); abort_to_idle(robot, None, 'cancelled DROP approach'); state = 'IDLE'
            else:
                dist = closest_forward_obstacle_mm(robot)
                cur = robot.get_pose()[:2]
                adv = math.hypot(cur[0] - _approach_start_pose[0], cur[1] - _approach_start_pose[1])
                if dist <= DROP_SHELF_STANDOFF_MM or adv >= DROP_APPROACH_MAX_MM or \
                        now - _approach_started_at > DROP_APPROACH_TIMEOUT_S:
                    robot.stop()
                    print(f'[FSM] DROP approach done — dist={dist:.0f}')
                    # BLOCKING: lower + open + raise + retreat
                    robot.step_enable(LIFT_STEPPER)
                    if not _place(robot, DROP_LIFT_PLACE_STEPS):
                        abort_to_idle(robot, None, 'lift failed at DROP'); state = 'IDLE'
                    else:
                        _retreat(robot, DROP_RETREAT_MM)
                        robot.set_led(LED.GREEN, 200)
                        _pause_until = now + DROP_POST_RELEASE_PAUSE_S
                        _after_pause = 'DROP_TURN_BACK'; state = 'PAUSING'
                else:
                    robot.set_velocity(DROP_APPROACH_VEL_MM_S, 0.0)

        elif state == 'DROP_TURN_BACK':
            heading = robot.get_pose()[2] + DROP_TURN_BACK_DEG
            print(f'[FSM] DROP — turn back {DROP_TURN_BACK_DEG:+.0f}° → {heading:.0f}°')
            motion_handle = _start_shelf_turn(robot, heading)
            state = 'DROP_TURN_BACK_WAIT'

        elif state == 'DROP_TURN_BACK_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled DROP turn back'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                # The shelf turn-back rarely lands the heading exactly; re-square
                # against the left wall with the lidar before the stop-sign leg.
                _align_started_at = 0.0
                _pause_until = now + 0.5; _after_pause = 'DROP_ALIGN_LEFT'; state = 'PAUSING'

        elif state == 'DROP_ALIGN_LEFT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled DROP align-left'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None:
                if motion_handle.is_finished(): motion_handle = None
            else:
                if _align_started_at == 0.0: _align_started_at = now; _align_iters = 0
                corr = side_wall_correction_deg(robot, DROP_TURN_BACK_ALIGN_FOV, 'left')
                if corr is None:
                    if now - _align_started_at >= WALL_ALIGN_TIMEOUT_S:
                        print('[ALIGN] DROP align-left: no fit, proceeding')
                        _align_started_at = 0.0; state = 'STOP_DRIVE'
                elif abs(corr) <= WALL_ALIGN_TOLERANCE_DEG or _align_iters >= WALL_ALIGN_MAX_ITERS:
                    print(f'[ALIGN] DROP align-left done ({corr:+.1f}°, {_align_iters} iter)')
                    _align_started_at = 0.0; state = 'STOP_DRIVE'
                else:
                    _align_iters += 1; motion_handle = _start_turn_to(robot, robot.get_pose()[2] + corr)

        # ===================================================================
        # STOP SIGN — drive forward until stop sign detected, wait, final drive
        # ===================================================================
        elif state == 'STOP_DRIVE':
            print(f'[FSM] STOP_SIGN — driving until sign confidence ≥ {MIN_STOP_SIGN_CONFIDENCE:.2f}')
            _approach_started_at = now
            _approach_start_pose = robot.get_pose()[:2]
            state = 'STOP_DRIVE_LOOP'

        elif state == 'STOP_DRIVE_LOOP':
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop(); abort_to_idle(robot, None, 'cancelled STOP drive'); state = 'IDLE'
            elif _see_stop_sign(robot):
                robot.stop()
                print(f'[FSM] STOP SIGN detected — driving {STOP_SIGN_APPROACH_MM:.0f} mm before stop')
                state = 'STOP_APPROACH'
            elif now - _approach_started_at > STOP_SIGN_TIMEOUT_S:
                robot.stop()
                print('[STOP] timeout — no stop sign, proceeding')
                _pause_until = now + STOP_SIGN_WAIT_S; _after_pause = 'STOP_FINAL_DRIVE'; state = 'PAUSING'
            else:
                robot.set_velocity(STOP_SIGN_VEL_MM_S, 0.0)

        elif state == 'STOP_APPROACH':
            # Stop sign read: keep driving, then stop + wait. Female drives an extra
            # STOP_SIGN_APPROACH_FEMALE_EXTRA_MM beyond the male/base distance.
            _label = _detected_gender[0] if _detected_gender else 'female'
            _approach_mm = STOP_SIGN_APPROACH_MM
            if _label != 'male':
                _approach_mm += STOP_SIGN_APPROACH_FEMALE_EXTRA_MM
            print(f'[FSM] STOP approach — gender={_label} drive={_approach_mm:.0f} mm')
            motion_handle = _start_drive(robot, _approach_mm, STOP_SIGN_VEL_MM_S,
                                         STOP_SIGN_TOLERANCE_MM, timeout=15.0)
            state = 'STOP_APPROACH_WAIT'

        elif state == 'STOP_APPROACH_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled STOP approach'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None
                robot.stop()
                print(f'[FSM] STOP — stopped, waiting {STOP_SIGN_WAIT_S:.1f}s')
                _pause_until = now + STOP_SIGN_WAIT_S; _after_pause = 'STOP_FINAL_DRIVE'; state = 'PAUSING'

        elif state == 'STOP_FINAL_DRIVE':
            print(f'[FSM] STOP final drive — {STOP_SIGN_FINAL_MM:.0f} mm')
            motion_handle = _start_drive(robot, STOP_SIGN_FINAL_MM, STOP_SIGN_VEL_FINAL_MM_S,
                                         STOP_SIGN_TOLERANCE_MM, timeout=15.0)
            state = 'STOP_FINAL_WAIT'

        elif state == 'STOP_FINAL_WAIT':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled final drive'); motion_handle = None; state = 'IDLE'
            elif motion_handle is not None and motion_handle.is_finished():
                motion_handle = None; state = 'DONE'

        # ===================================================================
        # DONE
        # ===================================================================
        elif state == 'DONE':
            robot.stop()
            robot.set_led(LED.GREEN, 200)
            robot.set_led(LED.BLUE, 200)
            # Mark the lift as already primed so re-entering IDLE does NOT
            # re-run prime_lift — at mission end the lift is already at carry and
            # the gripper is open, and re-priming would command another +carry
            # raise (driving the stepper up again) and move the servo. Keep both
            # still after the run finishes. (BTN_1 re-clears this to restart.)
            _lift_primed = True
            print('[FSM] DONE — mission complete. Press BTN_1 to restart.')
            state = 'IDLE'

        # ===================================================================
        # PAUSING (shared pause state)
        # ===================================================================
        elif state == 'PAUSING':
            if robot.was_button_pressed(Button.BTN_2):
                abort_to_idle(robot, motion_handle, 'cancelled during pause')
                motion_handle = None; state = 'IDLE'
            elif now >= _pause_until:
                state = _after_pause

        else:
            print(f'[FSM] unknown state: {state!r}')
            state = 'IDLE'

        # --- Rate limiter ---
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
