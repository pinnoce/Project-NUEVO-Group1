from __future__ import annotations

import time
from dataclasses import dataclass

from robot.robot import Robot

# =============================================================================
# Scripted navigation-only helpers
# =============================================================================
# This file is intentionally isolated from main.py and from gps_navigation.py.
#
# Behavior:
#   - Run scripted motion segments to visit 3 configured target locations.
#   - Run the configured drop-off motion segment.
#   - Do NOT enable servos, move the claw, close the claw, lift, or release.
# =============================================================================


@dataclass(frozen=True)
class MotionStep:
    duration_s: float
    linear_mm_s: float
    angular_deg_s: float
    label: str = ""


@dataclass(frozen=True)
class ScriptedTarget:
    name: str
    approach_segment: str


# Backward-compatible alias in case older test code imports PickupTarget.
PickupTarget = ScriptedTarget


# -----------------------------------------------------------------------------
# Course-specific scripted paths
# -----------------------------------------------------------------------------
# Replace these placeholder timing values with your measured course values.
# Each target segment should end with the rover at the target location.
# Because this is navigation-only, no claw action happens after each segment.

SCRIPTED_PATHS: dict[str, list[MotionStep]] = {
    "TEST_FORWARD": [
        MotionStep(2.0, 120.0, 0.0, "drive forward"),
    ],
    "GO_TO_BOTTOM_BUN_LOCATION": [
        MotionStep(1.2, 120.0, 0.0, "leave start"),
        MotionStep(0.7, 0.0, 35.0, "turn toward bottom bun location"),
        MotionStep(1.0, 90.0, 0.0, "drive to bottom bun location"),
    ],
    "GO_TO_PATTY_LOCATION": [
        MotionStep(0.8, -80.0, 0.0, "back away from bottom bun location"),
        MotionStep(0.9, 0.0, -40.0, "turn toward patty location"),
        MotionStep(1.2, 100.0, 0.0, "drive to patty location"),
    ],
    "GO_TO_TOP_BUN_LOCATION": [
        MotionStep(0.8, -80.0, 0.0, "back away from patty location"),
        MotionStep(0.8, 0.0, 40.0, "turn toward top bun location"),
        MotionStep(1.2, 100.0, 0.0, "drive to top bun location"),
    ],
    "GO_TO_DROPOFF_LOCATION": [
        MotionStep(0.8, -80.0, 0.0, "back away after top bun location"),
        MotionStep(1.0, 0.0, -45.0, "turn toward dropoff location"),
        MotionStep(2.0, 120.0, 0.0, "drive to dropoff location"),
    ],
}

SCRIPTED_TARGET_SEQUENCE: list[ScriptedTarget] = [
    ScriptedTarget("bottom_bun_location", "GO_TO_BOTTOM_BUN_LOCATION"),
    ScriptedTarget("patty_location", "GO_TO_PATTY_LOCATION"),
    ScriptedTarget("top_bun_location", "GO_TO_TOP_BUN_LOCATION"),
]

# Backward-compatible alias in case older test code imports SCRIPTED_OBJECT_SEQUENCE.
SCRIPTED_OBJECT_SEQUENCE = SCRIPTED_TARGET_SEQUENCE

# -----------------------------------------------------------------------------
# Internal state for non-blocking scripted motion.

_script_name: str | None = None
_script_step_index: int = 0
_script_step_started_at: float = 0.0


# =============================================================================
# Basic helpers
# =============================================================================


def reset_scripted_motion() -> None:
    global _script_name, _script_step_index, _script_step_started_at
    _script_name = None
    _script_step_index = 0
    _script_step_started_at = 0.0


def reset_all_scripted_state() -> None:
    reset_scripted_motion()


# =============================================================================
# Non-blocking motion functions
# =============================================================================


def drive_scripted_motion(robot: Robot, segment_name: str) -> bool:
    """
    Run one dead-reckoned motion segment without blocking the caller.

    Returns True once the segment is complete. Call this repeatedly from a loop
    running at DEFAULT_FSM_HZ. The function owns its own simple state machine.
    """
    global _script_name, _script_step_index, _script_step_started_at

    if segment_name not in SCRIPTED_PATHS:
        robot.stop()
        raise ValueError(f"Unknown scripted path: {segment_name}")

    now = time.monotonic()
    steps = SCRIPTED_PATHS[segment_name]

    if _script_name != segment_name:
        _script_name = segment_name
        _script_step_index = 0
        _script_step_started_at = now
        print(f"[SCRIPTED NAV] starting segment: {segment_name}")

    if _script_step_index >= len(steps):
        robot.stop()
        reset_scripted_motion()
        return True

    current_step = steps[_script_step_index]
    elapsed = now - _script_step_started_at

    if elapsed >= current_step.duration_s:
        _script_step_index += 1
        _script_step_started_at = now

        if _script_step_index >= len(steps):
            robot.stop()
            reset_scripted_motion()
            print(f"[SCRIPTED NAV] completed segment: {segment_name}")
            return True

        current_step = steps[_script_step_index]

    robot.set_velocity(current_step.linear_mm_s, current_step.angular_deg_s)
    return False


# =============================================================================
# Scripted 3-target navigation-only mission state machine
# =============================================================================


class ScriptedNavigationMission:
    """Non-blocking mission: visit 3 scripted target locations, then dropoff."""

    def __init__(self, targets: list[ScriptedTarget] | None = None) -> None:
        self.targets = targets if targets is not None else SCRIPTED_TARGET_SEQUENCE
        self.index = 0
        self.phase = "NAV_TO_TARGET"
        self.done = False

    def reset(self) -> None:
        self.index = 0
        self.phase = "NAV_TO_TARGET"
        self.done = False
        reset_all_scripted_state()

    def update(self, robot: Robot) -> bool:
        if self.done:
            robot.stop()
            return True

        if self.phase == "NAV_TO_TARGET":
            target = self.targets[self.index]
            if drive_scripted_motion(robot, target.approach_segment):
                print(f"[SCRIPTED MISSION] visited {target.name}; no pickup command sent")
                self.index += 1
                self.phase = "NAV_TO_DROPOFF" if self.index >= len(self.targets) else "NAV_TO_TARGET"
            return False

        if self.phase == "NAV_TO_DROPOFF":
            if drive_scripted_motion(robot, "GO_TO_DROPOFF_LOCATION"):
                robot.stop()
                self.done = True
                print("[SCRIPTED MISSION] navigation-only mission complete; no dropoff command sent")
                return True
            return False

        robot.stop()
        raise RuntimeError(f"Unknown scripted mission phase: {self.phase}")


# Backward-compatible alias in case an older test imports ScriptedPickupMission.
ScriptedPickupMission = ScriptedNavigationMission
