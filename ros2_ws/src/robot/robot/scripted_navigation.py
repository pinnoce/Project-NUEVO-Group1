import time
from dataclasses import dataclass

from robot.robot import Robot


@dataclass
class MotionStep:
    duration_s: float
    linear_mm_s: float
    angular_deg_s: float


# ---------------------------------------------------------------------------
# Scripted paths
# ---------------------------------------------------------------------------
# Scripted navigation does NOT know world coordinates.
# It just runs timed motion commands.
#
# Positive linear_mm_s = forward
# Negative linear_mm_s = backward
#
# Positive angular_deg_s = turn left / CCW
# Negative angular_deg_s = turn right / CW
#
# To tune the course:
#   1. Increase duration_s to go farther / turn longer.
#   2. Decrease duration_s to go shorter / turn less.
#   3. Increase linear_mm_s to drive faster.
#   4. Increase angular_deg_s magnitude to turn faster.
#
# The sequence below is only a starting template.
# You MUST tune these numbers on your actual course.


SCRIPTED_PATHS: dict[str, list[MotionStep]] = {
    "TEST_FORWARD": [
        MotionStep(duration_s=2.0, linear_mm_s=150.0, angular_deg_s=0.0),
    ],

    # Full no-pickup route:
    # Goes to the patty first, then bottom bun, then top bun, then dropoff.
    #
    # Replace these motions with your measured/tested sequence.
    "BURGER_NO_PICKUP": [
        # Go to patty first.
        MotionStep(duration_s=2.0, linear_mm_s=150.0, angular_deg_s=0.0),
        MotionStep(duration_s=0.7, linear_mm_s=0.0, angular_deg_s=45.0),
        MotionStep(duration_s=1.0, linear_mm_s=120.0, angular_deg_s=0.0),

        # Pause near patty.
        MotionStep(duration_s=0.5, linear_mm_s=0.0, angular_deg_s=0.0),

        # Go to bottom bun.
        MotionStep(duration_s=0.7, linear_mm_s=0.0, angular_deg_s=-45.0),
        MotionStep(duration_s=1.5, linear_mm_s=120.0, angular_deg_s=0.0),

        # Pause near bottom bun.
        MotionStep(duration_s=0.5, linear_mm_s=0.0, angular_deg_s=0.0),

        # Go to top bun.
        MotionStep(duration_s=1.0, linear_mm_s=0.0, angular_deg_s=45.0),
        MotionStep(duration_s=1.5, linear_mm_s=120.0, angular_deg_s=0.0),

        # Pause near top bun.
        MotionStep(duration_s=0.5, linear_mm_s=0.0, angular_deg_s=0.0),

        # Go to dropoff.
        MotionStep(duration_s=0.8, linear_mm_s=0.0, angular_deg_s=-45.0),
        MotionStep(duration_s=2.0, linear_mm_s=150.0, angular_deg_s=0.0),
    ],
}


_script_name: str | None = None
_script_step_index: int = 0
_script_step_started_at: float = 0.0


def reset_scripted_motion() -> None:
    global _script_name
    global _script_step_index
    global _script_step_started_at

    _script_name = None
    _script_step_index = 0
    _script_step_started_at = 0.0


def drive_scripted_motion(robot: Robot, segment_name: str) -> bool:
    """
    Run one scripted motion sequence.

    Returns:
        True when the full sequence is done.
        False while the sequence is still running.
    """
    global _script_name
    global _script_step_index
    global _script_step_started_at

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

        print(
            f"[SCRIPTED NAV] {segment_name}: "
            f"step {_script_step_index + 1}/{len(steps)}"
        )

    robot.set_velocity(
        current_step.linear_mm_s,
        current_step.angular_deg_s,
    )

    return False
