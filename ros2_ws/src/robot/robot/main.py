"""
main.py — student entry point
==============================
This is the only file you need to edit.

`run(robot)` is called by robot_node.py after ROS is set up and the Robot
instance is ready. Write your FSM here and call fsm.spin() at the end.

To run:
    ros2 run robot robot
"""

from robot.robot import Robot, FirmwareState
from robot.robot_fsm import RobotFSM


# ---------------------------------------------------------------------------
# LED identifiers for set_led()
#
# Usage:
#   self.robot.set_led(LED.GREEN, 255)   # full brightness
#   self.robot.set_led(LED.RED,   0)     # off
# ---------------------------------------------------------------------------

class LED:
    RED    = 0   # Discrete red LED    (PWM)
    GREEN  = 1   # Discrete green LED  (PWM)
    BLUE   = 2   # Discrete blue LED   (PWM)
    ORANGE = 3   # Discrete orange LED (PWM)
    PURPLE = 4   # Discrete purple LED (non-PWM — brightness is 0 or 255 only)

    OFF = 0      # Brightness: off
    ON  = 255    # Brightness: full on


class MyFSM(RobotFSM):
    """
    Example FSM with two states: IDLE and MOVING.

    Button 1 → start moving forward
    Button 2 → stop

    ┌────────┐  to_moving  ┌────────┐
    │  IDLE  │ ──────────> │ MOVING │
    │        │ <────────── │        │
    └────────┘   to_idle   └────────┘
    """

    # ------------------------------------------------------------------
    # Part 1 — State machine setup
    #
    # add_transition(from_state, event, to_state, action)
    #   from_state : which state this transition starts from
    #   event      : the name you pass to trigger() to fire it
    #   to_state   : which state to move to
    #   action     : method to call once, at the moment of transition
    #
    # The event name and action method share the same word so it is easy
    # to trace:  trigger("to_moving") → _on_to_moving()
    # ------------------------------------------------------------------

    def __init__(self, robot: Robot) -> None:
        super().__init__(robot, initial_state="IDLE")

        self.add_transition("IDLE",   "to_moving", "MOVING", action=self._on_to_moving)
        self.add_transition("MOVING", "to_idle",   "IDLE",   action=self._on_to_idle)

    # ------------------------------------------------------------------
    # Part 2 — Continuous logic (runs every spin cycle, ~20 Hz)
    #
    # update() is called repeatedly by spin(). Use it to:
    #   - read sensors or buttons
    #   - send continuous commands (LEDs, velocity adjustments)
    #   - decide when to trigger a transition
    #
    # Calling self.robot.xxx() here talks directly to hardware but does
    # NOT change the FSM state — it just sends a command right now.
    # Calling self.trigger("event") is what actually changes state.
    # ------------------------------------------------------------------

    def update(self) -> None:
        state = self.get_state()

        if state == "IDLE":
            # Continuous output while IDLE — runs every cycle
            self.robot.set_led(LED.GREEN, LED.OFF)
            self.robot.set_led(LED.RED,   LED.ON)

            # Transition condition: button 1 pressed → fire "to_moving" event
            # trigger() changes state to MOVING and calls _on_to_moving() once
            if self.robot.get_button(1):
                self.trigger("to_moving")

        elif state == "MOVING":
            # Continuous output while MOVING — runs every cycle
            self.robot.set_led(LED.RED,   LED.OFF)
            self.robot.set_led(LED.GREEN, LED.ON)

            # Transition condition: button 2 pressed → fire "to_idle" event
            # trigger() changes state to IDLE and calls _on_to_idle() once
            if self.robot.get_button(2):
                self.trigger("to_idle")

    # ------------------------------------------------------------------
    # Part 3 — Transition actions (called once, at the moment of change)
    #
    # These run exactly once when the transition fires — not every cycle.
    # Use them for one-shot commands: enabling the firmware, setting an
    # initial velocity, or cleanly stopping motors.
    # ------------------------------------------------------------------

    def _on_to_moving(self) -> None:
        """Called once when entering MOVING."""
        self.robot.set_state(FirmwareState.RUNNING)       # enable firmware
        self.robot.set_velocity(linear=100, angular=0.0)  # 100 mm/s forward

    def _on_to_idle(self) -> None:
        """Called once when entering IDLE."""
        self.robot.stop()                                  # halt motors
        self.robot.set_state(FirmwareState.IDLE)           # disable firmware


def run(robot: Robot) -> None:
    fsm = MyFSM(robot)
    fsm.spin(hz=20)
