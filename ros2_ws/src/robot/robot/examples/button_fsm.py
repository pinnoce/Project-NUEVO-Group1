"""
Example: Button-Triggered FSM
==============================
Demonstrates a multi-state FSM driven entirely by button presses.
No path planning — useful for testing hardware and understanding the FSM
structure before adding motion.

HOW TO APPLY THIS EXAMPLE
--------------------------
Copy ButtonFSM and run() into your main.py. Then run:

    ros2 run robot robot

You can also keep your own FSM and borrow individual state actions (e.g.,
the LED feedback pattern or the blocking button wait trick).

WHAT IT DOES
------------
States and transitions:

    IDLE ──(button 1)──► FORWARD   robot drives forward at 150 mm/s
    FORWARD ──(button 2)──► TURNING  robot turns 90° CCW in place
    TURNING ──(turn done)──► IDLE    robot stops; NeoPixel flashes red

Button 1 always restarts from IDLE.
Button 3 triggers ESTOP from any state.

TWO STYLES OF BUTTON HANDLING SHOWN
--------------------------------------
1. Polling in update()  — used for button 1 and 3.
   Fast response, but update() runs at the spin rate (default 20 Hz).

2. Blocking wait in an action — used to confirm the turn is done.
   Blocks the FSM loop; only use for short waits.
"""

import time

from robot.robot import Robot, FirmwareState
from robot.robot_fsm import RobotFSM


class ButtonFSM(RobotFSM):

    ALL_STATES = ["IDLE", "FORWARD", "TURNING"]

    def __init__(self, robot: Robot) -> None:
        super().__init__(robot, initial_state="IDLE")

        self.add_transition("IDLE",    "go",    "FORWARD", action=self._start_forward)
        self.add_transition("FORWARD", "turn",  "TURNING", action=self._start_turn)
        self.add_transition("TURNING", "done",  "IDLE",    action=self._on_idle)

        # ESTOP from any state
        for s in self.ALL_STATES:
            self.add_transition(s, "estop", "IDLE", action=self._emergency_stop)

    # ------------------------------------------------------------------
    # Polling  (called every spin cycle)
    # ------------------------------------------------------------------

    def update(self) -> None:
        # Button 3 — emergency stop from any state (polling style)
        if self.robot.get_button(3):
            self.trigger("estop")
            return

        state = self.get_state()

        if state == "IDLE":
            # Button 1 — start driving (polling style)
            if self.robot.get_button(1):
                self.trigger("go")

        elif state == "FORWARD":
            # Button 2 — begin turn (polling style)
            if self.robot.get_button(2):
                self.trigger("turn")

        elif state == "TURNING":
            # Transition when motion completes
            if not self.robot.is_moving():
                self.trigger("done")

    # ------------------------------------------------------------------
    # State actions
    # ------------------------------------------------------------------

    def _start_forward(self) -> None:
        self.robot.set_state(FirmwareState.RUNNING)
        self.robot.set_velocity(linear=150, angular_deg_s=0)
        self.robot.set_neopixel(0, 0, 100, 255)   # blue = moving

    def _start_turn(self) -> None:
        self.robot.stop()
        # Non-blocking turn; update() watches is_moving() for completion
        self.robot.turn_by(90, blocking=False, tolerance_deg=3)
        self.robot.set_neopixel(0, 255, 165, 0)   # orange = turning

    def _on_idle(self) -> None:
        self.robot.stop()
        # Flash NeoPixel red three times to signal return to IDLE
        for _ in range(3):
            self.robot.set_neopixel(0, 255, 0, 0)
            time.sleep(0.15)
            self.robot.set_neopixel(0, 0, 0, 0)
            time.sleep(0.15)

    def _emergency_stop(self) -> None:
        self.robot.cancel_motion()
        self.robot.stop()
        self.robot.set_neopixel(0, 255, 0, 0)   # solid red
        # Blocking wait: hold here until button 1 is pressed to re-arm
        # This intentionally blocks the FSM loop — the robot is stopped.
        self.robot.wait_for_button(1)
        self.robot.set_neopixel(0, 0, 0, 0)


def run(robot: Robot) -> None:
    fsm = ButtonFSM(robot)
    fsm.spin(hz=20)
