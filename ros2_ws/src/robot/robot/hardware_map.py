from __future__ import annotations

from enum import IntEnum


class Motor(IntEnum):
    DC_M1 = 1
    DC_M2 = 2
    DC_M3 = 3
    DC_M4 = 4


class Stepper(IntEnum):
    STEPPER_1 = 1
    STEPPER_2 = 2
    STEPPER_3 = 3
    STEPPER_4 = 4


class ServoChannel(IntEnum):
    CH_1 = 1
    CH_2 = 2
    CH_3 = 3
    CH_4 = 4
    CH_5 = 5
    CH_6 = 6
    CH_7 = 7
    CH_8 = 8
    CH_9 = 9
    CH_10 = 10
    CH_11 = 11
    CH_12 = 12
    CH_13 = 13
    CH_14 = 14
    CH_15 = 15
    CH_16 = 16


class Button(IntEnum):
    BTN_1 = 1
    BTN_2 = 2
    BTN_3 = 3
    BTN_4 = 4
    BTN_5 = 5
    BTN_6 = 6
    BTN_7 = 7
    BTN_8 = 8
    BTN_9 = 9
    BTN_10 = 10


class Limit(IntEnum):
    LIM_1 = 1
    LIM_2 = 2
    LIM_3 = 3
    LIM_4 = 4
    LIM_5 = 5
    LIM_6 = 6
    LIM_7 = 7
    LIM_8 = 8


class LED(IntEnum):
    RED = 0
    GREEN = 1
    BLUE = 2
    ORANGE = 3
    PURPLE = 4


# These rates mirror firmware/arduino/src/config.h.
UART_TASK_HZ = 100
SYS_STATE_RUN_HZ = 10
SYS_POWER_RUN_HZ = 10
DC_STATE_HZ = 50
STEP_STATE_HZ = 50
SERVO_STATE_HZ = 10
IMU_HZ = 25
KINEMATICS_HZ = 25
IO_INPUT_HZ = 50
IO_OUTPUT_HZ = 10

# Robot-side defaults should match the fastest upstream signal they consume.
DEFAULT_FSM_HZ = IO_INPUT_HZ
DEFAULT_NAV_HZ = KINEMATICS_HZ
