# Robot Node Design

## Overview

The `robot` package is structured in three layers on top of the existing `bridge` node:

```
┌──────────────────────────────────────────────┐
│  Layer 3 — Path Planner (PathPlanner)         │  pure pursuit; APF added when lidar ready
├──────────────────────────────────────────────┤
│  Layer 2 — FSM (RobotFSM base class)          │  student-authored logic
├──────────────────────────────────────────────┤
│  Layer 1 — Robot API (Robot)                  │  wraps all bridge topics/services
├──────────────────────────────────────────────┤
│  bridge node  (existing)                      │  raw ROS ↔ firmware TLV
└──────────────────────────────────────────────┘
```

`Robot` is a plain Python class, not a ROS node. It is constructed with a reference to
the live `RobotNode` and uses it to create subscriptions and publishers. The `RobotNode`
(`robot_node.py`) is the only ROS node in this package.

**Student entry point is `main.py`.** Students write their FSM there and do not touch
`robot_node.py`. `robot_node.py` handles all ROS setup and then calls `main.run(robot)`.

---

## Layer 1 — Robot (`robot.py`)

### Construction

```python
from enum import Enum

class Unit(Enum):
    MM   = 1.0    # native firmware units; no conversion
    INCH = 25.4   # 1 inch = 25.4 mm

class Robot:
    def __init__(self, node: rclpy.node.Node, unit: Unit = Unit.MM,
                 wheel_diameter_mm: float = 74.0,
                 wheel_base_mm: float = 333.0,
                 encoder_ppr: int = 1440):
```

`unit` controls every length/velocity input and output (position, velocity, wheel params).
Internally everything is stored and sent in mm / ticks. The conversion is:

```
mm = user_value × unit.value
user_value = mm / unit.value
ticks_per_s = mm_per_s × encoder_ppr / (π × wheel_diameter_mm)
```

### Internal state

`Robot` keeps a local mirror of firmware state, updated by ROS subscriptions. A
`threading.Lock` protects the shared state because the ROS spin thread writes it and the
FSM/path-planner threads read it.

---

### Full API

#### System

| Method | Blocking | Notes |
|---|---|---|
| `set_state(state: FirmwareState)` | yes (service) | transitions IDLE/RUNNING/ESTOP |
| `get_state() → FirmwareState` | no | reads cached `/sys_state` |
| `estop()` | yes | shorthand for `set_state(ESTOP)` |
| `reset_estop()` | yes | shorthand for `set_state(IDLE)` |
| `reset_odometry()` | no | publishes `/sys_odom_reset` |
| `get_power() → SystemPower` | no | cached `/sys_power` |

#### Pose / odometry

| Method | Notes |
|---|---|
| `get_pose() → (x, y, theta_deg)` | x, y in user units; theta in degrees |
| `get_velocity() → (vx, vy, v_theta_deg_s)` | vx, vy in user units/s; v_theta in degrees/s |
| `wait_for_pose_update(timeout=None) → bool` | blocks until next `/sensor_kinematics` tick (~25 Hz) |

#### Differential drive — velocity

| Method | Blocking | Notes |
|---|---|---|
| `set_velocity(linear, angular_deg_s)` | no | body-frame: linear in user-units/s, angular in degrees/s; firmware uses diff-drive mixing |
| `set_left_wheel(motor_id)` / `set_right_wheel(motor_id)` | no | configure which DC motors are used by diff-drive helpers |
| `set_motor_velocity(motor_id, velocity)` | no | per-motor; velocity in user-units/s → ticks/s |
| `stop()` | no | zero velocity on both drive motors; does NOT ESTOP |

Diff-drive mixing done by the API:
```
left_mm_s  = linear_mm_s − angular_rad_s × wheel_base_mm / 2
right_mm_s = linear_mm_s + angular_rad_s × wheel_base_mm / 2
```

#### Differential drive — navigation (motion commands)

All navigation commands check that firmware is in RUNNING state before sending.

`blocking=True` — method blocks until the goal is reached or `timeout` expires, then
returns `True` (success) or `False` (timeout).

`blocking=False` — method returns immediately; use the returned `MotionHandle` to poll
or wait later.

| Method | Default | Notes |
|---|---|---|
| `move_to(x, y, velocity, blocking=True, tolerance=10, timeout=None) → bool / MotionHandle` | blocking | navigate to (x, y) in user units at velocity user-units/s |
| `move_by(dx, dy, velocity, blocking=True, tolerance=10, timeout=None) → bool / MotionHandle` | blocking | relative move |
| `turn_to(angle_deg, blocking=True, tolerance_deg=2, timeout=None) → bool / MotionHandle` | blocking | rotate to absolute heading |
| `turn_by(delta_deg, blocking=True, tolerance_deg=2, timeout=None) → bool / MotionHandle` | blocking | rotate by delta |
| `is_moving() → bool` | — | True if a motion command is active |
| `cancel_motion()` | — | abort current navigation command; calls `stop()` |

`MotionHandle` (returned when `blocking=False`):
```python
handle.wait(timeout=None) → bool   # block until done
handle.is_done() → bool            # non-blocking poll
handle.cancel()                    # abort
```

#### DC motors — low-level

| Method | Blocking | Notes |
|---|---|---|
| `set_motor_pwm(motor_id, pwm)` | no | raw PWM −255…255 |
| `set_motor_position(motor_id, ticks, blocking=True, timeout=None)` | optional | position control mode |
| `enable_motor(motor_id, mode=DCMotorMode.VELOCITY)` | no | publishes `DCEnable`; mode follows firmware enum |
| `disable_motor(motor_id)` | no | publishes `DCEnable` |
| `home_motor(motor_id, blocking=True, timeout=None)` | optional | publishes `DCHome` |
| `reset_motor_position(motor_id)` | no | publishes `DCResetPosition` |
| `set_pid_gains(motor_id, loop_type, kp, ki, kd, max_output, max_integral)` | no | publishes `DCPidSet`; `loop_type` uses `DCPidLoop` (`POSITION=0`, `VELOCITY=1`) |
| `request_pid(motor_id, loop_type)` | no | publishes `DCPidReq`; result arrives on `/dc_pid_rsp` |
| `get_dc_state() → DCStateAll` | no | cached `/dc_state_all` |

#### Stepper motors

| Method | Blocking | Notes |
|---|---|---|
| `step_enable(stepper_id)` / `step_disable(stepper_id)` | no | |
| `step_move(stepper_id, steps, move_type=StepMoveType.RELATIVE, blocking=True, timeout=None)` | optional | publishes `StepMove`; `move_type` uses the firmware enum |
| `step_home(stepper_id, blocking=True, timeout=None)` | optional | publishes `StepHome` |
| `step_set_config(stepper_id, ...)` | no | publishes `StepConfigSet` |
| `get_step_state() → StepStateAll` | no | cached `/step_state_all` |

#### Servos

| Method | Notes |
|---|---|
| `set_servo(channel, angle_deg)` | maps angle to pulse width; publishes `ServoSet` |
| `enable_servo(channel)` / `disable_servo(channel)` | publishes `ServoEnable` |
| `get_servo_state() → ServoStateAll` | cached `/servo_state_all` |

#### IO — buttons, limits, LEDs

| Method | Blocking | Notes |
|---|---|---|
| `get_button(button_id) → bool` | no | reads bitmask from cached `IOInputState`; ids 1–10 on current firmware |
| `was_button_pressed(button_id, consume=True) → bool` | no | rising-edge latch captured in ROS callback |
| `wait_for_button(button_id, timeout=None) → bool` | yes | blocks until button pressed or timeout |
| `get_limit(limit_id) → bool` | no | same approach as buttons; ids 1–8 on current firmware |
| `was_limit_triggered(limit_id, consume=True) → bool` | no | rising-edge latch captured in ROS callback |
| `wait_for_limit(limit_id, timeout=None) → bool` | yes | blocks until limit triggered |
| `set_led(led_id, brightness, mode=None, period_ms=None, duty_cycle=500)` | no | publishes `IOSetLed`; `mode` uses `LEDMode` (`OFF`, `ON`, `BLINK`, `BREATHE`, `PWM`); blink/breathe default to 1000 ms and `duty_cycle=500` |
| `set_neopixel(index, r, g, b)` | no | publishes `IOSetNeopixel`; index is 0-based |

`wait_for_button` and `was_button_pressed` use work done inside the IO
subscription callback when the requested bit transitions from 0→1, so short
button presses are not missed even if the FSM loop runs slower. For simple
FSMs that watch one button per state, `get_button()` is often the cleaner
choice because it avoids carrying a latched edge across a later state change.

For LED timing, `duty_cycle` is in permille (`0..1000`). In `BLINK` mode it is
the ON-time share of the full period. In `BREATHE` mode it is the rise-time
share of the full period, with the remaining time used for the fade back down.

#### IMU

| Method | Notes |
|---|---|
| `get_imu() → SensorImu` | cached `/sensor_imu` |

#### Units

| Method | Notes |
|---|---|
| `set_unit(unit: Unit)` | changes scale factor; does not affect already-sent commands |
| `get_unit() → Unit` | |

---

## Layer 2 — RobotFSM (FSM base class)

Students subclass `RobotFSM` and implement their state machine. The base class
provides state registration, transition wiring, and a `spin()` loop.

```python
class RobotFSM:
    def __init__(self, robot: Robot): ...

    # --- builder API ---
    def add_transition(self,
                       from_state: str,
                       event: str,
                       to_state: str,
                       action: Callable = None,
                       guard: Callable[[], bool] = None): ...

    # --- runtime API ---
    def trigger(self, event: str): ...   # fire an event (thread-safe)
    def get_state(self) -> str: ...
    def spin(self, hz: float = 50): ... # runs the FSM update loop; blocks

    # --- hooks for subclasses ---
    def on_enter(self, state: str): ... # called on state entry; override per-state
    def on_exit(self, state: str): ...  # called on state exit; override per-state
    def update(self): ...               # called every spin cycle; override for polling logic
```

### Minimal student example

```python
class MyFSM(RobotFSM):
    def __init__(self, robot):
        super().__init__(robot)
        self.add_transition("IDLE",    "start",    "MOVING", action=self._start_move)
        self.add_transition("MOVING",  "arrived",  "IDLE",   action=self._celebrate)
        self.add_transition("MOVING",  "estop",    "STOPPED")
        self.add_transition("IDLE",    "estop",    "STOPPED")

    def update(self):
        if self.get_state() == "IDLE":
            if self.robot.get_button(1):
                self.trigger("start")

    def _start_move(self):
        self.robot.move_to(500, 0, 200, blocking=False)

    def _celebrate(self):
        self.robot.set_led(1, 255)
```

---

## Layer 3 — PathPlanner (`path_planner.py`)

Path planners are **pure algorithm classes** — no threads, no ROS subscriptions.
`Robot` calls `compute_velocity()` from its own internal navigation thread
(`Robot._nav_to_waypoints`). Students who want a custom algorithm subclass
`PathPlanner` and implement `compute_velocity()`.

### Base class

```python
class PathPlanner:
    def compute_velocity(
        self,
        pose: tuple[float, float, float],   # (x, y, theta_rad) in consistent units
        waypoints: list[tuple[float, float]],
        max_linear: float,                   # max forward speed
    ) -> tuple[float, float]:               # (linear, angular_rad_s)
        raise NotImplementedError

    def get_obstacles(self) -> list:
        return []   # override when 2D lidar is available
```

### PurePursuitPlanner (ships with the package)

```python
class PurePursuitPlanner(PathPlanner):
    def __init__(self, lookahead_dist=150, max_angular=2.0): ...
    def compute_velocity(self, pose, waypoints, max_linear): ...
```

### APFPlanner (stub — activated when lidar is ready)

```python
class APFPlanner(PathPlanner):
    """Artificial potential fields. Override get_obstacles() when /scan is live."""
```

### Thread model

```
Main thread  ──► RobotFSM.spin()
                   └─ calls robot.move_to(..., blocking=False)  → MotionHandle
                         └─ Robot starts _nav_thread
                               └─ _nav_thread calls planner.compute_velocity()
                                  and robot._send_body_velocity_mm() in a loop

ROS spin thread ──► updates Robot internal state (pose, buttons, etc.)
```

Locks:
- `Robot._lock` (threading.Lock) — protects all cached topic state
- `Robot._nav_cancel/done` (threading.Event) — coordinate nav thread lifecycle

---

## Key constants

These are constructor defaults in `Robot` and should match `firmware/arduino/src/config.h`:

| Constant | Value | Source |
|---|---|---|
| `wheel_diameter_mm` | 74.0 | `WHEEL_DIAMETER_MM` |
| `wheel_base_mm` | 333.0 | `WHEEL_BASE_MM` |
| `encoder_ppr` | 1440 | `ENCODER_PPR × ENCODER_4X` |
| Default left drive motor | 1 | `ODOM_LEFT_MOTOR + 1` |
| Default right drive motor | 2 | `ODOM_RIGHT_MOTOR + 1` |

Student code can override the left/right wheel mapping at startup via
`set_left_wheel()` / `set_right_wheel()` if the robot is wired differently.

---

## File layout (proposed)

```
robot/
├── robot/
│   ├── __init__.py
│   ├── hardware_map.py        # user-facing IDs + firmware-derived timing constants
│   ├── robot_node.py          # ROS node; constructs Robot, spins ROS, calls main.run()
│   ├── robot.py               # Robot class (Layer 1)
│   ├── robot_fsm.py           # RobotFSM base class (Layer 2)
│   ├── path_planner.py        # PathPlanner base + PurePursuitPlanner (Layer 3)
│   ├── main.py                # ← students edit this; entry point for their FSM
│   └── examples/
│       ├── square_drive.py    # drive a square using move_to
│       └── button_fsm.py      # button-triggered state machine
```

`main.py` is the only file students need to edit. It defines their FSM class and a
`run(robot)` function that `robot_node.py` calls after ROS is set up.
