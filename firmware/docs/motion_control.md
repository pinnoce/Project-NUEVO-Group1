# Motion Control Implementation

This document describes how DC motors, steppers, encoders, odometry, and servos are implemented in the current firmware.

## 1. DC Motor Control: Mixed ISR + Loop Pipeline

The DC motor path is intentionally split between Timer1 ISR work and loop-side compute.

### Why this exists

Earlier firmware versions placed too much control work in ISR context, which caused UART timing failures and RX overruns. The current design keeps the ISR short and moves the heavy math into the loop.

### Timer1 rate and slot model

Timer1 runs at `800 Hz`.

- one Timer1 overflow = one slot = `1.25 ms`
- four slots = one full DC round = `5 ms`
- each motor gets one slice per round = `200 Hz` effective apply cadence per motor

### What happens in `ISR(TIMER1_OVF_vect)`

The ISR in `arduino.ino` does three things:

1. reads Timer1 counter ticks for cheap timing measurement
2. calls `MotorControlCoordinator::servicePidIsrSlice(...)`
3. records the ISR slice time in `LoopMonitor`

`servicePidIsrSlice(...)` owns the real round-robin logic:

- slot `0` starts a new round
- slot `0` can publish staged outputs for the new round
- the current motor's feedback is latched
- the current motor's already-prepared output is applied
- the slot index is advanced for the next Timer1 tick
- when the last slot closes a running round, the full round span is returned for `pidr`

### What happens in loop context

The fast lane calls `fastMotorCompute()`, which calls `taskMotors()` whenever `MotorControlCoordinator` reports a new round is ready.

`taskMotors()` computes the next DC output set in software and stages it for the next publish.

This is a fixed one-round pipeline:

- round `N` is being applied
- loop computes round `N+1`
- slot `0` of the next round publishes round `N+1`

That gives predictable latency and much lower jitter than a just-in-time handoff.

## 2. `MotorControlCoordinator`

`MotorControlCoordinator` exists to keep the mixed control handoff understandable and centralized.

It owns:

- current ISR slot
- round count
- requested/computed/applied round numbers
- compute/apply sequence counters
- whether a round is ready for compute
- whether staged outputs are ready for publish
- per-window handoff counters used by `StatusReporter`

The status line:

`req/cmp/app=.../.../... | prep/app=.../... | pipe=...`

comes directly from this module.

The `ControlWin` counters mean:

- `missed`: a new round arrived while a previous one was still waiting for compute
- `late`: compute started in the final pre-publish slot
- `reused`: a round had to reuse the previous output because new output was not ready
- `cross`: compute finished after the requested round boundary was already crossed

## 3. `DCMotor`

Each `DCMotor` object owns one physical motor channel.

### Major responsibilities

- keep track of its pins
- read encoder position via an `IEncoderCounter`
- estimate velocity using a `IVelocityEstimator`
- run cascade position/velocity control
- stage and publish H-bridge outputs
- detect basic encoder-stuck / feedback failure conditions

### Control modes

Supported modes:

- `DC_MODE_DISABLED`
- `DC_MODE_POSITION`
- `DC_MODE_VELOCITY`
- direct PWM command path

### Two-part update model

`DCMotor` is split across two methods:

- `service()`
  - loop-side control compute
  - reads latched encoder feedback
  - runs fixed-point PID math
  - prepares `stagedDuty_` / `stagedDrive_`
- `update()`
  - ISR-side apply
  - writes already-prepared drive state to the H-bridge output pins / OCR register

This split is the main reason the current DC path fits within the UART-safe ISR budget.

### Fixed-point control implementation

The current control path uses a Q16 fixed-point implementation for the hot control logic.

That choice reduces floating-point cost in the control loop and keeps `service()` predictable on AVR. The older float PID helper remains in the file, but the main active path uses the fixed-point fields and helper routines.

## 4. Encoders and Velocity Estimation

### Encoder counters

The firmware supports:

- external interrupt channels for M1 and M2
- PCINT-backed channels for M3 and M4

That mapping is configured in `ISRScheduler`.

The encoder back-end is intentionally minimal:

- count edges
- provide count snapshots

Earlier expensive ISR behavior, such as per-edge timestamp work, was removed because it contributed to UART timing failures.

### Velocity estimator

Velocity estimation is handled by separate estimator objects attached to each `DCMotor`.

This lets encoder counting stay simple while the motor driver owns the control-side interpretation of that feedback.

## 5. Stepper Control

Stepper control is entirely separate from the DC path.

### `StepperManager`

`StepperManager` owns:

- creation of the four `StepperMotor` instances
- Timer3 configuration
- `timerISR()` dispatch across all stepper channels
- group operations such as `disableAll()` and `emergencyStopAll()`

### Timer3

Timer3 runs at `10 kHz`, so each overflow is `100 us`.

The ISR in `arduino.ino`:

1. samples Timer3 ticks for timing measurement
2. samples UART2 fault edges
3. calls `StepperManager::timerISR()`
4. records the stepper ISR time in `LoopMonitor`

### `StepperMotor`

Each stepper channel owns:

- step / dir / enable pins
- target motion state
- pulse generation state
- homing / limit logic

Stepper limit checks for homing are read directly in the stepper path, not through the slower `UserIO` cache. That is why the general input sampling cadence does not limit homing responsiveness.

## 6. Servos

Servo control is separate from the Timer1/Timer3 motor paths.

### Hardware model

Servos are driven through a PCA9685 connected over I2C.

`ServoController` owns:

- PCA9685 initialization
- output-enable control via OE pin
- pulse-width to PWM conversion
- single-channel and bulk-channel writes
- I2C health tracking

### Why servo writes are deferred

Servo commands come in through `MessageCenter`, but the actual PCA9685 writes are deferred into `processDeferredServo()`.

That means:

- decode handlers only update desired servo state
- the actual I2C transaction happens later from soft-task context

This avoids putting I2C latency into packet decode or ISR timing-critical code.

### What "enabled" means for servos

The current servo output state is controlled by two layers:

- firmware policy in `SystemManager`
- actual PCA9685 output enable in `ServoController`

If all servo channels are disabled, `processDeferredServo()` disables PCA9685 outputs. If any servo channel is enabled, it ensures the PCA9685 outputs are enabled before applying pulses.

## 7. Robot Kinematics / Odometry

`RobotKinematics` is a singleton-style static module.

It assumes a differential-drive model and uses the configured left/right DC motor channels as odometry sources.

The module maintains:

- `x`, `y`, `theta`
- instantaneous body-frame `vx`, `vy`, `vTheta`

It updates from:

- left/right encoder tick counts
- left/right motor velocity estimates

This module is used for telemetry and UI state, not for the low-level control loops.

## 8. Motion-Related Timing Signals

The most important motion timing fields in the human-readable report are:

- `pid`: one Timer1 ISR slice
- `pidr`: one full 4-slot Timer1 round
- `motor`: one loop-side DC compute round
- `step`: one Timer3 ISR service slice

For control correctness, the most useful status counters are usually:

- `req/cmp/app`
- `prep/app`
- `pipe`
- `ControlWin`

Those show whether the pipeline is coherent, not just whether a budget was exceeded.
