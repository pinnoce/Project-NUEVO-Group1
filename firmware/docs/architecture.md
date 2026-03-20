# Firmware Architecture

This document describes the current `v0.9.0` Arduino firmware as it is implemented in the codebase today. It is the top-level map for maintainers. Detailed subsystem behavior is split into focused companion documents:

- [`communication_and_state.md`](communication_and_state.md)
- [`motion_control.md`](motion_control.md)
- [`sensors_and_i2c.md`](sensors_and_i2c.md)
- [`technical_notes.md`](technical_notes.md)

## 1. What Runs Where

The firmware is intentionally split into three execution layers.

### Hard real-time ISR layer

These paths must stay short and deterministic.

| Path | Owner | Rate | Purpose |
|------|-------|------|---------|
| external INT / PCINT | encoder modules | edge-driven | count DC motor encoder transitions |
| `ISR(TIMER1_OVF_vect)` | `arduino.ino` + `MotorControlCoordinator` | `800 Hz` | one short DC latch/apply slice per tick |
| `ISR(TIMER3_OVF_vect)` | `arduino.ino` + `StepperManager` | `10 kHz` | stepper pulse service and UART2 fault-edge sampling |

No I2C, TLV parsing, EEPROM work, or large debug formatting is allowed here.

### Fast cooperative lane in `loop()`

`loop()` runs the fast lane before and after the periodic scheduler tick:

1. `StatusReporter::recordLoopGap()`
2. `StatusReporter::updateWindowPeaks()`
3. `Scheduler::serviceFastLane()`
4. `Scheduler::tickPeriodic()`
5. `Scheduler::serviceFastLane()`

The current fast-lane tasks are:

- UART RX drain
- UART TX drain
- motor round compute trigger service
- chunked status-report emission
- debug-log flush

These are "run when work is available" services, not fixed-rate periodic tasks.

### Periodic soft-task layer

`Scheduler::tickPeriodic()` is millis-based and runs at most one overdue task per pass.

| Task | Rate | What it does |
|------|------|--------------|
| `taskUART()` | `50 Hz` | heartbeat timeout cadence, deferred work, telemetry packing |
| `taskMotors()` | event-driven from fast lane | compute one full DC control round when requested |
| `taskSafety()` | `100 Hz` | evaluate heartbeat and battery fault conditions |
| `taskSensors()` | `100 Hz` | IMU/ultrasonic/voltage dispatch and button/limit sampling |
| `taskUserIO()` | `20 Hz` | LED patterns and deferred NeoPixel rendering |
| `StatusReporter::task()` | `1 Hz` by default | snapshot a human-readable system report |

## 2. What `arduino.ino` Owns

`firmware/arduino/arduino.ino` is intentionally kept as the readable top-level orchestration file. It should answer these questions at a glance:

- which global hardware-channel objects exist
- which ISRs are active
- which soft tasks are registered
- what the startup order is
- what the main loop order is

Detailed implementation is pushed into modules, but the actual runtime wiring remains visible in `arduino.ino`.

## 3. Startup Sequence

`setup()` performs the following sequence:

1. start debug serial and print the startup banner
2. initialize the scheduler
3. initialize `SystemManager`
4. initialize `MessageCenter`
5. initialize `SensorManager`
6. initialize `UserIO`
7. initialize `ServoController`
8. initialize `StepperManager`
9. initialize DC motors, encoders, and velocity estimators
10. initialize `MotorControlCoordinator`
11. initialize `StatusReporter`
12. attach encoder interrupts
13. register fast-lane tasks
14. register periodic tasks
15. configure Timer1 / Timer4 runtime hardware in `ISRScheduler`
16. resynchronize output pins affected by timer setup
17. transition `INIT -> IDLE`
18. print the startup summary

This order matters because several modules depend on earlier bring-up:

- `MessageCenter` expects UART and TLV buffers ready before communication starts
- `SensorManager` must initialize before battery/state policy becomes meaningful
- `ServoController` depends on the shared `Wire` bus
- Timer1 and Timer3 interrupts are enabled only after all dependent objects are ready

## 4. Core Modules and Their Real Responsibilities

### Runtime infrastructure

| File | Responsibility |
|------|----------------|
| `config.h` | user-facing compile-time configuration |
| `pins.h` | board mapping and direct register aliases |
| `utility.*` | small shared helpers for startup prints, timer-counter sampling, UART2 fault-edge sampling, and elapsed-time clamping |
| `Scheduler.*` | periodic soft-task registry plus fast cooperative lane |
| `ISRScheduler.*` | Timer1/Timer4 configuration and DC encoder interrupt attachment |

### System state and safety

| File | Responsibility |
|------|----------------|
| `SystemManager.*` | the firmware state machine and all transition policy |
| `modules/SafetyManager.*` | detect heartbeat and battery faults and forward them to `SystemManager` |

### Communication and reporting

| File | Responsibility |
|------|----------------|
| `modules/MessageCenter.*` | TLV decode/route/encode, heartbeat tracking, telemetry scheduling, deferred slow side effects |
| `modules/DebugLog.*` | queued human-readable USB debug output |
| `modules/StatusReporter.*` | 1 Hz chunked `[SYSTEM] / [TIMING] / [SENSORS] / [UART]` reporting |
| `modules/LoopMonitor.*` | timing stats and budget overrun tracking |

### Motion control

| File | Responsibility |
|------|----------------|
| `drivers/DCMotor.*` | one DC motor channel: feedback latch, control compute, staged output publish, H-bridge drive |
| `modules/MotorControlCoordinator.*` | Timer1 round bookkeeping and loop/ISR handoff for the fixed one-round pipeline |
| `modules/EncoderCounter*` | encoder count back-end for INT and PCINT channels |
| `modules/VelocityEstimator*` | encoder-based velocity estimation |
| `modules/DCMotorBringup.*` | repeated DC motor, encoder, and estimator initialization |
| `modules/StepperManager.*` | stepper-manager ownership of Timer3 configuration and multi-stepper dispatch |
| `drivers/StepperMotor.*` | one stepper channel: motion planning, pulse timing, homing switch checks |
| `drivers/ServoController.*` | PCA9685-based servo output over I2C |
| `modules/RobotKinematics.*` | differential-drive odometry from selected DC motor channels |

### Sensors and user I/O

| File | Responsibility |
|------|----------------|
| `modules/SensorManager.*` | IMU, ultrasonic, voltage monitoring, and magnetometer calibration state |
| `modules/PersistentStorage.*` | EEPROM-backed calibration persistence |
| `modules/UserIO.*` | button cache, limit-switch cache, discrete LED output modes, deferred NeoPixel state rendering |
| `drivers/IMUDriver.*` | ICM-20948 wrapper |
| `drivers/UltrasonicDriver.*` | SparkFun Qwiic ultrasonic wrapper |
| `drivers/NeoPixelDriver.*` | minimal NeoPixel wrapper used by `UserIO` |

## 5. Two Coding Patterns Used in the Firmware

### Static module pattern

Subsystems that exist exactly once are written as static modules:

- `SystemManager`
- `MessageCenter`
- `SensorManager`
- `StatusReporter`
- `UserIO`
- `StepperManager`
- `ServoController`

This is not an Arduino requirement. It is a deliberate firmware design choice:

- explicit `init()` calls in `setup()`
- no hidden constructor side effects before hardware is ready
- simple call sites like `SystemManager::triggerStartCommand()`

### Real object instances

Things that have multiple physical channels use real objects:

- `dcMotors[NUM_DC_MOTORS]`
- encoder instances
- velocity-estimator instances
- stepper instances inside `StepperManager`

Rule of thumb:

- one subsystem -> static module
- multiple hardware channels -> objects

## 6. State Ownership

`SystemManager` is the only module that decides whether a state transition is valid.

Other modules should not directly mutate firmware state. They should report intent or fault conditions.

Examples:

- `MessageCenter` receives `START/STOP/RESET/ESTOP` and calls `SystemManager` trigger functions
- `SafetyManager` detects battery or heartbeat faults and calls the safety trigger
- `UserIO` does not infer state; it only renders state that was queued by `SystemManager`

The transition rules themselves are documented in [`communication_and_state.md`](communication_and_state.md).

## 7. Why the Firmware Is Split This Way

This architecture was chosen to solve three real problems on the Mega 2560:

1. keep Timer1 and Timer3 short enough that UART2 RX does not overrun
2. keep heavy work such as TLV routing, I2C transactions, EEPROM writes, and debug output out of ISR context
3. keep `arduino.ino` readable enough that a maintainer can follow startup, loop order, and active ISRs without opening every module first

The more detailed implementation rules and tradeoffs are described in:

- [`communication_and_state.md`](communication_and_state.md)
- [`motion_control.md`](motion_control.md)
- [`sensors_and_i2c.md`](sensors_and_i2c.md)
- [`technical_notes.md`](technical_notes.md)
