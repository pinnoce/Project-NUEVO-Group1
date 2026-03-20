# Communication and State Flow

This document explains how Raspberry Pi communication, the firmware state machine, safety policy, and human-readable reporting are implemented in the current firmware.

## 1. UART / TLV Ownership

`MessageCenter` is the only module that owns the Raspberry Pi link on `Serial2`.

It is responsible for:

- configuring `Serial2`
- feeding bytes into the TLV decoder
- routing decoded commands to subsystems
- tracking heartbeat/liveness state
- batching telemetry back to the Pi
- queuing deferred slow side effects so decode handlers stay short

The link is intentionally split across two contexts:

- fast lane:
  - `MessageCenter::drainUart()`
  - `MessageCenter::drainTx()`
- periodic UART task:
  - `MessageCenter::processIncoming()`
  - `MessageCenter::processDeferred()`
  - `MessageCenter::sendTelemetry()`

This keeps RX/TX draining responsive without moving protocol work into ISR context.

## 2. Receive Path

The receive path is:

`Serial2 RX hardware -> fastDrainUart() -> tlvcodec decodePacket() -> decodeCallback() -> routeMessage()`

### `drainUart()`

`drainUart()` runs from the fast lane on every loop pass. It reads all available bytes from `Serial2` and feeds them directly into the TLV decoder.

Important design choice:

- there is no second large software RX queue on top of the decoder buffer
- bytes go straight into the decoder state machine

This avoids wasting SRAM and reduces the chance that a slow periodic task can starve the decoder.

### `decodeCallback()`

When a complete frame is decoded:

- frame-level decode errors increment the UART error counters
- valid frames increment the per-window activity counters
- every TLV in the frame is passed to `routeMessage()`

Oversize or malformed TLVs are rejected here and counted separately.

### `routeMessage()`

`routeMessage()` is where liveness is refreshed:

- every valid TLV refreshes `lastHeartbeatMs_`
- any non-heartbeat TLV also refreshes `lastCmdMs_`
- a heartbeat restore event is logged if the link had timed out earlier

Then the command or payload is dispatched by type.

## 3. Command Routing

`MessageCenter` does not own final state-transition policy. It only:

- validates payloads
- calls subsystem methods
- forwards transition requests to `SystemManager`

Examples:

- system commands:
  - `START -> SystemManager::triggerStartCommand()`
  - `STOP -> SystemManager::triggerStopCommand()`
  - `RESET -> SystemManager::triggerResetCommand()`
  - `ESTOP -> SystemManager::triggerEstopCommand()`
- DC motor enable/motion commands:
  - gated through `SystemManager::canEnableDriveActuator()` and `canRunDriveActuator()`
- servo enable commands:
  - gated through `SystemManager::canEnableServoActuator()`

This keeps the rules for "what is allowed in each state" in one place.

## 4. Deferred Work

Some received commands used to execute slow side effects immediately in decode context. That created timing spikes and link instability. The current firmware defers those operations.

### Deferred servo path

Servo commands do not write the PCA9685 directly from the decode handler.

Instead:

1. the decode handler updates cached desired servo state
2. `MessageCenter` marks servo work dirty
3. `processDeferredServo()` runs later from the UART soft task
4. `ServoController` performs the actual I2C writes there

This is important because PCA9685 I2C traffic is relatively slow and must not happen inside packet decode.

### Deferred magnetometer calibration path

Mag calibration commands also defer their slow actions:

- start / stop sampling
- save to EEPROM
- apply loaded offsets
- clear stored calibration

`processDeferredMagCal()` performs those operations later from soft-task context.

## 5. Telemetry Transmit Path

The transmit path is:

`sendTelemetry() -> append TLVs into shared txStorage_ -> sendFrame() -> queued drain via drainTx()`

### Frame batching

`sendTelemetry()` opens one frame, conditionally appends all telemetry TLVs that are due, and then queues one completed frame.

This avoids sending many small standalone frames and keeps telemetry scheduling centralized.

### Why `sendTelemetry()` checks for an in-flight frame first

The firmware uses one shared TX buffer. If the previous frame is still draining, rebuilding that same buffer would corrupt the bytes already being transmitted. The current code guards this before rebuilding the frame.

### Telemetry phase spreading

Telemetry groups are phase-shifted across 20 ms UART task slots to reduce burstiness:

- DC status
- kinematics
- ultrasonic
- I/O
- servo status
- IMU
- system status
- step status
- voltage

That staggering is one of the main reasons TX queue pressure is lower than earlier bring-up builds.

## 6. Heartbeat and Liveness

The heartbeat model is:

- any valid incoming TLV refreshes the heartbeat timer
- `processIncoming()` periodically checks timeout state
- `SafetyManager` decides when timeout should become a hard state fault

`MessageCenter` maintains:

- `heartbeatValid_`
- `lastHeartbeatMs_`
- `lastRxByteMs_`
- timeout and traffic counters

`SystemManager` decides whether heartbeat timeout matters in the current firmware state.

In the current policy:

- heartbeat timeout only trips a hard fault while `RUNNING`

## 7. State Machine Ownership

`SystemManager` is the single owner of firmware state.

The public trigger API is intentionally explicit:

- `triggerBootCompleted()`
- `triggerStartCommand()`
- `triggerStopCommand()`
- `triggerResetCommand()`
- `triggerEstopCommand()`
- `triggerSafetyFaultFromIsr()`

### Current transition rules

- `INIT -> IDLE` after boot
- `IDLE -> RUNNING` on `START`
- `RUNNING -> IDLE` on `STOP`
- `ERROR -> IDLE` on `RESET`
- `ESTOP -> IDLE` on `RESET`
- `any -> ESTOP` on ESTOP trigger
- `RUNNING -> ERROR` on configured safety faults

### Battery policy

The current battery policy is intentional and implemented in `SystemManager`:

- the firmware may enter `RUNNING` with no battery present
- actuator enable commands are ineffective while battery power is absent
- once battery power has been seen during that `RUNNING` session, loss of battery or critical/overvoltage trips `ERROR`
- `RESET` can still clear `ERROR` back to `IDLE`; actuators remain disabled until policy allows them again

## 8. SafetyManager

`SafetyManager` only detects fault conditions. It does not decide the state machine.

Its main checks are:

- heartbeat timeout
- battery under/over-voltage safety conditions

When a hard fault is found, it calls `SystemManager::triggerSafetyFaultFromIsr(flags)`.

The name is historical; the current safety check runs from the 100 Hz soft task, not from a hardware ISR. The important point is that the final state-transition decision still lives in `SystemManager`.

## 9. Human-Readable Reporting

### `StatusReporter`

`StatusReporter` produces the chunked:

- `[SYSTEM]`
- `[TIMING]`
- `[SENSORS]`
- `[UART]`

report once per configured status interval.

Two-stage design:

1. `StatusReporter::task()` snapshots state once per reporting period
2. `StatusReporter::emitChunk()` prints that snapshot in small pieces from the fast lane

That chunking is important because earlier monolithic status formatting caused large loop-gap spikes.

### `LoopMonitor`

`LoopMonitor` records:

- per-slot average / peak / max time
- budget thresholds
- fault masks when a budget is exceeded

It is a timing-budget monitor, not a communication monitor. Real link health still comes from:

- `dor`
- `fe`
- `crc`
- frame-length errors
- TLV errors

### `DebugLog`

All human-readable debug text is queued first and flushed later. This avoids long blocking writes to USB serial from sensitive code paths.

## 10. Where to Look Next

For the implementation details of the motion stack, use [`motion_control.md`](motion_control.md).

For IMU / ultrasonic / servo / PCA9685 / shared-I2C behavior, use [`sensors_and_i2c.md`](sensors_and_i2c.md).
