# NUEVO Communication Protocol
## v4 Logical Source of Truth

**Version:** 4.0  
**Date:** 2026-03-24  
**Status:** Current implemented logical source of truth for the compact TLV
protocol used by firmware, `nuevo_bridge`, and the UI.

---

## Table of Contents

0. [Purpose](#0-purpose)
1. [System Model](#1-system-model)
2. [Protocol Principles](#2-protocol-principles)
3. [Physical Layer and Framing](#3-physical-layer-and-framing)
4. [Firmware State Machine](#4-firmware-state-machine)
5. [Message Naming and Semantics](#5-message-naming-and-semantics)
6. [Bridge Bootstrap and Reset Recovery](#6-bridge-bootstrap-and-reset-recovery)
7. [Logical TLV Catalog](#7-logical-tlv-catalog)
8. [Streaming Policy](#8-streaming-policy)
9. [Diagnostics Policy](#9-diagnostics-policy)
10. [Implementation Notes](#10-implementation-notes)

---

## 0. Purpose

This file is the **human-readable source of truth** for the current protocol
revision.

It defines:

- what message families exist
- what each message is responsible for
- what should be streamed vs queried
- how the bridge should bootstrap and recover after firmware reset
- the design rules that keep the whole protocol consistent

This file does **not** own exact numeric IDs or exact payload byte layouts.
Those remain in:

| File | Purpose |
|---|---|
| [`../tlv_protocol/TLV_TypeDefs.json`](../tlv_protocol/TLV_TypeDefs.json) | Numeric TLV IDs |
| [`../tlv_protocol/TLV_Payloads.md`](../tlv_protocol/TLV_Payloads.md) | Exact payload layouts and sizes |

Maintenance workflow:

1. Update logical behavior here first.
2. Update `TLV_TypeDefs.json` and `TLV_Payloads.md`.
3. Regenerate generated type definitions.
4. Update firmware and bridge implementations.

---

## 1. System Model

```
Browser UI / ROS2 / Python API
            │
            ▼
      NUEVO Bridge
  - UART transport owner
  - command arbiter
  - static snapshot cache
  - reset/bootstrap manager
            │
            ▼
     Arduino firmware
  - real-time control
  - TLV routing
  - actuator ownership
  - sensor acquisition
```

Design intent:

- the Arduino owns real-time control and hardware state
- the bridge owns external interfaces, bootstrap, and cached static metadata
- the protocol should expose **live runtime state** separately from
  **static/config snapshots**

---

## 2. Protocol Principles

### 2.1 Intent-based message design

Every message should have one primary purpose:

- change behavior now
- change configuration
- request a snapshot
- return a snapshot
- stream live state
- stream high-rate telemetry

Mixed-purpose payloads should be avoided.

### 2.2 No required acknowledgements

The protocol remains **stream-oriented**, not request/ack driven.

- command messages do not require separate ACK TLVs
- success is normally observed through the next streamed state or response
- `REQ/RSP` is used for snapshots, not generic command acknowledgement

### 2.3 Stream only what changes fast

The current protocol explicitly separates:

- fast-changing runtime state
- slow-changing mutable configuration
- nearly static identity/capability information
- engineering/debug diagnostics

This keeps runtime streams focused on frequently changing data and moves static
or diagnostic data into dedicated snapshot messages.

### 2.4 Multi-TLV bundling remains first-class

Frames may contain multiple TLVs. This remains a core optimization.

- transport framing stays compact
- bundling due TLVs into one frame remains the normal streaming mode
- the transport remains compact and multi-TLV capable
- message meaning and grouping are defined by the logical catalog below

---

## 3. Physical Layer and Framing

The transport/framing layer stays the current compact TLV design.

### 3.1 Physical layer

| Parameter | Value |
|---|---|
| Interface | UART |
| Arduino port | `Serial2` |
| RPi port | board-configured serial port |
| Default baud | `200000` |
| Format | `8N1` |
| Duplex | full duplex |

### 3.2 Compact frame format

Frame header stays:

- `magic[4]`
- `numTotalBytes : uint16`
- `crc16 : uint16`
- `deviceId : uint8`
- `frameNum : uint8`
- `numTlvs : uint8`
- `flags : uint8`

TLV header stays:

- `tlvType : uint8`
- `tlvLen : uint8`

Current framing rules that remain:

- CRC16-CCITT
- maximum single TLV payload length = `255 bytes`
- multiple TLVs per frame supported and expected

This document is about **logical TLV structure**, not a new transport format.

---

## 4. Firmware State Machine

The protocol-visible firmware states remain:

- `INIT`
- `IDLE`
- `RUNNING`
- `ERROR`
- `ESTOP`

High-level behavior:

- `INIT` performs setup and enters `IDLE`
- `IDLE` is safe and accepts configuration
- `RUNNING` streams live control/sensor state
- `ERROR` disables actuators and exposes fault state
- `ESTOP` is the strict emergency state

The main system-level streamed state message is:

- `SYS_STATE`

not a large mixed `SYS_STATUS`.

---

## 5. Message Naming and Semantics

The protocol uses consistent naming:

| Suffix | Meaning |
|---|---|
| `*_CMD` | state-machine or discrete action command |
| `*_SET` | set targets or configuration |
| `*_REQ` | request one snapshot |
| `*_RSP` | one-shot response snapshot |
| `*_STATE` | periodic live runtime state |
| `*_TELEM` | high-rate measured data where "state" is not the best name |

Rules:

- `STATE` messages are streamed
- `REQ/RSP` messages are snapshot-oriented
- `SET` messages do not carry static data back inside the command itself
- a `SET` that changes queryable configuration should trigger an immediate
  matching `RSP`
- a `SET` that changes streamed runtime/output state should make the next
  `STATE` reflect the new value promptly

### 5.1 Timestamp rule

All streamed runtime/state/telemetry messages should carry a timestamp.

For bundled `*_ALL` messages:

- use one message-level timestamp
- do not duplicate timestamps per element unless absolutely necessary

This applies to:

- `DC_STATE_ALL`
- `STEP_STATE_ALL`
- `SERVO_STATE_ALL`
- `IO_INPUT_STATE`
- `IO_OUTPUT_STATE`
- `SENSOR_IMU`
- `SENSOR_KINEMATICS`
- `SENSOR_ULTRASONIC_ALL`
- `SYS_POWER`
- optionally `SYS_STATE`

### 5.2 Static vs dynamic split

The following should **not** be part of high-rate state streams:

- firmware version
- channel counts
- compile-time capabilities
- motor direction mask
- home-switch mapping
- PID gains
- stepper max speed / acceleration
- debug counters

Those belong in `RSP` or diagnostic messages.

### 5.3 Request / response rule

The formal `REQ/RSP` pattern does **not** require request correlation IDs.

Reason:

- the transport is point-to-point
- the bridge is the only requester
- snapshot traffic is low-rate

So the protocol rule is:

- bridge sends a typed `*_REQ`
- firmware returns the matching typed `*_RSP`
- bridge may serialize bootstrap/query traffic if it wants stricter ordering

---

## 6. Bridge Bootstrap and Reset Recovery

This is a required part of the new protocol design.

### 6.1 Why bootstrap exists

The firmware can reset externally:

- Arduino reset button (or from Arduino IDE)
- power cycle
- watchdog
- brownout
- USB/board reset during development

When this happens, the bridge must not assume its cached static/config state is
still valid.

### 6.2 Bridge bootstrap sequence

On serial connect or protocol resync, the bridge should:

1. wait for valid traffic from the firmware
2. request fresh static/config snapshots
3. cache those responses
4. only then treat the firmware state as fully synchronized

Recommended bootstrap queries:

- `SYS_INFO_REQ`
- `SYS_CONFIG_REQ`
- `DC_PID_REQ`
- `STEP_CONFIG_REQ`

If additional query-only snapshots exist later, they join the same bootstrap.

### 6.3 Reset detection

The bridge should detect a firmware reset by watching `SYS_STATE.uptimeMs`.

If:

- `uptimeMs` goes backwards, or
- `uptimeMs` drops sharply to a small value after having been large

the bridge should:

- assume firmware reset
- clear cached static/config snapshots
- re-run bootstrap queries

This behavior must be implemented and documented because static information is
no longer continuously streamed.

### 6.4 Runtime effect of reset

After reset, the firmware returns to its default safe startup state.

Therefore the bridge/UI must assume:

- actuators are disabled
- runtime config may be back to defaults
- any cached static/config mirror is invalid until refreshed
- any desired non-default runtime config must be re-applied by the bridge after
  bootstrap completes

---

## 7. Logical TLV Catalog

This section defines the **target logical message set**. Exact IDs and payloads
will be assigned later in `tlv_protocol/`.

### 7.1 System family

#### `SYS_HEARTBEAT` ↓

Keep this as the low-cost liveness message from bridge to firmware.

#### `SYS_CMD` ↓

System state-machine control:

- start
- stop
- reset
- estop

#### `SYS_STATE` ↑

Streamed live system-level state.

Should contain only live runtime/system information, such as:

- firmware state
- warning flags
- error flags
- runtime-present/health mask
- uptime
- last-RX age / liveness-related live data

Should **not** contain:

- firmware version
- compile-time config
- channel counts
- debug counters
- motor direction config

Recommended stream rate:

- `RUNNING`: `10 Hz`
- `ERROR`: `10 Hz`
- `IDLE`: `1 Hz`
- `ESTOP`: `1 Hz`

#### `SYS_POWER` ↑

This replaces the current `SENSOR_VOLTAGE` conceptually.

Contains:

- battery voltage
- 5V rail
- servo rail
- configured battery type / chemistry identifier
- optional future power-related live state if needed

Recommended stream rate:

- `RUNNING`: `10 Hz`
- `ERROR`: `10 Hz`
- `IDLE`: `1 Hz`

#### `SYS_INFO_REQ / SYS_INFO_RSP`

Static identity / capability snapshot.

Expected content:

- firmware version
- protocol version
- board identity
- supported feature mask
- configured/supported channel counts
- supported sensor capabilities
- compile-time hardware mappings that the UI needs to interpret other state
- limit-switch mask plus stepper/DC home-switch mappings

This is part of bootstrap and should not be part of a fast stream.

#### `SYS_CONFIG_REQ / SYS_CONFIG_RSP`

Snapshot of mutable system-level configuration.

Expected content:

- motor direction mask
- heartbeat timeout
- configured sensor mask
- NeoPixel count
- other mutable system settings that may differ from defaults

#### `SYS_CONFIG_SET` ↓

Set mutable system-level configuration.

This message sets mutable system-level configuration. It should not include
action-like commands such as odometry reset.

#### `SYS_ODOM_RESET` ↓

Dedicated command to reset odometry.

This is intentionally split out because it is an action, not configuration.

#### `SYS_DIAG_REQ / SYS_DIAG_RSP`

Query-only engineering diagnostics.

Candidate fields:

- free RAM
- loop timing summaries
- UART error counters
- queue drops
- other internal diagnostics useful for bring-up and debugging

These should not live in `SYS_STATE`.

---

### 7.2 DC motor family

#### `DC_ENABLE` ↓
Enable/disable a motor and select control mode.

#### `DC_SET_POSITION` ↓
Position target command.

#### `DC_SET_VELOCITY` ↓
Velocity target command.

#### `DC_SET_PWM` ↓
Open-loop PWM command.

#### `DC_RESET_POSITION` ↓
Reset one motor's encoder position to zero without requiring the motor to be enabled.

This is an encoder-baseline action, not a motion command.

#### `DC_HOME` ↓
Run a DC motor homing sequence toward its configured home limit.

The firmware uses the compile-time DC home-limit mapping from `config.h`. If no
home limit is configured for that motor, the command is rejected.

#### `DC_STATE_ALL` ↑

Streamed live DC runtime state for all motors.

Should include:

- enable/mode state
- measured position
- measured velocity
- current target
- actual PWM output
- current measurement if available
- per-motor fault flags
  - limit active
  - encoder fault
- one message-level timestamp

Should **not** include PID gains.

Recommended rate:

- `RUNNING`: `50 Hz`

#### `DC_PID_REQ / DC_PID_RSP`

Query-only PID snapshot for one motor or all motors, depending on final payload design.

#### `DC_PID_SET` ↓

Set PID gains.

After `DC_PID_SET`, the firmware should either:

- emit a matching `DC_PID_RSP`, or
- update a cached `RSP` state that the bridge can request immediately

---

### 7.3 Stepper family

#### `STEP_ENABLE` ↓
Enable/disable the stepper driver.

#### `STEP_MOVE` ↓
Issue absolute or relative move.

#### `STEP_HOME` ↓
Run homing sequence.

#### `STEP_CONFIG_REQ / STEP_CONFIG_RSP`

Query/response for motion configuration.

Should contain:

- max speed
- acceleration
- any persistent stepper config the UI needs to inspect

#### `STEP_CONFIG_SET` ↓

Set stepper motion parameters.

#### `STEP_STATE_ALL` ↑

Streamed live stepper runtime state.

Should include:

- enabled
- motion state
- limit/home flags
- current count
- target count
- current speed
- one message-level timestamp

Should **not** include:

- max speed
- acceleration

Naming cleanup:

- rename confusing fields like `commandedCount`
- prefer direct names such as `count` and `targetCount`, with a clear note that
  stepper position is open-loop command state, not measured feedback

Recommended rate:

- `RUNNING`: `50 Hz`

---

### 7.4 Servo family

#### `SERVO_ENABLE` ↓
Enable/disable channels.

#### `SERVO_SET` ↓
Set one or more pulse widths.

#### `SERVO_STATE_ALL` ↑

Streamed servo runtime/output state.

Should include:

- controller present/health state
- enabled mask
- commanded pulse widths
- one message-level timestamp

This is still commanded-state only; no feedback is implied.

Recommended rate:

- `RUNNING`: `10 Hz`

---

### 7.5 Sensor family

#### `SENSOR_IMU` ↑

High-rate IMU telemetry.

Should remain the main fused/raw IMU stream and continue to include:

- quaternion
- earth-frame linear acceleration
- raw accel/gyro/mag
- `magCalibrated`
- timestamp

Recommended rate:

- `RUNNING`: `50 Hz`

#### `SENSOR_KINEMATICS` ↑

Robot odometry/kinematics stream.

Should remain separate from IMU and include:

- pose
- velocity
- timestamp

Recommended rate:

- `RUNNING`: `50 Hz`

#### `SENSOR_ULTRASONIC_ALL` ↑

This message bundles all configured ultrasonic slots into one runtime
message containing:

- per-sensor status
- per-sensor distance
- one message-level timestamp

Recommended rate:

- `RUNNING`: `50 Hz`

Implementation note:

- the current Arduino firmware does not emit `SENSOR_ULTRASONIC_ALL`
- Pi-side components may still use this message family if ultrasonic sensing is
  exposed through the bridge later

#### `SENSOR_MAG_CAL_CMD` ↓
Keep this workflow-specific command.

#### `SENSOR_MAG_CAL_STATUS` ↑
Keep this workflow-specific status stream.

The magnetometer calibration workflow remains a special case:

- firmware handles sampling state
- bridge performs the hard/soft-iron fit
- bridge applies final calibration in one command

---

### 7.6 User IO family

#### `IO_SET_LED` ↓
Set discrete LED behavior.

Should include:

- `ledId`
- `mode`
- `brightness`
- `periodMs`
- `dutyCycle`

`mode` values are:

- `0`: `OFF`
- `1`: `ON`
- `2`: `BLINK`
- `3`: `BREATHE`
- `4`: `PWM`

Notes:

- `brightness` is the brightness ceiling for `ON`, `BLINK`, `BREATHE`, and `PWM`
- `periodMs` applies to `BLINK` and `BREATHE`
- `dutyCycle` is a permille timing parameter (`0..1000`)
- for `BLINK`, `dutyCycle` is the ON-time share of the full period
- for `BREATHE`, `dutyCycle` is the rise-time share of the full period; `500` is symmetric
- `dutyCycle` is ignored for `OFF`, `ON`, and `PWM`

#### `IO_SET_NEOPIXEL` ↓
Set NeoPixel behavior.

#### `IO_INPUT_STATE` ↑

Streamed user input state only.

Should include:

- button mask
- limit-active mask derived from the configured shared GPIO mapping
- one message-level timestamp

This exists specifically because input responsiveness matters more than output
mirror traffic.

Recommended rate:

- `RUNNING`: `50 Hz`

#### `IO_OUTPUT_STATE` ↑

Slower stream of output mirror state.

Should include:

- all discrete LED states/brightness
- NeoPixel current output state
- one message-level timestamp

Recommended rate:

- `RUNNING`: `10 Hz`

---

## 8. Streaming Policy

### 8.1 Stream classes

The protocol should be designed in stream classes, not as one-off rates only.

| Class | Typical use |
|---|---|
| `fast` | IMU, kinematics, responsive input state |
| `medium` | actuator runtime state |
| `slow` | system-level state and output mirrors |
| `query-only` | static info, config snapshots, diagnostics |
| `workflow` | special temporary streams like mag calibration |

### 8.2 Recommended default rates

| Message | IDLE | RUNNING | ERROR | ESTOP |
|---|---|---|---|---|
| `SYS_STATE` | 1 Hz | 10 Hz | 10 Hz | 1 Hz |
| `SYS_POWER` | 1 Hz | 10 Hz | 10 Hz | 1 Hz |
| `DC_STATE_ALL` | — | 50 Hz | — | — |
| `STEP_STATE_ALL` | — | 50 Hz | — | — |
| `SERVO_STATE_ALL` | — | 10 Hz | — | — |
| `IO_INPUT_STATE` | — | 50 Hz | — | — |
| `IO_OUTPUT_STATE` | — | 10 Hz | — | — |
| `SENSOR_IMU` | — | 50 Hz | — | — |
| `SENSOR_KINEMATICS` | — | 50 Hz | — | — |
| `SENSOR_ULTRASONIC_ALL` | — | 50 Hz | — | — |
| `SENSOR_MAG_CAL_STATUS` | on demand | workflow | on demand | — |

These are default design targets, not a promise that every platform revision
must use exactly these values.

### 8.3 Bundling policy

The bridge and firmware should continue to batch due streamed TLVs into one
frame where practical.

This is required to keep header overhead low, especially when several medium
and slow streams are due at the same time.

---

## 9. Diagnostics Policy

Engineering diagnostics are still important, but they should not dominate the
normal runtime protocol.

Therefore:

- diagnostic counters and loop timing should not live in `SYS_STATE`
- diagnostic data should be query-only by default
- if a streamed diagnostic mode is ever added, it should be explicitly
  opt-in and documented as engineering/debug traffic

Typical diagnostic content:

- free SRAM
- UART error totals
- loop timing summaries
- queue drops
- internal overrun counters

These belong under `SYS_DIAG_REQ / SYS_DIAG_RSP`.

---

## 10. Implementation Notes

The protocol described here is now the implemented compact-TLV layout.

Important current implementation notes:

- `SYS_STATE`, `SYS_POWER`, `SYS_INFO_*`, `SYS_CONFIG_*`, and `SYS_DIAG_*`
  are split and active
- `REQ/RSP` is the normal snapshot pattern
- the bridge uses `uptimeMs` regression to trigger bootstrap refresh after
  Arduino reset
- the Arduino-side runtime no longer supports lidar or ultrasonic sensing;
  those sensors are Pi-side only
- `SYS_POWER` now carries both rail voltages and the configured `batteryType`
