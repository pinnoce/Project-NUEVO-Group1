# TLV Payload Reference

This document defines the current wire payloads for the compact TLV protocol.

Source hierarchy:
- Logical protocol design: [`docs/COMMUNICATION_PROTOCOL.md`](../docs/COMMUNICATION_PROTOCOL.md)
- Numeric type IDs: [`TLV_TypeDefs.json`](./TLV_TypeDefs.json)
- Firmware wire structs: [`firmware/arduino/src/messages/TLV_Payloads.h`](../firmware/arduino/src/messages/TLV_Payloads.h)

This file owns exact payload layout and byte size. It should not duplicate the
higher-level behavioral rules from [`docs/COMMUNICATION_PROTOCOL.md`](../docs/COMMUNICATION_PROTOCOL.md).

All payloads are tightly packed and little-endian.

## Conventions

- `u8/u16/u32`: unsigned integer fields
- `i8/i16/i32`: signed integer fields
- `f32`: IEEE 754 float
- Channel and motor IDs on the wire are 0-based
- Timestamps are Arduino-side monotonic runtime counters

## System

### `SYS_HEARTBEAT = 1` ↓

`PayloadHeartbeat` — 5 bytes

| Field | Type | Notes |
|---|---|---|
| `timestamp` | `u32` | RPi milliseconds since boot |
| `flags` | `u8` | Reserved |

### `SYS_STATE = 2` ↑

`PayloadSysState` — 12 bytes

| Field | Type | Notes |
|---|---|---|
| `state` | `u8` | `INIT/IDLE/RUNNING/ERROR/ESTOP` |
| `warningFlags` | `u8` | Liveness/loop/no-battery/control-miss warnings |
| `errorFlags` | `u8` | Undervoltage/overvoltage/encoder/I2C/IMU errors |
| `runtimeFlags` | `u8` | Link/I2C/servo/battery/IMU health bits |
| `uptimeMs` | `u32` | Arduino uptime |
| `lastRxMs` | `u16` | Time since last TLV receive |
| `lastCmdMs` | `u16` | Time since last non-heartbeat command |

### `SYS_CMD = 3` ↓

`PayloadSysCmd` — 4 bytes

| Field | Type | Notes |
|---|---|---|
| `command` | `u8` | `START/STOP/RESET/ESTOP` |
| `reserved[3]` | `u8[3]` | Reserved |

### `SYS_INFO_REQ = 4` ↓

`PayloadSysInfoReq` — 4 bytes

### `SYS_INFO_RSP = 5` ↑

`PayloadSysInfoRsp` — 24 bytes

| Field | Type | Notes |
|---|---|---|
| `firmwareMajor/minor/patch` | `u8/u8/u8` | Firmware version |
| `protocolMajor/minor` | `u8/u8` | TLV protocol version |
| `boardRevision` | `u8` | Board revision code |
| `featureMask` | `u8` | Compile-time feature flags |
| `sensorCapabilityMask` | `u8` | IMU / ultrasonic capability bits |
| `dcMotorCount` | `u8` | Runtime-supported DC motor channels |
| `stepperCount` | `u8` | Runtime-supported stepper channels |
| `servoChannelCount` | `u8` | Servo channels |
| `ultrasonicMaxCount` | `u8` | Max bundled ultrasonic slots |
| `userLedCount` | `u8` | Discrete user LEDs |
| `maxNeoPixelCount` | `u8` | Supported NeoPixel count |
| `limitSwitchMask` | `u16` | Available limit-switch GPIO mask |
| `stepperHomeLimitGpio[4]` | `u8[4]` | Home-limit GPIO mapping, `0xFF` = none |
| `dcHomeLimitGpio[4]` | `u8[4]` | DC home-limit GPIO mapping, `0xFF` = none |

### `SYS_CONFIG_REQ = 6` ↓

`PayloadSysConfigReq` — 4 bytes

### `SYS_CONFIG_RSP = 7` ↑

`PayloadSysConfigRsp` — 8 bytes

| Field | Type | Notes |
|---|---|---|
| `motorDirMask` | `u8` | Direction inversion mask |
| `configuredSensorMask` | `u8` | Configured sensor set |
| `neoPixelCount` | `u8` | Active NeoPixel count |
| `reserved` | `u8` | Reserved |
| `heartbeatTimeoutMs` | `u16` | Link timeout |
| `reserved2` | `u16` | Reserved |

### `SYS_CONFIG_SET = 8` ↓

`PayloadSysConfigSet` — 8 bytes

| Field | Type | Notes |
|---|---|---|
| `motorDirMask` | `u8` | New inversion bits |
| `motorDirChangeMask` | `u8` | Which bits to apply |
| `neoPixelCount` | `u8` | `0 = no change` |
| `configuredSensorMask` | `u8` | `0xFF = no change` |
| `heartbeatTimeoutMs` | `u16` | `0 = no change` |
| `reserved` | `u16` | Reserved |

### `SYS_POWER = 9` ↑

`PayloadSysPower` — 12 bytes

| Field | Type | Notes |
|---|---|---|
| `batteryMv` | `u16` | Battery rail |
| `rail5vMv` | `u16` | 5 V rail |
| `servoRailMv` | `u16` | Servo rail |
| `batteryType` | `u8` | `BATTERY_TYPE` from firmware `config.h` |
| `reserved` | `u8` | Reserved |
| `timestamp` | `u32` | Measurement timestamp |

### `SYS_DIAG_REQ = 10` ↓

`PayloadSysDiagReq` — 4 bytes

### `SYS_DIAG_RSP = 11` ↑

`PayloadSysDiagRsp` — 24 bytes

| Field | Type | Notes |
|---|---|---|
| `freeSram` | `u16` | Free SRAM estimate |
| `loopTimeAvgUs` | `u16` | UART task avg |
| `loopTimeMaxUs` | `u16` | UART task max |
| `uartRxErrors` | `u16` | UART receive error count |
| `crcErrors` | `u16` | CRC decode failures |
| `frameErrors` | `u16` | Frame length / header failures |
| `tlvErrors` | `u16` | TLV header/length failures |
| `oversizeErrors` | `u16` | Oversize frame rejects |
| `txPendingBytes` | `u16` | Pending TX queue bytes |
| `reserved` | `u16` | Reserved |
| `txDroppedFrames` | `u32` | Dropped outgoing frames |

### `SYS_ODOM_RESET = 12` ↓

`PayloadSysOdomReset` — 4 bytes

## DC Motors

### `DC_ENABLE = 16` ↓

`PayloadDCEnable` — 4 bytes

| Field | Type | Notes |
|---|---|---|
| `motorId` | `u8` | 0-based |
| `mode` | `u8` | `DISABLED/POSITION/VELOCITY/PWM` |

### `DC_SET_POSITION = 17` ↓

`PayloadDCSetPosition` — 12 bytes

| Field | Type |
|---|---|
| `motorId` | `u8` |
| `targetTicks` | `i32` |
| `maxVelTicks` | `i32` |

### `DC_SET_VELOCITY = 18` ↓

`PayloadDCSetVelocity` — 8 bytes

### `DC_SET_PWM = 19` ↓

`PayloadDCSetPWM` — 4 bytes

### `DC_RESET_POSITION = 24` ↓

`PayloadDCResetPosition` — 4 bytes

| Field | Type | Notes |
|---|---|---|
| `motorId` | `u8` | 0-based |

### `DC_HOME = 25` ↓

`PayloadDCHome` — 8 bytes

| Field | Type | Notes |
|---|---|---|
| `motorId` | `u8` | 0-based |
| `direction` | `i8` | `+1` or `-1` |
| `homeVelocity` | `i32` | ticks/sec, `0 = firmware default` |

### `DC_STATE_ALL = 20` ↑

`PayloadDCStateAll` — 92 bytes

Per-motor sub-struct `DCMotorState` — 22 bytes:

| Field | Type |
|---|---|
| `mode` | `u8` |
| `faultFlags` | `u8` |
| `position` | `i32` |
| `velocity` | `i32` |
| `targetPos` | `i32` |
| `targetVel` | `i32` |
| `pwmOutput` | `i16` |
| `currentMa` | `i16` |

`faultFlags` bits:

- `0x01` = home limit currently active
- `0x02` = encoder fault latched

Frame layout:

| Field | Type |
|---|---|
| `motors[4]` | `DCMotorState[4]` |
| `timestamp` | `u32` |

### `DC_PID_REQ = 21` ↓

`PayloadDCPidReq` — 4 bytes

### `DC_PID_RSP = 22` ↑

`PayloadDCPidRsp` — 24 bytes

### `DC_PID_SET = 23` ↓

Alias of `PayloadDCPidRsp` — 24 bytes

## Steppers

### `STEP_ENABLE = 32` ↓

`PayloadStepEnable` — 4 bytes

### `STEP_MOVE = 33` ↓

`PayloadStepMove` — 8 bytes

### `STEP_HOME = 34` ↓

`PayloadStepHome` — 12 bytes

### `STEP_STATE_ALL = 35` ↑

`PayloadStepStateAll` — 68 bytes

Per-stepper sub-struct `StepperChannelState` — 16 bytes:

| Field | Type |
|---|---|
| `enabled` | `u8` |
| `motionState` | `u8` |
| `limitFlags` | `u8` |
| `reserved` | `i8` |
| `count` | `i32` |
| `targetCount` | `i32` |
| `currentSpeed` | `i32` |

### `STEP_CONFIG_REQ = 36` ↓

`PayloadStepConfigReq` — 4 bytes

### `STEP_CONFIG_RSP = 37` ↑

`PayloadStepConfigRsp` — 12 bytes

### `STEP_CONFIG_SET = 38` ↓

Alias of `PayloadStepConfigRsp` — 12 bytes

## Servos

### `SERVO_ENABLE = 48` ↓

`PayloadServoEnable` — 4 bytes

### `SERVO_SET = 49` ↓

Two supported payload variants:

- `PayloadServoSetSingle` — 4 bytes
- `PayloadServoSetBulk` — 34 bytes

### `SERVO_STATE_ALL = 50` ↑

`PayloadServoStateAll` — 40 bytes

## Sensors

### `SENSOR_IMU = 64` ↑

`PayloadSensorIMU` — 52 bytes

Quaternion, earth-frame acceleration, raw accel/gyro/mag, calibration flag, timestamp.

### `SENSOR_KINEMATICS = 65` ↑

`PayloadSensorKinematics` — 28 bytes

Differential-drive odometry and body-frame velocity.

### `SENSOR_ULTRASONIC_ALL = 66` ↑

`PayloadSensorUltrasonicAll` — 24 bytes

Per-sensor sub-struct `UltrasonicState` — 4 bytes:

| Field | Type |
|---|---|
| `status` | `u8` |
| `reserved` | `u8` |
| `distanceMm` | `u16` |

Frame layout:

| Field | Type |
|---|---|
| `configuredCount` | `u8` |
| `reserved[3]` | `u8[3]` |
| `sensors[4]` | `UltrasonicState[4]` |
| `timestamp` | `u32` |

### `SENSOR_MAG_CAL_CMD = 67` ↓

`PayloadMagCalCmd` — 52 bytes

| Field | Type | Notes |
|---|---|---|
| `command` | `u8` | `START/STOP/SAVE/APPLY/CLEAR` |
| `reserved[3]` | `u8[3]` | Reserved |
| `offsetX/Y/Z` | `f32/f32/f32` | Hard-iron offset |
| `softIronMatrix[9]` | `f32[9]` | Row-major 3×3 matrix |

### `SENSOR_MAG_CAL_STATUS = 68` ↑

`PayloadMagCalStatus` — 44 bytes

## User I/O

### `IO_SET_LED = 80` ↓

`PayloadSetLED` — 8 bytes

| Field | Type | Notes |
|---|---|---|
| `ledId` | `u8` | 0-based |
| `mode` | `u8` | `OFF/ON/BLINK/BREATHE/PWM` |
| `brightness` | `u8` | Brightness ceiling |
| `reserved` | `u8` | Reserved |
| `periodMs` | `u16` | Animation period for `BLINK/BREATHE` |
| `dutyCycle` | `u16` | Permille timing parameter; see [`docs/COMMUNICATION_PROTOCOL.md`](../docs/COMMUNICATION_PROTOCOL.md) for semantics |

### `IO_SET_NEOPIXEL = 81` ↓

`PayloadSetNeoPixel` — 4 bytes

### `IO_INPUT_STATE = 82` ↑

`PayloadIOInputState` — 8 bytes

### `IO_OUTPUT_STATE = 83` ↑

`PayloadIOOutputState` — 11 bytes fixed + `3 * neoPixelCount` appended bytes

| Field | Type |
|---|---|
| `ledBrightness[5]` | `u8[5]` |
| `neoPixelCount` | `u8` |
| `reserved` | `u8` |
| `timestamp` | `u32` |

The fixed payload may be followed by `neoPixelCount` RGB triplets, each encoded
as `r,g,b` bytes.

## Size Summary

| Type | Name | Dir | Size |
|---|---|---|---|
| 1 | `SYS_HEARTBEAT` | ↓ | 5 |
| 2 | `SYS_STATE` | ↑ | 12 |
| 3 | `SYS_CMD` | ↓ | 4 |
| 4 | `SYS_INFO_REQ` | ↓ | 4 |
| 5 | `SYS_INFO_RSP` | ↑ | 24 |
| 6 | `SYS_CONFIG_REQ` | ↓ | 4 |
| 7 | `SYS_CONFIG_RSP` | ↑ | 8 |
| 8 | `SYS_CONFIG_SET` | ↓ | 8 |
| 9 | `SYS_POWER` | ↑ | 12 |
| 10 | `SYS_DIAG_REQ` | ↓ | 4 |
| 11 | `SYS_DIAG_RSP` | ↑ | 24 |
| 12 | `SYS_ODOM_RESET` | ↓ | 4 |
| 16 | `DC_ENABLE` | ↓ | 4 |
| 17 | `DC_SET_POSITION` | ↓ | 12 |
| 18 | `DC_SET_VELOCITY` | ↓ | 8 |
| 19 | `DC_SET_PWM` | ↓ | 4 |
| 20 | `DC_STATE_ALL` | ↑ | 92 |
| 21 | `DC_PID_REQ` | ↓ | 4 |
| 22 | `DC_PID_RSP` | ↑ | 24 |
| 23 | `DC_PID_SET` | ↓ | 24 |
| 24 | `DC_RESET_POSITION` | ↓ | 4 |
| 25 | `DC_HOME` | ↓ | 8 |
| 32 | `STEP_ENABLE` | ↓ | 4 |
| 33 | `STEP_MOVE` | ↓ | 8 |
| 34 | `STEP_HOME` | ↓ | 12 |
| 35 | `STEP_STATE_ALL` | ↑ | 68 |
| 36 | `STEP_CONFIG_REQ` | ↓ | 4 |
| 37 | `STEP_CONFIG_RSP` | ↑ | 12 |
| 38 | `STEP_CONFIG_SET` | ↓ | 12 |
| 48 | `SERVO_ENABLE` | ↓ | 4 |
| 49 | `SERVO_SET` | ↓ | 4 or 34 |
| 50 | `SERVO_STATE_ALL` | ↑ | 40 |
| 64 | `SENSOR_IMU` | ↑ | 52 |
| 65 | `SENSOR_KINEMATICS` | ↑ | 28 |
| 66 | `SENSOR_ULTRASONIC_ALL` | ↑ | 24 |
| 67 | `SENSOR_MAG_CAL_CMD` | ↓ | 52 |
| 68 | `SENSOR_MAG_CAL_STATUS` | ↑ | 44 |
| 80 | `IO_SET_LED` | ↓ | 8 |
| 81 | `IO_SET_NEOPIXEL` | ↓ | 4 |
| 82 | `IO_INPUT_STATE` | ↑ | 8 |
| 83 | `IO_OUTPUT_STATE` | ↑ | 11 + `3n` |
