# Sensors and Shared I2C Bus

This document describes the sensor stack that is actually supported in the
current Arduino firmware.

## Supported on Arduino

- ICM-20948 IMU
- SparkFun Qwiic ultrasonic sensor(s)
- battery / 5 V / servo-rail voltage monitoring
- PCA9685 servo controller on the same `Wire` bus

## Not supported on Arduino

- Garmin / SparkFun LIDAR-Lite v4

The LIDAR-Lite v4 is now treated as a Raspberry Pi-side sensor only. The
Arduino firmware no longer contains a lidar driver, polling path, telemetry
path, or status reporting for it. If the robot uses this lidar, connect it to
the Pi-side Qwiic bus and use the Pi-side test tooling under
[`ros2_ws/tests/`](../../ros2_ws/tests/).

## 1. Shared `Wire` Bus Ownership

The Arduino Mega uses one global `Wire` bus for all I2C peripherals that remain
on the MCU side:

- IMU
- ultrasonic sensor(s)
- PCA9685 servo controller

The firmware does not implement a separate software I2C arbiter object. Bus
access is serialized by execution context instead:

- only one soft task runs at a time
- no `Wire` calls are allowed from Timer1 / Timer3 / encoder ISR paths
- deferred servo work runs from the UART soft task, not from decode context

So there is no true concurrent I2C access, but a slow transaction can still
lengthen a soft task and add loop jitter.

## 2. Why I2C Stays Out of ISRs

`Wire` is too slow and too opaque for the hard real-time budgets in this
firmware.

The current rules are:

- no I2C in `ISR(TIMER1_OVF_vect)`
- no I2C in `ISR(TIMER3_OVF_vect)`
- no I2C in encoder interrupts
- no I2C in TLV decode handlers

This is why:

- sensor polling lives in `SensorManager::tick()`
- servo writes are deferred and applied later from `MessageCenter::processDeferred()`
- magnetometer calibration save/apply/clear side effects are deferred as well

## 3. `SensorManager`

`SensorManager` is the central owner of Arduino-side sensing.

It runs from `taskSensors()` at `100 Hz` and dispatches internal work at three
rates:

- `update100Hz()` -> IMU + Fusion AHRS
- `update50Hz()` -> ultrasonic sensors
- `update10Hz()` -> voltage rails

`taskSensors()` also calls `UserIO::sampleInputs()` so button and limit-switch
sampling stays with state acquisition, not safety policy.

### IMU path

Files:

- `drivers/IMUDriver.*`
- `modules/SensorManager.*`
- `lib/Fusion/*`

Implementation:

- `IMUDriver` wraps the SparkFun ICM-20948 library
- the firmware does not use the DMP path
- `SensorManager::update100Hz()` checks `dataReady()`, reads the sensor, and
  updates `FusionWrapper`
- if magnetometer calibration is active, the fusion path is 9-DoF
- otherwise the firmware uses 6-DoF fusion without magnetometer correction

### Ultrasonic path

Files:

- `drivers/UltrasonicDriver.*`
- `modules/SensorManager.*`

Implementation:

- ultrasonic sensors are initialized in `SensorManager::init()`
- configured slots that do not respond are tracked as "configured but missing"
- `update50Hz()` iterates each configured ultrasonic slot and reads the latest
  distance from the SparkFun Qwiic ultrasonic board

Current timing model:

- blocking I2C reads
- scheduled at `50 Hz`
- timing reported separately in the status output as `ultra`

### Voltage monitoring

Voltage rails are not I2C devices. They are sampled through the ADC in
`SensorManager::updateVoltages()`.

Measured rails:

- battery input
- 5 V rail
- servo rail

These values feed:

- `SystemManager` battery-enable and battery-fault policy
- status reporting
- TLV telemetry

## 4. Magnetometer Calibration

Magnetometer calibration state lives in `SensorManager`, but command flow
starts in `MessageCenter`.

State machine:

- `IDLE`
- `SAMPLING`
- `COMPLETE`
- `SAVED`
- `ERROR`

Important implementation details:

- sampling is updated from the IMU update path
- persistence goes through `PersistentStorage`
- start/stop/save/apply/clear side effects are deferred out of message decode
  context

So the calibration flow touches both IMU I2C and EEPROM, while still staying
out of ISR paths.

## 5. PCA9685 Servo Controller on the Same Bus

This is the most important I2C interaction on the Arduino side.

Files:

- `drivers/ServoController.*`
- `modules/MessageCenter.*`

Implementation:

- `ServoController` owns PCA9685 initialization, OE control, pulse-width
  conversion, and grouped channel writes
- the PCA9685 shares the same `Wire` bus as the IMU and ultrasonic sensors
- servo I2C writes are never done directly from packet decode
- decode handlers only mark pending servo work
- `MessageCenter::processDeferred()` later applies the pending writes in soft
  task context

This deferred path exists specifically to keep the decode-side UART path short
and to avoid I2C work in timing-sensitive contexts.

## 6. Timing and Debug Output

There are two different sensor-related timing views in the status report.

### Whole soft sensor task

Reported as:

- `sensor a/b/c (...) us`

This is the timing of `taskSensors()` as a whole, including:

- `SensorManager::tick()`
- `UserIO::sampleInputs()`

### Ultrasonic loop only

Reported as:

- `ultra a/b/c (...) us`

This measures only the ultrasonic read loop inside `SensorManager::update50Hz()`.

The `[SENSORS]` block prints the current ultrasonic values, including slots
that are configured but missing.

## 7. Bus Settings and Stability Notes

Current bus settings are conservative on purpose:

- `I2C_BUS_CLOCK_HZ = 100000`
- `I2C_WIRE_TIMEOUT_US = 5000`

Those defaults prioritize recovery and shared-bus stability over raw throughput.

The servo controller, IMU, and ultrasonic sensors are expected to coexist on
that bus. If a new I2C device is added on the Arduino side, it should follow
the same rules:

- no ISR use
- no decode-context use
- explicit timeout behavior if the library can block

## 8. LIDAR-Lite v4 Note

The Project NUEVO firmware previously contained an Arduino-side LIDAR-Lite v4
integration. That path was removed because the lidar did not behave reliably on
the shared Arduino I2C topology.

Current project decision:

- Arduino firmware: no lidar support
- Raspberry Pi side: lidar support lives there instead

Keep that separation unless the hardware topology changes enough to justify a
new Arduino-side design.
