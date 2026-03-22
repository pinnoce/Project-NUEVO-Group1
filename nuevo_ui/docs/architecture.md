# NUEVO UI — Architecture Reference

Technical internals for developers extending or debugging the stack.

---

## System overview

```
Arduino Mega 2560
   │  UART @ 250 kbaud, compact TLV framing
   ▼
nuevo_bridge  (Python, FastAPI + asyncio)
   ├── serial_manager.py   — owns the serial port, heartbeat, decode loop
   ├── message_router.py   — registry: TLV type ↔ JSON topic
   ├── payloads.py         — ctypes structs matching TLV_Payloads.h exactly
   ├── ws_manager.py       — WebSocket connection pool, broadcast
   ├── auth.py / seal.py   — JWT authentication
   └── ros2_node.py        — optional ROS2 bridge (NUEVO_ROS2=1)
   │
   ├── /ws  (WebSocket)    — real-time telemetry + commands
   ├── /auth               — login, token validation
   └── /                   — serves compiled React SPA (production)
         │
    Browser (React SPA)
         ├── useWebSocket hook    — WS connection, auto-reconnect, dispatch to store
         ├── robotStore (Zustand) — all robot state, single source of truth
         └── Components           — subscribe to store slices, send via wsSend()
```

---

## Backend modules

### `serial_manager.py`
- Runs as an asyncio background task (`run_in_executor` for blocking `serial.read()`)
- Sends a heartbeat TLV frame every 200 ms (keeps Arduino alive)
- On each decoded frame: calls `message_router.handle_incoming(tlv_type, payload_bytes)`
- Exposes `send(tlv_type, ctypes_payload)` for outbound commands
- Tracks: `rx_count`, `tx_count`, `crc_errors`, connection state
- Auto-reconnects on serial disconnect with exponential backoff
- `NUEVO_MOCK=1`: replaces with a mock that generates synthetic telemetry at 10 Hz

### `message_router.py` — registry pattern
Adding support for a new TLV message type = **one dict entry + one ctypes struct**. No other changes needed.

```python
# Incoming: Arduino → decoded bytes → JSON → WebSocket broadcast
INCOMING_REGISTRY = {
    TLV_TYPE:  ("ws_topic_name",  PayloadStruct),
    ...
}

# Outgoing: browser JSON command → ctypes struct → TLV → Arduino
OUTGOING_REGISTRY = {
    "cmd_name": (TLV_TYPE, PayloadStruct),
    ...
}
```

The router auto-converts ctypes structs to/from JSON using field introspection. Array fields (e.g., `position[4]`) become JSON arrays automatically.

It also hosts bridge-side behaviors that need to observe telemetry and emit
commands. The main example is `MagCalibrationController`, which watches:

- `mag_cal_status` for sampling state and progress
- `imu` for raw magnetometer samples

When the user starts IMU calibration, the router-side controller collects raw
mag samples, fits hard-iron offset plus soft-iron matrix, and sends one
`mag_cal_cmd` apply packet back to the firmware.

### `payloads.py` — ctypes structs
All structs match `firmware/arduino/src/messages/TLV_Payloads.h` exactly.
Sizes are verified against firmware `STATIC_ASSERT_SIZE` macros:

| Struct | Size (bytes) |
|--------|-------------|
| PayloadHeartbeat | 5 |
| PayloadSystemStatus | 18 |
| PayloadSetPID | 24 |
| PayloadGetPID | 4 |
| PayloadResPID | 24 |
| PayloadDCEnable | 4 |
| PayloadDCSetPosition | 12 |
| PayloadDCSetVelocity | 12 |
| PayloadDCStatus | 24 |
| PayloadStepEnable | 4 |
| PayloadStepSetAccel | 12 |
| PayloadStepSetVel | 8 |
| PayloadStepMove | 8 |
| PayloadStepHome | 12 |
| PayloadStepStatus | 20 |
| PayloadServoEnable | 4 |
| PayloadServoSetSingle | 4 |
| PayloadSensorVoltage | 8 |
| PayloadSensorEncoder | 36 |
| PayloadSensorCurrent | 12 |
| PayloadIMU | 24 |
| PayloadSensorRange | 8 |
| PayloadMagCalCmd | 52 |
| PayloadMagCalStatus | 44 |
| PayloadSetLED | 8 |
| PayloadSetNeoPixel | 4 |
| PayloadButtonState | 8 |
| PayloadLimitState | 8 |

### `tlvcodec/`
Python TLV encoder/decoder matching the Arduino C implementation.
Located at `backend/tlvcodec/`. Installed as a local package alongside `nuevo_bridge`.
Source: lives at `backend/tlvcodec/` as the standalone Python TLV codec used by `nuevo_bridge`.

### Authentication
- JWT tokens issued at `/auth/login` (username + password from `users.json`)
- Every WebSocket connection must supply `?token=<jwt>` — rejected with close code `4001` if invalid
- `seal.py` handles token signing/verification (HMAC-SHA256)
- `users.json` stores bcrypt-hashed passwords; admin role can manage other users via the UI

---

## Frontend structure

```
src/
├── App.tsx                  # Auth gate → Dashboard
├── store/
│   ├── robotStore.ts        # Zustand: all robot telemetry state + dispatch()
│   └── authStore.ts         # Zustand: JWT token, username, role
├── hooks/
│   └── useWebSocket.ts      # WS connection, auto-reconnect, token in ?token=
├── lib/
│   ├── wsProtocol.ts        # TypeScript types for WS messages/commands
│   ├── wsSend.ts            # Global send() singleton (avoids prop drilling)
│   └── csvExport.ts         # CSV generation + browser download
└── components/
    ├── PowerSection.tsx
    ├── StepperSection.tsx
    ├── ServoSection.tsx
    ├── RpiSystemSection.tsx
    ├── ArduinoSystemSection.tsx
    ├── DCMotorSection.tsx
    ├── UserIOSection.tsx
    ├── LoginPage.tsx
    ├── UserManagementModal.tsx
    ├── dc/
    │   └── DCPlot.tsx       # uPlot real-time chart (position, velocity, current)
    └── common/
        ├── Modal.tsx
        └── RecordCSV.tsx    # Start/stop recording, download as CSV
```

### State flow
1. `useWebSocket` receives a JSON frame `{topic, data, ts}` from the bridge
2. Calls `robotStore.dispatch(topic, data, ts)` — updates the relevant store slice
3. Components subscribe to specific slices via `useRobotStore((s) => s.xyz)`
4. Commands go through `wsSend(cmd, data)` → WebSocket → bridge → Arduino

### IMU calibration flow

- `SensorSection` shows IMU data even when the magnetometer is uncalibrated
- if `imu.magCalibrated == 0`, the IMU card shows an `Uncalibrated` warning and
  a `Calibrate IMU` action
- clicking calibrate opens a modal with figure-8 + tilt instructions
- if the robot is `RUNNING`, the frontend first sends `sys_cmd(STOP)` and waits
  for `IDLE` before starting calibration
- once the firmware enters sampling mode, the bridge auto-fits the final
  hard/soft-iron calibration and sends `mag_cal_cmd(MAG_CAL_APPLY)`
- the UI closes on success, or returns to the uncalibrated state on cancel/fail

### Charts (uPlot)
- `DCPlot.tsx` uses [uPlot](https://github.com/leeoniya/uPlot) for high-performance real-time plotting
- Data stored in `robotStore` as fixed-size ring buffers (Float64Array)
- Renders at 30 fps via `requestAnimationFrame`; telemetry arrives at up to 100 Hz
- `RecordCSV` accumulates a separate array during recording, downloads on stop

---

## WebSocket protocol — full reference

### Frame format
```json
// Bridge → browser
{"topic": "<name>", "data": { ... }, "ts": 1234567890.123}

// Browser → bridge
{"cmd": "<name>", "data": { ... }}
```

### Incoming topics (bridge → browser)

```json
{"topic": "system_status", "data": {
  "uptimeMs": 123456, "loopHz": 994, "lastHeartbeatMs": 48,
  "errorFlags": 0, "motorEnableMask": 3, "stepperEnableMask": 0
}}

{"topic": "voltage", "data": {
  "batteryMv": 12400, "rail5vMv": 5020, "servoRailMv": 6000
}}

{"topic": "dc_status_all", "data": {
  "motors": [
    {"motorId": 0, "enabled": true, "mode": 2,
     "position": 1234, "velocity": 500,
     "targetPos": 0, "targetVel": 500,
     "pwmOutput": 128, "currentMa": 350},
    ...
  ]
}}

{"topic": "step_status_all", "data": {
  "steppers": [
    {"stepperId": 0, "enabled": true, "state": 1,
     "limitHit": 0, "position": 500, "targetPos": 1000,
     "currentVel": 200, "stepsRemaining": 500},
    ...
  ]
}}

{"topic": "servo_status_all", "data": {
  "pca9685Connected": true, "pca9685Error": 0,
  "channels": [
    {"channelNumber": 1, "enabled": true, "pulseUs": 1500},
    ...
  ]
}}

{"topic": "io_status", "data": {
  "buttonMask": 5, "limitSwitchMask": 2,
  "ledBrightness": [255, 0, 128, 0, 0],
  "neopixels": [{"r": 0, "g": 255, "b": 0}]
}}

{"topic": "imu", "data": {
  "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
  "accelX": 100, "accelY": -50, "accelZ": 16384,
  "gyroX": 0, "gyroY": 5, "gyroZ": -3,
  "magX": 200, "magY": 300, "magZ": -100,
  "calSys": 3, "calGyro": 3, "calAccel": 3, "calMag": 1
}}

{"topic": "connection", "data": {
  "serialConnected": true, "port": "/dev/ttyAMA0", "baud": 500000,
  "rxCount": 1234, "txCount": 567, "crcErrors": 2
}}
```

### Outgoing commands (browser → bridge)

```json
{"cmd": "dc_enable",       "data": {"motorId": 0, "enable": 1, "mode": 2}}
{"cmd": "dc_set_velocity", "data": {"motorId": 0, "targetVel": 1000, "maxAccel": 0}}
{"cmd": "dc_set_position", "data": {"motorId": 0, "targetPos": 5000, "maxVelocity": 0}}
{"cmd": "dc_set_pwm",      "data": {"motorId": 0, "pwm": 128}}
{"cmd": "set_pid",         "data": {"motorId": 0, "motorType": 0, "loopType": 1,
                                    "kp": 0.5, "ki": 0.1, "kd": 0.0,
                                    "maxOutput": 255.0, "maxIntegral": 100.0}}

{"cmd": "step_enable",     "data": {"stepperId": 0, "enable": 1}}
{"cmd": "step_move",       "data": {"stepperId": 0, "moveType": 0, "target": 1000}}
{"cmd": "step_set_params", "data": {"stepperId": 0, "maxVelocity": 500,
                                    "accel": 1000, "decel": 1000}}
{"cmd": "step_home",       "data": {"stepperId": 0, "direction": 1,
                                    "homeVelocity": 200, "backoffSteps": 50}}

{"cmd": "servo_enable",    "data": {"channel": 1, "enable": true}}
{"cmd": "servo_set",       "data": {"channel": 1, "pulseUs": 1500}}

{"cmd": "set_led",         "data": {"ledId": 0, "mode": 1, "brightness": 255}}
{"cmd": "set_neopixel",    "data": {"index": 0, "red": 0, "green": 255, "blue": 0}}

{"cmd": "sys_cmd",         "data": {"command": 4}}   // 4 = E-STOP
{"cmd": "mag_cal_cmd",     "data": {"command": 1}}
```

Full field definitions: `backend/nuevo_bridge/message_router.py`
