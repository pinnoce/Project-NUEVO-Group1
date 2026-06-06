# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MAE 162 lab robotics platform (Project NUEVO) — a two-wheeled mobile robot with Arduino firmware, a Raspberry Pi 5 running ROS2, a FastAPI bridge backend, and a React/TypeScript web UI.

## System Architecture

```
Browser (React/TS)
    ↕ WebSocket (JSON)
nuevo_bridge (FastAPI, runs on RPi)          ← nuevo_ui/backend/
    ├── owns Arduino UART serial (TLV protocol)
    └── optionally bridges to ROS2 topics
         ↕ ROS2 topics (bridge_interfaces msgs)
    bridge node (ros2_ws/src/bridge/)
         ↕ Robot API publishers/subscriptions
    robot.py  (ros2_ws/src/robot/robot/robot.py)  ← Layer 1: wraps all ROS topics
    main.py   (ros2_ws/src/robot/robot/main.py)   ← Layer 2: student FSM, you work here
    path_planner.py                               ← Layer 3: pure pursuit, APF, LAPF
         ↕ UART TLV (500 kbaud, /dev/ttyAMA0)
Arduino Mega 2560 firmware (firmware/arduino/)
```

The bridge node must be running before any `Robot` API calls work. The web UI and ROS code see the same live data — they are both driven by the same `nuevo_bridge` backend.

## Key Source Locations

| Path | Purpose |
|------|---------|
| `ros2_ws/src/robot/robot/main.py` | Student FSM — the only file students normally edit |
| `ros2_ws/src/robot/robot/robot.py` | Robot API source of truth |
| `ros2_ws/src/robot/robot/hardware_map.py` | Physical robot constants (wheel diameter, motor IDs, lidar params) |
| `ros2_ws/src/robot/robot/examples/` | Active worked examples; copy to `main.py` to run |
| `ros2_ws/src/bridge/bridge/bridge_node.py` | ROS ↔ TLV bridge node |
| `ros2_ws/src/bridge_interfaces/msg/` | All custom ROS message types |
| `nuevo_ui/backend/` | FastAPI backend (`nuevo_bridge`) |
| `nuevo_ui/frontend/` | React/TypeScript frontend |
| `firmware/arduino/arduino.ino` | Arduino entry point |
| `firmware/arduino/src/config.h` | Compile-time firmware tuning |
| `tlv_protocol/TLV_Payloads.md` | Wire payload byte layouts |
| `docs/COMMUNICATION_PROTOCOL.md` | Protocol behavioral source of truth |

## Running the Robot (RPi, hardware)

```bash
# Start ROS2 container (builds workspace on first run, ~60 s)
docker compose -f ros2_ws/docker/docker-compose.rpi.yml up -d --build --wait

# Watch logs
docker compose -f ros2_ws/docker/docker-compose.rpi.yml logs -f ros2_runtime

# Open a shell inside the running container
./ros2_ws/docker/enter_ros2.sh rpi

# Inside container — run the student robot program
ros2 run robot robot

# Inside container — run a specific example
cp ros2_ws/src/robot/robot/examples/motion_basics.py ros2_ws/src/robot/robot/main.py
ros2 run robot robot

# Verify container health (10 checks, all should pass)
./ros2_ws/docker/check_rpi_runtime.sh
```

## Running for Development (VM / no hardware)

```bash
# Start mock container
docker compose -f ros2_ws/docker/docker-compose.vm.yml up -d --build --wait

# Enter container
./ros2_ws/docker/enter_ros2.sh vm

# Inside container — launch bridge
ros2 launch bridge bridge.launch.py
```

## UI Development

```bash
# Quick start (mock mode, no hardware)
cd nuevo_ui && ./scripts/dev.sh
# Backend at http://localhost:8000, frontend at http://localhost:5173

# Manual backend (mock)
cd nuevo_ui/backend
pip install -e . --break-system-packages   # first time only
NUEVO_MOCK=1 python3 -m nuevo_bridge

# Manual frontend
cd nuevo_ui/frontend
npm install   # first time only
npm run dev

# Production build (deploy to backend static/)
cd nuevo_ui/frontend && npm ci && npm run build
cd .. && cp -r frontend/dist/. backend/static/
```

UI env vars: `NUEVO_MOCK=1` (simulate Arduino), `NUEVO_SERIAL_PORT=/dev/ttyAMA0`, `NUEVO_ROS2=1`.

## Running Tests

```bash
# APF/path planner unit tests (no ROS needed)
cd ros2_ws/src/robot
python3 -m pytest test/test_path_planner.py -v

# All robot unit tests
python3 -m pytest test/ -v

# Vision node tests
cd ros2_ws/src/vision
python3 -m pytest test/ -v

# Bridge firmware state service test
cd ros2_ws/src/bridge
python3 -m pytest test/ -v
```

## Arduino Firmware

Build and upload via Arduino IDE 2.x (board: `Arduino Mega or Mega 2560`). Serial buffer must be enlarged — add this to `platform.local.txt` for the AVR core:
```
compiler.cpp.extra_flags=-DSERIAL_RX_BUFFER_SIZE=512 -DSERIAL_TX_BUFFER_SIZE=256
```

CLI alternative:
```bash
arduino-cli compile --fqbn arduino:avr:mega firmware/arduino
arduino-cli upload --fqbn arduino:avr:mega -p /dev/ttyAMA0 firmware/arduino
```

## Student Code Patterns

`main.py` must export a `run(robot: Robot) -> None` function. `robot_node.py` calls it after ROS setup.

### Current `main.py` FSM (full competition mission)

The shipped `main.py` runs this sequence on `BTN_1` (with `BTN_2` cancelling to `IDLE` at any stage). Static check: every `state = 'X'` / `_after_pause = 'X'` target resolves to a defined `elif state == 'X'` (89 states, no orphans).

1. `IDLE` — orange LED; `prime_lift` raises the lift to `LIFT_CARRY_STEPS` and opens the gripper (once per IDLE visit, gated by `_lift_primed`). Waits for `BTN_1`.
2. **Traffic light** — `TL_TURN_LEFT` (turn `LOOK_LEFT_DEG`) → `TL_WATCH` (green-light detection: YOLO `traffic light` box ≥ confidence AND HSV `color.value=="green"` with `color.score` gate) → `TL_TURN_BACK`.
3. `MOV1_DRIVE` — drive forward to the left-bun shelf.
4. **Burger sub-FSM** — 3 stops `BUR_0..2` in order **left-bun → patty → right-bun** (rewritten 2026-06-06; replaces the old patty/LB/RB/stack 4-stop flow). Each stop runs the same regular 9-state pattern: `BUR_n_TURN` (relative turn-left `BUR_FACE_TURN_DEG[n]` to face the shelf) → `BUR_n_ALIGN` (forward-wall perpendicular square, FOV `BUR_PERP_FOV_DEG[n]`) → `BUR_n_APPROACH` (drive until `closest_forward_obstacle_mm ≤ SHELF_STANDOFF_MM`, then manipulate, then `_retreat(BUR_RETREAT_MM[n])`) → `BUR_n_TURN_BACK` (relative turn-right `BUR_UNFACE_TURN_DEG[n]`) → `BUR_n_PARA` (parallel left-wall square, FOV `BUR_PARA_FOV_DEG[n]`) → `BUR_n_DRIVE` (`BUR_TRAVEL_MM[n]` forward to line up beside the next item). Manipulation: BUR_0 picks the left bun with an empty gripper (`_pick(LIFT_SHELF_STEPS, …)`); BUR_1 and BUR_2 use `_place_layer_then_pick(place_steps, grip_deg)` (raise the held layer to `place_steps` → open to drop it onto the shelf item below → lower to `LIFT_SHELF_STEPS` → close → raise), so the burger is built bottom-up bun→patty→bun. Every travel leg is preceded by a parallel re-square, so the old `INTER_TURN_DEG`/`BUR_TRAVEL_12_ALIGN` LB→RB slip fix is gone. The last stop (BUR_2) skips its travel drive and hands to MOV2, ending **parallel** to the shelf (shelf on left), not facing it.
5. **MOV2** — scripted corridor: relative turns `MOV2_TURN_1..5` + lidar parallel-wall aligns (`MOV2_ALIGN_LEFT_*` / `MOV2_ALIGN_RIGHT_*`). Forward legs 1 & 5 are fixed-distance drives; **legs 2/3/4 are wall-standoff approaches** (`MOV2_APPROACH_2/3/4`: drive forward until `forward_wall_gap_mm ≤ MOV2_APPROACH_n_STANDOFF_MM`, capped by the old `MOV2_DRIVE_n_MM`). The corridor ends with one right-wall square (`MOV2_ALIGN_RIGHT_1`, a short `MOV2_DRIVE_5`, then a single **`MOV2_TURN_5`** = `+MOV2_TURN_5_DEG` left turn). A second redundant right-wall square (`MOV2_ALIGN_RIGHT_2`) was removed.
6. `LAPF_RUN` — `robot.lapf_to_goal(LAPF_GOAL, ...)` through the cone field. **Odometry is reset to `(0,0,90°)` in `MOV2_TURN_5_WAIT`** (right after the final turn, before LAPF), so `LAPF_GOAL = (0, 2700)` is a **robot-relative** straight-ahead goal — this discards the drift accumulated through burger+MOV2 so LAPF aims dead ahead. Everything downstream of LAPF is lidar-relative, so the mid-mission reset is safe.
7. **MOV3** — `MOV3_ALIGN_PERP` (perpendicular forward-wall square) → `MOV3_APPROACH` (until `MOV3_WALL_STANDOFF_MM`) → `MOV3_TURN` → parallel drive.
8. **Gender ID** — `get_detections("person")[i].attributes["gender"]` (≥ `MIN_PERSON_CONFIDENCE`) selects the drop-off distance.
9. **MOV4** — navigate to drop-off, gender-dependent final distance (`MOV4_DRIVE_FEMALE_MM` / `MOV4_DRIVE_MALE_MM`).
10. **Burger drop-off** — `DROP_TURN` → `DROP_ALIGN` → `DROP_APPROACH` → `DROP` (release) → `DROP_RETREAT` → `DROP_TURN_BACK` → **`DROP_ALIGN_LEFT`** (lidar parallel-left re-square of heading, FOV `DROP_TURN_BACK_ALIGN_FOV`) → stop-sign leg.
11. **Stop sign** — `STOP_DRIVE`/`STOP_DRIVE_LOOP` creep until `stop sign` confidence ≥ `MIN_STOP_SIGN_CONFIDENCE`, then **`STOP_APPROACH`** drives a **gender-dependent** distance further (male/base = `STOP_SIGN_APPROACH_MM`; female — and the no-detection default — adds `STOP_SIGN_APPROACH_FEMALE_EXTRA_MM`, reusing the `_detected_gender` from the MOV3 Gender ID), **`STOP_APPROACH_WAIT`** stops and waits `STOP_SIGN_WAIT_S`, then `STOP_FINAL_DRIVE` (`STOP_SIGN_FINAL_MM`). The detection timeout path skips the approach and goes straight to the wait.
12. `DONE` — sets `_lift_primed = True` so returning to `IDLE` does **not** re-raise the lift / move the servo. `BTN_1` re-clears it to restart.

**`main.py` is the canonical full-mission file** (it's what `robot_node.py` imports and runs; the old `main NEW w LIDAR.py` backup was removed).

**Test-harness siblings** are behaviour-trimmed full copies that, on `BTN_1`, jump straight into a sub-section (most use `TEST_SIMULATE_CARRY` to raise the lift + close the gripper, mimicking holding the burger). They differ from `main.py` only in the IDLE `BTN_1` jump, the `simulate_carry` helper, and a `# SEG…` end-redirect, so they diff cleanly. The three **LAPF-segment** harnesses split the mission at the LAPF boundary:

| File | `BTN_1` starts at | Ends at |
|------|-------------------|---------|
| `main_pre_lapf_test.py` | real traffic-light start (runs the real burger) | `DONE` right before LAPF (`MOV2_TURN_5_WAIT` → `DONE`) |
| `main_lapf_only_test.py` | `MOV2_DRIVE_5` (final drive + turn + odom reset) | `DONE` at the LAPF goal (`LAPF_WAIT` → `DONE`) |
| `main_post_lapf_test.py` | `MOV3_ALIGN_PERP` | natural `DONE` (drop-off + stop sign) |

(The older `main_post_burger_test.py` and `main_lapf_test.py` siblings carried stale tuning and were removed; these three LAPF-segment harnesses are the only test siblings.)

**Note (2026-06-06):** when the burger sub-FSM was rewritten to the 3-stop order, all three harness siblings were re-synced from the new `main.py` so they diff cleanly again (verified: each is `main.py` + only its IDLE jump / `simulate_carry` / SEG redirect). The re-sync also reset the harnesses' drifted tuning back to `main.py`'s values (`GRIPPER_CLOSE_PATTY_DEG`, `LIFT_MAX_VELOCITY`/`LIFT_ACCELERATION`, `SHELF_STANDOFF_MM`) — re-tune in `main.py` and re-sync rather than letting the harnesses drift.

**Launching a segment without `cp`:** `robot_node.py` takes a `mission_module` ROS param (default `robot.main`) and dynamically imports `run()` from it; `everything.launch.py` forwards a `mission_module` launch arg to the node (still bringing up rplidar + sensors + vision). Three thin launch files set it: `ros2 launch robot pre_lapf_test.launch.py` / `lapf_only_test.launch.py` / `post_lapf_test.launch.py`. (New launch files need a colcon rebuild to register in `share/robot/launch`.)

`configure_robot` calls `robot.enable_vision()` and (when `ENABLE_BURGER_ASSEMBLY=True`) `robot.enable_servo(GRIPPER_SERVO)` + `robot.step_set_config(LIFT_STEPPER, ...)`. `LOOK_HEADING_DEG = INITIAL_THETA_DEG + LOOK_LEFT_OFFSET_DEG` — change `LOOK_LEFT_OFFSET_DEG` (CCW positive) to aim the camera, not `INITIAL_THETA_DEG`.

#### Lift state model (no limit switch)

The lift stepper has no homing reference. To keep absolute-step semantics, `main.py` maintains a Python-side `_LIFT_LOGICAL_STEPS` (boot/park = 0 = shelf surface) and routes every move through `move_lift_to(robot, target)`, which sends a **RELATIVE** `step_move` with delta `target − _LIFT_LOGICAL_STEPS`, applies `_lift_signed(...)` to honor `LIFT_DIR_INVERTED`, and only updates the tracker on successful completion. The firmware's absolute step count is never consulted. Park the gripper at the shelf surface **before power-on** so logical 0 = physical shelf top. Diagnostic `[LIFT] target=… current=… delta=… firmware=…` prints make each move auditable. **The tracker is owned by `prime_lift` and the power-on park contract only** — `reset_mission` must NOT zero it (it's called at INIT *and* on `BTN_1`, and zeroing it after the lift was already raised caused a spurious double-raise / no-op-lower at the patty). `prime_lift` also re-issues `enable_servo` because `configure_robot` runs before the firmware reaches `RUNNING` and a post-Ctrl-C ESTOP drops the early servo-enable. It likewise re-applies the lift step config via `apply_lift_config(robot)` (after `step_enable`): `configure_robot`'s `step_set_config` lands in the boot window before the firmware link is up and is silently dropped, leaving the lift at the slow default so a carry move overruns `LIFT_MOVE_TIMEOUT_S`. **Lift blocking moves also depend on `bridge/ros_conversions.py` indexing `StepStateAll.steppers` by `stepper_number − 1`, not firmware payload order** — the firmware leads its payload with stepper 4, and if the array is filled in payload order the lift's `_wait_stepper_idle` reads the wrong (always-idle) stepper and every move times out / returns False even though the lift physically moved. This bridge fix is upstream-owned and regresses on `git merge upstream/main` — re-check it after any merge.

### Required startup sequence
```python
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)           # Unit.MM
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,       # 80.0 mm
        wheel_base=WHEEL_BASE,               # 290.0 mm
        initial_theta_deg=INITIAL_THETA_DEG, # 90.0°
        left_motor_id=LEFT_WHEEL_MOTOR,      # Motor.DC_M1
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,    # Motor.DC_M2
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

def start_robot(robot: Robot) -> None:
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)
```

### FSM loop skeleton
```python
def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            state = "IDLE"
        elif state == "IDLE":
            ...
        # Tick-rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
```

### Opt-in sensors (call in `configure_robot`)
```python
robot.enable_lidar()    # subscribe /scan → get_obstacles()
robot.enable_gps()      # subscribe /tag_detections → get_fused_pose()
robot.enable_imu()      # subscribe /sensor_imu → get_imu()
robot.enable_vision()   # subscribe /vision/detections → get_detections()
```

### MotionHandle pattern
```python
handle = robot.move_forward(300.0, velocity=100.0, tolerance=20.0, blocking=False)
# Poll from FSM:
if handle.is_finished():
    robot.stop()
# Or cancel:
handle.cancel(); handle.wait(timeout=1.0)
```

Only one high-level motion runs at a time — starting a second raises `RuntimeError`.

## Key Design Rules

- **Firmware state**: Motion commands silently drop unless firmware is in `RUNNING`. Always call `set_state(FirmwareState.RUNNING)` before moving.
- **Motor IDs are 1-based** (1–4). Servo channels are 1-based (1–16). Button/limit IDs are 1-based. NeoPixel indices are 0-based.
- **Unit system**: `set_unit()` affects all subsequent length/velocity calls. Methods ending in `_mm` always use raw millimeters. Angles are always degrees in the public API except `max_angular_rad_s` (always rad/s).
- **Angles**: CCW positive, 0° = robot forward (+x). Body frame: +x = forward, +y = left.
- **`hardware_map.py`** is the single source of truth for physical robot constants — import from there, not hardcoded values.
- **Protocol updates**: Change `docs/COMMUNICATION_PROTOCOL.md` first → update `tlv_protocol/TLV_TypeDefs.json` and `TLV_Payloads.md` → regenerate types → update firmware and bridge.
- **Generated files**: `/runtime_output` inside Docker containers is for artifacts (plots, images, logs). Do not write generated files into `ros2_ws/src/`.

## ROS Packages

| Package | Role |
|---------|------|
| `bridge` | UART TLV ↔ ROS topics bridge node; runs as the Docker entrypoint |
| `bridge_interfaces` | All custom msg/srv types shared across packages |
| `robot` | Robot API (`robot.py`) + student FSM (`main.py`) + planners |
| `sensors` | GPS node, mock lidar node |
| `vision` | YOLO-based vision node (NCNN on RPi, `/vision/detections`) |
| `rplidar_ros` | C1 lidar driver (publishes `/scan`) |
| `global_gps` | Jetson-side ArUco tag GPS, pushes to RPis over TCP 7777 |
