# Project NUEVO
![](/assets/NUEVO.png)

Lab project material for the MAE 162 robotics course (Winter/Spring 2026).

## Overview

A modular two-wheeled mobile robot platform designed for hands-on robotics education. Features customizable manipulators and a dual-layer control architecture for teaching embedded systems, ROS2, and mechatronics fundamentals.

## System Architecture

**Low-Level Control (Arduino)**
- Real-time motor control (DC, stepper, servo)
- GPIO, LEDs, and button inputs
- UART communication to Raspberry Pi

**High-Level Control (Raspberry Pi 5 + ROS2)**
- Decision-making and path planning
- Camera and GPS sensor processing
- ROS2 node orchestration

**Custom PCB**
- Integrates Arduino, motor drivers, and power management
- Standardized interface for educational reproducibility

## Repository Structure

```
├── firmware/       Arduino firmware and firmware-specific docs
├── nuevo_ui/       Raspberry Pi bridge + web UI
├── ros2_ws/        ROS2 workspace and Pi-side tests
├── tlv_protocol/   TLV type definitions, payload schemas, generators
├── NUEVO board/    PCB design files (schematics, layouts, BOM)
├── mechanical/     CAD files for chassis and manipulators
├── docs/           Cross-project architecture, protocol, and design docs
└── assets/         Shared repo assets
```



## Key Documents

| Document | Purpose |
|----------|---------|
| [docs/README.md](docs/README.md) | Cross-project documentation map and source-of-truth index |
| [docs/COMMUNICATION_PROTOCOL.md](docs/COMMUNICATION_PROTOCOL.md) | Current human-readable source of truth for protocol behavior, framing, and logical TLV design |
| [docs/DESIGN_GUIDELINES.md](docs/DESIGN_GUIDELINES.md) | Cross-project conventions, numbering rules, and protocol update workflow |
| [tlv_protocol/TLV_Payloads.md](tlv_protocol/TLV_Payloads.md) | Exact payload layouts and sizes |
| [firmware/README.md](firmware/README.md) | Arduino firmware overview, current features, and build instructions |
| [firmware/docs/README.md](firmware/docs/README.md) | Firmware subsystem documentation index |
| [NUEVO board/SPECIFICATIONS.md](NUEVO%20board/SPECIFICATIONS.md) | PCB hardware specifications |

## Current Mission Behavior (`ros2_ws/src/robot/robot/main.py`)

`main.py` runs the full competition mission FSM (started on `BTN_1`, `BTN_2` cancels to `IDLE` at any stage):

1. **IDLE** — orange LED; primes the lift to carry height and opens the gripper.
2. **Traffic light** — turn to face the light, wait for green, turn back.
3. **MOV1** — drive to the patty shelf.
4. **Burger assembly** — 3 shelf stops in order left bun → patty → right bun, each with turn-to-face, forward-wall square, standoff approach, lift manipulation (build bottom-up bun→patty→bun), retreat, turn-back, and parallel re-square.
5. **MOV2** — scripted corridor (relative turns + lidar parallel-wall aligns + wall-standoff approaches), ending with a final left turn and an **odometry reset**.
6. **LAPF** — `robot.lapf_to_goal(LAPF_GOAL)` through the cone field; the goal is robot-relative (straight ahead) thanks to the pre-LAPF reset.
7. **MOV3 → Gender ID → MOV4** — square/approach the wall, read the person's gender, drive a gender-dependent distance to the drop-off.
8. **Drop-off** — place the burger, re-square, then the stop-sign leg.
9. **Stop sign** — drive until the sign is detected, creep a fixed distance, stop and wait, then a final drive to **DONE**.

The full state-by-state breakdown lives in [CLAUDE.md](CLAUDE.md). Tune via the constants block near the top of `main.py`. Hardware constants (wheel diameter, wheel base, motor IDs) come from `ros2_ws/src/robot/robot/hardware_map.py`.

## Technologies

- **Embedded**: Arduino (C/C++)
- **High-Level**: ROS2 (Python/C++), Raspberry Pi 5
- **Communication**: UART serial protocol
- **Sensors**: Camera, GPS, encoders
- **Hardware**: Custom PCB, stepper/servo motors
