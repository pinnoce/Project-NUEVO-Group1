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

The robot node ships with a five-state FSM that demonstrates the full sensing → planning → control stack:

1. **IDLE** — orange LED; press `BTN_1` to start (`BTN_2` cancels at any stage).
2. **LOOK_LEFT** — turns `LOOK_LEFT_OFFSET_DEG` (default 40°) CCW from the initial heading so the camera faces the traffic light.
3. **WATCHING** — blue LED; polls `/vision/detections` for a `traffic light` detection whose `color = green` and `confidence ≥ MIN_TRAFFIC_LIGHT_CONFIDENCE` (default 0.50). Detections older than `VISION_STALE_SEC` (3 s) are ignored.
4. **TURN_BACK** — green LED; rotates back to `INITIAL_THETA_DEG` (90°).
5. **MOVING** — follows `PATH_CONTROL_POINTS` with pure pursuit (`purepursuit_follow_path`).

Tune via the constants block near the top of `main.py`. Hardware constants (wheel diameter, wheel base, motor IDs) come from `ros2_ws/src/robot/robot/hardware_map.py`.

## Technologies

- **Embedded**: Arduino (C/C++)
- **High-Level**: ROS2 (Python/C++), Raspberry Pi 5
- **Communication**: UART serial protocol
- **Sensors**: Camera, GPS, encoders
- **Hardware**: Custom PCB, stepper/servo motors
