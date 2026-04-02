# ROS2 Workspace

This folder contains the ROS2 side of Project NUEVO.

The current ROS2 design keeps the workspace small and beginner-friendly:

- `bridge_interfaces` defines the raw firmware and bridge-facing ROS interfaces
- `bridge` wraps the shared `nuevo_bridge` runtime and exposes ROS topics and services
- `robot` is the main robot application package
- `sensors` is for Raspberry Pi connected sensors outside the Arduino firmware
- `vision` is for camera and perception code

If you are new to ROS2, read these files in order:

1. [ROS2_NODES_DESIGN.md](ROS2_NODES_DESIGN.md)
2. [BRIDGE_RUNTIME.md](BRIDGE_RUNTIME.md)
3. this README


## Package Layout

```text
ros2_ws/
├── src/
│   ├── bridge_interfaces/   # raw bridge-facing ROS interfaces that mirror the current protocol
│   ├── bridge/              # ROS wrapper around the shared nuevo_bridge runtime
│   ├── robot/               # main robot logic, bringup, config, URDF, RViz
│   ├── sensors/             # Pi-side sensor nodes outside the Arduino firmware
│   └── vision/              # camera and perception nodes
├── docker/                  # Docker image, compose files, and entrypoint scripts
├── ROS2_NODES_DESIGN.md
├── BRIDGE_RUNTIME.md
└── RPI_SETUP.md             # legacy note; the setup steps are also included below
```


## Shared Runtime Model

The ROS bridge does not duplicate the web bridge.

- The shared Python runtime lives in [nuevo_ui/backend/nuevo_bridge](../nuevo_ui/backend/nuevo_bridge)
- The ROS package lives in [src/bridge](src/bridge)
- In ROS mode, one integrated `bridge` process owns:
  - one serial connection to the Arduino
  - one decode path
  - one FastAPI/WebSocket UI server
  - one ROS node

That means the UI and ROS always see the same decoded firmware state and use the
same outbound command path.


## Raw ROS API

The raw ROS topics use the same lowercase snake_case names as the bridge and
firmware protocol. Examples:

- `/sys_state`
- `/sys_power`
- `/dc_state_all`
- `/step_state_all`
- `/sensor_imu`
- `/sensor_kinematics`
- `/dc_set_velocity`
- `/dc_home`
- `/sensor_mag_cal_cmd`

This workspace intentionally treats those raw topics as the primary internal ROS
API. Standard ROS topics such as `/odom` or `/imu/data` can be added later as
adapters if needed.

See [docs/COMMUNICATION_PROTOCOL.md](../docs/COMMUNICATION_PROTOCOL.md) for the
protocol naming and semantics that these topics mirror.


## Command API Rule

Use the ROS API based on how the command behaves:

- use services for discrete actions where the caller expects success, rejection, or timeout
- use topics for continuous setpoints or frequently updated control inputs

Current example:

- `/set_firmware_state` uses `bridge_interfaces/srv/SetFirmwareState`
- `/dc_set_velocity` uses `bridge_interfaces/msg/DCSetVelocity`
- `/dc_set_pwm` uses `bridge_interfaces/msg/DCSetPwm`

The raw `/sys_cmd` topic still exists for low-level debugging, but normal ROS
callers should prefer `/set_firmware_state`.


## Docker Workflow

Docker is the default workflow for ROS2 because ROS2 Jazzy officially targets
Ubuntu 24.04, while the project may be developed on Raspberry Pi OS / Ubuntu,
macOS, or Windows.

Two compose files are provided:

| Platform | Compose file | Mode |
|---|---|---|
| Raspberry Pi with Arduino | `ros2_ws/docker/docker-compose.rpi.yml` | real hardware |
| macOS / Windows | `ros2_ws/docker/docker-compose.vm.yml` | mock bridge |

The current Docker image is a control-first ROS stack:

- it includes the shared bridge runtime and core ROS packages
- it does not yet include camera, OpenCV, libcamera, or YOLO dependencies
- those heavier vision dependencies will be added later when the `vision` package is activated

Important: `docker compose up` now builds the ROS workspace and then leaves the
`ros2_runtime` container idle. It does not automatically start any ROS node.
This is intentional so you can start and verify each node one at a time.


## Raspberry Pi UART Setup

If you are using a real Raspberry Pi 5 with the Arduino bridge, enable UART
first.

Edit `config.txt`:

```bash
sudo nano /boot/firmware/config.txt
```

Make sure it contains:

```text
enable_uart=1
dtoverlay=uart0-pi5
```

Then disable the serial console in `cmdline.txt`:

```bash
sudo nano /boot/firmware/cmdline.txt
```

Remove anything like:

```text
console=serial0,115200
console=ttyAMA0,115200
```

Reboot and verify:

```bash
ls -l /dev/ttyAMA0
```

You should see a device node similar to:

```text
crw-rw---- 1 root dialout ... /dev/ttyAMA0
```

The current bridge runtime uses:

```text
/dev/ttyAMA0 @ 200000 baud
```


## Build the Frontend First

The bridge serves the built web UI. Build the frontend once before starting the
ROS container:

```bash
cd nuevo_ui/frontend
npm install
npm run build
cd ../..
cp -r nuevo_ui/frontend/dist/. nuevo_ui/backend/static/
```


## 1. Start the Container

From the repository root, choose the compose file for your platform.

### Raspberry Pi

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE build
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f ros2_runtime
```

### macOS / Windows

```bash
COMPOSE=ros2_ws/docker/docker-compose.vm.yml
docker compose -f $COMPOSE build
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f ros2_runtime
```

Wait until the log shows that `colcon build` finished and the container is
ready. After that, leave the container running in the background.


## 2. Open a Shell Inside the Running Container

In a new terminal:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
```

At this point, no ROS nodes are running yet. The container is only prepared and
waiting.


## 3. Sanity Check the Workspace

Inside the container, verify that the packages and interfaces are available:

```bash
ros2 pkg list | grep -E 'bridge_interfaces|bridge|robot|sensors|vision'
ros2 interface show bridge_interfaces/msg/SystemPower
ros2 interface show bridge_interfaces/msg/DCStateAll
ros2 interface show bridge_interfaces/srv/SetFirmwareState
```

Expected result:

- all five package names appear
- the message and service definitions print correctly


## 4. Start the Bridge Node First

In the first container shell:

```bash
ros2 run bridge bridge
```

This starts the integrated ROS bridge, web UI server, and firmware transport.

Expected result in the terminal:

- the node starts without crashing
- in real hardware mode, the bridge connects to `/dev/ttyAMA0`
- in mock mode, the bridge reports mock operation


## 5. Verify the Bridge Node

Open a second terminal and enter the same running container:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash
```

Inside that second shell:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

Now run:

```bash
ros2 node list
ros2 topic list
ros2 service list | grep set_firmware_state
ros2 topic echo /sys_state
```

Expected result:

- `/bridge` appears in `ros2 node list`
- the raw bridge topics exist
- `/set_firmware_state` exists as a service
- `/sys_state` starts streaming

You can also verify the web bridge from the host:

```bash
curl http://localhost:8000/health
```


## 6. Test the Firmware State Service

From the second container shell, once `/sys_state` shows `IDLE`:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 1}"
```

Expected result:

- `target_state: 2` requests `RUNNING` and should succeed from `IDLE`
- `target_state: 1` requests `IDLE` and should succeed from `RUNNING`

Optional extra checks:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 4}"
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 1}"
```

- `target_state: 4` requests `ESTOP`
- `target_state: 1` should recover back to `IDLE`


## 7. Test Continuous Commands

Continuous motor control remains topic-based. First enter `RUNNING`, then send
a command:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"
ros2 topic pub --once /dc_set_velocity bridge_interfaces/msg/DCSetVelocity \
  "{motor_number: 1, target_ticks: 200}"
```

Expected result:

- the state transition happens through the service
- the DC command goes through the topic
- `/dc_state_all` reflects the new target


## 8. Start the Other Nodes One by One

The other packages are intentionally started manually so students can see what
each one does.

### Robot node

In a third shell:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run robot robot
```

Expected result:

- the node starts without crashing
- `ros2 node list` now includes `/robot`
- the example FSM in `src/robot/robot/main.py` starts and waits in the `IDLE` state

The robot node is the main student application layer. It exposes a three-layer
API over the bridge topics:

- **`Robot`** (Layer 1) — full hardware abstraction: DC motors, steppers, servos,
  LEDs, buttons, IMU, kinematics, and firmware state management
- **`RobotFSM`** (Layer 2) — base class for writing finite state machines; provides
  `add_transition()`, `trigger()`, and `spin()`
- **`PathPlanner` / `PurePursuitPlanner`** (Layer 3) — waypoint-following navigation

Students edit only `src/robot/robot/main.py` to implement their FSM.
See [ROBOT_NODE_DESIGN.md](src/robot/ROBOT_NODE_DESIGN.md) for full API reference.

### Sensors node

In another shell:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run sensors sensor_node
```

Expected result:

- the node starts
- the log prints `sensors package scaffold ready`

### Vision node

In another shell:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run vision vision_node
```

Expected result:

- the node starts
- the log prints `vision package scaffold ready`

The `robot` node is fully implemented. `sensors` and `vision` are scaffolds
pending hardware integration.


## 9. Stop and Reset

Useful commands:

```bash
# Stop the running container
docker compose -f $COMPOSE down

# Stop and also clear the ROS build/install cache
docker compose -f $COMPOSE down -v

# Rebuild the Docker image after dependency changes
docker compose -f $COMPOSE build

# View logs from the background container
docker compose -f $COMPOSE logs -f ros2_runtime
```


## Native Ubuntu 24.04 Workflow

Native ROS2 is optional. If you are on Ubuntu 24.04, you can build the
workspace without Docker after installing ROS2 Jazzy.

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The shared backend source must also be available on `PYTHONPATH` or pointed to
with `NUEVO_BRIDGE_SOURCE`:

```bash
export NUEVO_BRIDGE_SOURCE="$(pwd)/../nuevo_ui/backend"
ros2 run bridge bridge
```


## Beginner Notes

- `bridge_interfaces` is the raw firmware and bridge-facing interface package. It is not a node.
- `bridge` is the ROS package that starts the integrated UI + ROS bridge.
- `nuevo_bridge` is the shared Python runtime outside `ros2_ws`.
- `robot` is the main student application layer — it wraps the bridge API and is where FSM logic lives. `sensors` and `vision` are separate packages for future hardware expansion; they are currently scaffolds.
- `robot`, `sensors`, and `vision` are separate packages because they represent different responsibilities, not because every node needs its own package.
- discrete operations such as firmware state changes should use services
- continuous commands such as velocity or PWM should stay on topics
