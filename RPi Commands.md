# CAPSTONE RPi Commands

A practical playbook for working with the RPi. Use the **Quick Start** section as your default checklist after the Pi boots; drop into the **Reference** section when something is broken or you need to rebuild.

Every command below assumes you're in the repo root on the RPi. Set this once per terminal so the rest of the commands are short:

```bash
export COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
```

---

## Quick Start — every time you power on the RPi

Run these in order. Most days, that's all you need.

### 1. Pull the latest code from our fork
```bash
git pull
```
Grabs new commits from our own GitHub remote (not from `upstream/main`).

### 2. Start the docker container
```bash
docker compose -f $COMPOSE up -d
```
- `-d` runs in the background.
- Add `--wait` (i.e. `up -d --wait`) if you want the command to block until the healthcheck passes — useful in scripts. First-ever start takes ~60s because colcon has to build; subsequent starts are ~5s.
- The container auto-runs `ros2 launch bridge bridge.launch.py` as its main process, so the **bridge is already running** after this command finishes. You do NOT need to start the bridge manually.

If you also want to watch the bridge logs scroll by:
```bash
docker compose -f $COMPOSE logs -f ros2_runtime
```

### 3. Confirm the container is healthy
```bash
docker compose -f $COMPOSE ps
```
Look for `Up X seconds (healthy)`.

### 4. Enter the container to run your code
```bash
./ros2_ws/docker/enter_ros2.sh
```
This drops you into a bash shell inside the container with ROS2 already sourced. Equivalent long form:
```bash
docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

### 5. From inside the container, launch what you need

Run the robot + lidar + robot_gps all together (recommended):
```bash
ros2 launch robot everything.launch.py
```

Or run pieces individually:
```bash
ros2 run robot robot                            # just the robot FSM
ros2 launch vision vision_production.launch.py  # vision node
ros2 run sensors robot_gps                      # GPS receiver only
```

---

## When code changes — picking the right command

| What changed | What to run | Why |
|---|---|---|
| Python files in `ros2_ws/src/` (robot, bridge, sensors, vision) | `./ros2_ws/docker/restart.sh` | The container re-runs colcon build on startup, so a restart picks up Python edits. No image rebuild needed. |
| `.msg`/`.srv` files in `bridge_interfaces` | `./ros2_ws/docker/restart.sh` | Same as above — colcon regenerates message types on start. |
| Launch files (e.g. you added `everything.launch.py`) | `./ros2_ws/docker/restart.sh` | Launch files are installed by colcon — restart triggers a rebuild. |
| `Dockerfile`, `requirements.txt`, apt deps | `docker compose -f $COMPOSE build` then `up -d --wait` | The image itself has to change. |
| Frontend code (`nuevo_ui/frontend/`) | `./nuevo_ui/scripts/build.sh` | Rebuilds the React bundle the bridge serves. |

---

## When something is broken — full rebuild

If `restart.sh` doesn't fix it, escalate:

```bash
# Stop and remove the container (data volumes survive)
docker compose -f $COMPOSE down

# Rebuild the image from scratch
docker compose -f $COMPOSE build

# Bring it back up
docker compose -f $COMPOSE up -d --wait
```

Nuclear option (wipes the colcon build cache too — first start after this takes ~60s):
```bash
docker compose -f $COMPOSE down -v
docker compose -f $COMPOSE up -d --build --wait
```

To verify the workspace is healthy after a rebuild:
```bash
./ros2_ws/docker/check_rpi_runtime.sh
```

---

## Reference — what each command actually does

### Docker basics
| Command | What it does |
|---|---|
| `docker compose -f $COMPOSE up -d` | Start the container in the background. |
| `docker compose -f $COMPOSE up -d --wait` | Same, but block until healthcheck passes. |
| `docker compose -f $COMPOSE down` | Stop and remove the container (volumes persist). |
| `docker compose -f $COMPOSE build` | Rebuild the Docker image. |
| `docker compose -f $COMPOSE ps` | Show container status. |
| `docker compose -f $COMPOSE logs -f ros2_runtime` | Tail the container logs (Ctrl-C to detach — does NOT stop the container). |
| `docker compose -f $COMPOSE restart` | Restart the container without rebuilding the image (re-runs colcon build inside). |

### Helper scripts
| Script | What it does |
|---|---|
| `./ros2_ws/docker/enter_ros2.sh` | Opens a ROS2-sourced bash shell inside the container. Starts the container first if it isn't running. Pass `--build` to rebuild the image before entering. |
| `./ros2_ws/docker/restart.sh` | Restarts the container (re-runs colcon build, picks up Python/launch edits). No image rebuild. |
| `./ros2_ws/docker/check_rpi_runtime.sh` | 10 healthchecks — verifies all packages built and the bridge HTTP endpoint responds. |

### One-time setup (only after a fresh RPi flash)
```bash
sudo ./ros2_ws/setup_rpi.sh                              # enables UART, installs Docker, hardens SysRq
./nuevo_ui/scripts/build.sh                              # builds the frontend bundle
./ros2_ws/src/rplidar_ros/scripts/install_udev_rule.sh   # creates /dev/rplidar symlink
./ros2_ws/host_camera/install.sh                         # configures the host camera
```
**Don't run these every boot** — they're for the very first setup or after wiping the SD card. Reboots do NOT undo them.

---

## Common pitfalls

- **Forgetting to source ROS2 inside the container.** `enter_ros2.sh` does it for you. If you used plain `docker compose exec`, run `source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash` first.
- **Trying to run the bridge manually.** The container's main process is already `ros2 launch bridge bridge.launch.py`. Running it again will collide.
- **`ros2 launch` can't find a new launch file.** Restart the container so colcon installs it: `./ros2_ws/docker/restart.sh`.
- **`/tag_detections` has zero publishers.** The `robot_gps` node isn't running. Start it with `ros2 run sensors robot_gps`, or just use `ros2 launch robot everything.launch.py`.
- **`vision_node` stuck on `Waiting for camera device /dev/video10`.** The `pi-camera-feed` systemd service can't find the `v4l2loopback` kernel module — usually because the kernel was upgraded and DKMS didn't rebuild it for the new version. See *Camera feed / `/dev/video10` missing* below.

---

## Camera feed / `/dev/video10` missing

The Pi camera reaches ROS via `pi-camera-feed.service` → `rpicam-vid` → `ffmpeg` → `v4l2loopback` (`/dev/video10`). If `/dev/video10` doesn't exist or the service is stuck restarting:

```bash
systemctl status pi-camera-feed.service --no-pager
journalctl -u pi-camera-feed.service --no-pager -n 20
```

If the journal shows `modprobe: FATAL: Module v4l2loopback not found in directory /lib/modules/<kernel>`, DKMS lost the module on a kernel upgrade. Install the matching headers and rebuild:

```bash
sudo apt-get install -y linux-headers-$(uname -r) \
  && sudo dkms autoinstall \
  && sudo systemctl restart pi-camera-feed.service \
  && sleep 3 && ls -la /dev/video10
```

`/dev/video10` should appear as `crw-rw---- 1 root video …`.

---

## Inspecting runtime output from the host

Per-package state (lidar plots, vision overlays, logs) is written to `/runtime_output/<pkg>/…` *inside* the container. To view from the host without entering the container:

```bash
# Latest lidar body-frame plot
docker cp docker-ros2_runtime-1:/runtime_output/lidar/latest.png /tmp/lidar_latest.png
xdg-open /tmp/lidar_latest.png      # or any image viewer

# Latest vision debug frame
docker cp docker-ros2_runtime-1:/runtime_output/vision/latest.jpg /tmp/vision_latest.jpg
```

Auto-refreshing viewer (handy while tuning):
```bash
feh --reload 1 /tmp/lidar_latest.png   # re-copy in a loop or use --watch
```

---

## Quick ROS2 diagnostics (from inside the container)

| Question | Command |
|---|---|
| What nodes are alive? | `ros2 node list` |
| What's on a topic? | `ros2 topic echo /vision/detections` (Ctrl-C to stop) |
| Is `/scan` actually publishing? | `ros2 topic hz /scan --window 10` |
| What params does a node expose? | `ros2 param list /vision_node` |
| Tune a param live without restart | `ros2 param set /vision_node gender_female_threshold 0.4` |
| Trigger a one-off TF lookup | `ros2 run tf2_ros tf2_echo base_link laser_frame` |

---