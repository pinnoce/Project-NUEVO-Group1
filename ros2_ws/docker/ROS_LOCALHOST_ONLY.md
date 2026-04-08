# ROS2 Cross-Team Node Bleed — Fix Guide

## The Problem

ROS2 uses DDS (Data Distribution Service) for node discovery via UDP multicast.
By default all ROS2 nodes on the same network join domain 0, which means every
RPi in the lab can see every other team's nodes.

**Symptoms:**
- `ros2 node list` shows 3–10 bridge or robot nodes when only one should exist
- The NUEVO UI shows "view only" or drops commands intermittently
- The robot moves on its own, responding to another team's commands
- UI telemetry is noisy or jumps unexpectedly

**Root cause:** Without isolation, a command published by Team A's UI reaches
Team B's bridge node (and robot) because they share the same DDS multicast domain.

## The Fix

Setting `ROS_LOCALHOST_ONLY=1` tells DDS to communicate only over the loopback
interface (`127.0.0.1`). Nodes on other machines become completely invisible,
regardless of what domain ID they use.

This is set in `docker-compose.rpi.yml` under `environment` so it applies
automatically to every process in the container:

```yaml
environment:
  - ROS_LOCALHOST_ONLY=1
```

## Applying the Fix

Pull the update from upstream, merge, and restart the container:

```bash
# 1. Pull the fix from upstream
# If you have not added the upstream remote yet:
#   git remote add upstream https://github.com/pochihh/Project-NUEVO.git
git fetch upstream
git merge upstream/main

# 2. Restart the container (no rebuild needed — compose change only)
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE down
docker compose -f $COMPOSE up -d

# 3. Watch the logs until the build completes
docker compose -f $COMPOSE logs -f ros2_runtime
# Press Ctrl+C when ready
```

## Verifying the Fix

From inside the container, confirm only local nodes are visible:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash -c \
  "source /ros2_ws/install/setup.bash && ros2 node list"
```

You should only see nodes running on this RPi. Nodes from other teams' RPis
should no longer appear even if they are on the same network and running.

