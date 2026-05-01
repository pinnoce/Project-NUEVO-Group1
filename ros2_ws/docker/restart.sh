#!/usr/bin/env bash
# Restart the ROS2 container without rebuilding the image.
# Run from anywhere — detects Pi vs VM by checking for the rpi compose file arg or hostname.
set -e

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

if [ "${1}" = "vm" ]; then
    COMPOSE_FILE="docker-compose.vm.yml"
elif [ "${1}" = "rpi" ] || [ "${1}" = "" ]; then
    # Default to rpi when on the Pi; fall back to vm if compose file not relevant
    COMPOSE_FILE="docker-compose.rpi.yml"
else
    echo "Usage: restart.sh [rpi|vm]"
    exit 1
fi

echo "[restart] Using $COMPOSE_FILE"
docker compose -f "$REPO_ROOT/ros2_ws/docker/$COMPOSE_FILE" restart
echo "[restart] Done"
