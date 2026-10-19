#!/usr/bin/env bash
# Run INSIDE the desktop container. No normal demo may be running concurrently.
set -e
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch pai_robot demo.launch.py gui:=false controller:=false > /workspace/logs/integration.log 2>&1 &
pid=$!
trap 'kill -INT "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true' EXIT
sleep 5
ros2 run pai_robot smoke_check --move --timeout 60
