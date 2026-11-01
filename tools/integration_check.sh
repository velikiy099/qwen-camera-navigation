#!/usr/bin/env bash
# Run inside the desktop container; isolate DDS and Gazebo from a normal demo.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export ROS_DOMAIN_ID=73
export IGN_PARTITION="pai_integration_$$"
mkdir -p /workspace/logs
ros2 launch pai_robot demo.launch.py gui:=false controller:=false > /workspace/logs/integration.log 2>&1 &
pid=$!
trap 'kill -TERM "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true' EXIT
ros2 run pai_robot smoke_check --timeout 60 "$@"
