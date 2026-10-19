#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if command -v python3.12 >/dev/null; then PY=python3.12; else PY=python3; fi
init_env() {
    mkdir -p logs
    if [ ! -f .env ]; then
        "$PY" - <<'PY'
import os, secrets
with open('.env', 'x') as f:
    f.write('OLLAMA_URL=http://host.docker.internal:11435\nOLLAMA_MODEL=qwen2.5vl:3b\nOLLAMA_TOKEN=' + secrets.token_urlsafe(32) + '\n')
os.chmod('.env', 0o600)
print('Created .env with a private local-proxy token. Do not upload this file.')
PY
    fi
}
command_name="${1:-help}"
if [ "$#" -gt 0 ]; then shift; fi
case "$command_name" in
    help)
        echo 'Usage: ./run.sh init|build|up|proxy|doctor|sim|mission|check|camera|halt|shell|test|record|down|rebuild'
        echo 'Example: ./run.sh mission "Find the red box"'
        echo 'Example: ./run.sh sim controller:=false (for ./run.sh check --move)'
        exit 0 ;;
    init) init_env; exit 0 ;;
esac
init_env
set -a; source .env; set +a
if [ "$command_name" = proxy ]; then exec "$PY" tools/ollama_proxy.py; fi
compose=(docker compose -f compose.yaml)
if [ "$(uname -s)" = Linux ]; then compose+=(-f compose.linux.yaml); fi
if [ "${PAI_PLATFORM:-}" = linux/amd64 ]; then compose+=(-f compose.amd64.yaml); fi
ros_exec() { "${compose[@]}" exec desktop bash -c 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec "$@"' _ "$@"; }
case "$command_name" in
    build) "${compose[@]}" build ;;
    up) "${compose[@]}" up -d --no-build --pull never ;;
    doctor) ros_exec python3 /workspace/tools/doctor.py ;;
    sim) ros_exec ros2 launch pai_robot demo.launch.py "$@" ;;
    mission) ros_exec ros2 run pai_robot mission "$@" ;;
    check) ros_exec ros2 run pai_robot smoke_check "$@" ;;
    camera) ros_exec ros2 run rqt_image_view rqt_image_view /camera/image_raw ;;
    halt) ros_exec ros2 service call /halt std_srvs/srv/Trigger '{}' ;;
    shell) "${compose[@]}" exec desktop bash ;;
    test) ros_exec python3 -m unittest discover -s /workspace/src/pai_robot/test -v ;;
    rebuild) "${compose[@]}" exec desktop bash -c 'source /opt/ros/humble/setup.bash && cd /workspace && colcon build --symlink-install --packages-select pai_robot' ;;
    record) ros_exec ros2 bag record -o "/workspace/logs/bag_$(date +%Y%m%d_%H%M%S)" /camera/image_raw /scan /odom /mission /cmd_vel_raw /cmd_vel /robot_state /clock ;;
    down) "${compose[@]}" down ;;
    *) echo "Unknown subcommand: $command_name" >&2; exit 2 ;;
esac
