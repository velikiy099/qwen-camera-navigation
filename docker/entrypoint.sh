#!/usr/bin/env bash
set -e
mkdir -p "$XDG_RUNTIME_DIR" /workspace/logs
chmod 700 "$XDG_RUNTIME_DIR"
pids=()
cleanup() { for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT
trap 'exit 0' TERM INT
Xvfb "$DISPLAY" -screen 0 1280x800x24 +extension GLX +render -noreset -nolisten tcp > /workspace/logs/xvfb.log 2>&1 &
pids+=("$!")
ready=false
for _ in $(seq 1 60); do
    if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then ready=true; break; fi
    sleep 0.25
done
if [ "$ready" != true ]; then echo 'Xvfb failed. Read logs/xvfb.log'; exit 1; fi
openbox > /workspace/logs/openbox.log 2>&1 & pids+=("$!")
x11vnc -display "$DISPLAY" -listen 127.0.0.1 -rfbport 5900 -forever -shared -nopw -noxdamage > /workspace/logs/vnc.log 2>&1 & pids+=("$!")
websockify --web=/usr/share/novnc 0.0.0.0:80 127.0.0.1:5900 > /workspace/logs/novnc.log 2>&1 & pids+=("$!")
xterm -geometry 100x24+0+0 -T 'PAI ROS 2 terminal' -e bash > /workspace/logs/xterm.log 2>&1 &
echo 'Desktop ready. Open http://localhost:6080/vnc.html and run ./run.sh sim on host.'
wait -n "${pids[@]}"
