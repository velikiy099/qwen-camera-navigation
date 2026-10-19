#!/usr/bin/env python3
"""Read-only diagnostics in the running container. Does not display proxy tokens."""
import json
import os
import platform
import subprocess
import sys
from urllib.request import Request, build_opener, ProxyHandler

def main():
    failed = False
    print('Architecture:', platform.machine(), 'Python:', platform.python_version())
    print('ROS_DISTRO:', os.environ.get('ROS_DISTRO'))
    for cmd in [['ros2', 'pkg', 'prefix', 'pai_robot'], ['ros2', 'pkg', 'prefix', 'ros_gz_bridge'],
                ['ign', 'gazebo', '--versions'], ['glxinfo', '-B']]:
        print('\n$', ' '.join(cmd))
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
            print((result.stdout + result.stderr)[:5000])
            if result.returncode: failed = True
        except Exception as exc:
            print('FAIL:', exc); failed = True
    url = os.environ.get('OLLAMA_URL', 'http://host.docker.internal:11435').rstrip('/')
    token = os.environ.get('OLLAMA_TOKEN', '')
    print('\nOllama connection:', url, '(token not displayed)')
    try:
        req = Request(url + '/api/tags', headers={'Authorization': 'Bearer ' + token} if token else {})
        with build_opener(ProxyHandler({})).open(req, timeout=10) as response:
            data = json.load(response)
        names = [m.get('name') for m in data.get('models', [])]
        required = os.environ.get('OLLAMA_MODEL', 'qwen2.5vl:3b')
        print('Local models:', names)
        if required not in names:
            raise RuntimeError(f'{required} not installed in host Ollama')
    except Exception as exc:
        print('FAIL:', exc, '\nStart Ollama and ./run.sh proxy on the host.'); failed = True
    print('\nDiagnostics:', 'FAIL (review output above)' if failed else 'PASS (runtime mission still untested)')
    return int(failed)
if __name__ == '__main__': sys.exit(main())
