"""On-machine integration checks. Never substitutes synthetic images for sensors."""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan
from .images import to_bgra, png_from_bgra
from .ranges import Ranges, summarize_scan
from .ollama_client import OllamaClient

class Probe(Node):
    def __init__(self):
        super().__init__('pai_smoke_check')
        self.counts = dict(image=0, scan=0, clock=0, odom=0)
        self.image = None; self.ranges = Ranges.invalid(); self.sim = 0.0; self.pose = None
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
    def on_image(self, m):
        self.image = m; self.counts['image'] += 1
    def on_scan(self, m):
        self.ranges = summarize_scan(list(m.ranges), m.angle_min, m.angle_increment, m.range_min, m.range_max)
        self.counts['scan'] += 1
    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec / 1e9
        if t > self.sim: self.counts['clock'] += 1
        self.sim = t
    def on_odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.counts['odom'] += 1
    def cmd(self, v):
        m = Twist(); m.linear.x = float(v); self.pub.publish(m)

def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--timeout', type=float, default=45)
    p.add_argument('--vision', action='store_true', help='Send the received camera image to the real local VLM')
    p.add_argument('--move', action='store_true', help='Short physical motion check; requires controller:=false')
    a = p.parse_args(argv)
    rclpy.init(); node = Probe(); moved = False
    try:
        deadline = time.monotonic() + a.timeout
        while min(node.counts.values()) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print('Messages received:', node.counts)
        if min(node.counts.values()) < 3:
            raise RuntimeError('Missing image/scan/clock/odom. Check server, bridge and renderer first.')
        if not node.ranges.valid:
            raise RuntimeError('Lidar does not contain a valid 360-degree scan')
        m = node.image
        bgra = to_bgra(bytes(m.data), m.width, m.height, m.step, m.encoding)
        rgb_values = [bgra[i] for i in range(len(bgra)) if i % 4 != 3]
        if max(rgb_values) - min(rgb_values) < 10:
            raise RuntimeError('Camera is almost uniform; inspect renderer and saved image')
        output = Path(os.environ.get('PAI_LOG_DIR', '/workspace/logs'))
        output.mkdir(parents=True, exist_ok=True)
        png = png_from_bgra(bgra, m.width, m.height)
        (output/'smoke_camera.png').write_bytes(png)
        print(f'PASS: real camera {m.width}x{m.height} {m.encoding}, lidar, clock and odometry')
        if a.vision:
            print('Checking actual sensor PNG with local VLM...')
            colors, raw = OllamaClient().recognize(png)
            (output/'smoke_vision.json').write_text(json.dumps({'colors': sorted(colors), 'raw': raw}, indent=2))
            print('VLM replied:', sorted(colors), '(inspect image to judge correctness)')
        if a.move:
            if 'visual_robot' in node.get_node_names():
                raise RuntimeError('Motion test needs controller:=false; do not compete with the controller')
            if node.ranges.front < 0.8:
                raise RuntimeError('Not enough free space for motion test; restore default world')
            moved = True
            start_pose = node.pose; start_sim = node.sim
            end = time.monotonic() + 25
            while node.sim - start_sim < 1.5 and time.monotonic() < end:
                node.cmd(0.05)
                rclpy.spin_once(node, timeout_sec=0.05)
                time.sleep(0.03)
            for _ in range(10):
                node.cmd(0.0); rclpy.spin_once(node, timeout_sec=0.05)
            distance = math.hypot(node.pose[0] - start_pose[0], node.pose[1] - start_pose[1])
            if not 0.025 < distance < 0.25:
                raise RuntimeError(f'Motion test displacement unexpected: {distance:.4f} m')
            print(f'PASS: ROS Twist -> bridge -> Gazebo joints -> odometry, moved {distance:.4f} m')
        print('Smoke check is not a successful AI search-task result. Run and record missions separately.')
        return 0
    except Exception as exc:
        print('FAIL:', exc, file=sys.stderr)
        return 1
    finally:
        if moved and rclpy.ok():
            for _ in range(3): node.cmd(0.0); time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
