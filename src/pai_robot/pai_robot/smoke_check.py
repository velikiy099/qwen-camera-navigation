"""Check RGB, wheel odometry, clock and optionally a stationary VLM decision."""
from dataclasses import asdict
import argparse
import json
import os
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from .agent import CameraAgent
from .images import CHANNELS, png_from_image
from .ollama_client import OllamaClient
from .motion import OdometrySample


class Probe(Node):
    def __init__(self):
        super().__init__('pai_smoke_check')
        self.counts = dict(image=0, clock=0, odometry=0)
        self.odom_stamp = -1.0
        self.image = None
        self.image_stamp = self.sim = -1.0
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg):
        sample = OdometrySample.from_message(msg)
        if sample.stamp > self.odom_stamp:
            self.counts['odometry'] += 1
            self.odom_stamp = sample.stamp

    def on_image(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp > self.image_stamp:
            self.image, self.image_stamp = msg, stamp
            self.counts['image'] += 1

    def on_clock(self, msg):
        stamp = msg.clock.sec + msg.clock.nanosec / 1e9
        if stamp > self.sim:
            self.counts['clock'] += 1
        self.sim = stamp


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=45)
    parser.add_argument('--vision', action='store_true', help='Ask Qwen using a real sensor image; never execute the result')
    parser.add_argument('--instruction', default='Find the red box and approach it without touching it.')
    args = parser.parse_args(argv)
    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic() + args.timeout
        while min(node.counts.values()) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print('Messages received:', node.counts)
        if min(node.counts.values()) < 3:
            raise RuntimeError('Missing fresh RGB, wheel odometry or advancing clock; check server and bridge')
        msg = node.image
        png = png_from_image(bytes(msg.data), msg.width, msg.height, msg.step, msg.encoding)
        channels = CHANNELS[msg.encoding]
        values = [msg.data[y * msg.step + x * channels + c]
                  for y in range(msg.height) for x in range(msg.width) for c in range(3)]
        if max(values) - min(values) < 10:
            raise RuntimeError('Camera is almost uniform; inspect rendering')
        output = Path(os.environ.get('PAI_LOG_DIR', '/workspace/logs'))
        output.mkdir(parents=True, exist_ok=True)
        (output / 'smoke_camera.png').write_bytes(png)
        print(f'PASS: actual RGB camera {msg.width}x{msg.height} {msg.encoding}, wheel odometry and simulation clock')
        if args.vision:
            agent = CameraAgent(args.instruction, node.sim)
            client = OllamaClient()
            goal, _ = client.decide(png, agent.context())
            agent.lock_goal(goal)
            agent.observe_frame(png)
            action, raw = client.decide(png, agent.context())
            (output / 'smoke_decision.json').write_text(json.dumps(
                {'goal': asdict(goal), 'tool_call': asdict(action), 'raw': raw, 'executed': False}, ensure_ascii=False, indent=2))
            print('PASS: schema-valid VLM decision:', action.tool, '(not executed)')
        print('This check does not measure navigation, obstacle avoidance or arrival success.')
        return 0
    except Exception as exc:
        print('FAIL:', exc, file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
