"""ROS sensor subscriptions, asynchronous VLM, pure brain and Twist publisher.

No Webots API, simulator entity poses, runtime code generation or ROS work from
worker threads. The robot is commanded to STOP throughout slow VLM inference.
"""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
import csv
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .images import to_bgra, png_from_bgra
from .vision_tracking import find_target_blob
from .ranges import Ranges, summarize_scan
from .behavior import VisualSearchBrain, Command
from .ollama_client import OllamaClient

class VisualRobot(Node):
    def __init__(self):
        super().__init__('visual_robot')
        self.declare_parameter('vision_mode', 'ai')
        self.declare_parameter('log_dir', os.environ.get('PAI_LOG_DIR', '/workspace/logs'))
        self.declare_parameter('ai_timeout', 120.0)
        self.vision_mode = self.get_parameter('vision_mode').value
        if self.vision_mode not in ('ai', 'rules'):
            raise ValueError('vision_mode must be ai or rules')
        self.ai_timeout = float(self.get_parameter('ai_timeout').value)
        self.client = OllamaClient(timeout=self.ai_timeout)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='vlm')
        self.pending = None
        self.pending_epoch = 0
        self.pending_started = 0.0
        self.pending_path = ''
        self.epoch = 0
        self.counter = 0
        self.brain = None
        self.image = None
        self.image_seq = 0
        self.used_seq = -1
        self.last_image = self.last_scan = 0.0
        self.ranges = Ranges.invalid()
        self.blob = None
        self.candidate = False
        self.last_sim = None
        self.last_clock_advance = time.monotonic()
        self.last_logged = ''
        self.next_log = 0.0
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.log_dir = Path(self.get_parameter('log_dir').value) / ('run_' + stamp)
        self.log_dir.mkdir(parents=True, exist_ok=False)
        self.csv_handle = (self.log_dir / 'control.csv').open('w', newline='')
        self.csv = csv.writer(self.csv_handle)
        self.csv.writerow(['wall_time', 'sim_time', 'state', 'target', 'mode', 'desired_v', 'desired_w', 'front_m', 'blob_x', 'reason'])
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.status_pub = self.create_publisher(String, '/robot_state', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/mission', self.on_mission, 10)
        self.create_service(Trigger, '/halt', self.on_halt)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=self.steady)
        self.get_logger().info(f'Ready, vision_mode={self.vision_mode}. Waiting for /mission. Logs: {self.log_dir}')
        if self.vision_mode == 'rules':
            self.get_logger().warning('RULES-ONLY DIAGNOSTIC MODE: no image AI; not evidence of AI use')

    def on_image(self, msg):
        self.image = (bytes(msg.data), msg.width, msg.height, msg.step, msg.encoding)
        self.image_seq += 1
        self.last_image = time.monotonic()

    def on_scan(self, msg):
        self.ranges = summarize_scan(list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        self.last_scan = time.monotonic()

    def on_mission(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or data.get('target_color') not in ('red', 'blue'):
                raise ValueError('target_color must be red or blue')
            self.epoch += 1
            self.brain = VisualSearchBrain(data['target_color'])
            self.blob, self.candidate, self.used_seq = None, False, -1
            self.write_event({'event': 'mission', 'epoch': self.epoch, 'mission': data, 'vision_mode': self.vision_mode})
            self.get_logger().info(f"New mission: {self.brain.target}. Previous inference cannot authorize this mission.")
            self.send(Command(state='SEARCH', reason='New mission'))
        except (ValueError, TypeError) as exc:
            self.epoch += 1
            if self.brain:
                self.brain.fail('Invalid new mission')
            self.send(Command(state='ERROR', reason=str(exc)))
            self.get_logger().error(f'Invalid mission: {exc}')

    def on_halt(self, request, response):
        self.epoch += 1  # invalidate even an already-running worker result
        if self.brain:
            self.brain.fail('Operator halt. Send a new mission to restart.')
        self.send(Command(state='HALTED', reason='Operator halt'))
        response.success, response.message = True, 'Stopped; pending AI response invalidated'
        return response

    def write_event(self, event):
        event['wall_utc'] = datetime.now(timezone.utc).isoformat()
        with (self.log_dir / 'events.jsonl').open('a', encoding='utf-8') as out:
            out.write(json.dumps(event, ensure_ascii=False) + '\n')

    def tick(self):
        now = time.monotonic()
        sim = self.get_clock().now().nanoseconds / 1e9
        dt = 0.0 if self.last_sim is None else sim - self.last_sim
        if self.last_sim is not None and dt < -0.01:
            self.epoch += 1
            if self.brain:
                self.brain.fail('Simulation time moved backwards. Send a new mission after resetting.')
        if dt > 0:
            self.last_clock_advance = now
        self.last_sim = sim
        # Drain the worker only here; the worker never touches ROS or the brain.
        if self.pending is not None and self.pending.done():
            try:
                colors, raw = self.pending.result()
                valid = self.brain is not None and self.pending_epoch == self.epoch and self.brain.state == 'VERIFY'
                self.write_event({'event': 'vision', 'image': self.pending_path, 'model': self.client.model,
                                  'colors': sorted(colors), 'raw': raw, 'accepted_for_current_mission': valid,
                                  'latency_s': round(now - self.pending_started, 3)})
                if valid:
                    self.brain.record_vision(colors)
            except Exception as exc:
                if self.brain is not None and self.pending_epoch == self.epoch:
                    self.brain.fail(f'VLM failed: {exc}')
                self.write_event({'event': 'vision_error', 'error': str(exc)})
                self.get_logger().error(f'VLM failed, no success fallback: {exc}')
            self.pending = None
        if self.pending is not None and now - self.pending_started > self.ai_timeout + 3:
            if self.brain and self.pending_epoch == self.epoch:
                self.brain.fail('VLM deadline exceeded')
                self.epoch += 1
        if self.brain is None:
            self.send(Command(state='IDLE', reason='Send a mission'))
            return
        if self.image is None or now - self.last_image > 1.5 or now - self.last_scan > 1.5:
            self.send(Command(state='WAIT_SENSORS', reason='Fresh camera and lidar required'))
            return
        if now - self.last_clock_advance > 0.7 or sim <= 0:
            self.send(Command(state='WAIT_CLOCK', reason='Simulation paused or /clock missing'))
            return
        new_image = self.used_seq != self.image_seq
        try:
            if new_image:
                data, width, height, step, encoding = self.image
                self.bgra = to_bgra(data, width, height, step, encoding)
                self.used_seq = self.image_seq
                self.blob = find_target_blob(self.bgra, width, height, self.brain.target)
                alt = 'blue' if self.brain.target == 'red' else 'red'
                self.candidate = self.blob is not None or find_target_blob(self.bgra, width, height, alt) is not None
            action = self.brain.step(self.blob, self.candidate, self.ranges, max(0.0, min(dt, 0.25)), new_image=new_image)
            if action.request_vision and self.pending is None:
                self.send(Command(state='VERIFY', reason='Stopped for sensor image inference'))
                width, height = self.image[1:3]
                png = png_from_bgra(self.bgra, width, height)
                self.counter += 1
                snapshot = self.log_dir / f'camera_{self.counter:04d}.png'
                snapshot.write_bytes(png)
                if self.vision_mode == 'rules':
                    labels = {self.brain.target} if self.blob is not None else set()
                    self.brain.record_vision(labels)
                    self.write_event({'event': 'rules_only_confirmation', 'image': snapshot.name, 'ai_used': False})
                else:
                    self.pending_epoch, self.pending_path, self.pending_started = self.epoch, snapshot.name, now
                    self.pending = self.pool.submit(self.client.recognize, png)
                    self.get_logger().info(f'[VLM] Sent camera {snapshot.name}; motor command stays zero')
            self.send(action)
        except Exception as exc:
            self.brain.fail(f'Controller error: {exc}')
            self.send(Command(state='ERROR', reason=str(exc)))
            self.get_logger().error(str(exc))

    def send(self, action: Command):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(action.linear), float(action.angular)
        self.pub.publish(msg)
        now = time.monotonic()
        if action.state != self.last_logged or now >= self.next_log:
            target = self.brain.target if self.brain else ''
            record = {'state': action.state, 'reason': action.reason, 'target': target, 'vision_mode': self.vision_mode,
                      'desired_v': action.linear, 'desired_w': action.angular}
            status = String(); status.data = json.dumps(record)
            self.status_pub.publish(status)
            self.csv.writerow([time.time(), self.last_sim, action.state, target, self.vision_mode, action.linear,
                               action.angular, self.ranges.front, self.blob.center_x_norm if self.blob else '', action.reason])
            self.csv_handle.flush()
            if action.state != self.last_logged:
                self.get_logger().info(f'[{action.state}] {action.reason}')
            self.last_logged, self.next_log = action.state, now + 0.5

    def close(self):
        try:
            if rclpy.ok():
                self.send(Command(state='SHUTDOWN', reason='Controller closing'))
        finally:
            self.pool.shutdown(wait=False, cancel_futures=True)
            self.csv_handle.close()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisualRobot()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
