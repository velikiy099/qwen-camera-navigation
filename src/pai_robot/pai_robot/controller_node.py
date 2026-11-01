"""ROS adapter for the camera-only Qwen agent; all worker results are epoch-scoped."""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
import csv
import json
import math
import os
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .agent import CameraAgent, parse_mission
from .navigation import Goal
from .images import CHANNELS, png_from_image
from .ollama_client import InvalidToolResponse, OllamaClient
from .motion import OdometrySample, ODOM_MAX_AGE, wrap
from .mission_status import state_line


class VisualRobot(Node):
    def __init__(self):
        super().__init__('visual_robot')
        defaults = {'log_dir': os.environ.get('PAI_LOG_DIR', '/workspace/logs'),
                    'ai_timeout': 90.0, 'max_actions': 80,
                    'max_sim_seconds': 900.0, 'max_wall_seconds': 1200.0}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.ai_timeout = float(self.get_parameter('ai_timeout').value)
        self.max_actions = int(self.get_parameter('max_actions').value)
        self.max_sim = float(self.get_parameter('max_sim_seconds').value)
        self.max_wall = float(self.get_parameter('max_wall_seconds').value)
        if not 1 <= self.max_actions <= 500 or not all(math.isfinite(v) and v > 0 for v in (self.max_sim, self.max_wall)):
            raise ValueError('Invalid mission budget')
        self.client = OllamaClient(timeout=self.ai_timeout)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='vlm')
        self.pending = None
        self.pending_epoch = 0
        self.pending_started = 0.0
        self.pending_path = ''
        self.epoch = self.counter = 0
        self.agent = None
        self.idle_state, self.idle_reason = 'IDLE', 'Send a natural-language /mission'
        self.image = None
        self.image_time = None
        self.last_image = -1e12
        self.odom = None
        self.last_odom = -1e12
        self.last_sim = None
        self.last_clock_advance = -1e12
        self.mission_started = 0.0
        self.mission_id = None
        self.last_phase = None
        self.phase_started = 0.0
        self.observation = ''
        self.sensor_ready_once = False
        self.next_log = 0.0
        self.last_state = ''
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.log_dir = Path(self.get_parameter('log_dir').value) / ('run_' + stamp)
        self.log_dir.mkdir(parents=True, exist_ok=False)
        self.csv_handle = (self.log_dir / 'control.csv').open('w', newline='')
        self.csv = csv.writer(self.csv_handle)
        self.csv.writerow(['wall_utc', 'sim_time', 'epoch', 'state', 'step', 'requested_v', 'requested_w', 'reason'])
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.status_pub = self.create_publisher(String, '/robot_state', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/mission', self.on_mission, 10)
        self.create_service(Trigger, '/halt', self.on_halt)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=self.steady)
        self.get_logger().info(f'Camera agent with wheel-odometry execution ready; model={self.client.model}. Logs: {self.log_dir}')

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def write_event(self, event):
        with (self.log_dir / 'events.jsonl').open('a', encoding='utf-8') as out:
            out.write(json.dumps({'wall_utc': datetime.now(timezone.utc).isoformat(),
                                  'epoch': self.epoch, **event}, ensure_ascii=False, allow_nan=False) + '\n')

    def drain_results(self):
        if self.agent:
            for result in self.agent.recent_results:
                self.write_event({'event': 'tool_result', 'result': result})
            self.agent.recent_results.clear()

    def fail(self, reason, state='ERROR'):
        self.epoch += 1  # Includes already-running or already-completed HTTP responses.
        if self.agent:
            self.agent.stop(state, reason, self.sim_time())
            self.drain_results()
        self.idle_state, self.idle_reason = state, reason
        self.send(0.0, 0.0)
        self.write_event({'event': 'stopped', 'state': state, 'reason': reason})

    def on_image(self, msg):
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if not math.isfinite(stamp) or stamp < 0:
                raise ValueError('Invalid camera timestamp')
            if self.image_time is not None and stamp <= self.image_time:
                return  # A duplicate frame must not refresh the watchdog.
            channels = CHANNELS.get(msg.encoding, 0)
            if (not channels or not 0 < msg.width * msg.height <= 4096 * 4096
                    or msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * channels
                    or len(msg.data) < msg.step * msg.height):
                raise ValueError('Invalid camera encoding, dimensions or payload')
            self.image = (bytes(msg.data), msg.width, msg.height, msg.step, msg.encoding)
            self.image_time, self.last_image = stamp, time.monotonic()
        except (ValueError, TypeError, AttributeError) as exc:
            self.image = None
            if self.agent and not self.agent.terminal:
                self.fail(str(exc))

    def on_mission(self, msg):
        # Stop immediately, including when a replacement mission is invalid.
        self.pub.publish(Twist())
        try:
            instruction = parse_mission(msg.data)
        except (ValueError, TypeError) as exc:
            self.fail(f'Invalid mission: {exc}')
            return
        if self.agent and not self.agent.terminal:
            self.agent.stop('HALTED', 'Replaced by a new mission', self.sim_time())
            self.drain_results()
        self.epoch += 1
        self.agent = CameraAgent(instruction, self.sim_time(), max_actions=self.max_actions, max_sim_seconds=self.max_sim)
        self.mission_id = json.loads(msg.data).get('mission_id') or uuid.uuid4().hex
        self.observation = ''
        self.mission_started = time.monotonic()
        self.sensor_ready_once = False
        self.write_event({'event': 'mission', 'instruction': instruction, 'model': self.client.model,
                          'mission_id': self.mission_id,
                          'inputs': ['RGB camera', 'instruction', 'tool history'],
                          'robot_control_input': 'wheel_odometry', 'ground_truth_used': False})
        self.send(0.0, 0.0)

    def on_halt(self, request, response):
        self.fail('Operator halt; send a new mission to restart', 'HALTED')
        response.success, response.message = True, 'Stopped; pending model response invalidated'
        return response

    def on_odom(self, msg):
        try:
            sample = OdometrySample.from_message(msg)
            if self.odom is not None:
                dt = sample.stamp-self.odom.stamp
                if dt < 0:
                    raise ValueError('Wheel odometry clock moved backwards')
                if dt == 0:
                    return
                # Bounds include margin over the SDF velocity limits and sampling noise.
                if (math.hypot(sample.x-self.odom.x, sample.y-self.odom.y) > 0.2*dt+0.03
                        or abs(wrap(sample.yaw-self.odom.yaw)) > dt+0.1):
                    raise ValueError('Wheel odometry jumped; relative motion reference is invalid')
            self.odom, self.last_odom = sample, time.monotonic()
        except (ValueError, TypeError, AttributeError) as exc:
            self.odom, self.last_odom = None, -1e12
            if self.agent and not self.agent.terminal:
                self.fail(str(exc))

    def sensors_ready(self, now, sim):
        return (self.image is not None and self.image_time is not None
                and 0 <= now - self.last_image <= 1.5
                and 0 <= sim - self.image_time <= 0.6
                and self.odom is not None and 0 <= now-self.last_odom <= ODOM_MAX_AGE
                and 0 <= sim-self.odom.stamp <= ODOM_MAX_AGE
                and 0 <= now - self.last_clock_advance <= 0.7)

    def poll_response(self, now, sim):
        if self.pending is None:
            return
        if self.pending_epoch == self.epoch and now - self.pending_started > self.ai_timeout + 1:
            self.fail('VLM response deadline exceeded')
            return
        if not self.pending.done():
            return
        future, request_epoch, path = self.pending, self.pending_epoch, self.pending_path
        self.pending = None
        if request_epoch != self.epoch or not self.agent or self.agent.state != 'THINKING':
            self.write_event({'event': 'discarded_response', 'request_epoch': request_epoch})
            return
        try:
            action, raw = future.result()
        except InvalidToolResponse as exc:
            self.write_event({'event': 'rejected_response', 'camera_path': path, 'raw': exc.raw, 'reason': str(exc)})
            self.agent.reject(str(exc), sim)
            self.send(0.0, 0.0)
            return
        if isinstance(action, Goal):
            self.write_event({'event': 'mission_goal', 'raw': raw, 'goal': asdict(action)})
            self.observation = ''
        else:
            self.write_event({'event': 'decision', 'camera_path': path, 'raw': raw, 'tool_call': asdict(action)})
            self.observation = action.observation
        self.agent.accept(action, sim)

    def tick(self):
        try:
            self._tick()
        except Exception as exc:
            self.fail(f'{type(exc).__name__}: {exc}')
            self.get_logger().error(f'Mission stopped: {exc}')

    def _tick(self):
        now, sim = time.monotonic(), self.sim_time()
        previous_sim, self.last_sim = self.last_sim, sim
        if previous_sim is not None and sim < previous_sim:
            self.image = self.image_time = None
            self.odom, self.last_odom = None, -1e12
            self.last_clock_advance = -1e12
            if self.agent and not self.agent.terminal:
                self.fail('Simulation clock reset; send a new mission')
        elif previous_sim is not None and sim > previous_sim:
            self.last_clock_advance = now
        # Reap stale workers without allowing them to affect the new mission.
        if self.pending is not None and self.pending_epoch != self.epoch and self.pending.done():
            self.poll_response(now, sim)
        if not self.agent or self.agent.terminal:
            self.send(0.0, 0.0)
            return
        if now - self.mission_started > self.max_wall:
            self.fail('Mission wall-time budget exceeded')
            return
        if not self.sensors_ready(now, sim):
            if self.sensor_ready_once or now - self.mission_started > 10:
                self.fail('Camera, wheel odometry or simulation clock is stale, missing or inconsistent')
            else:
                self.send(0.0, 0.0)
            return
        self.sensor_ready_once = True
        # Advance before accepting a response so motion starts at the current clock.
        v, w = self.agent.advance(sim, self.odom)
        if self.agent.terminal:
            self.epoch += 1
        else:
            self.poll_response(now, sim)
            if (self.pending is None and self.agent.ready(sim, self.image_time)
                    and self.agent.begin_inference()):
                pixels, width, height, step, encoding = self.image
                png = png_from_image(pixels, width, height, step, encoding)
                self.counter += 1
                path = f'camera_{self.counter:04d}.png'
                (self.log_dir / path).write_bytes(png)
                self.agent.observe_frame(png)
                context = self.agent.context()
                self.write_event({'event': 'inference_request', 'camera_path': path,
                                  'image_sim_time': self.image_time, 'context': context})
                self.pending_epoch, self.pending_started, self.pending_path = self.epoch, now, path
                self.pending = self.pool.submit(self.client.decide, png, context)
        self.drain_results()
        if self.agent.state != 'EXECUTING':
            v = w = 0.0
        self.send(v, w)

    def send(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)
        state = self.agent.state if self.agent else self.idle_state
        reason = self.agent.reason if self.agent else self.idle_reason
        now = time.monotonic()
        step = self.agent.actions if self.agent else 0
        phase = (self.mission_id, state, step)
        changed = phase != self.last_phase
        if changed:
            self.last_phase, self.phase_started = phase, now
        if now < self.next_log and not changed:
            return
        self.next_log, self.last_state = now + 0.2, state
        sim = self.sim_time()
        status = {'state': state, 'reason': reason, 'epoch': self.epoch, 'step': step,
                  'instruction': self.agent.instruction if self.agent else '', 'model': self.client.model,
                  'sim_time': sim, 'requested_v': v, 'requested_w': w,
                  'motion_feedback': 'wheel_odometry', 'ground_truth_used': False, 'arrival_verified': False,
                  'arrival_reported_by_model': bool(state == 'DONE' and self.agent and self.agent.completion_source == 'model'),
                  'completion_source': self.agent.completion_source if self.agent else None, 'obstacle_watchdog': False,
                  'mission_id': self.mission_id, 'observation': self.observation,
                  'elapsed_wall_s': round(now-self.mission_started, 1) if self.agent else 0,
                  'state_elapsed_s': round(now-self.phase_started, 1), 'log_dir': str(self.log_dir),
                  'last_tool_result': ({key: self.agent.history[-1].get(key) for key in ('status', 'reason')}
                                       if self.agent and self.agent.history else None),
                  'motion_progress': None}
        if self.agent:
            status.update(fixed_goal=asdict(self.agent.goal) if self.agent.goal else None,
                          candidate_assessment=self.agent.assessment,
                          search_progress=self.agent.search.context(),
                          camera_arrival_checks_passed=bool(state == 'DONE' and self.agent.goal and
                              self.agent.goal.kind == 'approach' and self.agent.assessment.get('confirmed_in_two_frames')))
        if self.agent and self.agent.motion:
            motion = self.agent.motion
            turn = motion.tool == 'turn'
            status['motion_progress'] = {'tool': motion.tool,
                'goal': math.degrees(motion.amount) if turn else motion.amount,
                'current': math.degrees(motion.angle) if turn else motion.distance,
                'unit': 'deg' if turn else 'm'}
        if changed:
            self.get_logger().info(state_line(status))
            if self.agent and (state in ('DONE', 'STOPPED', 'HALTED', 'ERROR')):
                self.get_logger().info('Reason: ' + reason)
        message = String()
        message.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(message)
        self.csv.writerow([datetime.now(timezone.utc).isoformat(), sim, self.epoch, state, step, v, w, reason])
        self.csv_handle.flush()

    def close(self):
        self.epoch += 1
        if rclpy.ok():
            self.pub.publish(Twist())
        if self.agent and not self.agent.terminal:
            self.agent.stop('HALTED', 'Controller shutdown', self.sim_time())
            self.drain_results()
        if self.pending is not None:
            self.pending.cancel()
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.csv_handle.close()


def main(args=None):
    rclpy.init(args=args)
    node = VisualRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
