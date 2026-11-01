#!/usr/bin/env python3
"""Run a bounded actual Gazebo/Qwen trial inside the container and save evidence.

The agent uses RGB only. Optional evaluation reads ground truth independently
and instruments contacts on the environment, never feeding them to the agent.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('instruction')
    parser.add_argument('--hidden', action='store_true')
    parser.add_argument('--max-actions', type=int, default=20)
    parser.add_argument('--max-wall', type=float, default=240)
    parser.add_argument('--evaluate', action='store_true', help='Record independent simulator poses and environment contacts')
    parser.add_argument('--target', help='SDF target model name, used only by the independent evaluator')
    from pai_robot.navigation import DEFAULT_CLEARANCE_M
    parser.add_argument('--arrival-clearance', type=float, default=DEFAULT_CLEARANCE_M,
                        help='Independent maximum gap from vehicle front to target surface, meters')
    args = parser.parse_args()
    if args.evaluate and not args.target:
        parser.error('--evaluate requires --target')
    if not math.isfinite(args.arrival_clearance) or args.arrival_clearance <= 0:
        parser.error('--arrival-clearance must be positive and finite')
    output = Path(os.environ.get('PAI_LOG_DIR', '/workspace/logs')) / ('trial_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    output.mkdir(parents=True)
    # Keep this experiment independent of a user's normal simulation.
    os.environ['ROS_DOMAIN_ID'] = '74'
    os.environ['IGN_PARTITION'] = f'pai_trial_{os.getpid()}'
    command = ['ros2', 'launch', 'pai_robot', 'demo.launch.py', 'gui:=false',
               f'log_dir:={output}', f'max_actions:={args.max_actions}', f'max_wall_seconds:={args.max_wall}']
    evaluation = None
    if args.evaluate:
        from trial_evaluation import Evaluation
        source = Path('/workspace/src/pai_robot/worlds') / ('hidden_search.sdf' if args.hidden else 'visual_search.sdf')
        evaluation = Evaluation(output, source, args.target)
        evaluation.config['arrival_max_clearance_m'] = args.arrival_clearance
        command.append(f'world:={evaluation.world_path}')
    elif args.hidden:
        command.append('world:=/workspace/src/pai_robot/worlds/hidden_search.sdf')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    rclpy.init()
    node = Node('camera_agent_trial_observer')
    observations = {'camera_frames': 0, 'nonzero_commands': 0, 'last_command': [0.0, 0.0]}
    states = Counter()
    state = {}

    def camera(msg):
        observations['camera_frames'] += 1

    def velocity(msg):
        observations['last_command'] = [msg.linear.x, msg.angular.z]
        if abs(msg.linear.x) > 1e-8 or abs(msg.angular.z) > 1e-8:
            observations['nonzero_commands'] += 1

    def status(msg):
        state.update(json.loads(msg.data))
        states[state['state']] += 1

    node.create_subscription(Image, '/camera/image_raw', camera, qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel', velocity, 10)
    node.create_subscription(String, '/robot_state', status, 10)
    mission = node.create_publisher(String, '/mission', 10)
    began = time.monotonic()
    sent = False
    launcher = None
    error = ''
    evaluation_started = False
    terminal_seen = None
    try:
        with (output / 'launch.log').open('w') as log:
            launcher = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            while time.monotonic() - began < args.max_wall + 40:
                if launcher.poll() is not None:
                    raise RuntimeError(f'Launch exited: {launcher.returncode}')
                rclpy.spin_once(node, timeout_sec=.1)
                if evaluation and not evaluation_started and observations['camera_frames'] >= 3:
                    evaluation.start()
                    evaluation_started = True
                if not sent and time.monotonic() - began > 40:
                    raise RuntimeError('Camera, agent or evaluation startup timed out')
                if not sent and observations['camera_frames'] >= 3 and mission.get_subscription_count() and (not evaluation or evaluation.ready()):
                    message = String()
                    message.data = json.dumps({'instruction': args.instruction}, ensure_ascii=False)
                    mission.publish(message)
                    sent = True
                    print('Mission sent; evidence:', output, flush=True)
                if sent and state.get('state') in ('DONE', 'STOPPED', 'HALTED', 'ERROR'):
                    if terminal_seen is None:
                        terminal_seen = time.monotonic()
                    if time.monotonic() - terminal_seen >= 1.5:
                        break
            else:
                error = 'Trial runner wall-time limit exceeded'
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        topics = dict(node.get_topic_names_and_types())
        measured = evaluation.close() if evaluation else None
        if launcher is not None and launcher.poll() is None:
            os.killpg(launcher.pid, signal.SIGINT)
            try:
                launcher.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(launcher.pid, signal.SIGTERM)
                try:
                    launcher.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(launcher.pid, signal.SIGKILL)
                    launcher.wait(timeout=5)
        result = {'instruction': args.instruction, 'hidden_world': args.hidden,
                  'elapsed_wall_s': round(time.monotonic() - began, 2),
                  'mission_sent': sent, 'final_state': state, 'state_counts': dict(states),
                  **observations, 'ros_topics': topics, 'runner_error': error,
                  'navigation_success_verified': False, 'contact_measured': False}
        if measured:
            result['evaluation'] = measured
            result['arrival_policy_consistent'] = math.isclose(
                (state.get('fixed_goal') or {}).get('max_clearance_m', -1), args.arrival_clearance)
            result['contact_measured'] = measured.get('available', False)
            result['navigation_success_verified'] = bool(measured.get('available') and
                result['arrival_policy_consistent'] and
                measured.get('inside_goal_at_end') and measured.get('stationary_at_end') and
                measured.get('robot_contact_samples') == 0 and state.get('state') == 'DONE' and
                observations['last_command'] == [0.0, 0.0] and
                (not args.hidden or measured.get('screen_bypassed')))
        (output / 'trial.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return int(bool(error) or not sent)


if __name__ == '__main__':
    raise SystemExit(main())
