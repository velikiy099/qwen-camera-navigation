#!/usr/bin/env python3
"""Actual Gazebo motion check with scripted goals, not a VLM navigation test.

Uses the production ROS controller/guard and independently measures simulator
ground truth. Run inside the ROS container. No ground truth enters control.
"""
from dataclasses import asdict
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

from trial_evaluation import Evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cross-check', action='store_true', help='Use angles and distances absent from calibration')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '75'
    os.environ['IGN_PARTITION'] = f'pai_motion_check_{os.getpid()}'
    output = Path('/workspace/logs') / ('motion_check_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    output.mkdir()
    os.environ['PAI_LOG_DIR'] = str(output)
    goals = [('move', .3), ('move', -.15), ('turn', 90), ('move', .3), ('turn', -90), ('turn', 360)]
    if args.cross_check:
        goals = [('move', .6), ('turn', 60), ('move', -.2), ('turn', -120), ('turn', 270)]
    protocol = {'goals': goals, 'odometry_tolerance_m': .012, 'odometry_tolerance_deg': 1.5,
                'ground_truth_tolerance_m': .03, 'ground_truth_tolerance_deg': 5,
                'wall_timeout_s': 150, 'vlm_used': False, 'cross_check': args.cross_check}
    (output / 'protocol.json').write_text(json.dumps(protocol, indent=2))
    evaluation = Evaluation(output, '/workspace/src/pai_robot/worlds/visual_search.sdf', 'red_box')

    import rclpy
    from pai_robot.agent import ToolCall
    from pai_robot.navigation import Goal
    from pai_robot.controller_node import VisualRobot
    from pai_robot.motion import wrap

    class ScriptedGoals:
        model = 'scripted motion check; no VLM'
        index = 0

        def decide(self, png, context):
            if context.get('fixed_goal') is None:
                goal = Goal('inspect', 'Scripted motion check', 'unspecified', 'unspecified', 1.0)
                return goal, json.dumps(asdict(goal))
            tool, amount = goals[self.index] if self.index < len(goals) else ('stop', 0)
            self.index += 1
            action = ToolCall(tool=tool, distance_m=amount if tool=='move' else 0,
                              direction=('left' if amount >= 0 else 'right') if tool=='turn' else 'none',
                              angle_deg=abs(amount) if tool=='turn' else 0, observation='Scripted motion check',
                              memory='', target_visible=False, target_reached=False,
                              candidate_color='none', candidate_shape='none', ground_contact_px=[],
                              routes={d: 'clear' for d in ('left', 'forward', 'right', 'backward')},
                              next_subgoal='Execute the next scripted goal')
            return action, json.dumps(asdict(action))

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = VisualRobot()
    node.client = ScriptedGoals()
    launcher = None
    error = ''
    started = time.monotonic()
    evaluated = sent = False
    terminal_at = None
    try:
        with (output / 'launch.log').open('w') as log:
            launcher = subprocess.Popen(['ros2', 'launch', 'pai_robot', 'demo.launch.py', 'gui:=false',
                         'controller:=false', f'world:={evaluation.world_path}'],
                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            while time.monotonic()-started < protocol['wall_timeout_s']:
                if launcher.poll() is not None:
                    raise RuntimeError('Gazebo launch exited')
                rclpy.spin_once(node, timeout_sec=.05)
                if node.image is not None and node.odom is not None and not evaluated:
                    evaluation.start()
                    evaluated = True
                if not sent and evaluated and evaluation.ready() and node.sensors_ready(time.monotonic(), node.sim_time()):
                    node.on_mission(SimpleNamespace(data=json.dumps({'instruction': 'Scripted motor-control check'})))
                    sent = True
                if not sent and time.monotonic()-started > 40:
                    raise TimeoutError('Camera, odometry, clock or evaluation did not become ready')
                if sent and node.agent.terminal:
                    if terminal_at is None:
                        terminal_at = time.monotonic()
                    if time.monotonic()-terminal_at > 1.5:
                        break
            else:
                raise TimeoutError('Motion check timed out')
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        node.close()
        measured = evaluation.close()
        if launcher is not None and launcher.poll() is None:
            for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                os.killpg(launcher.pid, sig)
                try:
                    launcher.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    continue
        event_path = node.log_dir / 'events.jsonl'
        events = [json.loads(line) for line in event_path.read_text().splitlines()] if event_path.exists() else []
        results = [e['result'] for e in events if e['event']=='tool_result' and e['result'].get('motion_feedback')=='wheel_odometry']
        for result in results:
            samples = evaluation.samples
            if not samples:
                result['passed'] = False
                continue
            start = min(samples, key=lambda p: abs(p['sim_time']-result['started_sim_s']))
            end = min(samples, key=lambda p: abs(p['sim_time']-result['ended_sim_s']))
            path = [p for p in samples if start['sim_time'] <= p['sim_time'] <= end['sim_time']]
            distance = ((end['x']-start['x'])*math.cos(start['yaw']) +
                        (end['y']-start['y'])*math.sin(start['yaw']))
            angle = math.degrees(sum(wrap(b['yaw']-a['yaw']) for a,b in zip(path,path[1:])))
            move = result['tool_call']['tool']=='move'
            requested = result['tool_call']['distance_m' if move else 'angle_deg']
            if not move and result['tool_call']['direction'] == 'right':
                requested = -requested
            result.update(ground_truth_distance_m=distance, ground_truth_angle_deg=angle,
                          odometry_error=abs(result['odometry_distance_m' if move else 'odometry_angle_deg']-requested),
                          ground_truth_error=abs((distance if move else angle)-requested))
            result['passed'] = (result['status']=='completed'
                and result['odometry_error'] <= protocol['odometry_tolerance_m' if move else 'odometry_tolerance_deg']
                and result['ground_truth_error'] <= protocol['ground_truth_tolerance_m' if move else 'ground_truth_tolerance_deg'])
        report = {'protocol': protocol, 'results': results, 'evaluation': measured, 'runner_error': error,
                  'state': node.agent.state if node.agent else None,
                  'passed': not error and len(results)==len(goals) and all(r['passed'] for r in results)
                            and measured.get('available') and measured.get('stationary_at_end')
                            and measured.get('robot_contact_samples')==0}
        (output / 'motion_check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps({'directory': str(output), 'passed': report['passed'], 'state': report['state'],
                          'error': error, 'motions': [{k:r[k] for k in ('tool_call','odometry_error','ground_truth_error','elapsed_sim_s','passed') if k in r} for r in results]}, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
