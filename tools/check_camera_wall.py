#!/usr/bin/env python3
"""Reproduce wall contact and check RGB occlusion in an isolated Gazebo world.

Run inside the ROS container. Uses scripted actions through the production
controller, not a VLM. Contact and ground truth are evaluation-only inputs.
The red-pixel check is specific to this known test scene, never navigation.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import argparse
import io
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from PIL import Image as PILImage
from trial_evaluation import Evaluation
from pai_robot.navigation import GEOMETRY


CASES = {
    'frontal': {'pose': '-0.40 0 0.005 0 0 0', 'goals': [('move', .6)]},
    'oblique': {'pose': '-0.50 -0.30 0.005 0 0 0.523598775598', 'goals': [('move', .8)]},
    'rotation': {'pose': '-0.12 0 0.005 0 0 1.570796326795', 'goals': [('turn', -90), ('turn', 180)]},
    'unoccluded_control': {'pose': '-0.40 0 0.005 0 0 0', 'goals': [('observe', 0)], 'remove_wall': True},
}


def camera_position(pose):
    q = [float(pose['orientation'].get(k, 0)) for k in ('x', 'y', 'z', 'w')]
    norm = math.sqrt(sum(v*v for v in q))
    x, y, z, w = [v/norm for v in q]
    # Full quaternion rotation, including any contact-induced pitch and roll.
    cx, cz = GEOMETRY.camera_x_m, GEOMETRY.camera_height_m
    offset = ((1-2*(y*y+z*z))*cx + 2*(x*z+y*w)*cz,
              2*(x*y+z*w)*cx + 2*(y*z-x*w)*cz,
              2*(x*z-y*w)*cx + (1-2*(x*x+y*y))*cz)
    return [float(pose['position'].get(k, 0))+v for k, v in zip(('x', 'y', 'z'), offset)]


def box_margin(point, center, half_size):
    delta = [abs(p-c)-h for p, c, h in zip(point, center, half_size)]
    outside = math.sqrt(sum(max(v, 0)**2 for v in delta))
    return outside if outside else max(delta)  # Negative means inside the wall.


class CameraEvaluation(Evaluation):
    def __init__(self, *args):
        super().__init__(*args)
        screen = next((o for o in self.config['objects'] if o['name'] == 'occluding_screen'), None)
        self.wall_center = screen['pose'][:3] if screen else None
        self.wall_half_size = ([float(v)/2 for v in ET.fromstring(screen['geometry']).findtext('box/size').split()]
                               if screen else None)
        self.camera_samples = []

    def on_pose(self, message):
        super().on_pose(message)
        stamp = self.stamp(message)
        if self.camera_samples and stamp-self.camera_samples[-1]['sim_time'] < .099:
            return
        pose = next((p for p in message.get('pose', []) if p.get('name') == 'visual_bot'), None)
        if pose is not None:
            point = camera_position(pose)
            self.camera_samples.append({'sim_time': stamp, 'camera_xyz_m': point,
                'wall_margin_m': box_margin(point, self.wall_center, self.wall_half_size) if self.wall_center else None})


def run_case(output, label):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from pai_robot.agent import ToolCall
    from pai_robot.navigation import Goal
    from pai_robot.controller_node import VisualRobot
    from pai_robot.images import png_from_image

    case = CASES[label]
    output.mkdir()
    (output/'frames').mkdir()
    tree = ET.parse('/workspace/src/pai_robot/worlds/hidden_search.sdf')
    world = tree.getroot().find('world')
    world.find("model[@name='visual_bot']/pose").text = case['pose']
    if case.get('remove_wall'):
        world.remove(world.find("model[@name='occluding_screen']"))
    source = output/'source_world.sdf'
    tree.write(source, encoding='utf-8', xml_declaration=True)
    evaluation = CameraEvaluation(output, source, 'red_sphere')
    os.environ['PAI_LOG_DIR'] = str(output)
    os.environ['IGN_PARTITION'] = f'pai_camera_wall_{os.getpid()}_{label}'

    class ScriptedActions:
        model = 'scripted camera-wall regression; no VLM'
        index = 0

        def decide(self, png, context):
            if context.get('fixed_goal') is None:
                goal = Goal('inspect', 'Camera-wall regression', 'unspecified', 'unspecified', 1.0)
                return goal, json.dumps(asdict(goal))
            tool, amount = case['goals'][self.index] if self.index < len(case['goals']) else ('stop', 0)
            self.index += 1
            action = ToolCall(tool=tool, distance_m=amount if tool == 'move' else 0,
                direction=('left' if amount >= 0 else 'right') if tool == 'turn' else 'none',
                angle_deg=abs(amount) if tool == 'turn' else 0, observation='Camera-wall regression',
                memory='', target_visible=False, target_reached=False, candidate_color='none',
                candidate_shape='none', ground_contact_px=[],
                routes={d: 'clear' for d in ('left', 'forward', 'right', 'backward')},
                next_subgoal='Execute scripted action')
            return action, json.dumps(asdict(action))

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = VisualRobot()
    node.client = ScriptedActions()
    frames = []

    def capture(msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if frames and stamp-frames[-1]['sim_time'] < .099:
            return
        png = png_from_image(bytes(msg.data), msg.width, msg.height, msg.step, msg.encoding)
        with PILImage.open(io.BytesIO(png)) as image:
            # Isolate this fixture's red sphere from orange, yellow and purple.
            red = sum(r > 50 and g < .3*r and b < .3*r for r, g, b in image.convert('RGB').getdata())
        path = f'frames/{len(frames):04d}.png'
        (output/path).write_bytes(png)
        frames.append({'sim_time': stamp, 'path': path, 'red_pixels': red})

    subscription = node.create_subscription(Image, '/camera/image_raw', capture, qos_profile_sensor_data)
    launcher = None
    started = time.monotonic()
    evaluated = sent = False
    terminal_at = None
    error = ''
    log = (output/'launch.log').open('w')
    try:
        launcher = subprocess.Popen(['ros2', 'launch', 'pai_robot', 'demo.launch.py', 'gui:=false',
            'controller:=false', f'world:={evaluation.world_path}'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        while time.monotonic()-started < 60:
            if launcher.poll() is not None:
                raise RuntimeError('Gazebo launch exited')
            rclpy.spin_once(node, timeout_sec=.05)
            if node.image is not None and node.odom is not None and not evaluated:
                evaluation.start()
                evaluated = True
            if not sent and evaluated and evaluation.ready() and node.sensors_ready(time.monotonic(), node.sim_time()):
                node.on_mission(SimpleNamespace(data=json.dumps({'instruction': 'Scripted camera-wall regression'})))
                sent = True
            if sent and node.agent.terminal:
                if terminal_at is None:
                    terminal_at = time.monotonic()
                if time.monotonic()-terminal_at > 1.5:
                    break
        else:
            raise TimeoutError('Camera-wall check timed out')
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
        log.close()
        state = node.agent.state if node.agent else None
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

    margins = [p['wall_margin_m'] for p in evaluation.camera_samples if p['wall_margin_m'] is not None]
    blocked = not case.get('remove_wall', False)
    max_red = max((f['red_pixels'] for f in frames), default=0)
    checks = {'runner_ok': not error, 'evaluation_available': measured.get('available', False),
              'script_finished': state == 'STOPPED', 'frames_recorded': len(frames) >= 3,
              'stationary_at_end': measured.get('stationary_at_end', False)}
    if blocked:
        checks.update(contact_recorded=measured.get('robot_contact_samples', 0) > 0,
                      camera_outside_wall=bool(margins) and min(margins) > 0,
                      wall_beyond_near_clip=bool(margins) and min(margins) > GEOMETRY.camera_near_clip_m,
                      hidden_red_sphere_not_rendered=max_red == 0)
    else:
        checks['red_sphere_visible_without_wall'] = max_red >= 20
    report = {'case': label, 'protocol': case, 'camera_profile': GEOMETRY.context(),
              'vlm_used': False, 'state': state, 'runner_error': error, 'checks': checks,
              'passed': all(checks.values()), 'minimum_camera_wall_margin_m': min(margins) if margins else None,
              'maximum_red_pixels': max_red, 'frames': frames, 'camera_samples': evaluation.camera_samples,
              'evaluation': measured, 'controller_log_dir': str(node.log_dir)}
    (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    return {k: report[k] for k in ('case', 'checks', 'passed', 'minimum_camera_wall_margin_m', 'maximum_red_pixels', 'runner_error')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=list(CASES), help='Run one case instead of the full suite')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '76'
    output = args.output or Path('/workspace/logs')/('camera_wall_check_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    output.mkdir()
    results = []
    for label in ([args.case] if args.case else CASES):
        print('Starting '+label, flush=True)
        result = run_case(output/label, label)
        results.append(result)
        print(json.dumps(result), flush=True)
        (output/'results.json').write_text(json.dumps({'complete': False, 'results': results}, indent=2)+'\n')
    report = {'complete': True, 'passed': all(r['passed'] for r in results), 'results': results}
    (output/'results.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Output: '+str(output), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
