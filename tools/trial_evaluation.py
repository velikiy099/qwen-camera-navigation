"""Independent Gazebo evaluation; no metric or contact data is sent to ROS/Qwen."""
from collections import Counter
import json
import math
from pathlib import Path
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pai_robot.navigation import DEFAULT_CLEARANCE_M, GEOMETRY


def surface_clearance(x, y, yaw, target):
    """Evaluator-only exact planar distance: vehicle front to target surface."""
    x += GEOMETRY.robot_front_x_m * math.cos(yaw)
    y += GEOMETRY.robot_front_x_m * math.sin(yaw)
    px, py, _, _, _, angle = target['pose']
    dx, dy = x-px, y-py
    geometry = ET.fromstring(target['geometry'])
    if geometry.find('box') is not None:
        sx, sy, _ = map(float, geometry.findtext('box/size').split())
        local_x, local_y = dx*math.cos(angle)+dy*math.sin(angle), -dx*math.sin(angle)+dy*math.cos(angle)
        return math.hypot(max(abs(local_x)-sx/2, 0), max(abs(local_y)-sy/2, 0))
    radius = geometry.findtext('sphere/radius') or geometry.findtext('cylinder/radius')
    if radius is None:
        raise ValueError('Unsupported evaluation target geometry')
    return max(0.0, math.hypot(dx, dy)-float(radius))


def prepare_world(source, destination, target):
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    target_model = world.find(f"model[@name='{target}']")
    if target_model is None or target_model.findtext('static') != 'true':
        raise ValueError('Evaluation target must name an existing static model')
    objects = []
    for model in world.findall('model'):
        if model.findtext('static') != 'true' or model.get('name') == 'ground':
            continue
        pose = [float(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()]
        geometry = model.find('link/collision/geometry')
        objects.append({'name': model.get('name'), 'pose': pose,
                        'geometry': ET.tostring(geometry, encoding='unicode')})
        # Instrument the environment only. The robot retains exactly one RGB sensor.
        for link in model.findall('link'):
            sensor = ET.SubElement(link, 'sensor', name='evaluation_contact', type='contact')
            ET.SubElement(sensor, 'always_on').text = 'true'
            ET.SubElement(sensor, 'topic').text = '/evaluation/contacts'
            contact = ET.SubElement(sensor, 'contact')
            ET.SubElement(contact, 'topic').text = '/evaluation/contacts'
            for collision in link.findall('collision'):
                ET.SubElement(contact, 'collision').text = collision.get('name')
    ET.SubElement(world, 'plugin', name='ignition::gazebo::systems::Contact',
                  filename='libignition-gazebo-contact-system.so')
    ET.indent(tree, space='  ')
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    pose = [float(v) for v in target_model.findtext('pose').split()]
    return {'world': world.get('name'), 'target': target, 'target_xy': pose[:2],
            'arrival_max_clearance_m': DEFAULT_CLEARANCE_M,
            'arrival_reference': 'vehicle front to target near surface', 'objects': objects,
            'robot_sensors': [s.get('type') for s in world.findall("model[@name='visual_bot']//sensor")]}


class Evaluation:
    def __init__(self, output, world, target):
        self.output = Path(output)
        self.world_path = self.output / 'evaluation_world.sdf'
        self.config = prepare_world(world, self.world_path, target)
        self.samples = []
        self.contacts = Counter()
        self.contact_examples = []
        self.errors = []
        self.processes = []
        self.threads = []
        self.lock = threading.Lock()
        self.contact_topic_ready = False
        self.pose_file = (self.output / 'ground_truth.jsonl').open('w')
        self.contact_file = (self.output / 'contacts.jsonl').open('w')
        self.stderr = (self.output / 'evaluation_transport.log').open('w')

    @staticmethod
    def stamp(message):
        stamp = message.get('header', {}).get('stamp', {})
        return float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0)) / 1e9

    def on_pose(self, message):
        pose = next((p for p in message.get('pose', []) if p.get('name') == 'visual_bot'), None)
        if pose is None:
            return
        position, q = pose.get('position', {}), pose.get('orientation', {})
        x, y = float(position.get('x', 0)), float(position.get('y', 0))
        qx, qy, qz, qw = [float(q.get(k, 0)) for k in ('x', 'y', 'z', 'w')]
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        sample = {'sim_time': self.stamp(message), 'x': x, 'y': y, 'yaw': yaw,
                  'target_distance_m': math.hypot(x - self.config['target_xy'][0], y - self.config['target_xy'][1]),
                  'target_clearance_m': surface_clearance(x, y, yaw, next(obj for obj in self.config['objects'] if obj['name'] == self.config['target']))}
        if not all(math.isfinite(v) for v in sample.values()):
            raise ValueError('Non-finite evaluation pose')
        with self.lock:
            if self.samples and sample['sim_time'] - self.samples[-1]['sim_time'] < 0.099:
                return
            self.samples.append(sample)
            self.pose_file.write(json.dumps(sample) + '\n')
            self.pose_file.flush()

    def on_contacts(self, message):
        for contact in message.get('contact', []):
            names = []
            for key in ('collision1', 'collision2'):
                value = contact.get(key, '')
                names.append(value.get('name', str(value.get('id', ''))) if isinstance(value, dict) else str(value))
            if not any('visual_bot' in name for name in names):
                if len(self.contact_examples) < 3:
                    self.contact_examples.append(contact)
                continue
            pair = ' <-> '.join(sorted(names))
            with self.lock:
                self.contacts[pair] += 1
                # Contact samples are not independent collision events.
                if self.contacts[pair] == 1 or self.contacts[pair] % 50 == 0:
                    self.contact_file.write(json.dumps({'sim_time': self.stamp(message), 'pair': pair,
                                                       'sample_count': self.contacts[pair]}) + '\n')
                    self.contact_file.flush()

    def start(self):
        topics = [(f"/world/{self.config['world']}/dynamic_pose/info", self.on_pose),
                  ('/evaluation/contacts', self.on_contacts)]
        for topic, callback in topics:
            process = subprocess.Popen(['ign', 'topic', '-e', '-t', topic, '--json-output'],
                                       stdout=subprocess.PIPE, stderr=self.stderr, text=True)
            self.processes.append(process)
            thread = threading.Thread(target=self.read, args=(process, callback), daemon=True)
            thread.start()
            self.threads.append(thread)
        info = subprocess.run(['ign', 'topic', '-i', '-t', '/evaluation/contacts'],
                              capture_output=True, text=True, timeout=10)
        self.contact_topic_ready = info.returncode == 0 and 'msgs.Contacts' in info.stdout
        (self.output / 'contact_topic.txt').write_text(info.stdout + info.stderr)

    def read(self, process, callback):
        try:
            for line in process.stdout:
                if line.strip():
                    callback(json.loads(line))
        except Exception as exc:
            self.errors.append(f'{type(exc).__name__}: {exc}')

    def ready(self):
        return len(self.samples) >= 3 and self.contact_topic_ready and not self.errors

    def close(self):
        transport_ok = bool(self.processes) and all(p.poll() is None for p in self.processes)
        for p in self.processes:
            if p.poll() is None:
                p.terminate()
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=3)
        for thread in self.threads:
            thread.join(timeout=3)
        self.pose_file.close()
        self.contact_file.close()
        self.stderr.close()
        samples = self.samples
        if not samples:
            return {**self.config, 'available': False, 'errors': self.errors}
        last = samples[-1]
        recent = [p for p in samples if last['sim_time'] - p['sim_time'] <= 0.55]
        stationary = (len(recent) >= 4 and last['sim_time'] - recent[0]['sim_time'] >= 0.3
                      and all(math.hypot(p['x'] - last['x'], p['y'] - last['y']) < 0.01
                              and abs(math.atan2(math.sin(p['yaw'] - last['yaw']), math.cos(p['yaw'] - last['yaw']))) < 0.02
                              for p in recent))
        crossings = []
        screen = next((obj for obj in self.config['objects'] if obj['name'] == 'occluding_screen'), None)
        if screen:
            sx, sy = screen['pose'][:2]
            width = float(ET.fromstring(screen['geometry']).findtext('box/size').split()[1])
            for a, b in zip(samples, samples[1:]):
                if a['x'] < sx <= b['x']:
                    y = a['y'] + (b['y'] - a['y']) * (sx - a['x']) / (b['x'] - a['x'])
                    crossings.append({'y': y, 'clear_of_screen': abs(y - sy) > width / 2 + 0.19})
        return {**self.config, 'available': transport_ok and self.ready(), 'errors': self.errors,
                'sample_count': len(samples), 'initial_pose': samples[0], 'final_pose': last,
                'path_length_m': sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(samples,samples[1:])),
                'minimum_target_distance_m': min(p['target_distance_m'] for p in samples),
                'final_target_distance_m': last['target_distance_m'], 'stationary_at_end': stationary,
                'minimum_target_clearance_m': min(p['target_clearance_m'] for p in samples),
                'final_target_clearance_m': last['target_clearance_m'],
                'inside_goal_at_end': last['target_clearance_m'] <= self.config['arrival_max_clearance_m'],
                'contact_topic_ready': self.contact_topic_ready, 'robot_contact_samples': sum(self.contacts.values()),
                'contact_pairs': dict(self.contacts), 'unclassified_contact_examples': self.contact_examples,
                'screen_crossings': crossings, 'screen_bypassed': any(c['clear_of_screen'] for c in crossings)}
