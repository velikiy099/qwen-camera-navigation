#!/usr/bin/env python3
"""Syntax and RGB / robot-side odometry configuration checks; not a driving test."""
import ast
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml

root = Path(__file__).resolve().parents[1]
# Scope checks to this project, avoiding logs and linked external course materials.
python_files = sorted((root / 'src').rglob('*.py')) + sorted((root / 'tools').glob('*.py'))
for path in python_files:
    ast.parse(path.read_text(), filename=str(path), feature_version=(3, 10))
shell_files = [root / 'run.sh', *sorted((root / 'tools').glob('*.sh')), *sorted((root / 'docker').glob('*.sh'))]
for path in shell_files:
    subprocess.run(['bash', '-n', str(path)], check=True)
yaml_files = [*root.glob('*.yaml'), *(root / 'src').rglob('*.yaml'), *(root / '.github').rglob('*.yml')]
for path in yaml_files:
    yaml.safe_load(path.read_text())
ET.parse(root / 'src/pai_robot/package.xml')
bridge = yaml.safe_load((root / 'src/pai_robot/config/bridge.yaml').read_text())
assert {topic['ros_topic_name'] for topic in bridge} == {'/clock', '/camera/image_raw', '/cmd_vel', '/odom'}
for path in (root / 'src/pai_robot/worlds').glob('*.sdf'):
    world = ET.parse(path).getroot()
    assert not world.findall('.//include') and not world.findall('.//uri')
    assert [sensor.get('type') for sensor in world.findall('.//sensor')] == ['camera']
    assert not world.findall('.//depth_camera')
    assert world.findtext('.//plugin/odom_publish_frequency') == '20'
for module in ('controller_node.py', 'guard_node.py', 'smoke_check.py'):
    source = (root / 'src/pai_robot/pai_robot' / module).read_text()
    assert 'LaserScan' not in source and "'/scan'" not in source
print(f'PASS: {len(python_files)} Python files parse as Python 3.10; shell, YAML and XML syntax.')
print('PASS: one RGB camera; bridge connects camera, clock, command velocity and 20 Hz wheel odometry.')
print('This check does not measure rendering, physics, Qwen inference or navigation success.')
