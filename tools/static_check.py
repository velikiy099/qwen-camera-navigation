#!/usr/bin/env python3
"""Syntax/configuration checks only; not a Gazebo launch or ROS DDS test."""
import ast
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml
root=Path(__file__).resolve().parents[1]
python_files=list(root.rglob('*.py'))
for path in python_files:
    if '__pycache__' not in str(path):
        ast.parse(path.read_text(),filename=str(path),feature_version=(3,10))
for path in root.rglob('*.sh'):
    subprocess.run(['bash','-n',str(path)],check=True)
for path in root.rglob('*.yaml'): yaml.safe_load(path.read_text())
ET.parse(root/'src/pai_robot/package.xml')
world=ET.parse(root/'src/pai_robot/worlds/visual_search.sdf').getroot()
assert not world.findall('.//include') and not world.findall('.//uri')
print(f'PASS: {len(python_files)} Python files parse as Python 3.10; shell syntax, YAML, XML well-formed.')
print('Not performed: Docker Compose CLI validation, sdformat schema/plugin loading, Gazebo integration.')
