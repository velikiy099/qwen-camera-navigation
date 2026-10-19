from setuptools import find_packages, setup
from glob import glob
setup(name='pai_robot', version='3.0.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/pai_robot']),
                ('share/pai_robot', ['package.xml']),
                ('share/pai_robot/launch', glob('launch/*.launch.py')),
                ('share/pai_robot/config', glob('config/*.yaml')),
                ('share/pai_robot/worlds', glob('worlds/*.sdf'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='PAI project', maintainer_email='maintainer@example.invalid', license='MIT',
    description='Local VLM and visual search in ROS 2/Gazebo',
    entry_points={'console_scripts': [
        'visual_robot = pai_robot.controller_node:main',
        'velocity_guard = pai_robot.guard_node:main',
        'mission = pai_robot.mission_cli:main',
        'smoke_check = pai_robot.smoke_check:main']})
