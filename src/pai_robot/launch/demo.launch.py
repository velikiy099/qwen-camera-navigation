"""Fortress camera, bridge, watchdog and a single Qwen navigation agent."""
import os
import fcntl
import hashlib
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pai_robot.navigation import check_camera_profile


def start(context):
    share = Path(get_package_share_directory('pai_robot'))
    engine = LaunchConfiguration('render_engine').perform(context)
    if engine not in ('ogre', 'ogre2'):
        raise ValueError('render_engine must be ogre or ogre2')
    world = Path(LaunchConfiguration('world').perform(context))
    text = world.read_text().replace('<render_engine>ogre2</render_engine>', f'<render_engine>{engine}</render_engine>')
    check_camera_profile(text)
    # Fixed ROS topic names require one simulation per ROS domain, even when
    # different Gazebo partitions are used. Keep the lock until launch exits.
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    key = hashlib.sha256(domain.encode()).hexdigest()[:16]
    simulation_lock = open(f'/tmp/pai_simulation_ros_{key}.lock', 'a+')
    try:
        fcntl.flock(simulation_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        simulation_lock.close()
        raise RuntimeError('このROS領域ではsimが起動済みです。./run.sh status --watchで確認してください。'
                           ' 切り替える場合は既存のsimをCtrl+Cで終了してください。') from exc
    fd, path = tempfile.mkstemp(prefix='pai_world_', suffix='.sdf')
    with os.fdopen(fd, 'w') as out:
        out.write(text)
    server = ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '3', path], output='screen')
    gui = ExecuteProcess(cmd=['ign', 'gazebo', '-g', '--render-engine', engine], output='screen',
                         condition=IfCondition(LaunchConfiguration('gui')))
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_bridge',
                  parameters=[{'config_file': str(share / 'config/bridge.yaml')}], output='screen')
    guard = Node(package='pai_robot', executable='velocity_guard', parameters=[{'use_sim_time': True}], output='screen')
    control = Node(package='pai_robot', executable='visual_robot', output='screen',
                   parameters=[{'use_sim_time': True, 'log_dir': LaunchConfiguration('log_dir'),
                                'ai_timeout': float(LaunchConfiguration('ai_timeout').perform(context)),
                                'max_actions': int(LaunchConfiguration('max_actions').perform(context)),
                                'max_sim_seconds': float(LaunchConfiguration('max_sim_seconds').perform(context)),
                                'max_wall_seconds': float(LaunchConfiguration('max_wall_seconds').perform(context))}],
                   condition=IfCondition(LaunchConfiguration('controller')))
    events = [RegisterEventHandler(OnProcessExit(target_action=process,
              on_exit=[EmitEvent(event=Shutdown(reason='A core process exited; stopping simulation'))]))
              for process in (server, bridge, guard, control)]
    def cleanup(context, *args, **kwargs):
        Path(path).unlink(missing_ok=True)
        simulation_lock.close()
        return []
    events.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    return events + [server, gui, bridge, guard, control]


def generate_launch_description():
    share = Path(get_package_share_directory('pai_robot'))
    defaults = {'world': str(share / 'worlds/visual_search.sdf'), 'gui': 'true', 'controller': 'true',
                'render_engine': 'ogre2', 'log_dir': os.environ.get('PAI_LOG_DIR', '/workspace/logs'),
                'ai_timeout': '90.0', 'max_actions': '80', 'max_sim_seconds': '900.0', 'max_wall_seconds': '1200.0'}
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
                             + [OpaqueFunction(function=start)])
