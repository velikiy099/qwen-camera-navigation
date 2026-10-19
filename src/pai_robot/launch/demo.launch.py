"""Fortress server + optional GUI + bridge + independent safety guard + controller."""
import os
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def start(context):
    share = Path(get_package_share_directory('pai_robot'))
    engine = LaunchConfiguration('render_engine').perform(context)
    if engine not in ('ogre', 'ogre2'):
        raise ValueError('render_engine must be ogre or ogre2')
    world = Path(LaunchConfiguration('world').perform(context))
    text = world.read_text()
    # Change BOTH sensor rendering and GUI renderer together, without editing original SDF.
    text = text.replace('<render_engine>ogre2</render_engine>', f'<render_engine>{engine}</render_engine>')
    fd, path = tempfile.mkstemp(prefix='pai_world_', suffix='.sdf')
    with os.fdopen(fd, 'w') as out: out.write(text)
    server = ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '3', path], output='screen')
    gui = ExecuteProcess(cmd=['ign', 'gazebo', '-g', '--render-engine', engine], output='screen',
                         condition=IfCondition(LaunchConfiguration('gui')))
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_bridge',
                  parameters=[{'config_file': str(share / 'config/bridge.yaml')}], output='screen')
    guard = Node(package='pai_robot', executable='velocity_guard', parameters=[{'use_sim_time': True}], output='screen')
    control = Node(package='pai_robot', executable='visual_robot', output='screen',
                   parameters=[{'use_sim_time': True, 'vision_mode': LaunchConfiguration('vision_mode'),
                                'log_dir': LaunchConfiguration('log_dir')}],
                   condition=IfCondition(LaunchConfiguration('controller')))
    # Never leave the simulation receiving the last command if a core process dies.
    events = [RegisterEventHandler(OnProcessExit(target_action=p,
              on_exit=[EmitEvent(event=Shutdown(reason='A core process exited; shutting down simulation'))]))
              for p in (server, bridge, guard, control)]
    return events + [server, gui, bridge, guard, control]

def generate_launch_description():
    share = Path(get_package_share_directory('pai_robot'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(share/'worlds/visual_search.sdf')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('controller', default_value='true'),
        DeclareLaunchArgument('vision_mode', default_value='ai'),
        DeclareLaunchArgument('render_engine', default_value='ogre2'),
        DeclareLaunchArgument('log_dir', default_value=os.environ.get('PAI_LOG_DIR', '/workspace/logs')),
        OpaqueFunction(function=start)])
