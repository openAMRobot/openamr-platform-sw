"""Base and lift software-in-the-loop on Linux virtual CAN.

Creates the vcan interfaces, starts Gazebo with the SIL model, the device emulators and the
ROS-Gazebo bridge, and either the host test client that plays the STM32 CAN role
(stm32:=host, default) or nothing in its place (stm32:=renode) so a Renode machine can
attach to the same interfaces, or the SIMULATION ONLY reference process of openamr-platform-fw
tests/sim_reference at the pinned commit SIMREF_FW_SHA (stm32:=simref) with its bridge node.
Development evidence only.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

STM32_CHOICES = ('host', 'renode', 'simref')

# stm32:=simref runs openamr-platform-fw tests/sim_reference built at exactly this commit of
# branch sim/reference-controller (taken after its hosted checks were green). The binary must
# carry this SHA in simref_process.sha next to it (scripts/build_simref.sh writes it).
SIMREF_FW_REPO = 'https://github.com/openAMRobot/openamr-platform-fw.git'
SIMREF_FW_SHA = 'b6c890993bfdbc30180a7f04e5d514dce7294f19'
SIMREF_DEFAULT_BIN = os.path.join(os.path.expanduser('~'), '.cache', 'openamrobot_simref',
                                  SIMREF_FW_SHA, 'simref_process')


def simref_binary_problem(path):
    """None if path is the simref_process built at SIMREF_FW_SHA, else the reason."""
    if not os.path.isfile(path) or not os.access(path, os.X_OK):
        return 'simref_process not found at %s (run scripts/build_simref.sh)' % path
    stamp = path + '.sha'
    try:
        with open(stamp, 'r', encoding='utf-8') as f:
            sha = f.read().strip()
    except OSError:
        return 'no %s next to the binary' % stamp
    if sha != SIMREF_FW_SHA:
        return 'simref_process was built at %s, the launch file pins %s' % (sha, SIMREF_FW_SHA)
    return None


def _true(text):
    return str(text).lower() in ('true', '1', 'yes')


def simref_actions(config_file, simref_bin, gazebo):
    """The reference process in the STM32 role and its bridge node (stm32:=simref)."""
    with open(config_file, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    buses, d = cfg['buses'], cfg['drive']
    sock = (cfg.get('simref') or {}).get('socket', '/tmp/openamrobot_simref.sock')
    problem = simref_binary_problem(simref_bin)
    if problem:
        return [LogInfo(msg='BLOCKED: stm32:=simref: ' + problem),
                EmitEvent(event=Shutdown(reason='BLOCKED: ' + problem))]
    return [
        LogInfo(msg='stm32:=simref: SIMULATION ONLY reference at openamr-platform-fw %s '
                    'takes the STM32 role. Not safety evidence.' % SIMREF_FW_SHA),
        # respawn: a restart of the reference process is part of the tests (no auto enable)
        ExecuteProcess(
            name='simref_process',
            cmd=[simref_bin, '--can-drive', buses['drive']['interface'],
                 '--can-aux', buses['aux']['interface'], '--sock', sock,
                 '--wheel-radius', str(d['wheel_radius']),
                 '--wheel-separation', str(d['wheel_separation']),
                 '--left-sign', str(d['left_sign']), '--right-sign', str(d['right_sign'])],
            output='screen', respawn=True, respawn_delay=1.0),
        Node(package='openamrobot_sil', executable='simref_bridge', output='screen',
             parameters=[{'config_file': config_file, 'use_sim_time': gazebo}]),
    ]


def can_nodes(config_file, stm32, gazebo, simref_bin=SIMREF_DEFAULT_BIN):
    """The nodes that open vcan sockets."""
    params = [{'config_file': config_file}]
    nodes = [
        Node(package='openamrobot_sil', executable='zlac8015d_emulator', output='screen',
             parameters=params),
        Node(package='openamrobot_sil', executable='ctl017_emulator', output='screen',
             parameters=params + [{'gazebo': gazebo}]),
        Node(package='openamrobot_sil', executable='daly_bms_emulator', output='screen',
             parameters=params),
    ]
    if stm32 == 'host':
        nodes.append(Node(package='openamrobot_sil', executable='stm32_host_client',
                          output='screen', parameters=params + [{'use_sim_time': gazebo}]))
    elif stm32 == 'simref':
        nodes += simref_actions(config_file, simref_bin, gazebo)
    else:
        nodes.append(LogInfo(msg=(
            'stm32:=renode: no host test client. Attach the Renode machine to the vcan '
            'interfaces (docs/architecture/sil-topology.md, section 4).')))
    return nodes


def gazebo_actions(sil_dir, model, world, gui):
    description_dir = get_package_share_directory('openamrobot_description')
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)
    gz_args = ('-r -v 2 ' if gui else '-r -s -v 2 ') + world
    return [
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=':'.join([os.path.join(sil_dir, 'worlds'),
                            str(Path(description_dir).parent.resolve())])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'openamrobot', '-topic', '/robot_description',
                        '-x', '0', '-y', '0', '-z', '0.2']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(sil_dir, 'config',
                                                      'gz_bridge_sil.yaml')}]),
    ]


def build_actions(stm32, create_vcan, gazebo, gui, config_file, model, world,
                  simref_bin=SIMREF_DEFAULT_BIN, nav2=False):
    """All actions of the launch file for the given argument values."""
    if stm32 not in STM32_CHOICES:
        raise RuntimeError('stm32 must be one of %s, got %r' % (STM32_CHOICES, stm32))
    sil_dir = get_package_share_directory('openamrobot_sil')
    if stm32 == 'simref' and config_file == os.path.join(sil_dir, 'config', 'sil.yaml'):
        config_file = os.path.join(sil_dir, 'config', 'sil_simref.yaml')
    with open(config_file, 'r', encoding='utf-8') as f:
        buses = yaml.safe_load(f)['buses']
    interfaces = [buses[k]['interface'] for k in ('drive', 'bms', 'aux')]
    actions = []
    nodes = can_nodes(config_file, stm32, gazebo, simref_bin)
    if create_vcan:
        vcan_up = ExecuteProcess(
            cmd=['sh', os.path.join(sil_dir, 'scripts', 'sil_vcan.sh'), 'up'] + interfaces,
            output='screen')

        def after_vcan(event, context):
            if event.returncode == 0:
                return nodes
            return [LogInfo(msg='BLOCKED: vcan interfaces are not available, nothing started'),
                    EmitEvent(event=Shutdown(reason='BLOCKED: vcan unavailable'))]

        actions += [vcan_up,
                    RegisterEventHandler(OnProcessExit(target_action=vcan_up,
                                                       on_exit=after_vcan))]
    else:
        actions += nodes
    if gazebo:
        actions += gazebo_actions(sil_dir, model, world, gui)
    if nav2:
        if stm32 != 'simref':
            raise RuntimeError('nav2:=true is only offered with stm32:=simref')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sil_dir, 'launch', 'simref_nav2.launch.py')),
            launch_arguments={'use_sim_time': 'true' if gazebo else 'false'}.items()))
    return actions


def _setup(context):
    value = {name: LaunchConfiguration(name).perform(context)
             for name in ('stm32', 'create_vcan', 'gazebo', 'gui', 'config_file', 'model',
                          'world', 'simref_bin', 'nav2')}
    return build_actions(value['stm32'], _true(value['create_vcan']), _true(value['gazebo']),
                         _true(value['gui']), value['config_file'], value['model'],
                         value['world'], value['simref_bin'], _true(value['nav2']))


def generate_launch_description():
    sil_dir = get_package_share_directory('openamrobot_sil')
    return LaunchDescription([
        DeclareLaunchArgument(
            'stm32', default_value='host', choices=list(STM32_CHOICES),
            description='host: start the host test client that plays the STM32 CAN role. '
                        'renode: start nothing in its place, a Renode machine attaches. '
                        'simref: the SIMULATION ONLY reference process of openamr-platform-fw '
                        'at SIMREF_FW_SHA takes the STM32 role, fed by the simref bridge.'),
        DeclareLaunchArgument(
            'simref_bin', default_value=os.environ.get('SIMREF_BIN', SIMREF_DEFAULT_BIN),
            description='stm32:=simref: path of simref_process built at SIMREF_FW_SHA.'),
        DeclareLaunchArgument(
            'nav2', default_value='false',
            description='stm32:=simref only: start Nav2 (simref_nav2.launch.py) in front of '
                        'the bridge, so /cmd_vel is the collision_monitor output.'),
        DeclareLaunchArgument(
            'create_vcan', default_value='true',
            description='Create the vcan interfaces (needs root or passwordless sudo).'),
        DeclareLaunchArgument(
            'gazebo', default_value='true',
            description='Start Gazebo, the model and the bridge. false: emulators only, '
                        'with their internal plant models.'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Start the Gazebo GUI.'),
        DeclareLaunchArgument(
            'config_file', default_value=os.path.join(sil_dir, 'config', 'sil.yaml'),
            description='SIL configuration (buses, node IDs, drive, lift, BMS).'),
        DeclareLaunchArgument(
            'model', default_value=os.path.join(sil_dir, 'urdf', 'sil_robot.urdf.xacro'),
            description='Robot xacro. It must hold the wheel joints, a prismatic lift joint '
                        'and the two SIL Gazebo plugins of the default file.'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(sil_dir, 'worlds', 'sil_flat.sdf'),
            description='Gazebo world file.'),
        OpaqueFunction(function=_setup),
    ])
