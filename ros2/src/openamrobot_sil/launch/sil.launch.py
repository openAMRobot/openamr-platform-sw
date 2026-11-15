"""Base and lift software-in-the-loop on Linux virtual CAN.

Creates the vcan interfaces, starts Gazebo with the SIL model, the device emulators and the
ROS-Gazebo bridge, and in the STM32 slot one of:

  stm32:=host             the host test client plays the STM32 CAN role (default)
  stm32:=renode           the platform-fw telemetry image (preset RenodeTest, JSON console) runs in
                          Renode; FDCAN1, FDCAN2 and FDCAN3 on vcan_drive, vcan_bms and vcan_aux
                          through Renode's SocketCAN bridges. Nobody commands the drive.
  stm32:=renode_readonly  the micro-ROS image (preset TelemetryUrosSerialSimBench, same commit) on
                          the same three buses, plus a micro-ROS agent on its USART3 (a Renode
                          pseudo-terminal; serial transport, simulation and bench only), plus the
                          host test client, which sends the drive commands as SIL receive PDOs
                          and no SDO frame (the SDO uploads of 606Ch, 6041h, 603Fh and 2026h
                          belong to the firmware alone) and plays the CAN3 and CAN2 roles (NMT
                          start and commands of the lift, Daly polls) the image never plays.

In both Renode modes the drive bus audit captures vcan_drive for the whole run. The firmware
is a binary built elsewhere (scripts/sil_renode_firmware.sh); this file records which one.
Firmware-in-the-loop milestone for telemetry, development evidence only.
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
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

STM32_CHOICES = ('host', 'renode', 'renode_readonly')
RENODE_MODES = ('renode', 'renode_readonly')

# The firmware image that stm32:=renode and stm32:=renode_readonly run: openamr-platform-fw at
# this commit (head of fw/telemetry-uros, platform-fw PR #55, pinned after its hosted checks were
# green on it), CMake presets RenodeTest (renode) and TelemetryUrosSerialSimBench
# (renode_readonly) of boards/stm32/telemetry_image (same values as openamrobot_sil/renode_fw.py,
# checked by test/test_sil_launch.py). The node renode_firmware refuses an image whose banner,
# or in renode_readonly whose /base_fw/status, does not carry this commit and "params":"test".
FW_COMMIT = '763bdd3ffe4a5a582445f72d4845d8a0eedeba3c'
FW_PRESET = 'RenodeTest'
FW_PRESET_UROS = 'TelemetryUrosSerialSimBench'


def _true(text):
    return str(text).lower() in ('true', '1', 'yes')


def renode_nodes(config_file, renode, uros=False):
    """Drive bus audit and Renode firmware node. renode: dict of the renode_* arguments.

    uros: the micro-ROS image and its agent (stm32:=renode_readonly)."""
    sil_dir = get_package_share_directory('openamrobot_sil')
    audit = Node(package='openamrobot_sil', executable='drive_bus_audit', output='screen',
                 parameters=[{'config_file': config_file,
                              'capture_file': renode['capture_file']}])
    firmware = Node(
        package='openamrobot_sil', executable='renode_firmware', output='screen',
        parameters=[{'config_file': config_file, 'renode': renode['renode'],
                     'elf': renode['fw_elf_uros'] if uros else renode['fw_elf'],
                     'fw_dir': renode['fw_dir'],
                     'helpers': os.path.join(sil_dir, 'renode', 'sil_renode_helpers.py'),
                     'work_dir': renode['renode_work_dir'], 'can_link': 'socketcan',
                     'wait_for_audit': True,
                     'transport': 'uros' if uros else 'console',
                     'uros_agent': renode['uros_agent']}])
    # The firmware node exits on a wrong banner or a Renode failure: the whole run stops.
    stop = RegisterEventHandler(OnProcessExit(
        target_action=firmware,
        on_exit=[LogInfo(msg='renode_firmware ended: shutting the SIL launch down'),
                 EmitEvent(event=Shutdown(reason='renode_firmware ended'))]))
    return [audit, firmware, stop]


def can_nodes(config_file, stm32, gazebo, renode=None):
    """The nodes that open vcan sockets."""
    params = [{'config_file': config_file}]
    readonly = stm32 == 'renode_readonly'
    nodes = []
    if stm32 in RENODE_MODES:
        nodes += renode_nodes(config_file, renode or default_renode_args(), uros=readonly)
    nodes += [
        Node(package='openamrobot_sil', executable='zlac8015d_emulator', output='screen',
             parameters=params + [{'rpdo': readonly, 'stored_link_loss': readonly}]),
        Node(package='openamrobot_sil', executable='ctl017_emulator', output='screen',
             parameters=params + [{'gazebo': gazebo}]),
        Node(package='openamrobot_sil', executable='daly_bms_emulator', output='screen',
             parameters=params),
    ]
    if stm32 == 'host':
        nodes.append(Node(package='openamrobot_sil', executable='stm32_host_client',
                          output='screen', parameters=params + [{'use_sim_time': gazebo}]))
    elif readonly:
        nodes.append(Node(package='openamrobot_sil', executable='stm32_host_client',
                          output='screen',
                          parameters=params + [{'use_sim_time': gazebo,
                                                'drive_mode': 'command_only',
                                                'wait_for_audit': True}]))
    else:
        nodes.append(LogInfo(msg=(
            'stm32:=renode: firmware %s (%s) in Renode on the drive bus, no host test client. '
            'The image is read-only: nobody commands the drive.' % (FW_COMMIT, FW_PRESET))))
    return nodes


def default_renode_args():
    return {'renode': os.environ.get('SIL_RENODE', 'renode'),
            'fw_elf': os.environ.get('SIL_FW_ELF', ''),
            'fw_elf_uros': os.environ.get('SIL_FW_ELF_UROS', ''),
            'uros_agent': os.environ.get('SIL_UROS_AGENT', ''),
            'fw_dir': os.environ.get('SIL_FW_DIR', ''),
            'renode_work_dir': '/tmp/openamrobot_sil_renode',
            'capture_file': '/tmp/openamrobot_sil_vcan_drive.log'}


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


def build_actions(stm32, create_vcan, gazebo, gui, config_file, model, world, renode=None):
    """All actions of the launch file for the given argument values."""
    if stm32 not in STM32_CHOICES:
        raise RuntimeError('stm32 must be one of %s, got %r' % (STM32_CHOICES, stm32))
    sil_dir = get_package_share_directory('openamrobot_sil')
    with open(config_file, 'r', encoding='utf-8') as f:
        buses = yaml.safe_load(f)['buses']
    interfaces = [buses[k]['interface'] for k in ('drive', 'bms', 'aux')]
    actions = []
    nodes = can_nodes(config_file, stm32, gazebo, renode)
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
    return actions


RENODE_ARGS = ('renode', 'fw_elf', 'fw_elf_uros', 'uros_agent', 'fw_dir', 'renode_work_dir',
               'capture_file')


def _setup(context):
    value = {name: LaunchConfiguration(name).perform(context)
             for name in ('stm32', 'create_vcan', 'gazebo', 'gui', 'config_file', 'model',
                          'world') + RENODE_ARGS}
    actions = []
    if value['stm32'] in RENODE_MODES:
        uros = value['stm32'] == 'renode_readonly'
        actions.append(LogInfo(msg='firmware %s, preset %s, ELF %s%s' % (
            FW_COMMIT, FW_PRESET_UROS if uros else FW_PRESET,
            value['fw_elf_uros'] if uros else value['fw_elf'],
            ', micro-ROS agent %s' % value['uros_agent'] if uros else '')))
    return actions + build_actions(
        value['stm32'], _true(value['create_vcan']), _true(value['gazebo']),
        _true(value['gui']), value['config_file'], value['model'], value['world'],
        {k: value[k] for k in RENODE_ARGS})


def generate_launch_description():
    sil_dir = get_package_share_directory('openamrobot_sil')
    return LaunchDescription([
        DeclareLaunchArgument(
            'stm32', default_value='host', choices=list(STM32_CHOICES),
            description='host: the host test client plays the STM32 CAN role. renode: the '
                        'platform-fw telemetry image (%s, %s) in Renode on vcan_drive, vcan_bms '
                        'and vcan_aux, nobody commands the drive. renode_readonly: the %s image '
                        'with a micro-ROS agent on its USART3, plus the host test client '
                        'sending drive commands without any SDO.' % (FW_COMMIT[:12], FW_PRESET,
                                                                     FW_PRESET_UROS)),
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
        DeclareLaunchArgument(
            'renode', default_value=EnvironmentVariable('SIL_RENODE', default_value='renode'),
            description='Renode %s binary (renode mode). Default: $SIL_RENODE.' % '1.16.1'),
        DeclareLaunchArgument(
            'fw_elf', default_value=EnvironmentVariable('SIL_FW_ELF', default_value=''),
            description='telemetry_image.elf of preset %s at %s. Default: $SIL_FW_ELF.' % (
                FW_PRESET, FW_COMMIT)),
        DeclareLaunchArgument(
            'fw_elf_uros', default_value=EnvironmentVariable('SIL_FW_ELF_UROS', default_value=''),
            description='telemetry_image.elf of preset %s at %s (renode_readonly). Default: '
                        '$SIL_FW_ELF_UROS.' % (FW_PRESET_UROS, FW_COMMIT)),
        DeclareLaunchArgument(
            'uros_agent', default_value=EnvironmentVariable('SIL_UROS_AGENT', default_value=''),
            description='micro_ros_agent executable (renode_readonly), built by '
                        'scripts/sil_renode_firmware.sh. Default: $SIL_UROS_AGENT.'),
        DeclareLaunchArgument(
            'fw_dir', default_value=EnvironmentVariable('SIL_FW_DIR', default_value=''),
            description='openamr-platform-fw checkout at %s, for the Renode additions file. '
                        'Default: $SIL_FW_DIR.' % FW_COMMIT),
        DeclareLaunchArgument(
            'renode_work_dir', default_value='/tmp/openamrobot_sil_renode',
            description='Renode script, Renode log and USART3 console files.'),
        DeclareLaunchArgument(
            'capture_file', default_value='/tmp/openamrobot_sil_vcan_drive.log',
            description='candump -L capture of vcan_drive (renode modes).'),
        OpaqueFunction(function=_setup),
    ])
