"""Introspection of sil.launch.py without starting anything."""
import importlib.util
import os

from launch_ros.actions import Node
import pytest
import yaml

from conftest import PACKAGE_DIR

CONFIG = os.path.join(PACKAGE_DIR, 'config', 'sil.yaml')


@pytest.fixture(scope='module')
def sil_launch():
    spec = importlib.util.spec_from_file_location(
        'sil_launch', os.path.join(PACKAGE_DIR, 'launch', 'sil.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def executables(actions):
    return sorted(a.node_executable for a in actions if isinstance(a, Node))


def test_host_mode_starts_the_three_emulators_and_the_host_client(sil_launch):
    nodes = sil_launch.can_nodes(CONFIG, 'host', True)
    assert executables(nodes) == ['ctl017_emulator', 'daly_bms_emulator', 'stm32_host_client',
                                  'zlac8015d_emulator']


def test_renode_mode_starts_renode_and_the_audit_but_no_host_client(sil_launch):
    nodes = sil_launch.can_nodes(CONFIG, 'renode', True)
    assert executables(nodes) == ['ctl017_emulator', 'daly_bms_emulator', 'drive_bus_audit',
                                  'renode_firmware', 'zlac8015d_emulator']


def _text(value):
    if isinstance(value, tuple) and all(hasattr(v, 'text') for v in value):
        return yaml.safe_load(''.join(v.text for v in value))  # launch stores YAML text
    return value


def node_params(nodes, executable):
    """Parameters given to a Node action, as plain values (launch normalises them)."""
    node = [a for a in nodes if isinstance(a, Node) and a.node_executable == executable][0]
    out = {}
    for group in node._Node__parameters:
        for key, value in group.items():
            out[_text(key)] = _text(value)
    return out


def test_renode_readonly_mode_adds_the_command_only_host_client(sil_launch):
    nodes = sil_launch.can_nodes(CONFIG, 'renode_readonly', True, fw_args())
    assert executables(nodes) == ['ctl017_emulator', 'daly_bms_emulator', 'drive_bus_audit',
                                  'renode_firmware', 'stm32_host_client', 'zlac8015d_emulator']
    host = node_params(nodes, 'stm32_host_client')
    assert host['drive_mode'] == 'command_only' and host['wait_for_audit'] is True
    drive = node_params(nodes, 'zlac8015d_emulator')
    assert drive['rpdo'] is True and drive['stored_link_loss'] is True
    fw = node_params(nodes, 'renode_firmware')
    assert fw['can_link'] == 'socketcan' and fw['wait_for_audit'] is True
    # renode_readonly runs the micro-ROS image; the node starts the agent on its USART3.
    assert fw['transport'] == 'uros' and fw['elf'] == fw_args()['fw_elf_uros']
    assert fw['uros_agent'] == fw_args()['uros_agent']


def fw_args():
    return {'renode': 'r', 'fw_elf': '/e/console.elf', 'fw_elf_uros': '/e/uros.elf',
            'uros_agent': '/a/micro_ros_agent', 'fw_dir': '/fw', 'renode_work_dir': '/w',
            'capture_file': '/c'}


def test_renode_mode_runs_the_console_image(sil_launch):
    fw = node_params(sil_launch.can_nodes(CONFIG, 'renode', True, fw_args()), 'renode_firmware')
    assert fw['transport'] == 'console' and fw['elf'] == '/e/console.elf'


def test_host_mode_drive_emulator_has_no_rpdo(sil_launch):
    drive = node_params(sil_launch.can_nodes(CONFIG, 'host', True), 'zlac8015d_emulator')
    assert drive['rpdo'] is False and drive['stored_link_loss'] is False


def test_launch_file_records_the_pinned_firmware(sil_launch):
    from openamrobot_sil import renode_fw
    assert sil_launch.FW_COMMIT == renode_fw.FW_COMMIT == '763bdd3ffe4a5a582445f72d4845d8a0eedeba3c'
    assert sil_launch.FW_PRESET == renode_fw.FW_PRESET == 'RenodeTest'
    assert sil_launch.FW_PRESET_UROS == renode_fw.FW_PRESET_UROS == 'TelemetryUrosSerialSimBench'


def test_arguments_are_declared(sil_launch):
    from launch.actions import DeclareLaunchArgument
    declared = {a.name: a for a in sil_launch.generate_launch_description().entities
                if isinstance(a, DeclareLaunchArgument)}
    assert sorted(declared) == ['capture_file', 'config_file', 'create_vcan', 'fw_dir', 'fw_elf',
                                'fw_elf_uros', 'gazebo', 'gui', 'model', 'renode',
                                'renode_work_dir', 'stm32', 'uros_agent', 'world']
    assert declared['stm32'].choices == ['host', 'renode', 'renode_readonly']


def test_vcan_creation_names_the_three_buses(sil_launch):
    from launch.actions import ExecuteProcess
    actions = sil_launch.build_actions('renode', True, False, False, CONFIG, '', '')
    process = [a for a in actions if isinstance(a, ExecuteProcess)]
    assert len(process) == 1
    assert [str(c[0].text) for c in process[0].cmd][-4:] == ['up', 'vcan_drive', 'vcan_bms',
                                                           'vcan_aux']
    assert executables(actions) == []  # nodes start only after the interfaces exist


def test_unknown_stm32_value_is_refused(sil_launch):
    with pytest.raises(RuntimeError):
        sil_launch.build_actions('teensy', False, False, False, CONFIG, '', '')
