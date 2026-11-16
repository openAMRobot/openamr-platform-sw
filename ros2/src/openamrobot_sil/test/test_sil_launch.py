"""Introspection of sil.launch.py without starting anything."""
import importlib.util
import os

from launch_ros.actions import Node
import pytest

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


def test_renode_mode_starts_no_host_client(sil_launch):
    nodes = sil_launch.can_nodes(CONFIG, 'renode', True)
    assert executables(nodes) == ['ctl017_emulator', 'daly_bms_emulator', 'zlac8015d_emulator']


def test_arguments_are_declared(sil_launch):
    from launch.actions import DeclareLaunchArgument
    declared = {a.name: a for a in sil_launch.generate_launch_description().entities
                if isinstance(a, DeclareLaunchArgument)}
    assert sorted(declared) == ['config_file', 'create_vcan', 'gazebo', 'gui', 'model', 'stm32',
                                'world']
    assert declared['stm32'].choices == ['host', 'renode']


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
