"""Introspect drivers.launch.py: LiDAR model selection, without starting any process."""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'drivers.launch.py')

EXPECTED = {
    's3': {'serial_baudrate': 1000000, 'scan_mode': 'DenseBoost'},
    'a1': {'serial_baudrate': 115200, 'scan_mode': 'Standard'},
}


def _description():
    spec = importlib.util.spec_from_file_location('drivers_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _context(**configs):
    context = LaunchContext()
    context.launch_configurations.update(
        {'lidar_port': '/dev/null-test', 'teensy_port': '/dev/null-test'})
    context.launch_configurations.update(configs)
    return context


def _arg(name):
    args = [e for e in _description().entities
            if isinstance(e, DeclareLaunchArgument) and e.name == name]
    assert len(args) == 1
    return args[0]


def _active_lidar_nodes(model):
    context = _context(lidar_model=model)
    nodes = [e for e in _description().entities if isinstance(e, Node)]
    return [n for n in nodes if n.condition is None or n.condition.evaluate(context)], context


def test_lidar_model_argument():
    arg = _arg('lidar_model')
    assert arg.default_value[0].text == 's3'
    assert sorted(arg.choices) == ['a1', 's3']


def test_lidar_port_default_unchanged():
    assert _arg('lidar_port').default_value[0].text == (
        '/dev/serial/by-id/'
        'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')


def test_invalid_lidar_model_rejected():
    arg = _arg('lidar_model')
    with pytest.raises(Exception):
        arg.execute(_context(lidar_model='hokuyo'))


@pytest.mark.parametrize('model', sorted(EXPECTED))
def test_one_sllidar_node_per_model(model):
    nodes, context = _active_lidar_nodes(model)
    assert len(nodes) == 1
    node = nodes[0]
    assert node.node_package == 'sllidar_ros2'
    assert node.node_executable == 'sllidar_node'
    params = {}
    for entry in evaluate_parameters(context, node._Node__parameters):
        params.update(entry)
    assert params['channel_type'] == 'serial'
    assert params['serial_port'] == '/dev/null-test'
    assert params['frame_id'] == 'lidar_link'
    assert params['angle_compensate'] is True
    assert params['serial_baudrate'] == EXPECTED[model]['serial_baudrate']
    assert params['scan_mode'] == EXPECTED[model]['scan_mode']
    # sllidar_node publishes the relative topic 'scan' -> /scan; no remapping expected.
    assert not node.expanded_remapping_rules and not node._Node__remappings
