"""Introspect real_bringup.launch.py: lidar_model is declared and passed to the drivers."""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'real_bringup.launch.py')


def _description():
    spec = importlib.util.spec_from_file_location('real_bringup_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Resolve share directories to fake paths: no installed packages needed, nothing started.
    module.get_package_share_directory = lambda pkg: os.path.join('/fake-share', pkg)
    return module.generate_launch_description()


def _location(include):
    # The public .location is only a repr string until the include is expanded.
    source = include.launch_description_source
    return perform_substitutions(LaunchContext(), source._LaunchDescriptionSource__location)


def _drivers_include(description):
    includes = [e for e in description.entities if isinstance(e, IncludeLaunchDescription)
                and _location(e) == os.path.join(
                    '/fake-share', 'openamrobot_drivers', 'launch', 'drivers.launch.py')]
    assert len(includes) == 1
    return includes[0]


def test_lidar_model_declared_a1_default():
    args = [e for e in _description().entities
            if isinstance(e, DeclareLaunchArgument) and e.name == 'lidar_model']
    assert len(args) == 1
    assert args[0].default_value[0].text == 'a1'
    assert sorted(args[0].choices) == ['a1', 's3']


def test_lidar_model_passed_to_drivers():
    include = _drivers_include(_description())
    context = LaunchContext()
    for value in ('a1', 's3'):
        context.launch_configurations['lidar_model'] = value
        passed = {
            perform_substitutions(context, normalize_to_list_of_substitutions(k)):
            perform_substitutions(context, normalize_to_list_of_substitutions(v))
            for k, v in include.launch_arguments}
        assert passed == {'lidar_model': value}
