"""sim_bringup_launch.py must include map_saver_launch.py so /map_saver/save_map is up."""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch', 'sim_bringup_launch.py')


def _description(tmp_path):
    spec = importlib.util.spec_from_file_location('sim_bringup_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # sim_bringup_launch.py only uses the share dir to build path strings (no file I/O at
    # generation time) - a fake path is enough.
    module.get_package_share_directory = lambda pkg: str(tmp_path / pkg)
    return module.generate_launch_description()


def _location(include):
    source = include.launch_description_source
    return perform_substitutions(LaunchContext(), source._LaunchDescriptionSource__location)


def test_includes_map_saver(tmp_path):
    description = _description(tmp_path)
    matches = [
        e for e in description.entities
        if isinstance(e, IncludeLaunchDescription)
        and _location(e).endswith(os.path.join('launch', 'map_saver_launch.py'))
    ]
    assert len(matches) == 1, (
        'sim_bringup_launch.py must include map_saver_launch.py exactly once so '
        f'/map_saver/save_map is available; found {len(matches)} matching includes.')
