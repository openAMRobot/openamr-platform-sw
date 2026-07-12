"""Validate package resources without starting robot processes."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchDescription


SRC = Path(__file__).parents[2]


def test_all_package_manifests_are_well_formed_and_named_consistently():
    manifests = sorted(SRC.glob('*/package.xml'))
    assert manifests
    for manifest in manifests:
        root = ET.parse(manifest).getroot()
        assert root.findtext('name') == manifest.parent.name


def test_all_yaml_configuration_is_well_formed():
    yaml_files = sorted(SRC.glob('*/config/*.yaml')) + sorted(SRC.glob('*/launch/*.yml'))
    assert yaml_files
    for yaml_file in yaml_files:
        with yaml_file.open(encoding='utf-8') as stream:
            yaml.safe_load(stream)


@pytest.mark.parametrize('launch_file', sorted(SRC.glob('*/launch/*.py')), ids=lambda p: p.name)
def test_python_launch_files_load(launch_file):
    spec = importlib.util.spec_from_file_location(
        f'launch_test_{launch_file.parent.parent.name}_{launch_file.stem}', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert isinstance(description, LaunchDescription)
    assert description.entities


def test_gazebo_world_and_robot_descriptions_are_well_formed_xml():
    resources = [
        SRC / 'openamrobot_gazebo/worlds/walled_world.sdf',
        SRC / 'openamrobot_description/urdf/robot.sdf',
    ]
    for resource in resources:
        ET.parse(resource)
