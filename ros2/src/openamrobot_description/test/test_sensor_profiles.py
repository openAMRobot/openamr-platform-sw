# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
Checks of config/sensor_profiles.yaml against config/sensor_frames.yaml.

The profile file holds the behaviour of every base sensor (range, rate, noise) with the
source of each number. These tests keep it honest: same frame ids as sensor_frames.yaml,
no number without a source, no guessed number marked as unknown, and a named owner and
missing document for every unknown value.
"""

import os
import re

import pytest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, os.pardir, 'config')
STATUSES = {'datasheet', 'measured', 'estimated', 'unknown'}
SHA256 = re.compile(r'\b[0-9a-f]{64}\b')
URL = re.compile(r'https?://\S+')
# Frames and parts the profile must cover: every frame of sensor_frames.yaml.
EXPECTED_PART = {
    'lidar_link': 'RPLIDAR S3',
    'base_camera_link': 'Gemini 336L',
    'imu_link': 'ICM-42688-P',
    'us_front_left': 'MB7060',
    'us_front_right': 'MB7060',
    'us_rear_left': 'MB7060',
    'us_rear_right': 'MB7060',
    'tof_front': 'TF-Luna',
    'tof_rear': 'TF-Luna',
}


def load(name):
    with open(os.path.join(CONFIG, name)) as f:
        return yaml.safe_load(f)


PROFILES = load('sensor_profiles.yaml')
FRAMES = load('sensor_frames.yaml')['frames']


def leaves(node, path=()):
    """Yield (path, leaf) for every mapping that carries a 'value' key."""
    if isinstance(node, dict):
        if 'value' in node:
            yield path, node
            return
        for key, child in node.items():
            yield from leaves(child, path + (str(key),))


def all_leaves():
    for frame, entry in PROFILES['frames'].items():
        for path, leaf in leaves(entry['channels'], (frame,)):
            yield '.'.join(path), leaf


def test_every_profile_frame_exists_in_sensor_frames():
    unknown = sorted(set(PROFILES['frames']) - set(FRAMES))
    assert not unknown, 'frames not in sensor_frames.yaml: %s' % unknown


def test_every_sensor_frame_has_a_profile():
    missing = sorted(set(FRAMES) - set(PROFILES['frames']))
    assert not missing, 'sensor_frames.yaml frames without a profile: %s' % missing


def test_expected_parts_per_frame():
    assert set(PROFILES['frames']) == set(EXPECTED_PART)
    for frame, part in EXPECTED_PART.items():
        assert PROFILES['frames'][frame]['part'] == part, frame
        assert part in FRAMES[frame]['sensor'], (frame, FRAMES[frame]['sensor'])
        assert part in PROFILES['frames'][frame]['sensor'], frame


def test_profile_channels_are_present():
    for frame, entry in PROFILES['frames'].items():
        assert entry['channels'], frame
    channels = {f: set(e['channels']) for f, e in PROFILES['frames'].items()}
    assert channels['base_camera_link'] == {'depth', 'color'}
    assert channels['imu_link'] == {'gyro', 'accel', 'timestamp_skew'}
    for frame in ('lidar_link', 'us_front_left', 'tof_front'):
        assert channels[frame], frame
    assert {'noise_density', 'bias_stability', 'odr'} <= set(
        PROFILES['frames']['imu_link']['channels']['gyro'])
    assert {'noise_density', 'bias_stability', 'odr'} <= set(
        PROFILES['frames']['imu_link']['channels']['accel'])


def test_every_leaf_has_the_required_fields():
    count = 0
    for name, leaf in all_leaves():
        count += 1
        assert {'value', 'unit', 'source', 'status'} <= set(leaf), name
        assert leaf['status'] in STATUSES, (name, leaf['status'])
        assert isinstance(leaf['unit'], str) and leaf['unit'], name
    assert count > 40


@pytest.mark.parametrize('name,leaf', list(all_leaves()))
def test_non_null_value_has_a_source(name, leaf):
    if leaf['value'] is None:
        return
    assert isinstance(leaf['value'], (int, float)), name
    assert not isinstance(leaf['value'], bool), name
    source = leaf['source']
    assert isinstance(source, str) and source.strip(), 'no source for %s' % name
    assert leaf['status'] != 'unknown', '%s has a value but status unknown' % name
    assert SHA256.search(source) or URL.search(source), \
        'source of %s names neither a SHA-256 nor a URL: %s' % (name, source)


@pytest.mark.parametrize('name,leaf', list(all_leaves()))
def test_unknown_is_null_and_names_what_is_needed(name, leaf):
    if leaf['status'] == 'unknown':
        assert leaf['value'] is None, '%s is unknown but has a value' % name
        assert leaf['source'] is None, name
        assert str(leaf.get('needed', '')).strip(), 'no "needed" for %s' % name
        assert str(leaf.get('owner', '')).strip(), 'no "owner" for %s' % name
    else:
        assert leaf['value'] is not None, '%s has status %s but no value' % (
            name, leaf['status'])


def test_datasheet_sources_are_the_listed_documents():
    documents = {' '.join(text.split()) for text in PROFILES['documents'].values()}
    for name, leaf in all_leaves():
        if leaf['status'] == 'datasheet':
            assert ' '.join(leaf['source'].split()) in documents, name


def test_documents_carry_a_hash_or_a_vendor_url():
    for key, text in PROFILES['documents'].items():
        assert SHA256.search(text) or URL.search(text), key


def test_ranges_are_ordered():
    for frame, entry in PROFILES['frames'].items():
        for channel, values in entry['channels'].items():
            low = values.get('range_min', {}).get('value')
            highs = [v['value'] for k, v in values.items()
                     if k.startswith('range_max') and v['value'] is not None]
            for high in highs:
                if low is not None:
                    assert low < high, (frame, channel)


def test_values_agree_with_the_pinned_datasheet_numbers():
    f = PROFILES['frames']
    assert f['lidar_link']['channels']['scan']['range_min']['value'] == 0.05
    assert f['tof_front']['channels']['range']['range_min']['value'] == 0.20
    assert f['us_front_left']['channels']['range']['range_max']['value'] == 7.65
    depth = f['base_camera_link']['channels']['depth']
    color = f['base_camera_link']['channels']['color']
    assert (depth['fov_horizontal']['value'], depth['fov_vertical']['value']) == (90.0, 65.0)
    assert (color['fov_horizontal']['value'], color['fov_vertical']['value']) == (94.0, 68.0)
    assert depth['range_min']['value'] == 0.17


def test_same_part_has_the_same_profile():
    f = PROFILES['frames']
    for group in (('us_front_left', 'us_front_right', 'us_rear_left', 'us_rear_right'),
                  ('tof_front', 'tof_rear')):
        for frame in group[1:]:
            assert f[frame]['channels'] == f[group[0]]['channels'], frame


def test_no_private_material_in_the_profile_file():
    with open(os.path.join(CONFIG, 'sensor_profiles.yaml')) as f:
        text = f.read()
    for pattern in (r'drive\.google', r'docs\.google', r'slack\.com', r'@[a-z0-9-]+\.[a-z]+',
                    r'\$\s?\d', r'USD', r'EUR'):
        assert not re.search(pattern, text, re.IGNORECASE), pattern
