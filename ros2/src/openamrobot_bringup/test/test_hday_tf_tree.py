# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
TF tree of the H-day legacy profile: one parent per frame, connected, no cycles.

Built from the static_transform_publisher nodes of real_bringup.launch.py plus the EKF
transform odom -> base_link (ekf.yaml publish_tf, world_frame odom). map -> odom comes from
AMCL once the operator sets the pose; it is not a launch-time transform.
"""

import os

from launch import LaunchContext
from launch.utilities import perform_substitutions
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _real_bringup_nodes():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'real_bringup_launch', os.path.join(PKG, 'launch', 'real_bringup.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda pkg: os.path.join('/fake-share', pkg)
    return module.generate_launch_description().entities


def _static_edges():
    edges = []
    for entity in _real_bringup_nodes():
        arguments = getattr(entity, '_Node__arguments', None)
        if not arguments or getattr(entity, '_Node__node_executable', None) is None:
            continue
        words = [perform_substitutions(LaunchContext(), a) if not isinstance(a, str) else a
                 for a in arguments]
        # arguments are lists of substitutions, flatten the text
        flat = []
        for w in words:
            flat.append(w if isinstance(w, str) else ''.join(map(str, w)))
        if '--frame-id' in flat:
            parent = flat[flat.index('--frame-id') + 1]
            child = flat[flat.index('--child-frame-id') + 1]
            edges.append((parent, child))
    return edges


def _ekf_edge():
    with open(os.path.join(PKG, 'config', 'ekf.yaml'), encoding='utf-8') as f:
        cfg = yaml.safe_load(f)['ekf_filter_node']['ros__parameters']
    assert cfg['publish_tf'] is True
    assert cfg['world_frame'] == cfg['odom_frame']
    return cfg['odom_frame'], cfg['base_link_frame']


def test_tree_is_valid():
    edges = _static_edges() + [_ekf_edge()]
    assert len(edges) >= 6
    parents = {}
    for parent, child in edges:
        assert child not in parents, '%s has two parents: %s and %s' % (
            child, parents.get(child), parent)
        parents[child] = parent
    # every frame reaches the root without a cycle
    roots = set()
    for frame in parents:
        seen = [frame]
        while seen[-1] in parents:
            nxt = parents[seen[-1]]
            assert nxt not in seen, 'cycle through ' + nxt
            seen.append(nxt)
        roots.add(seen[-1])
    assert roots == {'odom'}, roots


def test_required_frames_present():
    edges = {c: p for p, c in _static_edges() + [_ekf_edge()]}
    for frame in ('base_link', 'lidar_link', 'imu_link', 'base_footprint'):
        assert frame in edges, frame
    assert edges['base_link'] == 'odom'
    assert edges['lidar_link'] == 'base_link'


def test_scan_frame_matches_a_published_frame():
    drivers = os.path.join(os.path.dirname(PKG), 'openamrobot_drivers', 'launch',
                           'drivers.launch.py')
    with open(drivers, encoding='utf-8') as f:
        assert "'frame_id': 'lidar_link'" in f.read()
    assert 'lidar_link' in {c: p for p, c in _static_edges()}


def test_no_robot_state_publisher_would_duplicate_static_frames():
    # The legacy profile publishes lidar_link itself; a URDF robot_state_publisher on top
    # would give lidar_link two parents. The H-day launch must not include the description.
    with open(os.path.join(PKG, 'launch', 'hday_legacy.launch.py'), encoding='utf-8') as f:
        code = ''.join(line for line in f if not line.lstrip().startswith('#'))
    body = code.split('"""', 2)[2]
    assert 'openamrobot_description' not in body
    assert 'robot_state_publisher' not in body
