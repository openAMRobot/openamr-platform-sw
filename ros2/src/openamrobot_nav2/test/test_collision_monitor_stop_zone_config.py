"""
Stop zone of the collision_monitor, checked against the sensor geometry (#35, #83).

Nikolai's measurement on main (platform-sw#83, #35): the only configured polygon
is `action_type: approach`, which slows the robot in proportion and never commands
zero, and its projected edge lies inside the LiDAR's blind zone. Gate A's A1
criterion needs a zone that commands zero and that a sensor can actually see.

This file holds the shipped nav2_params.yaml to those two properties. It is a
configuration check: it reads the shipped files and does not run collision_monitor.
The behavioural check (a real collision_monitor commanding zero for a scan point in
the zone) is a separate ROS test.

The numbers come from the repository, not from this file:
  - LiDAR mounting x and minimum range: config/sensor_frames.yaml (lidar_link) and the
    gpu_lidar <range><min> in urdf/gazebo_control.xacro;
  - robot footprint: the local costmap footprint in config/nav2_params.yaml.

Acceptance criteria this test encodes (proposed; the reviewers of the stop-zone PR
own them, and changing a constant below changes what the test accepts):
  1. at least one polygon has action_type "stop";
  2. its geometry is fixed (explicit points), in base_link, and it is enabled;
  3. it applies at the configured maximum forward speed;
  4. a scan point just beyond the LiDAR's minimum range, on the centre line, lies
     inside it, so the sensor can see something inside the zone at all;
  5. the same point offset to either side by the footprint's half-width lies inside
     it, so the zone covers the width the robot sweeps;
  6. min_points is low enough that an obstacle of OBSTACLE_MIN_WIDTH_M produces at
     least that many scan returns at that range.

Simulation results and configuration checks are not safety evidence.
"""
import math
from pathlib import Path
import re

import pytest
import yaml

_NAV2_ROOT = Path(__file__).resolve().parents[1]
_PARAMS_PATH = _NAV2_ROOT / 'config' / 'nav2_params.yaml'
_DESCRIPTION_ROOT = _NAV2_ROOT.parent / 'openamrobot_description'
_SENSOR_FRAMES_PATH = _DESCRIPTION_ROOT / 'config' / 'sensor_frames.yaml'
_GAZEBO_CONTROL_PATH = _DESCRIPTION_ROOT / 'urdf' / 'gazebo_control.xacro'

pytestmark = pytest.mark.skipif(
    not (_SENSOR_FRAMES_PATH.is_file() and _GAZEBO_CONTROL_PATH.is_file()),
    reason='needs openamrobot_description in the same source tree [#83]')

# Offset beyond the minimum range for the probe point (scan resolution is 0.01 m).
PROBE_MARGIN_M = 0.02
# Smallest obstacle the zone must be able to see (proposed criterion).
OBSTACLE_MIN_WIDTH_M = 0.05
# Angular resolution of the simulated scan: 360 samples over 2*pi (gazebo_control.xacro).
SCAN_ANGULAR_RESOLUTION_RAD = math.radians(1.0)


def _load_yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _lidar_x_in_base_link():
    frames = _load_yaml(_SENSOR_FRAMES_PATH)
    # The file nests sensors under a top-level key; find lidar_link wherever it is.
    stack = [frames]
    while stack:
        node = stack.pop()
        if isinstance(node, dict):
            if 'lidar_link' in node:
                return float(node['lidar_link']['xyz'][0])
            stack.extend(node.values())
    raise AssertionError('lidar_link not found in sensor_frames.yaml')


def _lidar_range_min():
    text = _GAZEBO_CONTROL_PATH.read_text(encoding='utf-8')
    start = text.index('name="lidar_sensor"')
    end = text.index('</lidar>', start)
    match = re.search(r'<range>\s*<min>\s*([0-9.]+)\s*</min>', text[start:end])
    assert match, 'no <range><min> found in the lidar_sensor block'
    return float(match.group(1))


def _footprint_points(params):
    footprint = params['local_costmap']['local_costmap']['ros__parameters']['footprint']
    return yaml.safe_load(footprint)


def _contains(polygon, x, y):
    """Ray casting; points exactly on an edge count as outside, so margins matter."""
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
    return inside


def _stop_zones(monitor):
    """
    Return (name, points, min_points, linear_range) for every stop zone.

    linear_range is (min, max) for velocity_polygon sub-polygons and None for a plain
    polygon, which applies at every speed.
    """
    zones = []
    for name in monitor.get('polygons', []):
        block = monitor.get(name, {})
        if block.get('action_type') != 'stop':
            continue
        if not block.get('enabled', True):
            continue
        min_points = int(block.get('min_points', 4))
        if block.get('type') == 'velocity_polygon':
            for sub in block.get('velocity_polygons', []):
                sub_block = block[sub]
                zones.append((
                    f'{name}.{sub}', yaml.safe_load(sub_block['points']), min_points,
                    (float(sub_block['linear_min']), float(sub_block['linear_max']))))
        elif 'points' in block:
            zones.append((name, yaml.safe_load(block['points']), min_points, None))
        else:
            zones.append((name, None, min_points, None))
    return zones


class TestStopZone:

    @classmethod
    def setup_class(cls):
        cls.params = _load_yaml(_PARAMS_PATH)
        cls.monitor = cls.params['collision_monitor']['ros__parameters']
        cls.lidar_x = _lidar_x_in_base_link()
        cls.range_min = _lidar_range_min()
        cls.footprint = _footprint_points(cls.params)
        cls.half_width = max(abs(p[1]) for p in cls.footprint)
        cls.max_forward_speed = float(
            cls.params['controller_server']['ros__parameters']['FollowPath']['max_vel_x'])
        cls.stop_zones = _stop_zones(cls.monitor)

    def _require_stop_zone(self):
        assert self.stop_zones, (
            'no enabled collision_monitor polygon has action_type "stop": only a '
            'slowdown ("approach") is configured, so nothing ever commands zero')

    def test_a_stop_zone_exists(self):
        self._require_stop_zone()

    def test_stop_zone_has_fixed_geometry_in_base_link(self):
        self._require_stop_zone()
        for name, points, _, _ in self.stop_zones:
            assert points is not None and len(points) >= 3, (
                f'stop zone {name} has no explicit points; a stop zone built from '
                f'the live footprint topic is the footprint itself, so it contains '
                f'nothing the robot has not already hit')
        assert self.monitor['base_frame_id'] == 'base_link'

    def test_stop_zone_applies_at_the_configured_forward_speed(self):
        self._require_stop_zone()
        applicable = [
            name for name, _, _, rng in self.stop_zones
            if rng is None or rng[0] <= self.max_forward_speed <= rng[1]]
        assert applicable, (
            f'no stop zone applies at the maximum forward speed '
            f'({self.max_forward_speed} m/s): {[z[0] for z in self.stop_zones]}')

    def _applicable_zones(self):
        self._require_stop_zone()
        return [
            (name, points, min_points) for name, points, min_points, rng in self.stop_zones
            if points and (rng is None or rng[0] <= self.max_forward_speed <= rng[1])]

    def test_a_scan_point_just_beyond_minimum_range_is_inside_the_zone(self):
        x = self.lidar_x + self.range_min + PROBE_MARGIN_M
        zones = self._applicable_zones()
        assert zones
        assert any(_contains(points, x, 0.0) for _, points, _ in zones), (
            f'no stop zone contains ({x:.3f}, 0.0) in base_link, the nearest centre-line '
            f'point the LiDAR can report (lidar x {self.lidar_x} m + minimum range '
            f'{self.range_min} m + {PROBE_MARGIN_M} m): the sensor can never see an '
            f'obstacle inside the zone')

    def test_the_zone_covers_the_width_the_robot_sweeps(self):
        x = self.lidar_x + self.range_min + PROBE_MARGIN_M
        zones = self._applicable_zones()
        assert zones
        for name, points, _ in zones:
            if _contains(points, x, 0.0):
                for y in (self.half_width, -self.half_width):
                    assert _contains(points, x, y), (
                        f'stop zone {name} does not contain ({x:.3f}, {y:+.3f}): it '
                        f'is narrower than the footprint half-width ({self.half_width} m)')
                return
        pytest.fail('no zone contains the centre-line probe point (see the test above)')

    def test_min_points_is_reachable_by_the_smallest_obstacle(self):
        x = self.lidar_x + self.range_min + PROBE_MARGIN_M
        range_from_lidar = self.range_min + PROBE_MARGIN_M
        returns = math.floor(
            OBSTACLE_MIN_WIDTH_M / (range_from_lidar * SCAN_ANGULAR_RESOLUTION_RAD))
        zones = self._applicable_zones()
        assert zones
        for name, points, min_points in zones:
            if _contains(points, x, 0.0):
                assert min_points <= returns, (
                    f'stop zone {name} needs {min_points} scan returns, but a '
                    f'{OBSTACLE_MIN_WIDTH_M} m obstacle at {range_from_lidar:.2f} m gives '
                    f'about {returns}: it would not trigger')
                return
        pytest.fail('no zone contains the centre-line probe point (see the test above)')
