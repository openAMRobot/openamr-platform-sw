# SPDX-License-Identifier: MIT
"""
Check that the base Gazebo sensors carry the numbers of config/sensor_profiles.yaml (#156).

The xacro is expanded with its default arguments and every SDF value of the base sensors is
compared with the profile key it comes from. A profile value that is null must leave no SDF
element: a number in the model without a source is a failure. The few values Gazebo needs and
the profile does not give are listed in UNSOURCED with the owner who closes them; once the
profile has the value, the model must use it. Topic names and frame ids are pinned: they are
contracts and this check fails if they move. Simulation only, not sensor acceptance.
"""
import math
import os
import xml.etree.ElementTree as ET

import pytest

import xacro

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, os.pardir, 'urdf', 'robo_urdf.urdf.xacro')
with open(os.path.join(HERE, os.pardir, 'config', 'sensor_profiles.yaml')) as f:
    FRAMES = yaml.safe_load(f)['frames']

TOF = ('tof_front', 'tof_rear')
US = ('us_front_left', 'us_front_right', 'us_rear_left', 'us_rear_right')

# Contracts: sensor name -> (type, gz topic, gz_frame_id, link). Not changed by #156.
CONTRACT = {
    'lidar_sensor': ('gpu_lidar', '/scan', 'lidar_link', 'lidar_link'),
    'imu_sensor': ('imu', 'imu', 'imu_link', 'imu_link'),
    'camera_sensor': ('camera', '/rgb_image', 'camera_optical_frame', 'camera_link'),
    'gemini_rgbd_sensor': ('rgbd_camera', '/base_camera/rgbd',
                           'base_camera_color_optical_frame', 'base_camera_link'),
    'tof_front_sensor': ('gpu_lidar', '/tof/front', 'tof_front', 'tof_front'),
    'tof_rear_sensor': ('gpu_lidar', '/tof/rear', 'tof_rear', 'tof_rear'),
    'us_front_left_sensor': ('gpu_lidar', '/ultrasonic/front_left', 'us_front_left',
                             'us_front_left'),
    'us_front_right_sensor': ('gpu_lidar', '/ultrasonic/front_right', 'us_front_right',
                              'us_front_right'),
    'us_rear_left_sensor': ('gpu_lidar', '/ultrasonic/rear_left', 'us_rear_left',
                            'us_rear_left'),
    'us_rear_right_sensor': ('gpu_lidar', '/ultrasonic/rear_right', 'us_rear_right',
                             'us_rear_right'),
}

# Values Gazebo needs that the profile leaves null, kept as they were and reported in #156.
# (sensor, sdf path) -> (model value, profile key, owner). Used only while the key is null.
UNSOURCED = {
    ('imu_sensor', 'update_rate'): (50.0, 'imu_link.gyro.odr', 'Evans'),
}
for _us in US:
    UNSOURCED[(_us + '_sensor', 'lidar/scan/horizontal/max_angle')] = (
        0.17, _us + '.range.beam_half_angle', 'Vineeth')
    UNSOURCED[(_us + '_sensor', 'lidar/scan/horizontal/min_angle')] = (
        -0.17, _us + '.range.beam_half_angle', 'Vineeth')


def profile(key):
    frame, channel, name = key.split('.')
    return FRAMES[frame]['channels'][channel][name]['value']


def rows():
    """Yield (sensor, sdf path, profile key, expected value or None)."""
    def r(sensor, path, key, conv=lambda v: v):
        value = profile(key)
        yield sensor, path, key, None if value is None else conv(value)

    yield from r('lidar_sensor', 'update_rate', 'lidar_link.scan.scan_rate')
    yield from r('lidar_sensor', 'lidar/range/min', 'lidar_link.scan.range_min')
    yield from r('lidar_sensor', 'lidar/range/max', 'lidar_link.scan.range_max_white')
    yield from r('lidar_sensor', 'lidar/range/resolution', 'lidar_link.scan.range_resolution',
                 lambda v: v * 0.001)
    yield from r('lidar_sensor', 'lidar/scan/horizontal/samples',
                 'lidar_link.scan.angular_resolution', lambda v: round(360.0 / v))
    yield from r('lidar_sensor', 'lidar/noise/stddev', 'lidar_link.scan.range_noise_stddev')

    g = 'gemini_rgbd_sensor'
    yield from r(g, 'update_rate', 'base_camera_link.depth.frame_rate_max')
    yield from r(g, 'camera/horizontal_fov', 'base_camera_link.depth.fov_horizontal',
                 math.radians)
    yield from r(g, 'camera/image/width', 'base_camera_link.depth.width_max')
    yield from r(g, 'camera/image/height', 'base_camera_link.depth.height_max')
    yield from r(g, 'camera/depth_camera/clip/near', 'base_camera_link.depth.range_min')
    yield from r(g, 'camera/depth_camera/clip/far', 'base_camera_link.depth.range_max')
    yield from r(g, 'camera/noise/stddev', 'base_camera_link.color.image_noise_stddev')

    yield from r('imu_sensor', 'update_rate', 'imu_link.gyro.odr')
    for axis in ('x', 'y', 'z'):
        for kind, ch in (('angular_velocity', 'gyro'), ('linear_acceleration', 'accel')):
            # Per-sample noise needs the firmware noise bandwidth (odr null): never written.
            yield ('imu_sensor', 'imu/%s/%s/noise/stddev' % (kind, axis),
                   'imu_link.%s.odr' % ch, None)
            yield from r('imu_sensor', 'imu/%s/%s/noise/bias_stddev' % (kind, axis),
                         'imu_link.%s.bias_stability' % ch)

    for frame in TOF:
        s = frame + '_sensor'
        rate = profile(frame + '.range.frame_rate_configured')
        if rate is None:
            yield from r(s, 'update_rate', frame + '.range.frame_rate_default')
        else:
            yield from r(s, 'update_rate', frame + '.range.frame_rate_configured')
        yield from r(s, 'lidar/range/min', frame + '.range.range_min')
        yield from r(s, 'lidar/range/max', frame + '.range.range_max_90pct')
        yield from r(s, 'lidar/range/resolution', frame + '.range.range_resolution',
                     lambda v: v * 0.01)
        yield from r(s, 'lidar/scan/horizontal/max_angle', frame + '.range.fov',
                     lambda v: math.radians(v) / 2)
        yield from r(s, 'lidar/scan/horizontal/min_angle', frame + '.range.fov',
                     lambda v: -math.radians(v) / 2)
        yield from r(s, 'lidar/noise/stddev', frame + '.range.range_noise_stddev')

    for frame in US:
        s = frame + '_sensor'
        yield from r(s, 'update_rate', frame + '.range.refresh_rate')
        yield from r(s, 'lidar/range/min', frame + '.range.range_min')
        yield from r(s, 'lidar/range/max', frame + '.range.range_max')
        yield from r(s, 'lidar/range/resolution', frame + '.range.range_resolution',
                     lambda v: v * 0.01)
        yield from r(s, 'lidar/scan/horizontal/max_angle', frame + '.range.beam_half_angle')
        yield from r(s, 'lidar/scan/horizontal/min_angle', frame + '.range.beam_half_angle',
                     lambda v: -v)
        yield from r(s, 'lidar/noise/stddev', frame + '.range.range_noise_stddev')


ROWS = list(rows())


@pytest.fixture(scope='module')
def sensors():
    root = ET.fromstring(xacro.process_file(XACRO).toxml())
    found = {}
    for gazebo in root.iter('gazebo'):
        for sensor in gazebo.findall('sensor'):
            assert sensor.get('name') not in found, sensor.get('name')
            found[sensor.get('name')] = (gazebo.get('reference'), sensor)
    return found


def test_every_sensor_is_accounted_for(sensors):
    assert set(sensors) == set(CONTRACT)


@pytest.mark.parametrize('name', sorted(CONTRACT))
def test_topic_and_frame_ids_are_unchanged(sensors, name):
    link, sensor = sensors[name]
    kind, topic, frame, ref = CONTRACT[name]
    assert sensor.get('type') == kind
    assert sensor.findtext('topic') == topic
    assert sensor.findtext('gz_frame_id') == frame
    assert link == ref
    assert sensor.find('pose').get('relative_to') == ref


@pytest.mark.parametrize('sensor,path,key,expected', ROWS,
                         ids=['%s:%s' % (row[0], row[1]) for row in ROWS])
def test_sdf_value_equals_the_profile(sensors, sensor, path, key, expected):
    element = sensors[sensor][1].find(path)
    if expected is None:
        if (sensor, path) in UNSOURCED:
            value, unsourced_key, owner = UNSOURCED[(sensor, path)]
            assert unsourced_key == key
            assert element is not None and float(element.text) == value, (
                '%s %s: expected the reported unsourced value %s (owner %s)'
                % (sensor, path, value, owner))
            return
        assert element is None, '%s %s = %s has no source: %s is null' % (
            sensor, path, element.text, key)
        if path.endswith('/stddev'):
            parent = path.rsplit('/', 1)[0]
            assert sensors[sensor][1].find(parent) is None, (
                '%s: %s block written without a sourced value' % (sensor, parent))
        return
    assert element is not None, '%s: %s missing (profile %s = %s)' % (
        sensor, path, key, expected)
    assert float(element.text) == pytest.approx(expected, rel=1e-9, abs=1e-12), (
        sensor, path, key)


def test_unsourced_values_are_only_where_the_profile_is_null():
    keys = {(row[0], row[1]): row[2] for row in ROWS}
    for (sensor, path), (_, key, owner) in UNSOURCED.items():
        assert keys.get((sensor, path)) == key, (sensor, path, key)
        assert profile(key) is None, '%s has a value now: use it, drop the row' % key
        assert owner


def test_no_noise_block_without_a_sourced_stddev(sensors):
    sourced = {(row[0], row[1]) for row in ROWS if row[3] is not None}
    for name, (_, sensor) in sensors.items():
        if name == 'camera_sensor':
            continue  # legacy Gate A camera, no profile (RG-SEN-036, #156)
        for parent in sensor.iter():
            noise = parent.find('noise')
            if noise is None:
                continue
            path = [p for p in ('lidar/noise/stddev', 'camera/noise/stddev')
                    if sensor.find(p) is noise.find('stddev')]
            assert path and (name, path[0]) in sourced, '%s: unsourced noise block' % name
