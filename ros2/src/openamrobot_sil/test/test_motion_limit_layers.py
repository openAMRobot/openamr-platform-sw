"""Base speed and deceleration limits of every command layer against the declared ceilings.

Declared (openAMRobot/.github decisions.yaml, read at 87c8486):
  SPEED-CEILING  Build 1 base speed at most 0.5 m/s in Nav2, smoother, ZLAC8015D and Gazebo.
  MASS-LEDGER    deceleration 1.5 m/s2 travel and 1.0 m/s2 hold-and-move, identical in Nav2, the
                 velocity smoother, the ZLAC8015D and Gazebo (D-29, a separate PR after #90).
No layer has a hold-and-move mode, so only the travel value is compared.

Each value is read from the file that configures the layer. A value that breaks its ceiling is a
failure unless DECLARED lists it with the value it has now and the tracking issue; a declared
value that changes (fixed or moved) fails too, so the table and the configuration change together.
No parameter is changed here: the Nav2 values belong to the Nav2 owners (#90, #91, #92, #113).
The table is in docs/architecture/motion_limits.md. Configuration evidence only, not a stop test.
"""
import math
import os
import re

import pytest
import yaml

from conftest import PACKAGE_DIR, REPO_ROOT
from openamrobot_sil import config
from openamrobot_sil.can_sil import zlac8015d_emulator as zlac

SPEED_CEILING = 0.5          # m/s, SPEED-CEILING
TRAVEL_DECELERATION = 1.5    # m/s2, MASS-LEDGER (D-29)
ISSUE = 'openAMRobot/openamr-platform-sw#172'
SRC = os.path.join(REPO_ROOT, 'ros2', 'src')
RPM_TO_RAD_S = 2.0 * math.pi / 60.0

# (layer, quantity): value configured today, for the rows that break the declared ceiling.
DECLARED = {
    ('nav2_controller', 'deceleration'): 2.5,
    ('velocity_smoother', 'deceleration'): 2.5,
    ('gazebo_diff_drive', 'speed'): 1.0,
    ('gazebo_diff_drive', 'reverse_speed'): 1.0,
    ('gazebo_diff_drive', 'deceleration'): 0.5,
    ('zlac8015d_emulator', 'speed'): 1000 * RPM_TO_RAD_S * 0.100,
    ('zlac8015d_emulator', 'deceleration'): 1000 * RPM_TO_RAD_S * 0.100 / 0.5,
}


def _yaml(*parts):
    with open(os.path.join(SRC, *parts), encoding='utf-8') as f:
        return yaml.safe_load(f)


def _diff_drive_tag(tag):
    with open(os.path.join(SRC, 'openamrobot_description', 'urdf', 'gazebo_control.xacro'),
              encoding='utf-8') as f:
        text = f.read()
    (value,) = re.findall(r'<%s>([-\d.]+)</%s>' % (tag, tag), text)
    return float(value)


def layers():
    """{(layer, quantity): (value, where)}; speeds in m/s as magnitudes, decelerations in m/s2."""
    nav = _yaml('openamrobot_nav2', 'config', 'nav2_params.yaml')
    fp = nav['controller_server']['ros__parameters']['FollowPath']
    sm = nav['velocity_smoother']['ros__parameters']
    dock = _yaml('openamrobot_docking', 'config', 'dock_trigger.yaml')
    dock = next(iter(dock.values()))['ros__parameters']
    radius = config.load(os.path.join(PACKAGE_DIR, 'config', 'sil.yaml'))['drive']['wheel_radius']
    drive = zlac.Zlac8015dEmulator(_NoBus())
    rpm_s = zlac.MAX_RPM / (drive.od.get(0x6084, 1) / 1000.0)  # 6084h ms per MAX_RPM
    nav2 = 'openamrobot_nav2/config/nav2_params.yaml'
    xacro = 'openamrobot_description/urdf/gazebo_control.xacro'
    emu = 'openamrobot_sil/vendor/can_sil/zlac8015d_emulator.py, sil.yaml wheel_radius'
    return {
        ('nav2_controller', 'speed'): (fp['max_vel_x'], nav2 + ' FollowPath.max_vel_x'),
        ('nav2_controller', 'speed_xy'): (fp['max_speed_xy'], nav2 + ' FollowPath.max_speed_xy'),
        ('nav2_controller', 'reverse_speed'): (-fp['min_vel_x'], nav2 + ' FollowPath.min_vel_x'),
        ('nav2_controller', 'deceleration'): (-fp['decel_lim_x'],
                                              nav2 + ' FollowPath.decel_lim_x'),
        ('velocity_smoother', 'speed'): (sm['max_velocity'][0], nav2 + ' max_velocity[0]'),
        ('velocity_smoother', 'reverse_speed'): (-sm['min_velocity'][0],
                                                 nav2 + ' min_velocity[0]'),
        ('velocity_smoother', 'deceleration'): (-sm['max_decel'][0], nav2 + ' max_decel[0]'),
        ('docking', 'speed'): (dock['drive_speed'], 'dock_trigger.yaml drive_speed'),
        ('docking', 'reverse_speed'): (dock['undock_reverse_speed'],
                                       'dock_trigger.yaml undock_reverse_speed'),
        ('gazebo_diff_drive', 'speed'): (_diff_drive_tag('max_linear_velocity'), xacro),
        ('gazebo_diff_drive', 'reverse_speed'): (-_diff_drive_tag('min_linear_velocity'), xacro),
        ('gazebo_diff_drive', 'deceleration'): (-_diff_drive_tag('min_linear_acceleration'),
                                                xacro),
        ('zlac8015d_emulator', 'speed'): (zlac.MAX_RPM * RPM_TO_RAD_S * radius,
                                          emu + ', 60FFh limit MAX_RPM'),
        ('zlac8015d_emulator', 'deceleration'): (rpm_s * RPM_TO_RAD_S * radius,
                                                 emu + ', 6084h default'),
    }


class _NoBus:
    """The emulator object dictionary is read without a bus; nothing is sent."""

    def send(self, *args, **kwargs):
        pass

    def recv(self, *args, **kwargs):
        return None


def ceiling_ok(quantity, value):
    if quantity == 'deceleration':
        return math.isclose(value, TRAVEL_DECELERATION, abs_tol=1e-9)
    return value <= SPEED_CEILING + 1e-12


LAYERS = layers()


@pytest.mark.parametrize('key', sorted(LAYERS), ids=['%s-%s' % k for k in sorted(LAYERS)])
def test_layer_against_the_declared_ceiling(key):
    value, where = LAYERS[key]
    if key in DECLARED:
        assert not ceiling_ok(key[1], value), (
            '%s %s = %g (%s) now meets the ceiling: remove it from DECLARED and from '
            'docs/architecture/motion_limits.md (%s)' % (key + (value, where, ISSUE)))
        assert math.isclose(value, DECLARED[key], rel_tol=1e-9), (
            '%s %s = %g (%s), declared %g: update DECLARED and the table (%s)'
            % (key + (value, where, DECLARED[key], ISSUE)))
    else:
        assert ceiling_ok(key[1], value), (
            '%s %s = %g (%s) breaks the ceiling (speed <= %g m/s, travel deceleration %g m/s2)'
            % (key + (value, where, SPEED_CEILING, TRAVEL_DECELERATION)))


def test_controller_and_smoother_agree():
    # Reverse is not compared: min_vel_x -0.05 (controller) is inside the smoother's -0.20.
    for q in ('speed', 'deceleration'):
        assert LAYERS[('nav2_controller', q)][0] == LAYERS[('velocity_smoother', q)][0], q


def test_declared_rows_exist():
    assert set(DECLARED) <= set(LAYERS)


def test_check_fails_on_an_undeclared_breach():
    assert not ceiling_ok('speed', 0.6)
    assert not ceiling_ok('deceleration', 2.5)
    assert ceiling_ok('speed', 0.5) and ceiling_ok('deceleration', 1.5)
