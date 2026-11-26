"""Configuration, model and bridge files agree with each other and with the description."""
import os
import re

import pytest
import yaml

from conftest import PACKAGE_DIR, REPO_ROOT
from openamrobot_sil import config
from openamrobot_sil.canbus import NotVirtualCan, open_bus
from openamrobot_sil.ctl017_gazebo import JOINT_ZERO_MM, PermitConfig
from openamrobot_sil.can_sil import ctl017_emulator as c

CFG = config.load(os.path.join(PACKAGE_DIR, 'config', 'sil.yaml'))


def read(*parts):
    with open(os.path.join(*parts), encoding='utf-8') as f:
        return f.read()


def test_wheel_geometry_equals_the_gazebo_diff_drive_plugin():
    xacro = read(REPO_ROOT, 'ros2', 'src', 'openamrobot_description', 'urdf',
                 'gazebo_control.xacro')
    radius = float(re.search(r'<wheel_radius>([\d.]+)</wheel_radius>', xacro).group(1))
    separation = float(re.search(r'<wheel_separation>([\d.]+)</wheel_separation>', xacro).group(1))
    assert CFG['drive']['wheel_radius'] == radius
    # The SIL value is the effective track of the simulated plant (calibration, upperbody-sw #136), not the geometric
    # one. It can only lie between the wheel centre planes and the outer wheel edges; the model keeps its own value.
    urdf = read(REPO_ROOT, 'ros2', 'src', 'openamrobot_description', 'urdf', 'robo_urdf.urdf.xacro')
    widths = {float(w) for w in re.findall(r'<cylinder radius="0\.098" length="([\d.]+)"/>', urdf)}
    assert widths == {0.046}, f'drive wheel collision width changed: {widths}'
    assert separation == 0.400
    assert separation <= CFG['drive']['wheel_separation'] <= separation + 0.046
    assert CFG['drive']['wheel_separation'] == 0.411


def test_the_effective_track_is_used_in_both_directions():
    from openamrobot_sil.zlac_gazebo import twist_to_wheel_speeds, wheel_speeds_to_twist
    radius, track = CFG['drive']['wheel_radius'], CFG['drive']['wheel_separation']
    left, right = twist_to_wheel_speeds(0.0, 0.5, radius, track)
    assert right - left == pytest.approx(0.5 * track / radius)
    assert wheel_speeds_to_twist(left, right, radius, track) == pytest.approx((0.0, 0.5))


def test_one_vcan_interface_per_bus_and_documented_node_ids():
    buses = CFG['buses']
    names = [buses[k]['interface'] for k in ('drive', 'bms', 'aux')]
    assert names == ['vcan_drive', 'vcan_bms', 'vcan_aux']
    assert buses['drive']['zlac_node_id'] == 1
    assert buses['aux']['ctl017_node_id'] == c.NODE_ID == 0x10
    assert buses['aux']['core_node_id'] == c.CORE_NODE_ID


def test_lift_joint_of_the_sil_model_matches_config_and_limits():
    xacro = read(PACKAGE_DIR, 'urdf', 'sil_robot.urdf.xacro')
    joint = re.search(r'<joint name="(\w+)" type="prismatic">(.*?)</joint>', xacro, re.S)
    assert joint.group(1) == CFG['lift']['joint']
    limit = re.search(r'<limit lower="([\d.]+)" upper="([\d.]+)" effort="(\d+)" '
                      r'velocity="([\d.]+)"/>', joint.group(2))
    assert [float(x) for x in limit.groups()] == [0.0, 0.350, 800.0, 0.018]
    assert float(limit.group(4)) * 1000 == c.FULL_SPEED_MM_S
    # joint 0, 0.175 and 0.350 m are the stops L1000, L1175 and L1350 of the v1 dictionary
    assert [round(p - JOINT_ZERO_MM, 3) for p in c.STOP_POS_MM] == [0.0, 175.0, 350.0]
    assert xacro.count('<joint_name>%s</joint_name>' % CFG['lift']['joint']) == 2


def test_bridge_does_not_pass_cmd_vel_straight_to_gazebo():
    rows = yaml.safe_load(read(PACKAGE_DIR, 'config', 'gz_bridge_sil.yaml'))
    to_gz = {r['ros_topic_name']: r['gz_topic_name'] for r in rows
             if r['direction'] == 'ROS_TO_GZ'}
    assert to_gz == {'/sil/drive/gz_cmd_vel': '/cmd_vel',
                     '/sil/lift/gz_cmd_vel': '/sil/gz/lift_cmd_vel'}


def test_permit_test_values_form_a_valid_layout():
    permit = config.permit_config(CFG)
    assert permit.length == 6
    assert permit.unpack(permit.pack(1, 0xBEEF, 7, 0)) == (1, 0xBEEF, 7, 0)
    assert permit.timeout_ms >= 2 * CFG['lift']['permit']['period_ms']
    assert permit.timeout_ms != c.SUPERVISION_MS  # timeouts are never merged
    assert permit.refused_result not in range(12)  # not a result code of the v1 table


@pytest.mark.parametrize('bad', [
    dict(cob_id=0x210), dict(cob_id=0x701), dict(flag_bits=12), dict(permit_value=0),
    dict(timeout_ms=0), dict(boot_bits=32, seq_bits=32)])
def test_permit_config_rejects_invalid_layouts(bad):
    values = dict(cob_id=0x410, flag_bits=8, boot_bits=16, seq_bits=16, reason_bits=8,
                  permit_value=1, timeout_ms=200, refused_result=0x80)
    values.update(bad)
    with pytest.raises(ValueError):
        PermitConfig.from_dict(values)


@pytest.mark.parametrize('channel', ['can0', 'slcan0', 'eth0', ''])
def test_only_vcan_interfaces_are_opened(channel):
    with pytest.raises(NotVirtualCan):
        open_bus(channel)
