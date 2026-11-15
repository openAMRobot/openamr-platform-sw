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

"""The EKF reads the raw firmware IMU topic and fuses only the yaw rate (decision D-10)."""

import os

import yaml

EKF_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'ekf.yaml')

# robot_localization order: x y z, roll pitch yaw, vx vy vz, vroll vpitch vyaw, ax ay az
VYAW_ONLY = [False, False, False,
             False, False, False,
             False, False, False,
             False, False, True,
             False, False, False]


def _params():
    with open(EKF_FILE, encoding='utf-8') as f:
        return yaml.safe_load(f)['ekf_filter_node']['ros__parameters']


def test_ekf_reads_imu_data_raw():
    assert _params()['imu0'] == '/imu/data_raw'


def test_ekf_fuses_only_imu_angular_velocity_z():
    assert _params()['imu0_config'] == VYAW_ONLY


def test_ekf_has_one_imu_input():
    assert [k for k in _params() if k.startswith('imu') and k[3:].isdigit()] == ['imu0']
