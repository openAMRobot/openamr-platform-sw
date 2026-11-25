# SPDX-License-Identifier: MIT
"""bt_navigator waits long enough for its servers on a loaded machine (platform-sw #165, OpenRAL review audits #88).
Pure Python over nav2_params.yaml: no ROS, no Nav2 start."""
import os

import yaml

PARAMS = os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml')


def test_bt_navigator_waits_30_s_for_its_action_servers():
    with open(PARAMS) as f:
        bt = yaml.safe_load(f)['bt_navigator']['ros__parameters']
    assert bt['wait_for_service_timeout'] >= 30000, 'milliseconds; 1000 aborted Nav2 on a loaded machine (#165)'
