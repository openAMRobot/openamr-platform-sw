# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
Wiring of the simulated base command path (#79): node, bridge config and parameters agree.

The docs describe `/cmd_vel -> sim_cmd_vel_timeout -> /sim/base_cmd_vel -> ros_gz_bridge -> gz
/cmd_vel`. These checks keep that description true: the bridge reads exactly what the timeout node
publishes, nothing bridges the ROS /cmd_vel straight to Gazebo again (that would bypass the
timeout), and the shipped parameters keep the 0.2 s of the real base. No ROS graph, no Gazebo.
"""

from pathlib import Path

from openamrobot_gazebo.cmd_vel_timeout import DEFAULT_TIMEOUT_S
from openamrobot_gazebo.sim_cmd_vel_timeout_node import INPUT_TOPIC, OUTPUT_TOPIC
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def _bridge_entries():
    return yaml.safe_load((CONFIG / 'gz_bridge.yaml').read_text())


def test_bridge_reads_what_the_timeout_node_publishes():
    entries = [e for e in _bridge_entries() if e['gz_topic_name'].lstrip('/') == 'cmd_vel']
    assert len(entries) == 1, entries
    assert entries[0]['ros_topic_name'] == OUTPUT_TOPIC
    assert entries[0].get('direction') == 'ROS_TO_GZ'
    assert INPUT_TOPIC == '/cmd_vel'


def test_nothing_bridges_the_ros_cmd_vel_straight_to_gazebo():
    # A second bridge entry from /cmd_vel would let a dead publisher's last command through the
    # timeout again, and the docs (TROUBLESHOOTING, ARCHITECTURE_OVERVIEW) would be wrong.
    # ros_gz_bridge treats an entry without `direction` as BIDIRECTIONAL.
    ros_side = [e['ros_topic_name'] for e in _bridge_entries()
                if e.get('direction', 'BIDIRECTIONAL') in ('ROS_TO_GZ', 'BIDIRECTIONAL')]
    assert INPUT_TOPIC not in ros_side, ros_side
    assert ros_side.count(OUTPUT_TOPIC) == 1, ros_side


def test_shipped_parameters_keep_the_real_base_timeout():
    params = yaml.safe_load((CONFIG / 'sim_cmd_vel_timeout.yaml').read_text())
    values = params['sim_cmd_vel_timeout']['ros__parameters']
    assert values['timeout_s'] == DEFAULT_TIMEOUT_S == 0.2
    assert 0.0 < values['tick_period_s'] < values['timeout_s']
    assert values['tick_period_s'] <= values['zero_repeat_period_s'] <= values['timeout_s']
