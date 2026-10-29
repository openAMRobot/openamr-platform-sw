# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
Single /cmd_vel publisher in the headless simulation bringup (PSW-03, evidence for #88).

Starts ``openamrobot_bringup bringup.launch.py sim:=true gui:=false use_rviz:=false``,
waits until every node managed by the Nav2 lifecycle managers is active, then checks:

* collision_monitor stays active for a 60 s idle window;
* /cmd_vel has exactly one publisher, collision_monitor (decided motion chain: every
  source into twist_mux, then velocity_smoother, then collision_monitor).

On main dock_trigger also publishes /cmd_vel, so the publisher check is an expected
failure that is strict: launch_testing runs unittest cases, where an unexpected success
fails the run, so CI turns red the day #35 lands until the marker is removed.

Run: ``launch_test test/sim/single_cmd_vel_publisher.launch.py`` (nightly job
sim-launch-tests.yml). The file name has no test_ prefix so that `colcon test` does not
collect it on every pull request. Simulation only: not physical acceptance and not safety evidence
(register TELEMETRY-NOT-SAFETY-EVIDENCE).
"""

import os
from pathlib import Path
import sys
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_graph  # noqa: E402

ACTIVE_DEADLINE_S = float(os.environ.get('OPENAMR_SIM_ACTIVE_DEADLINE_S', '300'))
IDLE_WINDOW_S = 60.0
EXPECTED_PUBLISHERS = ['/collision_monitor']


@pytest.mark.launch_test
def generate_test_description():
    bringup = os.path.join(get_package_share_directory('openamrobot_bringup'),
                           'launch', 'bringup.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={'sim': 'true', 'gui': 'false', 'use_rviz': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


# One probe node and the activation result, shared by the three test classes. unittest loads
# the classes in name order (TestA, TestB, TestC), so TestA runs first.
_PROBE = {}


def probe():
    if 'node' not in _PROBE:
        rclpy.init()
        _PROBE['node'] = rclpy.create_node('psw03_cmd_vel_probe')
    return _PROBE['node']


def tearDownModule():
    if 'node' in _PROBE:
        _PROBE.pop('node').destroy_node()
        rclpy.shutdown()


class TestA_Nav2LifecycleActive(unittest.TestCase):

    def test_nav2_lifecycle_active(self):
        elapsed, states = sim_graph.wait_all_active(probe(), ACTIVE_DEADLINE_S)
        print('PSW-03 lifecycle states: %s (%s)' % (states, sim_graph.domain_note()), flush=True)
        self.assertIsNotNone(elapsed, 'Nav2 lifecycle not active within %.0f s: %s'
                             % (ACTIVE_DEADLINE_S, states))
        print('PSW-03 Nav2 lifecycle active after %.1f s' % elapsed, flush=True)
        _PROBE['active_after_s'] = elapsed


class TestB_CollisionMonitorIdle(unittest.TestCase):

    def test_collision_monitor_stays_active_idle(self):
        self.assertIn('active_after_s', _PROBE, 'Nav2 lifecycle never became active')
        end = time.monotonic() + IDLE_WINDOW_S
        samples, bad = 0, []
        while time.monotonic() < end:
            state = sim_graph.lifecycle_state(probe(), 'collision_monitor')
            samples += 1
            if state != sim_graph.ACTIVE:
                bad.append(state)
            time.sleep(1.0)
        print('PSW-03 collision_monitor idle window %.0f s: %d samples, %d not active %s'
              % (IDLE_WINDOW_S, samples, len(bad), bad), flush=True)
        self.assertGreater(samples, 10)
        self.assertEqual(bad, [], 'collision_monitor left active during the idle window')


# launch_testing wraps test methods in functools.partial, which drops a method-level
# expectedFailure; the class-level marker survives. An unexpected success fails the run.
@unittest.expectedFailure  # xfail(strict=True): #35 #88 dock_trigger publishes /cmd_vel
class TestC_SingleCmdVelPublisher(unittest.TestCase):

    def test_cmd_vel_has_one_publisher_collision_monitor(self):
        self.assertIn('active_after_s', _PROBE, 'Nav2 lifecycle never became active')
        pubs = sim_graph.publishers(probe(), '/cmd_vel')
        print('PSW-03 /cmd_vel publishers: %s' % pubs, flush=True)
        self.assertEqual(pubs, EXPECTED_PUBLISHERS)
