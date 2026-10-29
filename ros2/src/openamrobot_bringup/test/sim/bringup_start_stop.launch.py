# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
Start and stop reliability of the headless simulation bringup (PSW-15).

The bringup (``sim:=true gui:=false use_rviz:=false``) is started and stopped three times
(launch_testing parametrize: one launch per cycle). Each cycle checks that every node
managed by the Nav2 lifecycle managers reaches active within a bounded time, records the
exit code of every process after the stop, and checks that no ros2 or gz process is left.

The apriltag_node SIGSEGV on stop (#84) is a separate, expected failure.

Run: ``launch_test test/sim/bringup_start_stop.launch.py`` (nightly job
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
LEFTOVER_GRACE_S = 30.0
# Exit codes accepted after the stop: clean exit, or ended by SIGINT / SIGTERM.
CLEAN_EXIT = (0, -2, -15, 130, 143)
# Command-line fragments of processes the bringup starts.
OWNED = ('/opt/ros/', 'gz sim', 'gz-sim', 'parameter_bridge',
         str(Path(get_package_share_directory('openamrobot_bringup')).parents[2]))
APRILTAG = 'apriltag_node'
# ros2 and gz processes that existed before each cycle started; they are not counted as
# left over by that cycle.
BEFORE = {}
# Exit codes of apriltag_node in the cycle that just stopped (set by TestStopIsClean).
LAST_EXIT_CODES = {}


@pytest.mark.launch_test
@launch_testing.parametrize('cycle', [1, 2, 3])
def generate_test_description(cycle):
    bringup = os.path.join(get_package_share_directory('openamrobot_bringup'),
                           'launch', 'bringup.launch.py')
    BEFORE[cycle] = {pid for pid, _ in leftover_processes()}
    print('PSW-15 cycle %d: start (%d ros2 or gz processes already running)'
          % (cycle, len(BEFORE[cycle])), flush=True)
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={'sim': 'true', 'gui': 'false', 'use_rviz': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestStartReachesActive(unittest.TestCase):

    def test_all_managed_nodes_active(self, cycle):
        rclpy.init()
        node = rclpy.create_node('psw15_probe_%d' % cycle)
        try:
            elapsed, states = sim_graph.wait_all_active(node, ACTIVE_DEADLINE_S)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        print('PSW-15 cycle %d states: %s' % (cycle, states), flush=True)
        self.assertIsNotNone(elapsed, 'cycle %d: not all managed nodes active within %.0f s: %s'
                             % (cycle, ACTIVE_DEADLINE_S, states))
        print('PSW-15 cycle %d: all managed nodes active after %.1f s' % (cycle, elapsed),
              flush=True)


def ancestors():
    """Return the process ids of this process and its parents."""
    pids, pid = set(), os.getpid()
    while pid > 1:
        pids.add(pid)
        try:
            with open('/proc/%d/stat' % pid) as stream:
                pid = int(stream.read().rsplit(')', 1)[1].split()[1])
        except OSError:
            break
    return pids


def leftover_processes():
    """Return [(pid, command line)] of ros2 or gz processes other than this test."""
    skip, out = ancestors(), []
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) in skip:
            continue
        try:
            with open('/proc/%s/cmdline' % entry, 'rb') as stream:
                cmd = stream.read().replace(b'\0', b' ').decode(errors='replace').strip()
        except OSError:
            continue
        if cmd and any(f in cmd for f in OWNED) and 'launch_test' not in cmd:
            out.append((int(entry), cmd[:160]))
    return out


def exit_codes(proc_info):
    """Return {launch process name: exit code} of every process the launch started."""
    codes = {}
    for info in proc_info:
        name = os.path.basename(str(info.action.process_details['name']))
        codes[name] = info.returncode
    return codes


@launch_testing.post_shutdown_test()
class TestStopIsClean(unittest.TestCase):

    def test_exit_codes_recorded_and_clean(self, proc_info, cycle):
        codes = exit_codes(proc_info)
        LAST_EXIT_CODES[APRILTAG] = {n: c for n, c in codes.items() if APRILTAG in n}
        print('PSW-15 cycle %d exit codes: %s' % (cycle, codes), flush=True)
        self.assertTrue(codes, 'no process exit codes recorded')
        bad = {n: c for n, c in codes.items() if APRILTAG not in n and c not in CLEAN_EXIT}
        self.assertEqual(bad, {}, 'cycle %d: unclean exit codes %s' % (cycle, bad))

    def test_no_leftover_ros2_or_gz_processes(self, cycle):
        def new():
            return [p for p in leftover_processes() if p[0] not in BEFORE.get(cycle, set())]
        end = time.monotonic() + LEFTOVER_GRACE_S
        left = new()
        while left and time.monotonic() < end:
            time.sleep(1.0)
            left = new()
        print('PSW-15 cycle %d leftover processes: %s' % (cycle, left), flush=True)
        self.assertEqual(left, [], 'cycle %d: processes left after the stop' % cycle)


# Non-strict xfail for the known crash: unittest loads classes in name order, so
# TestStopIsClean has recorded this cycle's exit codes before this class is set up. The test
# is an expected failure only in a cycle where apriltag_node crashed; a clean exit passes.
# (A class-level expectedFailure would be strict, and the crash does not happen on every stop.)
@launch_testing.post_shutdown_test()
class TestStopZApriltagNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        codes = LAST_EXIT_CODES.get(APRILTAG, {})
        crashed = any(c not in CLEAN_EXIT for c in codes.values())
        cls.__unittest_expecting_failure__ = crashed  # xfail(strict=False): #84 apriltag crash

    def test_apriltag_node_exits_cleanly(self, proc_info, cycle):
        codes = {n: c for n, c in exit_codes(proc_info).items() if APRILTAG in n}
        xfail = ' (XFAIL #84)' if self.__unittest_expecting_failure__ else ''
        print('PSW-15 cycle %d apriltag exit codes: %s%s' % (cycle, codes, xfail), flush=True)
        self.assertTrue(codes, 'apriltag_node did not run')
        self.assertTrue(all(c in CLEAN_EXIT for c in codes.values()), codes)
