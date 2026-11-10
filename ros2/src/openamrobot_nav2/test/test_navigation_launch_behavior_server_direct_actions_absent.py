"""
Live proof that backup/drive_on_heading/assisted_teleop never advertise.

D-19 (platform-sw#124), panthera-momagdii's #90 review: each is its own
directly-callable behavior_server action, independent of whatever the BT
does - drive_on_heading accepts a negative target/speed, assisted_teleop
forwards whatever linear.x a teleop operator sends, both reverse-capable
while registered. Starts the real navigation_launch.py as a subprocess
and confirms via real nav2_msgs/action ActionClients that none of the
three ever becomes available - the same mechanism any external client
would use to discover the server before submitting a goal.

See test_nav2_params_no_reverse.py for the static (no ROS graph) version
of this same guarantee, and test_navigation_launch_waypoint_follower_
server_absent.py for the sibling test this one is modelled on.
"""
import os
import signal
import subprocess
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #90, panthera-momagdii's D-19 review

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from nav2_msgs.action import AssistedTeleop, BackUp, DriveOnHeading  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

_ACTIONS = (
    ('backup', BackUp),
    ('drive_on_heading', DriveOnHeading),
    ('assisted_teleop', AssistedTeleop),
)


def _start_navigation_launch():
    share = get_package_share_directory('openamrobot_nav2')
    launch_file = os.path.join(share, 'launch', 'navigation_launch.py')
    # Own process group - today's established fix for the ros2-launch-starts-
    # a-real-child orphan footgun (see test_stop_contract_full_chain.py's
    # own _start_mux for the same reasoning).
    return subprocess.Popen(
        ['ros2', 'launch', launch_file, 'use_sim_time:=false'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)


def _stop_navigation_launch(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10.0)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=10.0)


class TestBehaviorServerDirectActionsNeverAvailable(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._proc = _start_navigation_launch()
        self.node = rclpy.create_node('behavior_server_direct_action_absence_probe')

    def tearDown(self):
        _stop_navigation_launch(self._proc)
        self.node.destroy_node()

    def test_none_of_the_three_reverse_capable_actions_ever_becomes_available(self):
        # Sanity check first: this absence proves nothing if the launch
        # itself never came up at all (e.g. a missing nav2 apt package in a
        # minimal CI container) - everything would be equally absent then,
        # not just these three specifically. /safety/stop_lock_release is
        # built in-repo (openamrobot_nav2) with no nav2-core apt
        # dependency, so it is a reliable "the launch actually ran" signal
        # (found the hard way: platform-sw#90, nav-status-tests.yml's
        # minimal container never had nav2-bt-navigator/nav2-behaviors/
        # nav2-controller/nav2-planner/nav2-collision-monitor/nav2-
        # lifecycle-manager installed, which made this and the sibling
        # waypoint-follower-absent test vacuously pass there).
        release_client = self.node.create_client(Trigger, '/safety/stop_lock_release')
        self.assertTrue(
            release_client.wait_for_service(timeout_sec=20.0),
            'navigation_launch.py never came up at all (safety_stop_gate_node '
            'never advertised /safety/stop_lock_release) - the absence check '
            'below would be meaningless if nothing in the launch ever started')
        release_client.destroy()

        clients = [ActionClient(self.node, action_type, name) for name, action_type in _ACTIONS]
        # Generous window: long enough for every other node in the launch
        # (controller_server, planner_server, bt_navigator, ...) to fully
        # start up and settle, even without a map/TF tree converging - if
        # any of these existed, this is ample time for it to advertise.
        deadline = time.monotonic() + 15.0
        available = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            for (name, _), client in zip(_ACTIONS, clients):
                if name not in available and client.wait_for_server(timeout_sec=0.0):
                    available.append(name)
            if len(available) == len(_ACTIONS):
                break
        for client in clients:
            client.destroy()
        self.assertEqual(
            available, [],
            f'{available} became available as action servers - D-19 '
            '(platform-sw#124): each remains reverse-capable while registered '
            "in behavior_server's behavior_plugins")
