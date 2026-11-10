"""
Live proof that /follow_waypoints never exists against a real navigation_launch.py.

Alex's #90 finding at 2e0b8f3, acceptance criterion: "prove an external
client cannot submit a follow_waypoints goal."

Starts the real navigation_launch.py as a subprocess with the shipped
nav2_params.yaml - no map/TF/costmap convergence needed for this specific
check, since we only care that nothing ever advertises /follow_waypoints,
not that the rest of the stack reaches an active lifecycle state. A real
nav2_msgs/action/FollowWaypoints ActionClient's wait_for_server is the
same mechanism any external client would use to discover the server
before submitting a goal.

See test_navigation_launch_no_waypoint_follower.py for the static (no
ROS graph) version of this same guarantee.
"""
import os
import signal
import subprocess
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #90, Alex's D-14 finding

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from nav2_msgs.action import FollowWaypoints, Spin  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402


def _start_navigation_launch():
    share = get_package_share_directory('openamrobot_nav2')
    launch_file = os.path.join(share, 'launch', 'navigation_launch.py')
    # Own process group (today's established fix for the ros2-launch-starts-
    # a-real-child orphan footgun - see test_stop_contract_full_chain.py's
    # own _start_mux for the same reasoning): signalling only the `ros2
    # launch` wrapper leaves its child processes running after the test.
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


class TestFollowWaypointsServerNeverExists(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._proc = _start_navigation_launch()
        self.node = rclpy.create_node('follow_waypoints_absence_probe')

    def tearDown(self):
        _stop_navigation_launch(self._proc)
        self.node.destroy_node()

    def test_follow_waypoints_action_server_never_becomes_available(self):
        # Positive control first (panthera-momagdii's #90 review, raised
        # against the sibling behavior-server-direct-actions test but the
        # same gap applies here): checking only a safety_stop_gate_node
        # service proves that one separate, no-nav2-core-apt-dependency
        # node started - it says nothing about whether the actual Nav2
        # stack (bt_navigator, behavior_server, ...) that would otherwise
        # run waypoint_follower ever came up. 'spin' stays registered in
        # behavior_plugins specifically so this test has a true positive to
        # wait for: it only becomes available once behavior_server itself
        # is configured and active.
        spin_client = ActionClient(self.node, Spin, 'spin')
        behavior_server_up = False
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if spin_client.wait_for_server(timeout_sec=0.0):
                behavior_server_up = True
                break
        spin_client.destroy()
        self.assertTrue(
            behavior_server_up,
            'behavior_server never advertised spin (its own registered, '
            'non-reverse-capable action) - the Nav2 stack likely never came '
            'up at all, which would make the follow_waypoints absence below '
            'meaningless')

        client = ActionClient(self.node, FollowWaypoints, 'follow_waypoints')
        # Generous window: long enough for every other node in the launch
        # (controller_server, planner_server, bt_navigator, ...) to fully
        # start up and settle, even without a map/TF tree converging - if
        # waypoint_follower existed, this is ample time for it to advertise.
        deadline = time.monotonic() + 15.0
        became_available = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if client.wait_for_server(timeout_sec=0.0):
                became_available = True
                break
        client.destroy()
        self.assertFalse(
            became_available,
            'a /follow_waypoints action server appeared - navigation_launch.py '
            "must not start waypoint_follower (Alex's D-14 finding on #90)")
