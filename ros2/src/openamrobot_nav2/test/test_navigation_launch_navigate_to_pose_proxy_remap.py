"""
Live proof that navigate_to_pose is proxied, against a real navigation_launch.py.

platform-sw#124, D-13; BotshareAI's #90 review (11 Nov): confirms
navigation_launch.py's own remap actually takes effect at launch time, not
just that the Python source says so - the public navigate_to_pose name must
be served by navigate_to_pose_goal_proxy_node, and bt_navigator's own real
action server must still be reachable at navigate_to_pose_real (proving the
remap renamed it rather than disabling it). No map/TF/costmap convergence
needed for this - server advertisement happens on construction, not on
reaching an active lifecycle state.

This does NOT depend on nav2-bt-navigator/nav2-behaviors/nav2-controller/
nav2-planner/nav2-collision-monitor/nav2-lifecycle-manager being installed
for its own public-name half (navigate_to_pose_goal_proxy_node is pure
openamrobot_nav2), but the navigate_to_pose_real half does, same dependency
footprint as the other live navigation_launch.py tests - see
nav-status-tests.yml's own comment on why those stay local-only.
"""
import os
import signal
import subprocess
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #90/#124

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402


def _start_navigation_launch():
    share = get_package_share_directory('openamrobot_nav2')
    launch_file = os.path.join(share, 'launch', 'navigation_launch.py')
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


class TestNavigateToPoseProxyRemap(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._proc = _start_navigation_launch()
        self.node = rclpy.create_node('navigate_to_pose_proxy_remap_probe')

    def tearDown(self):
        _stop_navigation_launch(self._proc)
        self.node.destroy_node()

    def _wait_for_server(self, name, timeout_s):
        client = ActionClient(self.node, NavigateToPose, name)
        deadline = time.monotonic() + timeout_s
        available = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if client.wait_for_server(timeout_sec=0.0):
                available = True
                break
        client.destroy()
        return available

    def test_public_name_is_served_by_the_proxy_and_real_name_still_reachable(self):
        self.assertTrue(
            self._wait_for_server('navigate_to_pose', 20.0),
            'navigate_to_pose never became available - the proxy node never '
            'started or never claimed the public name')
        self.assertTrue(
            self._wait_for_server('navigate_to_pose_real', 20.0),
            'navigate_to_pose_real never became available - bt_navigator was '
            'not actually remapped to it, or never started')


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
