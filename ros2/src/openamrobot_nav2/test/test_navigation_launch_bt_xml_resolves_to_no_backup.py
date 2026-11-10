"""
Live proof that bt_navigator resolves its bt_xml params to the no-backup files.

D-19 audit, platform-sw#124, Alex's finding on #90.

Starts the real navigation_launch.py as a subprocess with the shipped
nav2_params.yaml and queries bt_navigator's own live parameters via a real
rclpy parameter client - the same mechanism `ros2 param get` uses. A static
check of nav2_params.yaml alone cannot prove this: the YAML only holds a
relative placeholder string, resolved to an absolute path at launch time
via RewrittenYaml (see navigation_launch.py's own param_substitutions).

See test_behavior_trees_no_backup.py for the static check that the BT
files themselves never contain a BackUp node.
"""
import os
import signal
import subprocess
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #90, Alex's D-19 audit finding

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from rcl_interfaces.srv import GetParameters  # noqa: E402


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


class TestBtNavigatorResolvesToNoBackupFiles(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._proc = _start_navigation_launch()
        self.node = rclpy.create_node('bt_xml_resolution_probe')

    def tearDown(self):
        _stop_navigation_launch(self._proc)
        self.node.destroy_node()

    def _get_bt_navigator_param(self, name, timeout_s=90.0):
        # 30.0 was enough locally but timed out on the hosted CI runner
        # (shared, under load from the rest of this same suite already
        # having launched the full stack several times) - this is the
        # full Nav2 stack (controller_server, planner_server, bt_navigator,
        # behavior_server, collision_monitor, lifecycle_manager, ...), not
        # a lightweight launch, so it needs real margin here.
        client = self.node.create_client(GetParameters, '/bt_navigator/get_parameters')
        deadline = time.monotonic() + timeout_s
        while not client.wait_for_service(timeout_sec=0.5) and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(client.service_is_ready(), '/bt_navigator never came up')
        future = client.call_async(GetParameters.Request(names=[name]))
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        client.destroy()
        self.assertTrue(future.done(), f'get_parameters({name!r}) timed out')
        return future.result().values[0].string_value

    def test_default_nav_to_pose_bt_xml_resolves_to_the_no_backup_file(self):
        value = self._get_bt_navigator_param('navigate_to_pose.default_nav_to_pose_bt_xml')
        self.assertTrue(
            value.endswith(os.path.join('behavior_trees', 'navigate_to_pose_no_backup.xml')),
            f'bt_navigator resolved default_nav_to_pose_bt_xml to {value!r}, expected it to '
            'end with behavior_trees/navigate_to_pose_no_backup.xml')

    def test_default_nav_through_poses_bt_xml_resolves_to_the_no_backup_file(self):
        value = self._get_bt_navigator_param(
            'navigate_through_poses.default_nav_through_poses_bt_xml')
        self.assertTrue(
            value.endswith(
                os.path.join('behavior_trees', 'navigate_through_poses_no_backup.xml')),
            f'bt_navigator resolved default_nav_through_poses_bt_xml to {value!r}, expected '
            'it to end with behavior_trees/navigate_through_poses_no_backup.xml')
