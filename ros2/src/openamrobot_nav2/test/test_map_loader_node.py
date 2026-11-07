"""
Node-level regression tests for map_loader_node.py.

openamrobot-interfaces#34, platform-sw#36/#73. Real rclpy, same convention
as the other node tests in this package - skipped here if openamr_nav_msgs
isn't installed, run for real in a ROS 2 workspace. The real map_server's
own ~/load_map service is replaced by a fake one on the probe node so each
test controls exactly what "loading" returns, without needing a real map
file on disk for every scenario; test_map_loader_against_real_map_server.py
covers the real nav2_map_server integration.
"""
import os
import tempfile
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')

from nav2_msgs.srv import LoadMap  # noqa: E402,I100
from nav_msgs.msg import OccupancyGrid  # noqa: E402,I100
from openamr_nav_msgs.msg import MapIdentity  # noqa: E402
from openamrobot_nav2.map_identity import grid_fingerprint, map_identity_hash  # noqa: E402
from openamrobot_nav2.map_loader_node import MapLoaderNode  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

_FAKE_LOAD_MAP_SERVICE = '/map_loader_node_test/fake_load_map'

# A tiny, valid 2x2 PGM - enough for load_map_identity_source to read, same
# fixture style as test_map_identity.py.
_TINY_PGM = b'P5\n2 2\n255\n' + bytes([10, 20, 30, 40])
_MAP_YAML = """\
image: map.pgm
resolution: 0.05
origin: [-1.0, -2.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary
"""


def _write_map_fixture(tmpdir) -> str:
    yaml_path = os.path.join(tmpdir, 'map.yaml')
    with open(yaml_path, 'w') as f:
        f.write(_MAP_YAML)
    with open(os.path.join(tmpdir, 'map.pgm'), 'wb') as f:
        f.write(_TINY_PGM)
    return yaml_path


def _grid(data=(0, 100, -1, 0)):
    msg = OccupancyGrid()
    msg.data = list(data)
    return msg


class NodeTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.map_yaml_path = _write_map_fixture(self.tmpdir.name)
        # A fake service name unique to this file, NOT /map_server/load_map:
        # a real nav2_map_server (test_map_loader_against_real_map_server.py
        # starts one) stays discoverable on the same domain for a while
        # after its own test finishes, and these tests would then talk to it
        # instead of the fake below - "unreachable" and "returns a failure
        # code" both silently became "a real map loaded fine".
        self.node = MapLoaderNode(
            parameter_overrides=[
                Parameter('map_yaml_path', value=self.map_yaml_path),
                Parameter('boot_load_delay_s', value=0.0),
                Parameter('load_map_service_name', value=_FAKE_LOAD_MAP_SERVICE),
            ])
        self.probe = rclpy.create_node('map_loader_probe')
        self._load_requests = []
        self._load_result = LoadMap.Response.RESULT_SUCCESS
        self._load_grid = _grid()
        self._load_map_service = self.probe.create_service(
            LoadMap, _FAKE_LOAD_MAP_SERVICE, self._on_load_map)
        self._identities = []
        self.probe.create_subscription(MapIdentity, '/map_identity', self._identities.append, 10)
        # One persistent executor per node, added once - matches how the
        # real node runs (rclpy.spin keeps the same executor for the
        # node's whole lifetime), rather than the plain module-level
        # rclpy.spin_once(node, ...) convenience function, which adds and
        # removes the node on every single call.
        self._node_executor = SingleThreadedExecutor()
        self._node_executor.add_node(self.node)
        self._probe_executor = SingleThreadedExecutor()
        self._probe_executor.add_node(self.probe)

    def tearDown(self):
        self.tmpdir.cleanup()
        self._node_executor.remove_node(self.node)
        self._probe_executor.remove_node(self.probe)
        self.node.destroy_node()
        self.probe.destroy_node()

    def _on_load_map(self, request, response):
        self._load_requests.append(request)
        response.result = self._load_result
        if self._load_result == LoadMap.Response.RESULT_SUCCESS:
            response.map = self._load_grid
        return response

    def _spin_both(self, timeout_s=1.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self._node_executor.spin_once(timeout_sec=0.02)
            self._probe_executor.spin_once(timeout_sec=0.02)

    def _call_reload(self):
        """
        Call ~/reload and spin long enough for the fire-and-forget load.

        The Trigger response only confirms the request was accepted (see
        map_loader_node.py's module docstring on why this isn't
        synchronous) - callers check /map_identity's own load_generation
        (via self._identities) for the real outcome, same as a real
        consumer would.
        """
        client = self.probe.create_client(Trigger, '/map_loader_node/reload')
        self.assertTrue(client.wait_for_service(timeout_sec=2.0))
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < end:
            self._spin_both(timeout_s=0.05)
        self.assertTrue(future.done(), 'reload call timed out')
        result = future.result()
        self._spin_both(timeout_s=1.0)  # let the fire-and-forget load resolve
        return result


class TestBootLoad(NodeTestCase):

    def test_a_successful_boot_load_publishes_identity(self):
        self._spin_both(timeout_s=1.5)
        self.assertEqual(len(self._load_requests), 1, 'load_map was not called at boot')
        self.assertTrue(self._identities, 'no /map_identity was ever published')

        identity = self._identities[-1]
        self.assertEqual(identity.load_generation, 1)
        self.assertEqual(identity.grid_fingerprint, grid_fingerprint(self._load_grid.data))
        expected_hash = map_identity_hash(
            resolution=0.05, origin_x=-1.0, origin_y=-2.0, origin_yaw=0.0,
            negate=False, occupied_thresh=0.65, free_thresh=0.196, mode='trinary',
            image_bytes=_TINY_PGM)
        self.assertEqual(identity.artifact_hash, expected_hash)

    def test_the_load_request_names_the_configured_map(self):
        self._spin_both(timeout_s=1.5)
        self.assertEqual(len(self._load_requests), 1)
        self.assertIn(self.map_yaml_path, self._load_requests[0].map_url)

    def test_a_failed_boot_load_publishes_no_identity(self):
        self._load_result = LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST
        self._spin_both(timeout_s=1.5)
        self.assertEqual(len(self._load_requests), 1, 'load_map was not called at boot')
        self.assertFalse(
            self._identities,
            'a failed load must never publish an identity - there is nothing to report')


class TestReload(NodeTestCase):

    def test_reload_calls_load_map_again_and_increments_generation(self):
        self._spin_both(timeout_s=1.5)
        self.assertEqual(self._identities[-1].load_generation, 1)

        result = self._call_reload()
        self.assertTrue(result.success, 'the request itself should be accepted')
        self.assertEqual(len(self._load_requests), 2)
        self.assertEqual(self._identities[-1].load_generation, 2)

    def test_a_reload_is_rejected_up_front_when_load_map_is_unreachable(self):
        self._spin_both(timeout_s=1.5)
        self._load_map_service.destroy()
        self._spin_both(timeout_s=0.5)  # let the node's client notice it is gone

        result = self._call_reload()
        self.assertFalse(result.success)
        self.assertEqual(
            len(self._load_requests), 1,
            'no second attempt should have been made against an unreachable service')

    def test_a_reload_that_fails_keeps_the_previous_identity_retained(self):
        self._spin_both(timeout_s=1.5)
        first_identity = self._identities[-1]

        self._load_result = LoadMap.Response.RESULT_INVALID_MAP_DATA
        result = self._call_reload()
        self.assertTrue(result.success, 'the request itself should still be accepted')
        self.assertEqual(
            len(self._identities), 1,
            'a failed reload must not publish a new identity over the retained one')
        self.assertEqual(self._identities[-1], first_identity)

    def test_a_reload_with_changed_map_content_publishes_a_different_fingerprint(self):
        self._spin_both(timeout_s=1.5)
        first_fingerprint = self._identities[-1].grid_fingerprint

        self._load_grid = _grid((0, 0, 0, 0))  # different content, same loader
        self._call_reload()
        self.assertNotEqual(self._identities[-1].grid_fingerprint, first_fingerprint)


if __name__ == '__main__':
    unittest.main(verbosity=2)
