"""
Real nav2_map_server integration for map_loader_node.py.

openamrobot-interfaces#34, platform-sw#36/#73. test_map_loader_node.py
covers the node's own logic against a fake ~/load_map service; this file
instead brings up the real nav2_map_server process, started empty (an
explicit empty yaml_filename - the single-loader decision itself, see
_start_map_server), and confirms a real MapLoaderNode against it drives a
real LoadMap call end to end and publishes a /map_identity that matches
disk.

Skips cleanly if the real ros2 CLI or the nav2_map_server package isn't
installed - same convention as this package's other real-process test
files (test_twist_mux_startup_and_restart.py, etc. on the stop-gate work).
"""
import shutil
import signal
import subprocess
import time
import unittest
from pathlib import Path  # noqa: I100

import pytest

rclpy = pytest.importorskip('rclpy')


def _map_server_available():
    if shutil.which('ros2') is None:
        return False
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'nav2_map_server'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10.0)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


pytestmark = pytest.mark.skipif(
    not _map_server_available(),
    reason='needs the real ros2 CLI AND the nav2_map_server package installed [#36/#73]')

from openamr_nav_msgs.msg import MapIdentity  # noqa: E402,I100
from openamrobot_nav2.map_identity import load_map_identity_source, map_identity_hash  # noqa: E402
from openamrobot_nav2.map_loader_node import MapLoaderNode  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

_NODE_NAME = 'map_server'
# Real nav2_map_server rejects a synthetic tiny PGM with RESULT_INVALID_MAP_
# METADATA (verified) - use this package's own already-shipped real map
# fixture instead, the same one navigation_status_node's default
# map_yaml_path points at.
_MAP_YAML_PATH = str(
    Path(__file__).resolve().parents[1] / 'maps' / 'my_map.yaml')


def _lifecycle_set(label, timeout_s=10.0):
    end = time.monotonic() + timeout_s
    last_result = None
    while time.monotonic() < end:
        last_result = subprocess.run(
            ['ros2', 'lifecycle', 'set', f'/{_NODE_NAME}', label],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5.0)
        if last_result.returncode == 0:
            return
        time.sleep(0.3)
    stderr = last_result.stderr.decode(errors='replace') if last_result else '(never ran)'
    raise AssertionError(
        f'ros2 lifecycle set /{_NODE_NAME} {label} never succeeded within '
        f'{timeout_s}s - last stderr: {stderr}')


def _start_map_server():
    # yaml_filename explicitly empty, not omitted - map_server declares it
    # with no default, so never setting it at all throws during configure.
    # An explicit empty string is nav2_map_server's own documented "no
    # initial map should be used" case (its on_configure logs exactly that
    # and skips loading) - this is "start empty," per the single-loader
    # decision this file exists to verify, not a missing parameter.
    return subprocess.Popen(
        ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
         '-r', '__node:=map_server', '-p', 'yaml_filename:=""'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _stop_map_server(proc):
    if proc is not None and proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5.0)


class TestMapLoaderAgainstRealMapServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.map_yaml_path = _MAP_YAML_PATH
        self._server_proc = _start_map_server()
        _lifecycle_set('configure')
        _lifecycle_set('activate')

        self.node = MapLoaderNode(
            parameter_overrides=[
                Parameter('map_yaml_path', value=self.map_yaml_path),
                Parameter('boot_load_delay_s', value=0.0),
            ])
        self.probe = rclpy.create_node('map_loader_real_probe')
        self._identities = []
        self.probe.create_subscription(MapIdentity, '/map_identity', self._identities.append, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._executor.add_node(self.probe)

    def tearDown(self):
        self._executor.remove_node(self.node)
        self._executor.remove_node(self.probe)
        self.node.destroy_node()
        self.probe.destroy_node()
        _stop_map_server(self._server_proc)

    def _spin(self, timeout_s=10.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end and not self._identities:
            self._executor.spin_once(timeout_sec=0.05)

    def test_boot_load_against_the_real_map_server_publishes_a_matching_identity(self):
        self._spin(timeout_s=10.0)
        self.assertTrue(
            self._identities,
            '/map_identity was never published against the real map_server')

        identity = self._identities[-1]
        self.assertEqual(identity.load_generation, 1)
        expected_hash = map_identity_hash(**load_map_identity_source(self.map_yaml_path))
        self.assertEqual(identity.artifact_hash, expected_hash)
        # Just confirm a fingerprint came back at all, not its exact value -
        # this file's job is the real end-to-end wiring, not re-deriving
        # nav2_map_server's own pixel conversion (see map_identity.py's
        # docstring for why nothing in this package tries to do that).
        self.assertTrue(identity.grid_fingerprint)


if __name__ == '__main__':
    unittest.main(verbosity=2)
