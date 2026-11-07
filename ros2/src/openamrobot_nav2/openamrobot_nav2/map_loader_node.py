#!/usr/bin/env python3
"""
Single map loader (openamrobot-interfaces#34, platform-sw#36/#73).

Owns the only supported map-loading path, at boot and for every supported
reload: calls nav2_map_server's own ~/load_map service itself, rather than
letting map_server auto-load from a yaml_filename parameter at launch. That
makes this node the one place that actually knows which artifact was
requested and what grid nav2_map_server returned for it, instead of a
status observer trying to infer the same thing from map_yaml_path, launch
timing, or the first /map message it happens to see - all of which Mohamed
Sayed's platform-sw#73 review showed cannot actually establish map identity
(a same-geometry, same-content-looking map already active before the
observer starts, or the observer restarting while a different map stayed
active throughout, both read as the configured map with every one of those
heuristics).

Publishes /map_identity (transient-local) on every successful load: the
artifact hash of the map.yaml + image file actually requested
(map_identity.map_identity_hash, interfaces#28's SiteConfigStatus.map_version
definition), a load generation counter, and a fingerprint of the
OccupancyGrid nav2_map_server's own load_map response returned - not a
later, separately-observed /map message, which could have been produced by
any caller bypassing this node entirely. A failed load never publishes a
new identity; the previous one (if any) stays retained and valid.

Does not touch /map itself and is not a replacement for map_server - it is
a thin, deliberate wrapper around the one load_map call every supported
boot/reload path must go through, so that call's own result is available
to publish, not just inferred afterwards.

[Why fire-and-forget, not async/await] An async def callback that awaits
call_async(...) directly does not reliably resume in this environment: a
suspended coroutine's resume callback is scheduled onto the owning
executor, but that scheduling was verified (two independent minimal
reproductions, one driven by manual spin_once() calls, one by real
background spin() threads - same result both times) to never actually run
when the client and the service it calls live on separate node/executor
instances, which is the normal case for this node calling map_server's own
service. Plain call_async(request) + future.add_done_callback(...) has no
such problem - the done callback fires correctly from _take_client's own
synchronous handling of the response - so every call here uses that
instead, and /map_identity's own generation field is what a caller
actually confirming a reload took should watch, not a synchronous
service response.
"""
import os

from ament_index_python.packages import get_package_share_directory
from nav2_msgs.srv import LoadMap
from openamr_nav_msgs.msg import MapIdentity
from openamrobot_nav2.map_identity import (
    grid_fingerprint, load_map_identity_source, map_identity_hash)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


class MapLoaderNode(Node):

    def __init__(self, parameter_overrides=None):
        super().__init__('map_loader_node', parameter_overrides=parameter_overrides or [])

        default_map_yaml_path = os.path.join(
            get_package_share_directory('openamrobot_nav2'), 'maps', 'my_map.yaml')
        self.declare_parameter('map_yaml_path', default_map_yaml_path)
        # Only relevant to tests - lets a test avoid waiting for the real
        # service discovery delay the boot load would otherwise need.
        self.declare_parameter('boot_load_delay_s', 1.0)
        # map_server's load_map service, as a parameter rather than a
        # hardcoded absolute name: a namespaced map_server would otherwise
        # be unreachable, and a test needs to be able to point this at its
        # own fake without colliding with a real map_server that happens
        # to be discoverable on the same domain.
        self.declare_parameter('load_map_service_name', '/map_server/load_map')

        self._load_client = self.create_client(
            LoadMap, self.get_parameter('load_map_service_name').value)
        self._generation = 0
        self._last_load_ok = None  # None until the first attempt resolves

        identity_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._identity_pub = self.create_publisher(MapIdentity, '/map_identity', identity_qos)

        # A deliberate service call to trigger a reload, same pattern as the
        # stop gate's own engage/release services - not a topic an operator
        # could accidentally publish to. Confirms the request was issued,
        # not that the load itself succeeded - see the module docstring's
        # [Why fire-and-forget] note; watch /map_identity's load_generation
        # for actual confirmation.
        self.create_service(Trigger, '~/reload', self._on_reload)

        delay_s = float(self.get_parameter('boot_load_delay_s').value)
        self._boot_timer = self.create_timer(max(delay_s, 0.001), self._on_boot_timer)

    def _on_boot_timer(self):
        self._boot_timer.cancel()
        self._load()

    def _on_reload(self, request, response):
        del request
        if not self._load_client.service_is_ready():
            response.success = False
            response.message = f'{self._load_client.srv_name} is not reachable'
            return response
        self._load()
        response.success = True
        response.message = (
            'reload requested - watch /map_identity load_generation for confirmation')
        return response

    def _load(self) -> None:
        """
        Call ~/load_map for the current map_yaml_path, fire-and-forget.

        _on_load_response does the rest once it returns. map_yaml_path is
        read fresh from this node's own parameter on every
        call, not cached at construction, so a reload can target a
        different map after an operator updates the parameter.
        """
        map_yaml_path = self.get_parameter('map_yaml_path').value
        if not self._load_client.service_is_ready():
            self.get_logger().error(f'{self._load_client.srv_name} is not available')
            return
        # LoadMap.srv's own comment documents a "file://" URL, but the real
        # map_io on this Jazzy build passes map_url straight to its YAML
        # loader without stripping a scheme first - a "file://" prefix
        # becomes part of the literal (nonexistent) filename it tries to
        # open and fails with RESULT_INVALID_MAP_METADATA. Verified against
        # the real service: a bare absolute path works, a "file://"
        # one does not.
        request = LoadMap.Request(map_url=map_yaml_path)
        future = self._load_client.call_async(request)
        future.add_done_callback(lambda f: self._on_load_response(f, map_yaml_path))

    def _on_load_response(self, future, map_yaml_path: str) -> None:
        response = future.result()
        if response is None:
            self.get_logger().error('load_map call did not complete')
            self._last_load_ok = False
            return

        if response.result != LoadMap.Response.RESULT_SUCCESS:
            self.get_logger().error(
                f'load_map for {map_yaml_path} failed with result code {response.result}')
            self._last_load_ok = False
            return

        try:
            artifact_hash = map_identity_hash(**load_map_identity_source(map_yaml_path))
        except Exception as exc:
            self.get_logger().error(f'failed to hash {map_yaml_path} after loading it: {exc}')
            self._last_load_ok = False
            return

        self._generation += 1
        identity = MapIdentity()
        identity.artifact_hash = artifact_hash
        identity.load_generation = self._generation
        identity.grid_fingerprint = grid_fingerprint(response.map.data)
        self._identity_pub.publish(identity)
        self._last_load_ok = True
        self.get_logger().info(f'loaded {map_yaml_path} (generation {self._generation})')


def main(args=None):
    rclpy.init(args=args)
    node = MapLoaderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
