#!/usr/bin/env python3
"""
On-demand image gate for AprilTag detection.

AprilTag detection (``apriltag_node``) is CPU-heavy: it runs the quad detector
on EVERY camera frame (~1.6 cores on the Pi 5). It is only needed during the
final dock approach, NOT while the robot navigates — running it always-on
starves the Nav2 planner/controller (the whole machine sits at load ~8 on 4
cores, and goals take seconds to start).

This gate sits between the camera and ``apriltag_node``::

    /camera/image_raw ──(gate)──▶ /apriltag/image_in ──▶ apriltag_node

The gate keeps ONE subscription to the camera and republishes to
``/apriltag/image_in`` ONLY while enabled:

* DISABLED (default): incoming frames are dropped here, so ``apriltag_node``
  receives nothing and idles at ~0 % CPU. ``apriltag_node`` STAYS ALIVE
  (already initialised, already subscribed).
* ENABLED: frames are forwarded, so ``apriltag_node`` gets the very next one.

Because ``apriltag_node`` never restarts and the camera never stops, toggling is
just flipping a boolean — effectively instant (< 100 ms), no process start, no
camera warm-up. That is the whole point: the docking sequence can flip detection
on "just before it needs it" without a latency penalty.

Note on the design: we deliberately keep a *permanent* subscription and gate the
*republish*, rather than creating/destroying the subscription on each toggle.
Creating a subscription from inside the SetBool service callback (single-threaded
executor) is unreliable — the new subscription is discovered by DDS but its
callback is never serviced, so no frames flow. The gate's own receive cost
(deserialising the camera stream) is negligible at the camera frame rate and far
below apriltag's detection cost, so this trades a tiny constant overhead for
correctness. The heavy ~1.6-core detection is fully gated either way.

Toggle with the ``std_srvs/SetBool`` service (default ``/apriltag/set_enabled``)::

    ros2 service call /apriltag/set_enabled std_srvs/srv/SetBool "{data: true}"

``dock_trigger`` calls it automatically: ENABLE once the robot reaches the
staging zone (Phase 1 done), DISABLE when the sequence ends (docked / failed /
undock). An operator UI can use the same service for a manual button.

QoS: RELIABLE on both sides, matching the camera (camera_ros publishes RELIABLE
KEEP_LAST 1) and apriltag's image_transport subscriber (RELIABLE KEEP_LAST 10).
A best-effort reader of the reliable depth-1 camera gets STARVED once apriltag is
also running (frames stop arriving) — measured on the Pi; a reliable reader does
not. Do not "optimise" this back to sensor-data/best-effort.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class AprilTagGate(Node):
    def __init__(self):
        super().__init__('apriltag_gate')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/apriltag/image_in')
        # start_enabled=false so apriltag stays idle until something asks for
        # detection. Set true for standalone apriltag testing (no dock_trigger).
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('service_name', 'set_enabled')
        # Throttle: forward at most this many fps to apriltag (drop the rest).
        # apriltag processes ~10-12 fps on the Pi 5; feeding it the full camera
        # rate overflows its input queue -> the detections it emits are 300+ ms
        # stale (backlog). Capping the input near apriltag's capacity keeps the
        # queue empty -> ~1-frame latency, at FULL resolution (NO precision loss,
        # unlike lowering the camera resolution). 0 = unthrottled.
        self.declare_parameter('max_fps', 10.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        service_name = self.get_parameter('service_name').value
        self.enabled = bool(self.get_parameter('start_enabled').value)
        max_fps = float(self.get_parameter('max_fps').value)
        self._min_interval = (1.0 / max_fps) if max_fps > 0 else 0.0
        self._last_fwd = None

        # RELIABLE everywhere to MATCH the camera (camera_ros publishes RELIABLE
        # KEEP_LAST 1) and apriltag's image_transport subscriber (RELIABLE
        # KEEP_LAST 10). A best-effort reader of the reliable depth-1 camera gets
        # starved once apriltag is also running; a reliable reader does not.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Image, self.output_topic, qos)
        # ONE permanent subscription (see module docstring). No traffic is
        # republished until self.enabled.
        self.sub = self.create_subscription(
            Image, self.input_topic, self._on_image, qos)

        self.srv = self.create_service(SetBool, service_name, self.on_set_enabled)
        self.get_logger().info(
            f"AprilTag gate ready, {'ENABLED' if self.enabled else 'DISABLED'}. "
            f"{self.input_topic} -> {self.output_topic} "
            f"(toggle with SetBool on '{service_name}').")

    def _on_image(self, msg):
        # Forward only while enabled; otherwise drop -> apriltag gets nothing.
        if not self.enabled:
            return
        # Throttle to max_fps: drop frames so apriltag is never overfed (which
        # backs up its input queue and makes its detections stale). Full-res
        # frames are still forwarded, just fewer of them.
        if self._min_interval > 0.0:
            now = self.get_clock().now()
            if self._last_fwd is not None and \
                    (now - self._last_fwd).nanoseconds * 1e-9 < self._min_interval:
                return
            self._last_fwd = now
        self.pub.publish(msg)

    def on_set_enabled(self, request, response):
        was = self.enabled
        self.enabled = bool(request.data)
        if self.enabled != was:
            self.get_logger().info(
                'AprilTag gate ENABLED — forwarding to apriltag.' if self.enabled
                else 'AprilTag gate DISABLED — apriltag idle (~0% CPU).')
        response.success = True
        response.message = 'apriltag gate ' + ('enabled' if self.enabled else 'disabled')
        return response


def main():
    rclpy.init()
    node = AprilTagGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
