#!/usr/bin/env python3
"""PROTOTYPE for issue #35 (Proposed, simulation only; implementation owner Kunal).

Docking guard mode for collision_monitor. Instead of lifecycle-deactivating
collision_monitor for the dock (which breaks its bond with
lifecycle_manager_navigation and makes the manager reset the whole navigation
stack), this node keeps collision_monitor ACTIVE and switches its polygons:

  NAV  : FootprintApproach enabled,  DockStop disabled
  DOCK : FootprintApproach disabled, DockStop enabled

DOCK applies while dock_trigger reports docking/undocking/docked and no
NavigateToPose goal is running (Phase 1 is a normal Nav2 goal). The applied
mode is read back from collision_monitor and published, latched, on
/docking/guard_mode, so the active guard is observable.

In the real implementation the switch belongs where dock_trigger calls
_pause_collision_monitor() and _resume_collision_monitor(); this separate node
only exists so the prototype does not modify dock_trigger.py.

This switches a functional guard; it is not a safety function. It does not
publish velocity.
"""
from action_msgs.msg import GoalStatus, GoalStatusArray
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

DOCK_STATES = {'docking', 'undocking', 'docked'}
ACTIVE_GOAL = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
               GoalStatus.STATUS_CANCELING}
MODES = {
    'NAV': {'FootprintApproach.enabled': True, 'DockStop.enabled': False},
    'DOCK': {'FootprintApproach.enabled': False, 'DockStop.enabled': True},
}


class DockGuardMode(Node):

    def __init__(self):
        super().__init__('dock_guard_mode')
        self.monitor = self.declare_parameter('collision_monitor_node', 'collision_monitor').value
        self.dock_status = 'idle'
        self.nav_active = False
        self.applied = None
        self.pending = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, '/docking/guard_mode', latched)
        self.client = AsyncParameterClient(self, self.monitor)
        self.create_subscription(String, '/dock_trigger_status', self._on_status, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self._on_nav_status, 10)
        self.create_timer(0.2, self._tick)
        self._publish('UNKNOWN')

    def _on_status(self, msg):
        if msg.data in DOCK_STATES or msg.data in {'idle', 'failed'}:
            self.dock_status = msg.data

    def _on_nav_status(self, msg):
        self.nav_active = any(s.status in ACTIVE_GOAL for s in msg.status_list)

    def _wanted(self):
        return 'DOCK' if self.dock_status in DOCK_STATES and not self.nav_active else 'NAV'

    def _publish(self, text):
        self.mode_pub.publish(String(data=text))

    def _tick(self):
        wanted = self._wanted()
        if wanted == self.applied or self.pending is not None:
            return
        if not self.client.services_are_ready():
            self.client.wait_for_services(timeout_sec=0.0)
            return
        params = [Parameter(k, value=v) for k, v in MODES[wanted].items()]
        self.pending = wanted
        self.client.set_parameters(params).add_done_callback(self._on_set)

    def _on_set(self, future):
        wanted, self.pending = self.pending, None
        res = future.result()
        if res is None or not all(r.successful for r in res.results):
            self.get_logger().error(f'collision_monitor rejected guard mode {wanted}')
            self._publish('FAILED')
            return
        self.client.get_parameters(list(MODES[wanted])).add_done_callback(
            lambda f: self._on_readback(wanted, f))

    def _on_readback(self, wanted, future):
        res = future.result()
        got = dict(zip(MODES[wanted], [v.bool_value for v in res.values])) if res else {}
        if got != MODES[wanted]:
            self.get_logger().error(f'guard mode {wanted} not confirmed: {got}')
            self._publish('FAILED')
            return
        self.applied = wanted
        self.get_logger().info(f'collision_monitor guard mode {wanted} confirmed: {got}')
        self._publish(wanted)


def main():
    rclpy.init()
    node = DockGuardMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
