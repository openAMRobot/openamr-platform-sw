"""Helpers shared by the SIL nodes.

Clock contract (docs/architecture/sil-topology.md section 6): device-step and publish timers run
on the node's ROS time (C4), which follows Gazebo ``/clock`` with ``use_sim_time`` and stops in a
pause. Only the process-health timer runs on host steady time (C5). ``NodeClock`` is the one
simulation clock of the process that the device models are given.
"""
import json

from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import String

from openamrobot_sil.sim_time import ClockCoordinator, FedClock, SteadyClock

STEADY = Clock(clock_type=ClockType.STEADY_TIME)
HEALTH_PERIOD_S = 0.1


def device_timer(node, period_s, callback):
    """Timer on the node's ROS time (C4). With use_sim_time it stops while Gazebo is paused."""
    return node.create_timer(period_s, callback)


def health_timer(node, period_s, callback):
    """Timer on host steady time (C5): process health only, never a device timer."""
    return node.create_timer(period_s, callback, clock=STEADY)


class NodeClock:
    """The simulation clock of one SIL process.

    With ``use_sim_time`` the clock is a ``FedClock`` fed from ``/clock`` by a
    ``ClockCoordinator``; a steady health timer declares a stall when ``/clock`` stops (C5) and
    logs "simulation stalled", never a device fault. Without ``use_sim_time`` (internal plant)
    the clock is host steady time (C1). ``on_stall`` is called once per stall, on the health
    timer, so the node can hold its plant command at zero while Gazebo does not run.
    ``add_check(fn)``: ``fn()`` returns None or a process-health problem to log.
    """

    def __init__(self, node, on_stall=None):
        self.node = node
        self.on_stall = on_stall
        self.sim = bool(node.get_parameter('use_sim_time').value)
        self._checks = []
        self._problems = {}
        if self.sim:
            self.coordinator = ClockCoordinator(FedClock())
            self.clock = self.coordinator.clock
            node.create_subscription(
                ClockMsg, '/clock', self.on_clock,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        else:
            self.coordinator = None
            self.clock = SteadyClock()
        health_timer(node, HEALTH_PERIOD_S, self.on_health)

    def add_check(self, fn):
        self._checks.append(fn)

    def on_clock(self, msg):
        event = self.coordinator.feed(msg.clock.sec + msg.clock.nanosec * 1e-9)
        if event == 'reset':
            self.node.get_logger().warning(
                'simulation time moved backward to %.3f s: emulated devices power cycle' %
                self.clock.now())
        elif event == 'resumed':
            self.node.get_logger().info('simulation resumed at %.3f s' % self.clock.now())

    def on_health(self):
        if self.coordinator is not None:
            report = self.coordinator.check()
            if report:
                self.node.get_logger().warning(report)
                if self.on_stall is not None:
                    self.on_stall()
        for fn in self._checks:
            problem = fn()
            if problem != self._problems.get(fn):
                self._problems[fn] = problem
                if problem:
                    self.node.get_logger().error('process health: %s' % problem)

    def status(self):
        if self.coordinator is not None:
            return self.coordinator.status()
        return {'domain': 'steady', 'fed': False, 'generation': 0, 'paused': False, 'stalls': 0,
                'resets': 0}


def thread_check(name, worker):
    """Process-health check: the worker thread of an emulator or stand-in still runs (C5)."""
    def check():
        thread = getattr(worker, '_thread', None)
        if thread is not None and not thread.is_alive():
            return '%s thread stopped' % name
        return None
    return check


def publish_json(publisher, data):
    msg = String()
    msg.data = json.dumps(data, sort_keys=True)
    publisher.publish(msg)


def joint_value(msg, name, field):
    """Value of one joint in a JointState, or None."""
    try:
        i = list(msg.name).index(name)
    except ValueError:
        return None
    values = getattr(msg, field)
    return values[i] if i < len(values) else None
