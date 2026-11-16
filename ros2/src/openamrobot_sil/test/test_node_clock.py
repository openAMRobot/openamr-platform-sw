# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Node side of the clock contract: device timers on ROS time (C4), health on steady time (C5),
one fed clock per process from /clock. Needs rclpy (runs in CI sil-tests)."""
from builtin_interfaces.msg import Time
import pytest
import rclpy
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock as ClockMsg

from openamrobot_sil.nodes.common import NodeClock, device_timer, health_timer
from openamrobot_sil.sim_time import FedClock, ManualClock, SteadyClock


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def factory(use_sim_time):
        node = Node('sil_clock_test_%d' % len(nodes), parameter_overrides=[
            Parameter('use_sim_time', value=use_sim_time)])
        nodes.append(node)
        return node
    yield factory
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def clock_type(timer):
    return (getattr(timer, 'clock', None) or timer._clock).clock_type


def test_device_timer_runs_on_ros_time_and_health_timer_on_steady_time(make_node):
    node = make_node(True)
    assert clock_type(device_timer(node, 0.1, lambda: None)) == ClockType.ROS_TIME
    assert clock_type(health_timer(node, 0.1, lambda: None)) == ClockType.STEADY_TIME


def test_sim_time_node_feeds_one_clock_from_clock_messages(make_node):
    node = make_node(True)
    stalls = []
    sim = NodeClock(node, on_stall=lambda: stalls.append(1))
    assert isinstance(sim.clock, FedClock)
    sim.on_clock(ClockMsg(clock=Time(sec=5, nanosec=500000000)))
    assert sim.clock.now() == pytest.approx(5.5)
    wall = ManualClock()
    sim.coordinator.steady = wall
    sim.on_clock(ClockMsg(clock=Time(sec=5, nanosec=600000000)))
    wall.advance(1.5)
    sim.on_health()
    sim.on_health()
    assert stalls == [1] and sim.status()['stalls'] == 1 and sim.status()['paused']
    sim.on_clock(ClockMsg(clock=Time(sec=0)))
    assert sim.status()['resets'] == 1 and not sim.status()['paused']


def test_node_without_sim_time_uses_steady_time(make_node):
    sim = NodeClock(make_node(False))
    assert isinstance(sim.clock, SteadyClock) and sim.status()['domain'] == 'steady'
