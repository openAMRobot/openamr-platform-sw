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


# --- the health callback is a simulation-only hold, not a second command authority ------------

import os  # noqa: E402
import types  # noqa: E402

from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

from openamrobot_sil.canbus import open_bus  # noqa: E402
from openamrobot_sil.nodes import ctl017_node, zlac8015d_node  # noqa: E402

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'config', 'sil.yaml')


@pytest.fixture
def sim_time_ros():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'config_file:=' + CONFIG])
    yield
    rclpy.shutdown()


def drain(bus):
    out = []
    msg = bus.recv(timeout=0.05)
    while msg is not None:
        out.append(msg)
        msg = bus.recv(timeout=0.05)
    return out


def device_view(kind, node):
    if kind == 'drive':
        s = node.emu.state()
        return (s['enabled'], s['target_rpm'], s['demand_rpm'], s['nmt_state'], s['faults'],
                s['clock_stalls'], s['clock_resets'])
    s = node.emu.state_dict()
    return (s['drive_on'], s['target_stop'], s['state'], s['nmt_state'], s['faults'],
            s['cmd_result'], s['clock_stalls'], s['clock_resets'])


@pytest.mark.parametrize('kind', ['drive', 'lift'])
def test_health_callback_only_effect_is_the_simulation_only_plant_hold(sim_time_ros, monkeypatch,
                                                                       kind):
    module = zlac8015d_node if kind == 'drive' else ctl017_node
    channel = 'virtual:health-%s' % kind
    monkeypatch.setattr(module, 'open_bus', lambda _iface: open_bus(channel))
    node = module.ZlacNode() if kind == 'drive' else module.Ctl017Node()
    spy = open_bus(channel)
    try:
        if kind == 'drive':
            node.emu.stop()          # the emulator thread reacts on its own step; not tested here
        drain(spy)
        published = []
        node.cmd_pub = types.SimpleNamespace(publish=published.append)   # spy on the hold
        wall = ManualClock()
        node.sim.coordinator.steady = wall
        node.sim.on_clock(ClockMsg(clock=Time(sec=5)))
        before = device_view(kind, node)
        node.sim.on_health()                               # no stall: no effect at all
        assert published == []
        wall.advance(1.5)
        for _ in range(5):
            node.sim.on_health()                           # one stall, one hold
        hold = Twist() if kind == 'drive' else Float64(data=0.0)
        assert published == [hold]                         # zero to the Gazebo plant only
        assert drain(spy) == []                            # no CAN frame from the health path
        assert device_view(kind, node) == before           # the emulated device untouched
        assert node.sim.status()['stalls'] == 1
    finally:
        node.shutdown()
        node.destroy_node()
        spy.shutdown()
