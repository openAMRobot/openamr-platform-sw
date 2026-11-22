# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
sim_cmd_vel_timeout node (#79) on a simulated clock, without Gazebo.

The test publishes /clock itself, so every time below is simulation time: the node forwards
/cmd_vel to /sim/base_cmd_vel, sends zero once 0.2 s of simulation time pass without a command,
resumes on the next command, and does not time out while the simulation clock stands still.
"""

import os
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, Vector3
from openamrobot_gazebo.sim_cmd_vel_timeout_node import SimCmdVelTimeout
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock

# Own DDS domain, so a simulator or another test on this machine cannot publish /clock into it.
DOMAIN_ID = 100 + os.getpid() % 100


class Harness(Node):
    """Publishes /clock and /cmd_vel, records /sim/base_cmd_vel."""

    def __init__(self, context):
        super().__init__('sim_cmd_vel_timeout_test', context=context)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.out = []  # linear.x of every forwarded or zero command
        self.create_subscription(Twist, '/sim/base_cmd_vel',
                                 lambda m: self.out.append(m.linear.x), 10)


@pytest.fixture
def rig():
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=DOMAIN_ID)
    node = SimCmdVelTimeout(context=context,
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    harness = Harness(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(harness)

    def spin(wall_s=0.05):
        end = time.monotonic() + wall_s
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.005)

    def set_clock(sim_s):
        sec = int(sim_s)
        nanosec = int(round((sim_s - sec) * 1e9))
        harness.clock_pub.publish(Clock(clock=Time(sec=sec, nanosec=nanosec)))
        spin()

    def wait_for_matching():
        end = time.monotonic() + 10.0
        while time.monotonic() < end:
            if (harness.clock_pub.get_subscription_count()
                    and harness.cmd_pub.get_subscription_count()
                    and node.pub.get_subscription_count()):
                return
            spin(0.1)
        raise TimeoutError('publishers and subscriptions did not match')

    wait_for_matching()
    yield node, harness, spin, set_clock
    executor.shutdown()
    node.destroy_node()
    harness.destroy_node()
    rclpy.shutdown(context=context)


def command(harness, spin, vx):
    harness.cmd_pub.publish(Twist(linear=Vector3(x=vx)))
    spin()


def test_forwards_then_zero_after_0_2_s_of_simulation_time(rig):
    node, harness, spin, set_clock = rig
    set_clock(10.0)
    command(harness, spin, 0.2)
    assert harness.out and harness.out[-1] == pytest.approx(0.2)
    for step in range(1, 16):  # 10.01 .. 10.15 s: still within the timeout
        set_clock(10.0 + step * 0.01)
    assert 0.0 not in harness.out, harness.out
    for step in range(16, 30):  # past 10.2 s
        set_clock(10.0 + step * 0.01)
    assert harness.out[-1] == 0.0
    assert node.timeout.timeouts == 1


def test_next_command_resumes(rig):
    node, harness, spin, set_clock = rig
    set_clock(20.0)
    command(harness, spin, 0.2)
    for step in range(1, 30):
        set_clock(20.0 + step * 0.01)
    assert harness.out[-1] == 0.0
    command(harness, spin, 0.15)
    assert harness.out[-1] == pytest.approx(0.15)
    assert not node.timeout.stale


def test_clock_reset_holds_zero_and_the_log_does_not_print_a_negative_age(rig, capfd):
    # A simulation reset puts the clock before the last command. The base must stop (the command
    # is from before the reset) and the log must say why, not "no /cmd_vel for -4.980 s".
    node, harness, spin, set_clock = rig
    set_clock(10.0)
    command(harness, spin, 0.2)
    for step in range(1, 5):
        set_clock(10.0 + step * 0.01)
    capfd.readouterr()
    for step in range(4):                  # rclpy re-arms its timer after a backward jump:
        set_clock(5.0 + step * 0.01)       # the first tick comes 0.02 s into the new timeline
    log = capfd.readouterr().err
    assert harness.out[-1] == 0.0
    assert node.timeout.timeouts == 1
    assert 'went back' in log and 'holding the simulated base at zero' in log, log
    assert ' -' not in log.split('sim_cmd_vel_timeout]:')[-1], f'negative age in the log: {log}'


def test_command_before_the_first_clock_message_is_held_at_zero_once_the_clock_starts(rig):
    # Before the first /clock message the node clock reads 0 and its timer does not run, so the
    # budget does not elapse while the simulation has not started (Gazebo publishes /clock, so
    # nothing moves either). A command received then counts as older than the timeout as soon as
    # the clock starts at a later time: fail-safe, the base is held at zero until the next command.
    node, harness, spin, set_clock = rig
    assert node.get_clock().now().nanoseconds == 0
    command(harness, spin, 0.2)
    spin(0.6)                              # wall time passes, the clock has not started
    assert harness.out == [pytest.approx(0.2)]
    assert node.timeout.timeouts == 0
    set_clock(5.0)                         # the simulation already ran: first /clock is at 5 s
    set_clock(5.02)
    assert harness.out[-1] == 0.0
    assert node.timeout.timeouts == 1
    command(harness, spin, 0.15)           # non-latching: the next command passes
    assert harness.out[-1] == pytest.approx(0.15)
    assert not node.timeout.stale


def test_no_timeout_while_the_simulation_clock_stands_still(rig):
    # At a real-time factor below 1 a lot of wall time passes per simulated second; only simulation
    # time counts. One wall second with the clock held at +0.1 s must not stop the base.
    node, harness, spin, set_clock = rig
    set_clock(30.0)
    command(harness, spin, 0.2)
    set_clock(30.1)
    spin(1.0)
    assert 0.0 not in harness.out, harness.out
    assert node.timeout.timeouts == 0
