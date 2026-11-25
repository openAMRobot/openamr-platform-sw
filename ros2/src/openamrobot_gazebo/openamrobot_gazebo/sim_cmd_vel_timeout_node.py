# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
Simulation-only command timeout between /cmd_vel and Gazebo's DiffDrive (#79).

Subscribes to /cmd_vel, forwards every command to /sim/base_cmd_vel (bridged to the gz /cmd_vel
topic the DiffDrive system reads, config/gz_bridge.yaml) and, once no command has arrived for
`timeout_s` of the node clock, publishes zero at once and repeats it every `zero_repeat_period_s`
until the next command. One zero already stops DiffDrive; the repeat is for a bridge that was not
running when the timeout fired. It runs on simulation time (use_sim_time), so it behaves the same
at any real-time factor.

/sim/base_cmd_vel is internal to openamrobot_gazebo: only this node publishes it and only the
bridge reads it. Nothing else may use it; it is not an interface of the platform.

Clock edge cases (all fail-safe, pinned by test_sim_cmd_vel_timeout_node.py): before the first
/clock message the node clock reads 0 and its timer does not run, so nothing times out while the
simulation (the source of /clock) has not started; a command received then counts as older than
the timeout as soon as the clock starts at a later time, and the base is held at zero until the
next command. A clock that goes backwards (a simulation reset) also counts as stale.

It makes the simulated base stop when its commands stop, as the real base controller does. It is
not the firmware watchdog and not safety evidence. Every timeout is logged with its age so a test
or a trace can tell "the chain sent its own zero" apart from "the timeout had to stop the base".
"""

from geometry_msgs.msg import Twist
from openamrobot_gazebo.cmd_vel_timeout import (
    CommandTimeout, DEFAULT_TIMEOUT_S, DEFAULT_ZERO_REPEAT_S, describe_stale, validate_tick_period,
    validate_zero_repeat_period, zero_is_due)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

INPUT_TOPIC = '/cmd_vel'
OUTPUT_TOPIC = '/sim/base_cmd_vel'


class SimCmdVelTimeout(Node):
    """Forward /cmd_vel to the simulated base; hold zero after the command timeout."""

    def __init__(self, **kwargs):
        super().__init__('sim_cmd_vel_timeout', **kwargs)
        self.declare_parameter('timeout_s', DEFAULT_TIMEOUT_S)
        self.declare_parameter('tick_period_s', 0.02)
        self.declare_parameter('zero_repeat_period_s', DEFAULT_ZERO_REPEAT_S)
        timeout_s = float(self.get_parameter('timeout_s').value)
        tick_period_s = float(self.get_parameter('tick_period_s').value)
        zero_repeat_s = float(self.get_parameter('zero_repeat_period_s').value)
        validate_tick_period(timeout_s, tick_period_s)
        validate_zero_repeat_period(timeout_s, tick_period_s, zero_repeat_s)
        self.timeout = CommandTimeout(timeout_s)
        self.zero_repeat_s = zero_repeat_s
        self._last_zero_s = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, OUTPUT_TOPIC, qos)
        self.create_subscription(Twist, INPUT_TOPIC, self._on_cmd_vel, qos)
        # The timer runs on the node clock: simulation time when use_sim_time is true.
        self.create_timer(tick_period_s, self._on_tick)
        self.get_logger().info(
            f'{INPUT_TOPIC} -> {OUTPUT_TOPIC}, zero after {timeout_s:.3f} s without a command, '
            f'repeated every {zero_repeat_s:.3f} s (tick {tick_period_s:.3f} s, use_sim_time '
            f'{self.get_parameter("use_sim_time").value}); simulation only')

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd_vel(self, msg):
        if self.timeout.on_command(self._now_s()):
            self.get_logger().info('command resumed after a timeout')
        self._last_zero_s = None
        self.pub.publish(msg)

    def _on_tick(self):
        now = self._now_s()
        hold_zero, newly = self.timeout.on_tick(now)
        if not hold_zero:
            return
        if newly:
            self.get_logger().warning(
                f'{describe_stale(self.timeout.age(now), self.timeout.timeout_s)} '
                f'(timeout {self.timeout.timeouts})')
        if newly or zero_is_due(now, self._last_zero_s, self.zero_repeat_s):
            self.pub.publish(Twist())
            self._last_zero_s = now


def main(args=None):
    rclpy.init(args=args)
    node = SimCmdVelTimeout()
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
