"""Helpers shared by the SIL nodes."""
import json

from rclpy.clock import Clock, ClockType
from std_msgs.msg import String

STEADY = Clock(clock_type=ClockType.STEADY_TIME)


def steady_timer(node, period_s, callback):
    """Timer on the host monotonic clock.

    The emulators measure their timeouts on the host monotonic clock (as the vendored code
    does), so the nodes pace on it too. A timer on ROS time would stop when Gazebo pauses.
    """
    return node.create_timer(period_s, callback, clock=STEADY)


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
