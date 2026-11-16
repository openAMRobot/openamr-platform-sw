"""ZLAC8015D emulator whose wheels are the Gazebo wheels.

The CANopen behaviour (NMT, heartbeat, SDO, velocity mode, 2000h link-loss, faults) is the
vendored openamr-platform-fw emulator, unchanged. This subclass replaces only the plant:
the ramped speed demand of the drive goes to Gazebo, and 606Ch reports the wheel speeds
that Gazebo measures. Protocol evidence only. Nothing here shows that a real wheel stops.
"""
import math
import time

from openamrobot_sil.can_sil import zlac8015d_emulator

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


class GazeboZlac8015d(zlac8015d_emulator.Zlac8015dEmulator):
    """One ZLAC8015D node. Sub-index 1 is the left axis and 2 the right axis in this layer.

    The vendor documents do not settle which sub-index is which wheel or the sign per axis
    (actuator ledger Q-02). left_sign and right_sign are SIL parameters, default +1.
    """

    def __init__(self, bus, node_id=1, heartbeat_ms=100, left_sign=1, right_sign=1,
                 feedback_timeout_s=0.5):
        super().__init__(bus, node_id, heartbeat_ms)
        self.signs = (float(left_sign), float(right_sign))
        self.feedback_timeout_s = feedback_timeout_s
        self._feedback_rad_s = None
        self._feedback_time = 0.0

    def set_wheel_feedback(self, left_rad_s, right_rad_s):
        """Wheel joint speeds from Gazebo, rad/s, positive forward."""
        with self.lock:
            self._feedback_rad_s = (float(left_rad_s), float(right_rad_s))
            self._feedback_time = time.monotonic()

    def feedback_fresh(self):
        return (self._feedback_rad_s is not None and
                time.monotonic() - self._feedback_time <= self.feedback_timeout_s)

    def demand_rad_s(self):
        """Speed the drive asks of each wheel now (left, right), rad/s, positive forward."""
        with self.lock:
            return tuple(self.signs[i] * self.rpm[i] * RPM_TO_RAD_S for i in (0, 1))

    def tick(self, dt):
        super().tick(dt)
        # The vendored tick wrote its own ramp model into 606Ch. With Gazebo attached the
        # encoder value is the Gazebo wheel speed instead. Without fresh feedback (protocol
        # tests, Gazebo not started) the vendored model stays.
        if self.feedback_fresh():
            for i in (0, 1):
                rpm = self.signs[i] * self._feedback_rad_s[i] / RPM_TO_RAD_S
                self.od.set(0x606C, i + 1, int(round(rpm * 10)))

    def state(self):
        with self.lock:
            return {
                'node_id': self.node_id,
                'nmt_state': self.nmt_state,
                'statusword': self.od.get(0x6041, 0),
                'faults': self.faults,
                'enabled': self.enabled,
                'quick_stop': self.quick_stop,
                'target_rpm': [self.od.get(0x60FF, 1), self.od.get(0x60FF, 2)],
                'demand_rpm': [round(self.rpm[0], 2), round(self.rpm[1], 2)],
                'actual_rpm': [self.od.get(0x606C, 1) / 10.0, self.od.get(0x606C, 2) / 10.0],
                'link_loss_ms': self.od.get(0x2000, 0),
                'link_lost': self.link_lost,
                'link_loss_events': self.link_loss_events,
                'bus_off': self.bus_off,
                'feedback': 'gazebo' if self.feedback_fresh() else 'internal',
            }


def wheel_speeds_to_twist(left_rad_s, right_rad_s, wheel_radius, wheel_separation):
    """Body twist (v m/s, w rad/s) of a differential drive from its wheel speeds."""
    v = wheel_radius * (left_rad_s + right_rad_s) / 2.0
    w = wheel_radius * (right_rad_s - left_rad_s) / wheel_separation
    return v, w


def twist_to_wheel_speeds(v, w, wheel_radius, wheel_separation):
    """Wheel speeds (left, right) rad/s of a differential drive from a body twist."""
    left = (v - w * wheel_separation / 2.0) / wheel_radius
    right = (v + w * wheel_separation / 2.0) / wheel_radius
    return left, right
