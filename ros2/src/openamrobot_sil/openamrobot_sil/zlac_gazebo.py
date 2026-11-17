"""ZLAC8015D emulator whose wheels are the Gazebo wheels.

The CANopen behaviour (NMT, heartbeat, SDO, velocity mode, 2000h link-loss, faults) is the
vendored openamr-platform-fw emulator, unchanged. This subclass replaces only the plant:
the ramped speed demand of the drive goes to Gazebo, and 606Ch reports the wheel speeds
that Gazebo measures. Protocol evidence only. Nothing here shows that a real wheel stops.

Time (clock contract, docs/architecture/sil-topology.md section 6): the freshness of the Gazebo
wheel feedback is judged by receipt time on the given clock (C1, C6). A time reset or a stall of
that clock is a power cycle of the drive (the vendored ``power_cycle()``): targets zero, drive
disabled, boot-up frame. The vendored 2000h link-loss time and the node tick still read host
monotonic time until openamr-platform-fw gives them an injectable clock.

First clock (issue #160, same rule as the CTL-017 node, clock-reset contract R1, Proposed):
before the first ``/clock`` the drive is not powered. It sends no frame (boot-up, heartbeat),
drops every received frame and runs no step. At the first clock time it powers up through the
vendored ``power_cycle()`` (saved parameters restored, disabled, zero targets, boot-up frame).
A simulation that has not started has no powered device; no decision logic of the vendored
emulator changes.
"""
import math

from openamrobot_sil.can_sil import zlac8015d_emulator
from openamrobot_sil.sim_time import ClockWatch, Stamp, SteadyClock, is_fresh

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


class GazeboZlac8015d(zlac8015d_emulator.Zlac8015dEmulator):
    """One ZLAC8015D node. Sub-index 1 is the left axis and 2 the right axis in this layer.

    The vendor documents do not settle which sub-index is which wheel or the sign per axis
    (actuator ledger Q-02). left_sign and right_sign are SIL parameters, default +1.
    """

    def __init__(self, bus, node_id=1, heartbeat_ms=100, left_sign=1, right_sign=1,
                 feedback_timeout_s=0.5, clock=None):
        self.clock = clock if clock is not None else SteadyClock()
        self._clock_watch = ClockWatch(self.clock)
        self.clock_resets = 0
        self.clock_stalls = 0
        self.powered = self.clock.fed   # False until the first clock time (#160, R1)
        self.pre_clock_drops = 0        # frames received before the first clock
        super().__init__(bus, node_id, heartbeat_ms)
        self.signs = (float(left_sign), float(right_sign))
        self.feedback_timeout_s = feedback_timeout_s
        self._feedback_rad_s = None
        self._feedback_stamp = None

    def set_wheel_feedback(self, left_rad_s, right_rad_s):
        """Wheel joint speeds from Gazebo, rad/s, positive forward."""
        with self.lock:
            self._feedback_rad_s = (float(left_rad_s), float(right_rad_s))
            self._feedback_stamp = Stamp(self.clock)

    def feedback_fresh(self):
        return (self._feedback_rad_s is not None and
                is_fresh(self._feedback_stamp, self.clock, self.feedback_timeout_s))

    def _power_on(self):
        """First clock time: power-up. Returns False while the clock is not fed yet."""
        if self.powered:
            return True
        if not self.clock.fed:
            return False
        self.powered = True
        self._clock_watch = ClockWatch(self.clock)
        self.power_cycle()   # vendored: saved parameters, disabled, zero targets, boot-up
        return True

    def _send(self, arb_id, data):
        if not self.powered:
            return  # not powered before the first clock: no boot-up, no heartbeat
        super()._send(arb_id, data)

    def _follow_clock(self):
        """A reset or a stall of the clock is a power cycle: nothing from before it survives."""
        event = self._clock_watch.poll()
        if event == 'reset':
            self.clock_resets += 1
            self.power_cycle()
        elif event == 'stall':
            self.clock_stalls += 1
            self.power_cycle()

    def demand_rad_s(self):
        """Speed the drive asks of each wheel now (left, right), rad/s, positive forward."""
        with self.lock:
            return tuple(self.signs[i] * self.rpm[i] * RPM_TO_RAD_S for i in (0, 1))

    def _handle(self, msg):
        if not self._power_on():
            self.pre_clock_drops += 1   # not powered yet: the frame is lost
            return
        self._follow_clock()  # a frame received after a reset or stall meets the new state
        super()._handle(msg)

    def tick(self, dt):
        if not self._power_on():
            return
        self._follow_clock()
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
                'clock_resets': self.clock_resets,
                'clock_stalls': self.clock_stalls,
                'powered': self.powered,
                'pre_clock_drops': self.pre_clock_drops,
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
