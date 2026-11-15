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
                 feedback_timeout_s=0.5, cia402_statusword=False, commissioned_link_loss_ms=None):
        super().__init__(bus, node_id, heartbeat_ms)
        self.signs = (float(left_sign), float(right_sign))
        self.feedback_timeout_s = feedback_timeout_s
        self._feedback_rad_s = None
        self._feedback_time = 0.0
        # Off by default, so the stm32:=host mode keeps the vendored behaviour. With
        # cia402_statusword the statusword shows the CiA 402 states that the reference's
        # enable sequence checks (can1-drive-link.md 5.3: 0021h, 0023h, 0027h under mask
        # 006Fh, switch-on disabled 0040h under 004Fh). The vendored emulator sets those bits
        # only when enabled. SIL behaviour, not a model of the real drive.
        self.cia402_statusword = bool(cia402_statusword)
        self.cia_state = 'SOD'
        self.transition_delay_s = 0.0   # failure injection: statusword lags the controlword
        self.abort_controlword = None   # failure injection: this controlword is aborted
        self._pending = None            # (due time, cia state, enabled)
        if commissioned_link_loss_ms is not None:
            # A commissioned drive (can1-drive-link.md 5.2): 2000h written and saved.
            self.od.set(0x2000, 0, int(commissioned_link_loss_ms))
            self.eeprom[(0x2000, 0)] = int(commissioned_link_loss_ms)

    # --- SIL failure injection for the reference's enable sequence --------------------
    def inject_transition_delay(self, delay_s):
        with self.lock:
            self.transition_delay_s = max(0.0, float(delay_s))

    def inject_abort_controlword(self, word):
        with self.lock:
            self.abort_controlword = word

    def on_write(self, index, sub, value):
        if index == 0x6040 and self.abort_controlword is not None and value == self.abort_controlword:
            return zlac8015d_emulator.ABORT_GENERAL
        if index == 0x6040 and self.cia402_statusword:
            before_enabled = self.enabled
            result = super().on_write(index, sub, value)
            if result is None:
                if value & 0x80:
                    nxt = 'SOD'
                else:
                    nxt = {0x06: 'RTSO', 0x07: 'SO', 0x0F: 'OE', 0x00: 'SOD'}.get(value & 0x0F)
                if nxt == 'OE' and self.faults:
                    nxt = None
                if nxt is not None and self.transition_delay_s > 0:
                    # the transition, and the enable it carries, happen only after the delay
                    self._pending = (time.monotonic() + self.transition_delay_s, nxt, self.enabled)
                    self.enabled = before_enabled
                elif nxt is not None:
                    self.cia_state = nxt
            return result
        return super().on_write(index, sub, value)

    def power_cycle(self):
        with self.lock:
            self.cia_state = 'SOD'
            self._pending = None
        super().power_cycle()

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
        if self._pending is not None and time.monotonic() >= self._pending[0]:
            _, self.cia_state, self.enabled = self._pending
            self._pending = None
        super().tick(dt)
        if self.cia402_statusword:
            if not self.enabled and self.cia_state == 'OE':
                self.cia_state = 'SOD' if self.faults else 'RTSO'
            if self.faults:
                sw = 0x0018
            else:
                sw = 0x0010 | {'SOD': 0x0060, 'RTSO': 0x0021, 'SO': 0x0023, 'OE': 0x0027}[self.cia_state]
                if self.quick_stop:
                    sw &= ~0x0020
            self.od.set(0x6041, 0, sw)
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
                'cia_state': self.cia_state if self.cia402_statusword else None,
                'transition_delay_s': self.transition_delay_s,
                'abort_controlword': self.abort_controlword,
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
