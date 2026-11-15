"""ZLAC8015D emulator whose wheels are the Gazebo wheels.

The CANopen behaviour (NMT, heartbeat, SDO, velocity mode, 2000h link-loss, faults) is the
vendored openamr-platform-fw emulator, unchanged. This subclass replaces only the plant:
the ramped speed demand of the drive goes to Gazebo, and 606Ch reports the wheel speeds
that Gazebo measures. Protocol evidence only. Nothing here shows that a real wheel stops.
"""
import math
import struct
import time

from openamrobot_sil.can_sil import zlac8015d_emulator

RPM_TO_RAD_S = 2.0 * math.pi / 60.0

# SIL receive PDOs for stm32:=renode_readonly. SIL test values, not a proposal and not the
# real drive's PDO mapping (no mapping is recorded for the ZLAC8015D in this project). They let
# the host test client command the drive without any SDO, so the SDO channel of the drive is
# the firmware's alone.
#   RPDO1 200h + node, 8 bytes: 60FFh:01 int32, 60FFh:02 int32 (target velocity, r/min)
#   RPDO2 300h + node, 2 bytes: 6040h controlword
RPDO1_BASE = 0x200
RPDO2_BASE = 0x300


class GazeboZlac8015d(zlac8015d_emulator.Zlac8015dEmulator):
    """One ZLAC8015D node. Sub-index 1 is the left axis and 2 the right axis in this layer.

    The vendor documents do not settle which sub-index is which wheel or the sign per axis
    (actuator ledger Q-02). left_sign and right_sign are SIL parameters, default +1.
    """

    def __init__(self, bus, node_id=1, heartbeat_ms=100, left_sign=1, right_sign=1,
                 feedback_timeout_s=0.5, rpdo=False, stored_link_loss_ms=None):
        super().__init__(bus, node_id, heartbeat_ms)
        self.signs = (float(left_sign), float(right_sign))
        self.feedback_timeout_s = feedback_timeout_s
        self._feedback_rad_s = None
        self._feedback_time = 0.0
        self.rpdo = bool(rpdo)
        self.rpdo_frames = 0
        self.rpdo_rejected = 0
        if stored_link_loss_ms is not None:
            # As if 2000h had been written and saved with 2010h before this run: the value is
            # in the emulated EEPROM and active from power-up, with no SDO write in the run.
            self.od.set(0x2000, 0, int(stored_link_loss_ms))
            self.eeprom[(0x2000, 0)] = int(stored_link_loss_ms)

    def on_rx_frame(self, msg):
        super().on_rx_frame(msg)
        if not self.rpdo or msg.is_extended_id or msg.is_remote_frame:
            return
        if self.nmt_state != zlac8015d_emulator.NMT_OPERATIONAL:
            return  # CiA 301: PDOs only in OPERATIONAL
        data = bytes(msg.data)
        if msg.arbitration_id == RPDO1_BASE + self.node_id and len(data) == 8:
            self.rpdo_frames += 1
            for sub, value in zip((1, 2), struct.unpack('<ii', data)):
                if self.on_write(0x60FF, sub, value):
                    self.rpdo_rejected += 1
        elif msg.arbitration_id == RPDO2_BASE + self.node_id and len(data) >= 2:
            self.rpdo_frames += 1
            if self.on_write(0x6040, 0, struct.unpack('<H', data[:2])[0]):
                self.rpdo_rejected += 1

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
                'rpdo': self.rpdo,
                'rpdo_frames': self.rpdo_frames,
                'rpdo_rejected': self.rpdo_rejected,
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
