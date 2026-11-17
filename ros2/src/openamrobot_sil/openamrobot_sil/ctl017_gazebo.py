"""CTL-017 lift node emulator whose lift is the Gazebo lift joint.

The CANopen behaviour and the lift model (commands, stop sensors, Hall count, faults) are
the vendored openamr-platform-fw emulator, unchanged. It follows the CTL-017 v1 object
dictionary of openamr-upperbody-fw PR #15, which is a proposal. This subclass adds two things:

1. The plant. The lift position comes from the Gazebo lift joint and the motor command goes
   to it. The Hall count and the three stop sensors follow that position.
2. The permit gate, off unless a PermitConfig is given. The v1 dictionary has no permit
   object. The rules below are the ones the permit contract proposal of openamr-upperbody-fw
   (docs/ctl017_permit_contract.md, branch night-shift/fw2-ctl017-infra-2026-11-14, 8dc7236)
   lists as fixed by the owner decision on platform-fw #20. COB-ID, field widths, the permit
   code, the timeout and the refusal result are not decided anywhere. PermitConfig carries
   them as SIL test values and they are not proposals.

Time (clock contract, docs/architecture/sil-topology.md section 6, rule C1): every CTL-017 timer
(Hall rate window, stall, move deadline, keep-alive, heartbeat consumer, standstill, permit
freshness) reads the vendored ``_now_ms()`` hook, which this subclass points at the clock it is
given: the process ``FedClock`` fed from Gazebo ``/clock`` in a SIL launch, host steady time
for the internal plant. The timeout values are not changed (rule C3). A time reset is a power
cycle of the node; a stall halts the lift and invalidates the permit, so motion needs a new
command after the resume.

Emulation of documented device behaviour only. Not a safety function.
"""
import struct

from openamrobot_sil.can_sil import canopen_common, ctl017_emulator as c
from openamrobot_sil.sim_time import ClockWatch, SteadyClock

# Joint position 0 m is the L1000 stop. The v1 dictionary puts L1000 5 mm above the lower
# end (0x2100:01 = 5000 um), L1175 at 180 mm and L1350 at 355 mm (0x2100:02, :03).
JOINT_ZERO_MM = c.STOP_POS_MM[0]
# Position changes below this are ignored. SIL value: a tenth of one Hall count (0.2092 mm).
POSITION_DEADBAND_MM = 0.02


class PermitConfig:
    """Wire layout and timing of the lift permit PDO. SIL test values, not proposals."""

    def __init__(self, cob_id, flag_bits, boot_bits, seq_bits, reason_bits, permit_value,
                 timeout_ms, refused_result):
        for bits in (flag_bits, boot_bits, seq_bits, reason_bits):
            if bits not in (8, 16, 32):
                raise ValueError('permit field widths are 8, 16 or 32 bits')
        if flag_bits + boot_bits + seq_bits + reason_bits > 64:
            raise ValueError('permit fields exceed 8 bytes')
        if not 0x081 <= cob_id <= 0x57F:
            raise ValueError('permit COB-ID must lie in 0x081 to 0x57F')
        if cob_id in (c.EMCY, c.RPDO1, c.RPDO2, c.TPDO1, c.TPDO2):
            raise ValueError('permit COB-ID collides with a CTL-017 object')
        if permit_value == 0:
            raise ValueError('0 always means no permit')
        if timeout_ms <= 0:
            raise ValueError('permit timeout must be above 0')
        self.cob_id = cob_id
        self.widths = (flag_bits, boot_bits, seq_bits, reason_bits)
        self.permit_value = permit_value
        self.timeout_ms = timeout_ms
        self.refused_result = refused_result
        self.fmt = '<' + ''.join({8: 'B', 16: 'H', 32: 'I'}[b] for b in self.widths)
        self.length = struct.calcsize(self.fmt)

    @classmethod
    def from_dict(cls, d):
        return cls(d['cob_id'], d['flag_bits'], d['boot_bits'], d['seq_bits'],
                   d['reason_bits'], d['permit_value'], d['timeout_ms'], d['refused_result'])

    def pack(self, flag, boot, seq, reason):
        masks = [(1 << b) - 1 for b in self.widths]
        return struct.pack(self.fmt, flag & masks[0], boot & masks[1], seq & masks[2],
                           reason & masks[3])

    def unpack(self, data):
        return struct.unpack(self.fmt, bytes(data[:self.length]))


class GazeboCtl017(c.Ctl017Emulator):
    def __init__(self, bus, node_id=c.NODE_ID, start_pos_mm=None, operational=False,
                 permit=None, clock=None):
        # Set before the vendored constructor: its _reinit() reads _now_ms().
        self.clock = clock if clock is not None else SteadyClock()
        self._clock_watch = ClockWatch(self.clock)
        self.clock_resets = 0
        self.clock_stalls = 0
        self.permit_cfg = permit
        self.external = False
        self._joint_mm = None
        self.cmd_velocity_mm_s = 0.0
        self.permit_refusals = 0
        self.permit_stops = 0
        self._permit_reset()
        super().__init__(bus, node_id, start_pos_mm, operational)

    # --- time: the clock contract ----------------------------------------
    def _now_ms(self):
        """Overrides the vendored hook: every CTL-017 timer reads the given clock (C1)."""
        return clock_ms(self.clock)

    def _follow_clock(self):
        """Apply a reset or a stall of the clock before this step uses the time."""
        event = self._clock_watch.poll()
        if event == 'reset':
            # Time moved backward: power cycle. Boot-up frame, PRE-OPERATIONAL, targets and
            # permit dropped, unhomed unless the lift stands at a stop sensor.
            self.clock_resets += 1
            self.power_cycle()
        elif event in ('stall', 'resume'):
            # Pause longer than the stall bound: the vendored STOP command, and the permit is
            # invalid until a new permit frame arrives. Repeated at the resume, so a frame or
            # command received during the pause does not carry over. No fault is raised.
            if event == 'stall':
                self.clock_stalls += 1
            self._cmd_stop()
            self.permit_valid = False

    def tick(self, dt):
        self._follow_clock()
        super().tick(dt)

    # --- plant: the Gazebo lift joint -------------------------------------
    def attach_external_plant(self, joint_position_m):
        """Take the lift position from Gazebo from now on. Call before start()."""
        with self.lock:
            self.external = True
            self._joint_mm = JOINT_ZERO_MM + joint_position_m * 1000.0
            self.true_mm = min(c.TRAVEL_MM, max(0.0, self._joint_mm))
            self._start_pos_mm = self.true_mm

    def set_joint_position(self, joint_position_m):
        with self.lock:
            self._joint_mm = JOINT_ZERO_MM + joint_position_m * 1000.0

    def demand_velocity_m_s(self):
        """Lift speed the motor model asks for now, m/s, positive up."""
        with self.lock:
            return self.cmd_velocity_mm_s / 1000.0

    def _plant(self, dt):
        driving = self.drive_on and self.motor_power
        duty = self.param(c.P_DUTY_HOMING if self.state == c.HOMING else c.P_DUTY_MOVE)
        self.cmd_velocity_mm_s = (c.FULL_SPEED_MM_S * duty / 1000.0 * self.dir
                                  if driving else 0.0)
        if not self.external:
            super()._plant(dt)
            return
        new_true = min(c.TRAVEL_MM, max(0.0, self._joint_mm))
        actual = new_true - self.true_mm
        if abs(actual) < POSITION_DEADBAND_MM:
            return  # solver noise of a resting joint must not look like Hall pulses
        self.true_mm = new_true
        if not self.hall_dead:
            self.count_f += (actual / c.MM_PER_HALL * self.hall_rate_pct / 100.0
                             * self.hall_sign * self.hall_dir)

    # --- permit gate --------------------------------------------------------
    def _permit_reset(self):
        self.permit_valid = False   # false at boot
        self.permit_rx_ms = 0
        self.permit_boot = None
        self.permit_seq = None
        self.permit_reason = 0
        self.permit_rx_count = 0

    def _reinit(self):
        self._permit_reset()
        super()._reinit()

    def permit_fresh(self, now=None):
        if self.permit_cfg is None:
            return True  # v1 baseline: no permit object, no gate
        now = self._now_ms() if now is None else now
        return self.permit_valid and now - self.permit_rx_ms < self.permit_cfg.timeout_ms

    def _nmt(self, cmd):
        super()._nmt(cmd)
        self.permit_valid = False  # a permit seen before an NMT transition is not kept

    def on_rx_frame(self, msg):
        self._follow_clock()
        cfg = self.permit_cfg
        if cfg is None or msg.arbitration_id != cfg.cob_id:
            super().on_rx_frame(msg)
            return
        if self.nmt_state != canopen_common.NMT_OPERATIONAL:
            return
        if len(msg.data) < cfg.length:
            self.rpdo_len_errors += 1  # too short: no refresh
            return
        flag, boot, seq, reason = cfg.unpack(msg.data)
        self.permit_rx_count += 1
        boot_changed = self.permit_boot is not None and boot != self.permit_boot
        self.permit_boot, self.permit_seq, self.permit_reason = boot, seq, reason
        # A new producer boot withdraws the permit. This frame is not trusted. The next
        # frame of the new boot is (SIL assumption: the contract leaves the rule open).
        self.permit_valid = flag == cfg.permit_value and not boot_changed
        if self.permit_valid:
            self.permit_rx_ms = self._now_ms()

    def _cmd_move(self, stop, now):
        if not self.permit_fresh(now):
            self.permit_refusals += 1
            return self.permit_cfg.refused_result
        return super()._cmd_move(stop, now)

    def _cmd_home(self, now):
        if not self.permit_fresh(now):
            self.permit_refusals += 1
            return self.permit_cfg.refused_result
        return super()._cmd_home(now)

    def _check_drive(self, now):
        if not self.permit_fresh(now):
            # Withdrawn or expired while moving: halt and drop the target. No fault and no
            # replay: motion needs a new command after a new permit.
            self.permit_stops += 1
            self._drive_stop()
            self._discard_target()
            self.state = c.STOPPING
            return
        super()._check_drive(now)

    def state_dict(self):
        with self.lock:
            now = self._now_ms()
            return {
                'node_id': self.node_id,
                'nmt_state': self.nmt_state,
                'state': self.state,
                'flags': self.flags(now),
                'position_mm': round(self.position_mm(), 2) if self.position_valid else None,
                'true_mm': round(self.true_mm, 2),
                'hall_count': self.hall_count,
                'stop_sensors': self.sensor_bits(),
                'active_stop': self._active_stop_id(now),
                'target_stop': self.target_stop,
                'cmd_result': self.cmd_result,
                'faults': self.faults,
                'drive_on': self.drive_on,
                'cmd_velocity_mm_s': self.cmd_velocity_mm_s,
                'plant': 'gazebo' if self.external else 'internal',
                'permit_gate': self.permit_cfg is not None,
                'permit_fresh': self.permit_fresh(now),
                'permit_rx_count': self.permit_rx_count,
                'permit_refusals': self.permit_refusals,
                'permit_stops': self.permit_stops,
                'clock_resets': self.clock_resets,
                'clock_stalls': self.clock_stalls,
            }


def clock_ms(clock):
    """Milliseconds of a clock of the contract, the time base of every CTL-017 timer."""
    return int(clock.now() * 1000)
