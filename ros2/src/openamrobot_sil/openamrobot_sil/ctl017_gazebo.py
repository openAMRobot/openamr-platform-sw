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

Emulation of documented device behaviour only. Not a safety function.
"""
import struct
import time

from openamrobot_sil.can_sil import canopen_common, ctl017_emulator as c

# Joint position 0 m is the L1000 stop. The v1 dictionary puts L1000 5 mm above the lower
# end (0x2100:01 = 5000 um), L1175 at 180 mm and L1350 at 355 mm (0x2100:02, :03).
JOINT_ZERO_MM = c.STOP_POS_MM[0]
# Position changes below this are ignored. SIL value: a tenth of one Hall count (0.2092 mm).
POSITION_DEADBAND_MM = 0.02
# State names in the order of the emulator's state constants, for the start-up diagnostics.
STATE_NAMES = ('INIT', 'UNHOMED', 'HOMING', 'IDLE', 'MOVING', 'STOPPING', 'FAULT')


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
    # Optional observer, callable(str), for start-up diagnostics. It only reads state; its
    # failure is swallowed, and with none set nothing here changes the emulator.
    diag = None

    def __init__(self, bus, node_id=c.NODE_ID, start_pos_mm=None, operational=False,
                 permit=None):
        self.permit_cfg = permit
        self.external = False
        self._joint_mm = None
        self.cmd_velocity_mm_s = 0.0
        self.permit_refusals = 0
        self.permit_stops = 0
        self._permit_reset()
        super().__init__(bus, node_id, start_pos_mm, operational)

    # --- plant: the Gazebo lift joint -------------------------------------
    def attach_external_plant(self, joint_position_m):
        """Take the lift position from Gazebo from now on. Call before start()."""
        with self.lock:
            self.external = True
            self._joint_mm = JOINT_ZERO_MM + joint_position_m * 1000.0
            self.true_mm = min(c.TRAVEL_MM, max(0.0, self._joint_mm))
            self._start_pos_mm = self.true_mm
            self._diag('attach joint_m=%.6f joint_mm=%.3f true_mm=%.3f'
                       % (joint_position_m, self._joint_mm, self.true_mm))

    # --- start-up diagnostics (observe only) ------------------------------
    def _diag(self, text):
        fn = self.diag
        if fn is None:
            return
        try:
            fn(text)
        except Exception:
            pass    # a diagnostics failure must never change the emulator

    def _diag_init(self, when):
        if self.diag is None:
            return
        try:
            offsets = [round(self.true_mm - p, 3) for p in c.SENSOR_POS_MM]
            inside = [abs(o) <= c.SENSOR_WINDOW_MM for o in offsets]
            self._diag(
                'finish_init %s init_age_ms=%d state=%s homed=%s position_valid=%s joint_mm=%s '
                'true_mm=%.3f sensor_bits=0b%s offsets_mm=%s window_mm=%.1f in_window=%s '
                'hall_supply=%s faults=%s both_limits=%s' % (
                    when, self._now_ms() - self.t0, STATE_NAMES[self.state], self.homed,
                    self.position_valid,
                    None if self._joint_mm is None else round(self._joint_mm, 3), self.true_mm,
                    format(self.sensor_bits(), '03b'), offsets, c.SENSOR_WINDOW_MM, inside,
                    self.hall_supply, self.faults, self.both_limits()))
        except Exception:
            pass

    def _finish_init(self):
        self._diag_init('before')
        super()._finish_init()
        self._diag_init('after')

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
            }


def wall_ms():
    return int(time.monotonic() * 1000)
