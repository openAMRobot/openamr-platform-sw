#!/usr/bin/env python3
"""Software-in-the-loop emulator of the upper-body controller CTL-017 (CANopen node 0x10, CAN3).

PROPOSAL, TODO(Evans). The object dictionary, PDO layouts, command checks and fault rules follow
the CTL-017 v1 proposal in openamr-upperbody-fw PR #15 (docs/ctl017_object_dictionary.md), which
in turn follows the CAN3 lift message set (platform-fw #12, decisions D1 to D20) and P-03 items
29 and 30 A7. The dictionary itself is generated into ctl017_od_v1.py by gen_ctl017_od.py. This
emulator follows the #15 proposal pending Evans: when he adopts or changes the dictionary,
regenerate the module and adjust the behaviour here. Not a safety function: the hardwired E-stop
removes lift motor power independently of CAN3.

Frames (node 0x10, classic CAN, 500 kbit/s):
  NMT 0x000 in; EMCY 0x090 out; heartbeat 0x710 out (100 ms); core heartbeat 0x701 consumed (300 ms)
  TPDO1 0x190  state, flags, position u16 (0.1 mm, 0xFFFF unknown), active_stop_id, last_seq,
               cmd_result, status_counter. On change of bytes 0 to 6 (inhibit 10 ms) and every 100 ms.
  TPDO2 0x290  inputs (bit 0..2 stop sensors L1000 L1175 L1350, bit 3 second E-stop, bit 4 Hall
               supply good, bit 5..7 spare), target_stop_id, motor_current u16 mA, velocity i16
               0.1 mm/s, faults F01..F08 u8, aux flags u8 (bit 0 F09, bit 1 light, bit 2 Getac).
  RPDO1 0x210  seq, op (0 STOP, 1 MOVE_TO_STOP, 2 CLEAR_FAULT, 3 START_HOMING), stop_id (3 bytes)
  RPDO2 0x310  head light on, dim level 0..100, Getac enable (3 bytes)
  SDO 0x610/0x590 expedited. PDOs run only in OPERATIONAL (the core sends NMT start).
Supervision (D12 alternative a): during a move or homing the lift stops with F07 when the RPDO1
keep-alive (0x1400:05, 300 ms) or the core heartbeat (0x1016:01, 300 ms, monitored after the first
heartbeat) is missing.

Usage: python3 ctl017_emulator.py --channel vcan0 [--operational]
As a library: see tests/test_ctl017.py. Everything marked TODO(Evans) is a placeholder.
"""
import argparse
import struct
import time

import can

from canopen_common import (ABORT_BAD_VALUE, ABORT_DEVICE_STATE, HB_BASE, NMT_OPERATIONAL,
                            NMT_PRE_OPERATIONAL, CanopenNode, ObjectDict)
from ctl017_od_v1 import OD_V1

NODE_ID = 0x10
CORE_NODE_ID = 0x01
RPDO1 = 0x200 + NODE_ID
RPDO2 = 0x300 + NODE_ID
TPDO1 = 0x180 + NODE_ID
TPDO2 = 0x280 + NODE_ID
EMCY = 0x080 + NODE_ID

# states
INIT, UNHOMED, HOMING, IDLE, MOVING, STOPPING, FAULT = range(7)
# ops
OP_STOP, OP_MOVE_TO_STOP, OP_CLEAR_FAULT, OP_START_HOMING = range(4)
# TPDO1 flags
F_POSITION_VALID, F_AT_STOP, F_MOVING, F_LOWER, F_UPPER, F_FAULT, F_MOTOR_POWER, F_RECOVERY = (
    1 << i for i in range(8))
# TPDO2 inputs
IN_STOP0, IN_STOP1, IN_STOP2, IN_ESTOP2, IN_HALL_SUPPLY = (1 << i for i in range(5))
# TPDO2 aux flags
AUX_F09, AUX_LIGHT_ON, AUX_GETAC = 1, 2, 4
# cmd_result
(R_OK, R_BAD_STOP, R_POS_INVALID, R_FAULT, R_LIMIT, R_BUSY, R_RECOVERY, R_CLEAR_UNMET,
 R_HOMING_UNMET, R_BAD_OP, R_SEQ_CONFLICT, R_NO_MOTOR_POWER) = range(12)
R_NONE = 0xFF
# fault ids (bit n-1 in the mask)
F01, F02, F03, F04, F05, F06, F07, F08, F09 = range(1, 10)
INVALIDATES_POSITION = (1 << (F01 - 1)) | (1 << (F02 - 1)) | (1 << (F05 - 1)) | (1 << (F09 - 1))

# Plant (simulation values, not hardware facts)
FULL_SPEED_MM_S = 18.0      # P-03 item 16 no-load lift speed, TODO(Evans) loaded
TRAVEL_MM = 360.0           # test value: 5 mm + 350 mm span + 5 mm, TODO(Evans) real stroke
SENSOR_POS_MM = [5.0, 180.0, 355.0]  # TODO(Evans): physical sensor positions, calibrate
SENSOR_WINDOW_MM = 1.0      # TODO(Evans): IME12 switching window as mounted
MM_PER_HALL = 0.2092        # TODO(Evans): confirm for the ordered column model
MOVING_CURRENT_MA = 1000    # simulation placeholder only, TODO(Evans): measure on the motor
INIT_SETTLE_MS = 50         # TODO(Evans) (#15 init_settle_ms)
DEFAULT_START_MM = 100.0    # between L1000 and L1175
SUPERVISION_MS = 300        # D15 (defaults in the OD: 0x1400:05 and 0x1016:01)
STOP_POS_MM = [p / 1000.0 for p in (5000, 180000, 355000)]  # OD defaults 0x2100:01..03

_SIZES = {"UNSIGNED8": (1, False), "BOOLEAN": (1, False), "UNSIGNED16": (2, False),
          "UNSIGNED32": (4, False), "INTEGER16": (2, True), "INTEGER32": (4, True)}
_ACCESS = {"const": "ro", "ro": "ro", "rw": "rw", "rw (PRE-OP only)": "preop",
           "rww (RPDO only)": "local"}


def build_od():
    od = ObjectDict()
    for index, sub, _name, typ, access, _pdo, default in OD_V1:
        size, signed = _SIZES[typ]
        od.add(index, sub, size, default, signed=signed, rw=_ACCESS[access])
    return od


# OD parameter names (index, sub) used by the model
P_HALL_NM = (0x2101, 1)
P_UP_POSITIVE = (0x2101, 2)
P_STOP_TOL = (0x2101, 3)
P_XCHECK_TOL = (0x2101, 4)
P_EDGE_UP = (0x2101, 5)
P_EDGE_DOWN = (0x2101, 6)
P_STOP_LEAD = (0x2101, 7)
P_OVERRUN = (0x2101, 8)
P_DUTY_MOVE = (0x2101, 9)
P_DUTY_HOMING = (0x2101, 10)
P_MOVE_SPEED = (0x2101, 11)
P_HOMING_SPEED = (0x2101, 12)
P_HALL_WINDOW = (0x2101, 13)
P_HALL_GRACE = (0x2101, 14)
P_MOVE_MIN = (0x2101, 15)
P_MOVE_MAX = (0x2101, 16)
P_HOME_MIN = (0x2101, 17)
P_HOME_MAX = (0x2101, 18)
P_STALL_MS = (0x2101, 19)
P_WRONG_DIR = (0x2101, 20)
P_TIMEOUT_PCT = (0x2101, 21)
P_TIMEOUT_MARGIN = (0x2101, 22)
P_OC_MA = (0x2101, 23)
P_OC_MS = (0x2101, 24)
P_INRUSH_MS = (0x2101, 25)
P_QUIET_MS = (0x2101, 27)
P_STANDSTILL_MS = (0x2101, 28)
P_END_WIRED = (0x2101, 29)
P_HOME_CONTINUE = (0x2101, 30)
P_REREF = (0x2101, 32)
P_STOP_POS = [(0x2100, 1), (0x2100, 2), (0x2100, 3)]
P_DEADLINE = (0x1400, 5)


class Ctl017Emulator(CanopenNode):
    """CTL-017 node. Thread-safe through the base class lock; inject faults through the
    inject_* and set_* methods."""

    sdo_when_stopped = False

    def __init__(self, bus, node_id=NODE_ID, start_pos_mm=None, operational=False):
        super().__init__(bus, node_id, 100, build_od())
        self._start_pos_mm = DEFAULT_START_MM if start_pos_mm is None else start_pos_mm
        self.true_mm = self._start_pos_mm
        # injections (the plant, not the firmware)
        self.hall_dead = False
        self.hall_rate_pct = 100
        self.hall_dir = 1
        self.forced_sensors = 0
        self.motor_power = True
        self.hall_supply = True
        self.estop2_pressed = False
        self.spare_in = [False, False, False]
        self.extra_current_ma = 0
        self.emcy_log = []          # fault numbers, in the order they were raised
        self.emcy_resets = 0
        self._operational_at_start = operational
        self._reinit()

    # --- (re)initialisation ------------------------------------------------
    def _reinit(self):
        self.t0 = self._now_ms()
        self.state = INIT
        self.count_f = 0.0
        self.last_count = 0
        self.ref_count = 0
        self.ref_pos_mm = 0.0
        self.homed = False
        self.position_valid = False
        self.faults = 0
        self.new_faults = 0
        self.last_faults = 0
        self.any_cmd = False
        self.last_seq = 0
        self.last_op = 0
        self.last_stop = 0
        self.cmd_result = R_NONE
        self.target_stop = 0xFF
        self.target_mm = 0.0
        self.drive_on = False
        self.dir = 0
        self.arriving = False
        self.f08_armed = False
        self.drive_on_ms = self.drive_off_ms = self.t0
        self.move_deadline_ms = 0
        self.win_start_ms = self.t0
        self.win_start_count = 0
        self.last_pulse_ms = self.t0
        self.dir_ref_count = 0
        self.over_active = False
        self.over_since_ms = 0
        self.last_count_change_ms = self.t0
        self.last_keepalive_ms = self.t0
        self.vel_t0_ms = self.t0
        self.vel_c0 = 0
        self.velocity = 0
        self.current_ma = 0
        self.stop_prev = [False] * 3
        self.supervision_lost = False
        self.hbc = "waiting"        # off, waiting, active, lost
        self.hbc_last_ms = self.t0
        self.core_hb_seen = False
        self.head_light_on = 0
        self.head_light_dim = 100   # TODO(Evans) default
        self.getac_enable = 0       # TODO(Evans) boot state
        self.counter = 0
        self.rpdo_len_errors = 0
        self.tpdo_last = [None, None]
        self.tpdo_last_ms = [0, 0]
        self.tpdo_sent_once = [False, False]
        self.last_prev_sensors = 0
        self.nmt_state = NMT_PRE_OPERATIONAL
        self.err_history = []
        self._od_reset_defaults()
        self.hbc_config()
        if self._operational_at_start:
            self.nmt_state = NMT_OPERATIONAL
        self._update_od()

    def _od_reset_defaults(self):
        for index, sub, _n, _t, _a, _p, default in OD_V1:
            self.od.set(index, sub, default)

    @staticmethod
    def _now_ms():
        return int(time.monotonic() * 1000)

    def param(self, key):
        return self.od.get(*key)

    # --- plant ---------------------------------------------------------------
    @property
    def hall_sign(self):
        return 1 if self.param(P_UP_POSITIVE) else -1

    @property
    def hall_count(self):
        return int(round(self.count_f))

    def position_mm(self):
        """Position from the Hall count and the last reference (what the firmware reports)."""
        dc = self.hall_sign * (self.hall_count - self.ref_count)
        return self.ref_pos_mm + dc * self.param(P_HALL_NM) / 1.0e6

    counted = property(position_mm)

    def sensors(self):
        return self.sensor_bits()

    def sensor_bits(self):
        bits = self.forced_sensors
        for i, p in enumerate(SENSOR_POS_MM):
            if abs(self.true_mm - p) <= SENSOR_WINDOW_MM:
                bits |= 1 << i
        return bits & 7

    def lower_limit(self):
        return bool(self.param(P_END_WIRED)) and self.true_mm <= 0.0

    def upper_limit(self):
        return bool(self.param(P_END_WIRED)) and self.true_mm >= TRAVEL_MM

    def both_limits(self):
        return self.lower_limit() and self.upper_limit()

    def stop_mm(self, k):
        return self.param(P_STOP_POS[k]) / 1000.0

    # --- injection API -----------------------------------------------------------
    def inject_stall(self, on=True):
        self.hall_dead = on

    def inject_hall_slip(self, mm):
        """Hall count drifts from the true position (lost or extra pulses)."""
        with self.lock:
            self.count_f += mm / MM_PER_HALL

    def inject_hall_rate(self, pct):
        self.hall_rate_pct = pct

    def inject_hall_reversed(self, on=True):
        self.hall_dir = -1 if on else 1

    def inject_current(self, ma):
        self.extra_current_ma = ma

    def set_motor_power(self, present):
        with self.lock:
            self.motor_power = present

    def set_hall_supply(self, ok):
        with self.lock:
            self.hall_supply = ok

    def set_estop2(self, pressed):
        self.estop2_pressed = pressed

    def power_cycle(self, start_pos_mm=None):
        """Node power loss: everything volatile returns to its default, the lift is unhomed."""
        with self.lock:
            if start_pos_mm is not None:
                self.true_mm = start_pos_mm
            self._operational_at_start = False
            self._reinit()
            self._send_boot()

    # --- NMT -----------------------------------------------------------------------
    def _nmt(self, cmd):
        was_op = self.nmt_state == NMT_OPERATIONAL
        if cmd == 0x81:
            self._cmd_stop()
            self.power_cycle()
            return
        super()._nmt(cmd)
        if cmd == 0x01 and not was_op:
            self.tpdo_sent_once = [False, False]
        if was_op and self.nmt_state != NMT_OPERATIONAL:
            self._cmd_stop()  # no RPDO can arrive any more: stop at once
        if cmd == 0x82:
            self.hbc_config()

    # --- heartbeat consumer ----------------------------------------------------------
    def hbc_config(self):
        entry = self.od.get(0x1016, 1)
        node, ms = (entry >> 16) & 0xFF, entry & 0xFFFF
        self.hbc = "off" if (ms == 0 or node == 0 or node > 127) else "waiting"
        self.supervision_lost = False

    def _consumer_node(self):
        return (self.od.get(0x1016, 1) >> 16) & 0xFF

    # --- SDO writes --------------------------------------------------------------------
    def motion_in_progress(self):
        return (self.drive_on or self.state in (MOVING, HOMING, STOPPING)
                or not self._standstill(self._now_ms()))

    def _config_valid(self):
        if self.param(P_HALL_NM) <= 0 or self.param(P_STOP_TOL) <= 0 or self.param(P_XCHECK_TOL) <= 0:
            return False
        pos = [self.param(k) for k in P_STOP_POS]
        if any(b <= a for a, b in zip(pos, pos[1:])) or pos[0] < 0 or pos[-1] > 6553000:
            return False
        return True

    def on_write(self, index, sub, value):
        key = (index, sub)
        if index in (0x2100, 0x2101) or key in (P_DEADLINE, (0x1800, 3), (0x1801, 3)):
            if self.motion_in_progress():
                return ABORT_DEVICE_STATE
            if index == 0x2101 and sub in (2, 29, 30, 31, 32) and value > 1:
                return ABORT_BAD_VALUE
            old = self.od.get(index, sub)
            self.od.set(index, sub, value)
            if not self._config_valid():
                self.od.set(index, sub, old)
                return ABORT_BAD_VALUE
            return None
        if index == 0x2011:
            if (sub == 2 and value > 100) or (sub != 2 and value > 1):
                return ABORT_BAD_VALUE
            self.od.set(index, sub, value)
            self._apply_aux()
            return None
        if key == (0x1003, 0):
            if value != 0:
                return ABORT_BAD_VALUE
            self.err_history = []
            self.od.set(0x1003, 0, 0)
            return None
        if key == (0x1016, 1):
            if (value & 0xFF000000) or (value >> 16) > 127:
                return ABORT_BAD_VALUE
            self.od.set(index, sub, value)
            self.hbc_config()
            return None
        if key == (0x1017, 0):
            self.od.set(index, sub, value)
            self.heartbeat_ms = value
            return None
        self.od.set(index, sub, value)
        return None

    def _apply_aux(self):
        self.head_light_on = self.od.get(0x2011, 1)
        self.head_light_dim = min(100, self.od.get(0x2011, 2))
        self.getac_enable = self.od.get(0x2011, 3)

    # --- helpers following the #15 lift core ------------------------------------------
    def _standstill(self, now):
        return now - self.last_count_change_ms >= self.param(P_QUIET_MS)

    def _sensor_count(self):
        return bin(self.sensor_bits()).count("1")

    def _rest_state(self):
        if self.faults:
            return FAULT
        if self.position_valid and self.homed and not self.both_limits():
            return IDLE
        return UNHOMED

    def _set_reference(self, pos_mm):
        self.ref_count = self.last_count
        self.ref_pos_mm = pos_mm

    def _drive_start(self, direction, timeout_ms, now):
        self.dir = direction
        self.drive_on = True
        self.arriving = False
        self.drive_on_ms = now
        self.move_deadline_ms = now + timeout_ms
        self.win_start_ms = now
        self.win_start_count = self.last_count
        self.last_pulse_ms = now
        self.dir_ref_count = self.last_count
        self.over_active = False
        self.f08_armed = False

    def _drive_stop(self, now=None):
        if self.drive_on:
            self.drive_on = False
            self.drive_off_ms = self._now_ms() if now is None else now
            self.f08_armed = True

    def _discard_target(self):
        self.target_stop = 0xFF
        self.arriving = False

    def _set_target(self, k):
        self.target_stop = k
        self.target_mm = self.stop_mm(k)

    def _raise(self, fault_id):
        bit = 1 << (fault_id - 1)
        self.new_faults |= bit & ~self.faults
        self.faults |= bit
        if bit & INVALIDATES_POSITION:
            self.position_valid = False
            self.homed = False
        self._drive_stop()
        self._discard_target()
        self.state = FAULT

    def _timeout_ms(self, distance_mm, speed_um_s):
        t = abs(distance_mm) * 1000.0 / speed_um_s * 1000.0  # ms
        t = int(t) * self.param(P_TIMEOUT_PCT) // 100 + self.param(P_TIMEOUT_MARGIN)
        return int(t)

    def _active_stop(self, now):
        if not self.position_valid or self.drive_on or not self._standstill(now):
            return -1
        pos = self.position_mm()
        tol = self.param(P_STOP_TOL) / 1000.0
        sensors = self.sensor_bits()
        for k in range(3):
            if sensors & (1 << k) and abs(pos - self.stop_mm(k)) <= tol:
                return k
        return -1

    # --- commands (RPDO1) ---------------------------------------------------------------
    def _command(self, seq, op, stop, now):
        if self.any_cmd and seq == self.last_seq:
            if op == self.last_op and stop == self.last_stop:
                self.last_keepalive_ms = now  # keep-alive: refresh, do not execute again
                return
            self.cmd_result = R_SEQ_CONFLICT  # no refresh, no state change
            return
        self.any_cmd = True
        self.last_seq, self.last_op, self.last_stop = seq, op, stop
        if op == OP_STOP:
            res = self._cmd_stop()
        elif op == OP_MOVE_TO_STOP:
            res = self._cmd_move(stop, now)
        elif op == OP_CLEAR_FAULT:
            res = self._cmd_clear(now)
        elif op == OP_START_HOMING:
            res = self._cmd_home(now)
        else:
            res = R_BAD_OP
        if res == R_OK:
            self.last_keepalive_ms = now
        self.cmd_result = res

    def _cmd_stop(self):
        if self.drive_on or self.state in (MOVING, HOMING):
            self._drive_stop()
            self._discard_target()
            if self.state != FAULT:
                self.state = STOPPING
        else:
            self._discard_target()
        return R_OK

    def _cmd_move(self, stop, now):
        if stop >= 3:
            return R_BAD_STOP
        if self.faults:
            return R_FAULT
        if self.state == INIT or self.motion_in_progress():
            return R_BUSY
        if not self.homed:
            return R_RECOVERY
        if not self.position_valid:
            return R_POS_INVALID
        if self._active_stop(now) == stop:
            return R_OK  # already there: no-op
        if not self.motor_power:
            return R_NO_MOTOR_POWER
        diff = self.stop_mm(stop) - self.position_mm()
        direction = 1 if diff >= 0 else -1
        if (direction > 0 and self.upper_limit()) or (direction < 0 and self.lower_limit()):
            return R_LIMIT
        self._set_target(stop)
        self.state = MOVING
        self._drive_start(direction, self._timeout_ms(diff, self.param(P_MOVE_SPEED)), now)
        return R_OK

    def _cmd_clear(self, now):
        if self.motion_in_progress():
            return R_BUSY
        if not self.faults:
            return R_OK  # D9: accepted no-op
        keep = 0
        if self.both_limits():
            keep |= 1 << (F01 - 1)
        if self._sensor_count() > 1:
            keep |= 1 << (F09 - 1)
        if self._current_ma() > self.param(P_OC_MA):
            keep |= 1 << (F03 - 1)
        if not self.hall_supply:
            keep |= 1 << (F05 - 1)
        self.faults &= keep
        self.state = self._rest_state()
        return R_CLEAR_UNMET if self.faults else R_OK

    def _cmd_home(self, now):
        if self.faults:
            return R_FAULT
        if self.state == INIT or self.motion_in_progress():
            return R_BUSY
        n = self._sensor_count()
        if self.both_limits() or not self.hall_supply or n > 1:
            return R_HOMING_UNMET
        if n == 1:
            k = [i for i in range(3) if self.sensor_bits() & (1 << i)][0]
            if not self.homed:
                self._set_reference(self.stop_mm(k))
                self.homed = True
                self.position_valid = True
            if k == 0 or not self.param(P_HOME_CONTINUE):
                self.state = self._rest_state()
                return R_OK
        if not self.motor_power:
            return R_NO_MOTOR_POWER
        if self.lower_limit():
            return R_LIMIT
        span = self.stop_mm(2) - self.stop_mm(0) + 2 * self.param(P_STOP_TOL) / 1000.0
        if self.homed and self.param(P_HOME_CONTINUE):
            self._set_target(0)
        else:
            self._discard_target()
        self.state = HOMING
        self._drive_start(-1, self._timeout_ms(span, self.param(P_HOMING_SPEED)), now)
        return R_OK

    # --- frames in -------------------------------------------------------------------------
    def on_rx_frame(self, msg):
        now = self._now_ms()
        if msg.arbitration_id == HB_BASE + self._consumer_node() and self.hbc != "off" and len(msg.data) >= 1:
            self.hbc_last_ms = now
            if self.hbc == "lost":
                self.supervision_lost = False
            self.hbc = "active"
            return
        if self.nmt_state != NMT_OPERATIONAL:
            return
        if msg.arbitration_id == RPDO1:
            if len(msg.data) < 3:
                self.rpdo_len_errors += 1
                return
            seq, op, stop = msg.data[:3]
            self.od.set(0x2010, 1, seq)
            self.od.set(0x2010, 2, op)
            self.od.set(0x2010, 3, stop)
            self._command(seq, op, stop, now)
        elif msg.arbitration_id == RPDO2:
            if len(msg.data) < 3:
                self.rpdo_len_errors += 1
                return
            self.od.set(0x2011, 1, 1 if msg.data[0] else 0)
            self.od.set(0x2011, 2, min(100, msg.data[1]))
            self.od.set(0x2011, 3, 1 if msg.data[2] else 0)
            self._apply_aux()

    # --- control step (mirrors lift_step) -------------------------------------------------
    def _current_ma(self):
        driving = self.drive_on and self.motor_power
        return (MOVING_CURRENT_MA if driving else 0) + self.extra_current_ma

    def _plant(self, dt):
        if self.drive_on and self.motor_power:
            duty = self.param(P_DUTY_HOMING if self.state == HOMING else P_DUTY_MOVE)
            dx = FULL_SPEED_MM_S * duty / 1000.0 * dt * self.dir
            new_true = min(TRAVEL_MM, max(0.0, self.true_mm + dx))
            actual = new_true - self.true_mm
            self.true_mm = new_true
            if not self.hall_dead:
                self.count_f += (actual / MM_PER_HALL * self.hall_rate_pct / 100.0
                                 * self.hall_sign * self.hall_dir)

    def tick(self, dt):
        now = self._now_ms()
        self._plant(dt)
        # inputs
        prev_count = self.last_count
        self.last_count = self.hall_count
        if self.last_count != prev_count:
            self.last_count_change_ms = now
            self.last_pulse_ms = now
        self.current_ma = self._current_ma()
        sensors = self.sensor_bits()
        stop_now = [bool(sensors & (1 << k)) for k in range(3)]
        self._update_velocity(now)
        if self.hbc == "active" and now - self.hbc_last_ms > (self.od.get(0x1016, 1) & 0xFFFF):
            self.hbc = "lost"
            self.supervision_lost = True
        if self.state == INIT:
            if now - self.t0 >= INIT_SETTLE_MS:
                self._finish_init()
        else:
            self._check_global(now)
            if self.drive_on:
                self._check_drive(now)
            if self.drive_on:
                self._sensor_edges(stop_now)
            if self.drive_on:
                self._handle_target(now)
            if self.state == STOPPING and not self.drive_on and self._standstill(now):
                self._discard_target()
                self.state = self._rest_state()
            elif self.state not in (MOVING, HOMING, STOPPING, FAULT):
                self.state = self._rest_state()
            if self.f08_armed:
                if self._standstill(now):
                    self.f08_armed = False
                elif now - self.drive_off_ms >= self.param(P_STANDSTILL_MS):
                    self.f08_armed = False
                    self._raise(F08)
        self.stop_prev = stop_now
        self._emit_emcy()
        self._update_od(now)
        if self.nmt_state == NMT_OPERATIONAL:
            self._publish(now)

    def _finish_init(self):
        n = self._sensor_count()
        if self.both_limits():
            self._raise(F01)
        if n > 1:
            self._raise(F09)
        elif n == 1 and self.hall_supply and not self.faults:
            k = [i for i in range(3) if self.sensor_bits() & (1 << i)][0]
            self._set_reference(self.stop_mm(k))  # power-up at a sensor: valid without homing
            self.homed = True
            self.position_valid = True
        self.state = self._rest_state()

    def _update_velocity(self, now):
        dt = now - self.vel_t0_ms
        if dt < 100:
            if not self.drive_on and self._standstill(now):
                self.velocity = 0
            return
        dc = self.hall_sign * (self.last_count - self.vel_c0)
        v = int(dc * self.param(P_HALL_NM) / dt / 100)
        self.velocity = max(-32767, min(32767, v))
        self.vel_t0_ms = now
        self.vel_c0 = self.last_count

    def _check_global(self, now):
        if self.both_limits() and not self.faults & (1 << (F01 - 1)):
            self._raise(F01)
        if self._sensor_count() > 1 and not self.faults & (1 << (F09 - 1)):
            self._raise(F09)
        if not self.hall_supply and (self.homed or self.position_valid):
            moving = self.drive_on or not self._standstill(now)
            self.homed = False
            self.position_valid = False
            if moving:
                self._raise(F05)
            elif self.state not in (MOVING, HOMING, STOPPING):
                self.state = self._rest_state()

    def _check_drive(self, now):
        if self.supervision_lost or now - self.last_keepalive_ms >= self.param(P_DEADLINE):
            self._raise(F07)
            return
        if not self.motor_power:  # hardwired chain removed power: halt, keep counting, no fault
            self._drive_stop()
            self._discard_target()
            self.state = STOPPING
            return
        on_for = now - self.drive_on_ms
        if on_for >= self.param(P_INRUSH_MS) and self.current_ma > self.param(P_OC_MA):
            if not self.over_active:
                self.over_active = True
                self.over_since_ms = now
            elif now - self.over_since_ms >= self.param(P_OC_MS):
                self._raise(F03)
                return
        else:
            self.over_active = False
        if (self.dir > 0 and self.upper_limit()) or (self.dir < 0 and self.lower_limit()):
            self._raise(F02)
            return
        if now - self.move_deadline_ms >= 0:
            self._raise(F04)
            return
        progress = self.hall_sign * (self.last_count - self.dir_ref_count) * self.dir
        if progress > 0:
            self.dir_ref_count = self.last_count
        elif -progress > self.param(P_WRONG_DIR):
            self._raise(F02)
            return
        if on_for < self.param(P_HALL_GRACE):
            self.win_start_ms = now
            self.win_start_count = self.last_count
            return
        if now - self.last_pulse_ms >= self.param(P_STALL_MS):
            self._raise(F03)  # stall: no Hall pulses while driving
            return
        w = now - self.win_start_ms
        if w >= self.param(P_HALL_WINDOW):
            cps = abs(self.last_count - self.win_start_count) * 1000 // max(1, w)
            homing = self.state == HOMING
            lo = self.param(P_HOME_MIN if homing else P_MOVE_MIN)
            hi = self.param(P_HOME_MAX if homing else P_MOVE_MAX)
            recent = now - self.last_pulse_ms < self.param(P_HALL_WINDOW) // 2
            if cps > hi or (cps != 0 and recent and cps < lo):
                self._raise(F02)
                return
            self.win_start_ms = now
            self.win_start_count = self.last_count

    def _sensor_edges(self, stop_now):
        for k in range(3):
            if not self.drive_on:
                return
            if not stop_now[k] or self.stop_prev[k]:
                continue
            edge = self.param(P_EDGE_UP if self.dir > 0 else P_EDGE_DOWN) / 1000.0
            expect = self.stop_mm(k) + edge
            if not self.homed:
                if self.state != HOMING or self.dir > 0:
                    continue
                self._set_reference(expect)
                self.homed = True
                self.position_valid = True
                self._set_target(0 if self.param(P_HOME_CONTINUE) else k)
                continue
            if abs(self.position_mm() - expect) > self.param(P_XCHECK_TOL) / 1000.0:
                self._raise(F02)
                return
            if self.param(P_REREF):
                self._set_reference(expect)
            if self.state == HOMING and self.target_stop == 0xFF:
                self._set_target(0 if self.param(P_HOME_CONTINUE) else k)

    def _handle_target(self, now):
        if self.target_stop == 0xFF or not self.position_valid:
            return
        remaining = (self.target_mm - self.position_mm()) * self.dir
        lead = self.param(P_STOP_LEAD) / 1000.0
        overrun = self.param(P_OVERRUN) / 1000.0
        if self.sensor_bits() & (1 << self.target_stop) and remaining <= lead:
            self._drive_stop()
            self.arriving = True
            self.state = STOPPING
        elif remaining < -overrun:
            self._raise(F02)

    # --- status, EMCY, PDOs ---------------------------------------------------------------
    def error_register(self):
        r = 0
        if self.faults:
            r |= 0x01 | 0x80
        if self.faults & (1 << (F03 - 1)):
            r |= 0x02
        if self.faults & (1 << (F07 - 1)):
            r |= 0x10
        return r

    def _emit_emcy(self):
        raised, self.new_faults = self.new_faults, 0
        for n in range(1, 10):
            if raised & (1 << (n - 1)):
                code = 0xFF00 + n
                self.emcy_log.append(n)
                self.err_history.insert(0, code | (n << 16) | ((self.faults & 0xFF) << 24))
                del self.err_history[8:]
                self._send_emcy(code, n)
        if not self.faults and self.last_faults:
            self.emcy_resets += 1
            self._send_emcy(0x0000, 0)
        self.last_faults = self.faults

    def _send_emcy(self, code, n):
        if self.nmt_state not in (NMT_PRE_OPERATIONAL, NMT_OPERATIONAL):
            return
        self._send(EMCY, struct.pack("<HBBH2x", code, self.error_register(), n, self.faults))

    def flags(self, now=None):
        now = self._now_ms() if now is None else now
        f = 0
        if self.position_valid:
            f |= F_POSITION_VALID
        if self._active_stop(now) >= 0:
            f |= F_AT_STOP
        if self.drive_on or not self._standstill(now):
            f |= F_MOVING
        if self.lower_limit():
            f |= F_LOWER
        if self.upper_limit():
            f |= F_UPPER
        if self.faults:
            f |= F_FAULT
        if self.motor_power:
            f |= F_MOTOR_POWER
        if not self.homed:
            f |= F_RECOVERY
        return f

    def _pos_raw(self):
        if not self.position_valid:
            return 0xFFFF
        return max(0, min(0xFFFE, int(self.position_mm() * 10)))

    def _active_stop_id(self, now):
        k = self._active_stop(now)
        return 0xFF if k < 0 else k

    def inputs_byte(self):
        b = self.sensor_bits()
        if self.estop2_pressed:
            b |= IN_ESTOP2
        if self.hall_supply:
            b |= IN_HALL_SUPPLY
        for i, on in enumerate(self.spare_in):
            if on:
                b |= 1 << (5 + i)
        return b

    def aux_flags(self):
        f = AUX_F09 if self.faults & (1 << (F09 - 1)) else 0
        if self.head_light_on:
            f |= AUX_LIGHT_ON
        if self.getac_enable:
            f |= AUX_GETAC
        return f

    def status1(self, counter, now=None):
        now = self._now_ms() if now is None else now
        return struct.pack("<BBHBBBB", self.state, self.flags(now), self._pos_raw(),
                           self._active_stop_id(now), self.last_seq, self.cmd_result, counter)

    def status2(self):
        return struct.pack("<BBHhBB", self.inputs_byte(), self.target_stop,
                           max(0, min(0xFFFF, self.current_ma)), self.velocity,
                           self.faults & 0xFF, self.aux_flags())

    def _update_od(self, now=None):
        now = self._now_ms() if now is None else now
        s1 = self.status1(self.counter, now)
        for sub, val in enumerate(struct.unpack("<BBHBBBB", s1), start=1):
            self.od.set(0x2000, sub, val)
        for sub, val in enumerate(struct.unpack("<BBHhBB", self.status2()), start=1):
            self.od.set(0x2001, sub, val)
        self.od.set(0x2002, 0, self.faults)
        self.od.set(0x1001, 0, self.error_register())
        n = len(self.err_history)
        self.od.set(0x1003, 0, n)
        for i in range(8):
            self.od.set(0x1003, i + 1, self.err_history[i] if i < n else 0)
        self.od.set(0x2300, 1, self.hall_count)
        self.od.set(0x2300, 3, self.rpdo_len_errors)

    def _publish(self, now):
        for k, (cob, change_mask, inhibit_key, event_key) in enumerate((
                (TPDO1, 0x7F, (0x1800, 3), (0x1800, 5)),
                (TPDO2, 0xC3, (0x1801, 3), (0x1801, 5)))):
            if k == 0:
                payload = self.status1(self.counter, now)
            else:
                payload = self.status2()
            last = self.tpdo_last[k]
            since = now - self.tpdo_last_ms[k]
            inhibit_ms = (self.od.get(*inhibit_key) + 9) // 10
            event_ms = self.od.get(*event_key)
            changed = last is not None and any(
                change_mask & (1 << b) and payload[b] != last[b] for b in range(8))
            due = (not self.tpdo_sent_once[k] or (changed and since >= inhibit_ms)
                   or (event_ms and since >= event_ms))
            if not due:
                continue
            if k == 0:
                self.counter = (self.counter + 1) & 0xFF
                payload = self.status1(self.counter, now)
            self._send(cob, payload)
            self.tpdo_last[k] = payload
            self.tpdo_last_ms[k] = now
            self.tpdo_sent_once[k] = True

    def state_line(self):
        with self.lock:
            return ("ctl017 nmt=0x%02X state=%d flags=0x%02X pos=%.1f mm hall=%d sensors=0x%X "
                    "faults=0x%04X motor_power=%s light=%d getac=%d" % (
                        self.nmt_state, self.state, self.flags(), self.position_mm(),
                        self.hall_count, self.sensor_bits(), self.faults, self.motor_power,
                        self.head_light_on, self.getac_enable))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--start-pos-mm", type=float, default=None)
    ap.add_argument("--operational", action="store_true",
                    help="start in OPERATIONAL (real node waits for NMT start from the core)")
    args = ap.parse_args()
    bus = can.Bus(interface="socketcan", channel=args.channel)
    node = Ctl017Emulator(bus, start_pos_mm=args.start_pos_mm,
                          operational=args.operational).start()
    try:
        while True:
            print(node.state_line(), flush=True)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        bus.shutdown()


if __name__ == "__main__":
    main()
