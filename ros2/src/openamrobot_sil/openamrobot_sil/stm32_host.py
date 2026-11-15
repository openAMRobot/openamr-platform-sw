"""Host test client: plays the CAN role of the STM32 base controller until the firmware runs.

TEST STAND-IN, vcan only. It is not the firmware and it copies none of its logic. It sends
what the emulators need to be exercised: the drive bring-up and wheel targets on CAN1, the
Daly polls on CAN2, and NMT start, core heartbeat, lift commands, keep-alive and the lift
permit frames on CAN3. It holds no safety function and takes no stop decision: it has no
command timeout, no E-stop input and no fault reaction. With stm32:=renode it is not started.
With stm32:=renode_readonly it runs with DriveHost mode "command_only" next to the firmware.
"""
import math
import random
import struct
import threading
import time

import can

from openamrobot_sil import daly_bms
from openamrobot_sil.can_sil import canopen_common, ctl017_emulator as c
from openamrobot_sil.zlac_gazebo import (RPDO1_BASE, RPDO2_BASE, RPM_TO_RAD_S,
                                         twist_to_wheel_speeds, wheel_speeds_to_twist)

DRIVE_MODES = ('sdo', 'command_only')


class SdoForbidden(RuntimeError):
    """The command-only drive client tried to send an SDO request."""


class _CommandOnlyBus:
    """Bus wrapper of the command-only drive client: it refuses every frame to the SDO server
    of the drive (600h + node), so the client cannot take the SDO channel from the firmware."""

    def __init__(self, bus, node_id):
        self._bus = bus
        self._sdo_id = canopen_common.SDO_RX_BASE + node_id
        self.sent = 0
        self.refused = 0

    def send(self, msg, timeout=None):
        if not msg.is_extended_id and msg.arbitration_id == self._sdo_id:
            self.refused += 1
            raise SdoForbidden('command-only drive client: SDO frame %03X refused'
                               % msg.arbitration_id)
        self._bus.send(msg, timeout)
        self.sent += 1


class _Worker:
    period_s = 0.02

    def __init__(self):
        self.lock = threading.RLock()
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def _run(self):
        next_t = time.monotonic()
        while self._running:
            self.step()
            next_t += self.period_s
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()

    def step(self):
        raise NotImplementedError


class DriveHost(_Worker):
    """CAN1 role: configure and enable the ZLAC8015D, write 60FFh, read 606Ch, integrate
    odometry from the speeds the drive reports.

    mode "command_only" (stm32:=renode_readonly): the firmware image in Renode owns the SDO
    channel of the drive and reads 606Ch, 6041h, 603Fh and 2026h itself. The client then sends
    no SDO frame at all, only the SIL receive PDOs of zlac_gazebo (controlword enable sequence,
    then the wheel targets every cycle), reads nothing and integrates no odometry. 2000h is not
    written either; the emulator starts with it stored (stored_link_loss_ms).
    """

    period_s = 0.02  # 50 Hz control cycle assumed in platform-fw docs/architecture/can1-drive-link.md

    def __init__(self, bus, node_id, link_loss_ms, wheel_radius, wheel_separation,
                 left_sign=1, right_sign=1, sdo_timeout_s=0.1, mode='sdo'):
        super().__init__()
        if mode not in DRIVE_MODES:
            raise ValueError('drive mode must be one of %s, got %r' % (DRIVE_MODES, mode))
        self.mode = mode
        self.node_id = node_id
        self.cmd_bus = None
        if mode == 'command_only':
            self.cmd_bus = _CommandOnlyBus(bus, node_id)
            self.sdo = None
        else:
            self.sdo = canopen_common.SdoClient(bus, node_id, timeout=sdo_timeout_s)
        self.link_loss_ms = int(link_loss_ms)
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.signs = (left_sign, right_sign)
        self.silent = False       # True: send nothing on CAN1 (link-loss test)
        self.target_v = 0.0
        self.target_w = 0.0
        self.configured = False
        self.sdo_errors = 0
        self.statusword = None
        self.fault_code = None
        self.wheel_rad_s = (0.0, 0.0)
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.samples = 0
        self._last_sample = None
        self._cycle = 0

    def set_twist(self, v, w):
        with self.lock:
            self.target_v, self.target_w = float(v), float(w)

    def set_silent(self, silent):
        with self.lock:
            self.silent = bool(silent)
            if silent:
                self.configured = False  # bring the drive up again when the link returns
                self._last_sample = None

    def target_rpm(self):
        left, right = twist_to_wheel_speeds(self.target_v, self.target_w, self.wheel_radius,
                                            self.wheel_separation)
        return (int(round(self.signs[0] * left / RPM_TO_RAD_S)),
                int(round(self.signs[1] * right / RPM_TO_RAD_S)))

    def _bring_up(self):
        s = self.sdo
        s.write(0x60FF, 1, 0, 4, signed=True)
        s.write(0x60FF, 2, 0, 4, signed=True)
        s.write(0x6060, 0, 3, 1, signed=True)          # profile velocity
        s.write(0x2000, 0, self.link_loss_ms, 2)       # drive link-loss time
        for word in (0x06, 0x07, 0x0F):                # CiA 402 enable sequence
            s.write(0x6040, 0, word, 2)
        self.configured = True

    def _send_pdo(self, arb_id, data):
        self.cmd_bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))

    def _step_command_only(self, rpm):
        try:
            if not self.configured:
                for word in (0x06, 0x07, 0x0F):            # CiA 402 enable sequence, RPDO2
                    self._send_pdo(RPDO2_BASE + self.node_id, struct.pack('<H', word))
                self.configured = True
            self._send_pdo(RPDO1_BASE + self.node_id, struct.pack('<ii', rpm[0], rpm[1]))
        except can.CanError:
            with self.lock:
                self.sdo_errors += 1
                self.configured = False

    def step(self):
        with self.lock:
            if self.silent:
                return
            rpm = self.target_rpm()
        if self.mode == 'command_only':
            self._step_command_only(rpm)
            return
        try:
            if not self.configured:
                self._bring_up()
            self.sdo.write(0x60FF, 1, rpm[0], 4, signed=True)
            self.sdo.write(0x60FF, 2, rpm[1], 4, signed=True)
            left = self.sdo.read(0x606C, 1, signed=True) / 10.0
            right = self.sdo.read(0x606C, 2, signed=True) / 10.0
            self._cycle += 1
            if self._cycle % 5 == 0:
                self.statusword = self.sdo.read(0x6041, 0)
                self.fault_code = self.sdo.read(0x603F, 0)
        except (TimeoutError, canopen_common.SdoAbort, can.CanError):
            with self.lock:
                self.sdo_errors += 1
                self.configured = False
                self._last_sample = None
            return
        now = time.monotonic()
        with self.lock:
            wl = self.signs[0] * left * RPM_TO_RAD_S
            wr = self.signs[1] * right * RPM_TO_RAD_S
            self.wheel_rad_s = (wl, wr)
            self.v, self.w = wheel_speeds_to_twist(wl, wr, self.wheel_radius,
                                                   self.wheel_separation)
            if self._last_sample is not None:
                dt = now - self._last_sample
                self.yaw += self.w * dt
                self.x += self.v * math.cos(self.yaw) * dt
                self.y += self.v * math.sin(self.yaw) * dt
            self._last_sample = now
            self.samples += 1

    def state(self):
        with self.lock:
            return {'mode': self.mode, 'configured': self.configured, 'silent': self.silent,
                    'pdo_frames_sent': self.cmd_bus.sent if self.cmd_bus else 0,
                    'sdo_frames_refused': self.cmd_bus.refused if self.cmd_bus else 0,
                    'sdo_errors': self.sdo_errors, 'statusword': self.statusword,
                    'fault_code': self.fault_code, 'target_rpm': list(self.target_rpm()),
                    'wheel_rad_s': list(self.wheel_rad_s), 'samples': self.samples,
                    'x': self.x, 'y': self.y, 'yaw': self.yaw, 'v': self.v, 'w': self.w}


class LiftHost(_Worker):
    """CAN3 role: NMT start, core heartbeat, RPDO1 command with keep-alive, permit frames."""

    period_s = 0.01
    HEARTBEAT_S = 0.1
    KEEPALIVE_S = 0.1

    def __init__(self, bus, node_id=c.NODE_ID, core_node_id=c.CORE_NODE_ID, permit=None,
                 permit_period_s=0.05):
        super().__init__()
        self.bus = bus
        self.node_id = node_id
        self.core_node_id = core_node_id
        self.permit_cfg = permit
        self.permit_period_s = permit_period_s
        self.permit = False          # false at boot
        self.permit_silent = False   # True: stop sending permit frames (expiry test)
        self.boot_id = random.getrandbits(16) or 1
        self.permit_seq = 0
        self.seq = 0
        self.cmd = None
        self.status = None
        self.status2 = None
        self.node_nmt = None
        self.emcy = []
        self.silent = False
        self._t_hb = self._t_ka = self._t_permit = self._t_nmt = 0.0

    def _send(self, arb_id, data):
        try:
            self.bus.send(can.Message(arbitration_id=arb_id, data=bytes(data),
                                      is_extended_id=False))
        except can.CanError:
            pass

    def command(self, op, stop=0):
        with self.lock:
            self.seq = (self.seq + 1) & 0xFF
            self.cmd = (self.seq, op, stop)
            self._send(0x200 + self.node_id, self.cmd)
            self._t_ka = time.monotonic()

    def set_permit(self, permit):
        with self.lock:
            self.permit = bool(permit)
            self._t_permit = 0.0  # send the new state at once

    def step(self):
        now = time.monotonic()
        msg = self.bus.recv(timeout=0)
        while msg is not None:
            self._rx(msg)
            msg = self.bus.recv(timeout=0)
        with self.lock:
            if self.silent:
                return
            if now - self._t_hb >= self.HEARTBEAT_S:
                self._t_hb = now
                self._send(canopen_common.HB_BASE + self.core_node_id,
                           [canopen_common.NMT_OPERATIONAL])
            if (self.node_nmt is not None and self.node_nmt != canopen_common.NMT_OPERATIONAL
                    and now - self._t_nmt >= 0.5):
                self._t_nmt = now
                self.cmd = None  # never replay a command into a restarted node
                self._send(canopen_common.NMT_ID, [0x01, self.node_id])
            if self.cmd is not None and now - self._t_ka >= self.KEEPALIVE_S:
                self._t_ka = now
                self._send(0x200 + self.node_id, self.cmd)
            cfg = self.permit_cfg
            if (cfg is not None and not self.permit_silent
                    and now - self._t_permit >= self.permit_period_s):
                self._t_permit = now
                self.permit_seq += 1
                flag = cfg.permit_value if self.permit else 0
                self._send(cfg.cob_id, cfg.pack(flag, self.boot_id, self.permit_seq, 0))

    def _rx(self, msg):
        if msg.is_extended_id:
            return
        data = bytes(msg.data)
        with self.lock:
            if msg.arbitration_id == 0x180 + self.node_id and len(data) == 8:
                state, flags, pos, stop, last_seq, res, counter = struct.unpack('<BBHBBBB', data)
                self.status = {'state': state, 'flags': flags,
                               'position_mm': None if pos == 0xFFFF else pos / 10.0,
                               'active_stop': stop, 'last_seq': last_seq, 'cmd_result': res,
                               'counter': counter}
            elif msg.arbitration_id == 0x280 + self.node_id and len(data) == 8:
                inputs, target, current, velocity, faults, aux = struct.unpack('<BBHhBB', data)
                self.status2 = {'inputs': inputs, 'target_stop': target,
                                'motor_current_ma': current, 'velocity_mm_s': velocity / 10.0,
                                'faults': faults, 'aux': aux}
            elif msg.arbitration_id == 0x080 + self.node_id and len(data) >= 4:
                self.emcy.append(data[3])
                del self.emcy[:-16]
            elif msg.arbitration_id == canopen_common.HB_BASE + self.node_id and len(data) == 1:
                self.node_nmt = data[0]

    def state(self):
        with self.lock:
            out = {'node_nmt': self.node_nmt, 'permit': self.permit,
                   'permit_silent': self.permit_silent, 'seq': self.seq,
                   'cmd': list(self.cmd) if self.cmd else None, 'emcy': list(self.emcy)}
            out.update(self.status or {})
            out.update(self.status2 or {})
            return out


class BmsHost(_Worker):
    """CAN2 role: poll the Daly BMS. 5 Hz is the rate the BOM CAN buses sheet proposes."""

    period_s = 0.2
    POLLED = (daly_bms.ID_SOC, daly_bms.ID_MOS, daly_bms.ID_FAILURE)

    def __init__(self, bus, bms_addr=0x01, host_addr=0x40, response_timeout_s=0.05):
        super().__init__()
        self.bus = bus
        self.bms_addr = bms_addr
        self.host_addr = host_addr
        self.response_timeout_s = response_timeout_s
        self.values = {}
        self.answers = 0
        self.timeouts = 0
        self.last_answer = None

    def step(self):
        for data_id in self.POLLED:
            try:
                self.bus.send(can.Message(
                    arbitration_id=daly_bms.daly_id(data_id, self.bms_addr, self.host_addr),
                    data=bytes(8), is_extended_id=True))
            except can.CanError:
                continue
            want = daly_bms.daly_id(data_id, self.host_addr, self.bms_addr)
            end = time.monotonic() + self.response_timeout_s
            decoded = None
            while time.monotonic() < end:
                msg = self.bus.recv(timeout=max(0.0, end - time.monotonic()))
                if msg is not None and msg.is_extended_id and msg.arbitration_id == want:
                    decoded = daly_bms.decode_response(data_id, msg.data)
                    break
            with self.lock:
                if decoded is None:
                    self.timeouts += 1
                else:
                    self.values.update(decoded)
                    self.answers += 1
                    self.last_answer = time.monotonic()

    def state(self):
        with self.lock:
            out = dict(self.values)
            out['answers'] = self.answers
            out['timeouts'] = self.timeouts
            out['age_s'] = (None if self.last_answer is None
                            else round(time.monotonic() - self.last_answer, 3))
            return out
