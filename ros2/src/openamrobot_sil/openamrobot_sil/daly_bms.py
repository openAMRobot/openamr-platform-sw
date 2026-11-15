"""Minimal Daly BMS emulator on CAN2: pack voltage, current, SOC, MOS state and faults.

PROTOCOL NOT CONFIRMED for the delivered unit. The frames follow the layout that the
receive-only decoder of openamr-platform-fw PR #51 documents (firmware/telemetry
daly_rx.h at fcf81f8): 29-bit identifier 0x18 << 24 | DataID << 16 | destination << 8 |
source, 8 data bytes, Data IDs 0x90 to 0x98, big-endian fields. That decoder states that
newer Daly firmware may use a different protocol. When the owner confirms the protocol by a
capture, this emulator follows the capture.

Request and response: the host sends DataID with destination = BMS address. The emulator
answers the same DataID with destination = host address. Current is reported as raw =
current_a * 10 + 30000 and no direction is named, as in the decoder.
"""
import struct
import threading

import can

PRIORITY = 0x18
ID_SOC = 0x90
ID_CELL_MINMAX = 0x91
ID_TEMP_MINMAX = 0x92
ID_MOS = 0x93
ID_STATUS = 0x94
ID_FAILURE = 0x98
SUPPORTED = (ID_SOC, ID_CELL_MINMAX, ID_TEMP_MINMAX, ID_MOS, ID_STATUS, ID_FAILURE)
CURRENT_OFFSET = 30000
TEMP_OFFSET = 40


def daly_id(data_id, destination, source):
    return (PRIORITY << 24) | (data_id << 16) | (destination << 8) | source


def split_id(arbitration_id):
    """Return (priority, data_id, destination, source) of a 29-bit Daly identifier."""
    return ((arbitration_id >> 24) & 0x1F, (arbitration_id >> 16) & 0xFF,
            (arbitration_id >> 8) & 0xFF, arbitration_id & 0xFF)


class DalyBmsEmulator:
    def __init__(self, bus, bms_addr=0x01, host_addr=0x40, n_cells=8, n_temps=1,
                 pack_voltage_v=25.6, current_a=0.0, soc_percent=80.0, capacity_ah=84.0,
                 temperature_c=25):
        self.bus = bus
        self.bms_addr = bms_addr
        self.host_addr = host_addr
        self.n_cells = n_cells
        self.n_temps = n_temps
        self.pack_voltage_v = pack_voltage_v
        self.current_a = current_a
        self.soc_percent = soc_percent
        self.capacity_ah = capacity_ah
        self.temperature_c = temperature_c
        self.charge_state = 0
        self.charge_mos_on = True
        self.discharge_mos_on = True
        self.failure = bytearray(8)
        self.offline = False  # no answer at all, as after a BMS disconnect of its CAN side
        self.requests = 0
        self.unsupported = 0
        self.foreign = 0
        self.life = 0
        self.lock = threading.RLock()
        self._running = False
        self._thread = None

    # --- injection ------------------------------------------------------------
    def inject_failure(self, byte_index, mask):
        """Set raw bits in failure byte 0 to 7 (0x98). Bytes 0 to 3: even bit level 1, odd
        bit level 2. The meaning of each bit belongs to the protocol document."""
        with self.lock:
            self.failure[byte_index] |= mask & 0xFF

    def clear_failures(self):
        with self.lock:
            self.failure = bytearray(8)

    def set_mos(self, charge_on, discharge_on):
        with self.lock:
            self.charge_mos_on, self.discharge_mos_on = bool(charge_on), bool(discharge_on)

    def set_values(self, pack_voltage_v=None, current_a=None, soc_percent=None):
        with self.lock:
            if pack_voltage_v is not None:
                self.pack_voltage_v = float(pack_voltage_v)
            if current_a is not None:
                self.current_a = float(current_a)
            if soc_percent is not None:
                self.soc_percent = min(100.0, max(0.0, float(soc_percent)))

    # --- frames ---------------------------------------------------------------
    def payload(self, data_id):
        with self.lock:
            if data_id == ID_SOC:
                v = int(round(self.pack_voltage_v * 10))
                i = int(round(self.current_a * 10)) + CURRENT_OFFSET
                return struct.pack('>HHHH', v, v, i, int(round(self.soc_percent * 10)))
            if data_id == ID_CELL_MINMAX:
                mv = int(round(self.pack_voltage_v * 1000 / self.n_cells))
                return struct.pack('>HBHBxx', mv, 1, mv, self.n_cells)
            if data_id == ID_TEMP_MINMAX:
                t = self.temperature_c + TEMP_OFFSET
                return struct.pack('>BBBBxxxx', t, 1, t, 1)
            if data_id == ID_MOS:
                self.life = (self.life + 1) & 0xFF
                remaining = int(self.capacity_ah * 1000 * self.soc_percent / 100.0)
                return struct.pack('>BBBBI', self.charge_state, int(self.charge_mos_on),
                                   int(self.discharge_mos_on), self.life, remaining)
            if data_id == ID_STATUS:
                return struct.pack('>BBBBBHx', self.n_cells, self.n_temps, 0, 1, 0, 0)
            if data_id == ID_FAILURE:
                return bytes(self.failure)
        return None

    def handle(self, msg):
        if not msg.is_extended_id:
            return
        priority, data_id, destination, source = split_id(msg.arbitration_id)
        if priority != PRIORITY or destination != self.bms_addr or source != self.host_addr:
            self.foreign += 1
            return
        self.requests += 1
        if self.offline:
            return
        data = self.payload(data_id)
        if data is None:
            self.unsupported += 1  # 0x95 to 0x97 and unknown Data IDs: no answer
            return
        self.bus.send(can.Message(arbitration_id=daly_id(data_id, self.host_addr, self.bms_addr),
                                  data=data, is_extended_id=True))

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
        while self._running:
            msg = self.bus.recv(timeout=0.02)
            if msg is not None:
                self.handle(msg)

    def state(self):
        with self.lock:
            return {
                'pack_voltage_v': self.pack_voltage_v, 'current_a': self.current_a,
                'soc_percent': self.soc_percent, 'charge_mos_on': self.charge_mos_on,
                'discharge_mos_on': self.discharge_mos_on, 'failure': self.failure.hex(),
                'offline': self.offline, 'requests': self.requests,
                'unsupported': self.unsupported, 'foreign': self.foreign,
            }


def decode_response(data_id, data):
    """Decode one response payload the way the firmware decoder does. Host test client use."""
    data = bytes(data)
    if len(data) != 8:
        return None
    if data_id == ID_SOC:
        v, acquired, i, soc = struct.unpack('>HHHH', data)
        return {'pack_voltage_v': v * 0.1, 'acquired_voltage_v': acquired * 0.1,
                'current_a': (i - CURRENT_OFFSET) * 0.1, 'soc_percent': soc * 0.1}
    if data_id == ID_MOS:
        state, chg, dis, life, remaining = struct.unpack('>BBBBI', data)
        return {'charge_state': state, 'charge_mos_on': bool(chg),
                'discharge_mos_on': bool(dis), 'bms_life': life, 'remaining_mah': remaining}
    if data_id == ID_FAILURE:
        return {'failure_raw': data.hex(),
                'any_level1': any(b & 0x55 for b in data[:4]),
                'any_level2': any(b & 0xAA for b in data[:4]),
                'any_hw_failure': any(data[4:7])}
    return None
