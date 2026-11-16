"""Minimal CANopen helpers shared by the SIL emulators (SDO expedited, NMT, heartbeat)."""
import struct
import threading
import time

import can

NMT_ID = 0x000
SDO_TX_BASE = 0x580  # node to master
SDO_RX_BASE = 0x600  # master to node
HB_BASE = 0x700

NMT_OPERATIONAL = 0x05
NMT_STOPPED = 0x04
NMT_PRE_OPERATIONAL = 0x7F

ABORT_NO_OBJECT = 0x06020000
ABORT_NO_SUBINDEX = 0x06090011
ABORT_READ_ONLY = 0x06010002
ABORT_WRITE_ONLY = 0x06010001
ABORT_BAD_VALUE = 0x06090030
ABORT_GENERAL = 0x08000000
ABORT_LOCAL_CONTROL = 0x08000021   # object written by a PDO only
ABORT_DEVICE_STATE = 0x08000022    # write outside PRE-OPERATIONAL or during motion

SIZE_CS = {1: 0x2F, 2: 0x2B, 4: 0x23}  # download request command byte by size
CS_SIZE = {v: k for k, v in SIZE_CS.items()}
UPLOAD_RESP = {1: 0x4F, 2: 0x4B, 4: 0x43}


class ObjectDict:
    """index, sub -> dict(size, signed, value, rw).

    rw is "rw", "ro", "wo", "local" (SDO write refused with 0x08000021, PDO only) or "preop"
    (SDO write only in PRE-OPERATIONAL, else 0x08000022).
    """

    def __init__(self):
        self.entries = {}

    def add(self, index, sub, size, value=0, signed=False, rw="rw"):
        self.entries[(index, sub)] = {"size": size, "signed": signed, "value": value, "rw": rw}

    def has_index(self, index):
        return any(i == index for i, _ in self.entries)

    def get(self, index, sub):
        return self.entries[(index, sub)]["value"]

    def set(self, index, sub, value):
        self.entries[(index, sub)]["value"] = value


def pack_value(value, size, signed):
    fmt = {1: "b", 2: "h", 4: "i"}[size] if signed else {1: "B", 2: "H", 4: "I"}[size]
    return struct.pack("<" + fmt, value)


def unpack_value(data, signed):
    size = len(data)
    fmt = {1: "b", 2: "h", 4: "i"}[size] if signed else {1: "B", 2: "H", 4: "I"}[size]
    return struct.unpack("<" + fmt, bytes(data))[0]


class CanopenNode:
    """Base class: NMT slave, heartbeat producer, expedited SDO server over an OD.

    Subclasses override on_write(index, sub, value) -> abort code or None, on_read(),
    on_rx_frame(msg) and tick(dt). The node runs on its own thread; use start()/stop().
    """

    tick_s = 0.005
    sdo_when_stopped = True  # CTL-017 sets False (no SDO in STOPPED); ZLAC behaviour not recorded

    def __init__(self, bus, node_id, heartbeat_ms=100, od=None):
        self.bus = bus
        self.node_id = node_id
        self.od = od or ObjectDict()
        self.heartbeat_ms = heartbeat_ms
        self.nmt_state = NMT_PRE_OPERATIONAL
        self.bus_off = False  # simulated bus-off: node neither sends nor receives
        self.heartbeat_enabled = True
        self.lock = threading.RLock()
        self._thread = None
        self._running = False
        self._last_hb = 0.0
        self.rx_count = 0

    # hooks
    def on_write(self, index, sub, value):
        return None

    def on_rx_frame(self, msg):
        pass

    def swallow_write(self, index, sub, value):
        """Failure injection hook: return True to drop an SDO write with no response."""
        return False

    def tick(self, dt):
        pass

    # control
    def start(self):
        self._running = True
        self._send_boot()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def _send(self, arb_id, data):
        if self.bus_off:
            return
        self.bus.send(can.Message(arbitration_id=arb_id, data=bytes(data), is_extended_id=False))

    def _send_boot(self):
        self._send(HB_BASE + self.node_id, [0x00])

    def set_bus_off(self, state):
        """Simulate bus-off: the node falls silent and ignores frames until cleared."""
        with self.lock:
            self.bus_off = state
            if not state:
                self._send_boot()

    def _run(self):
        last = time.monotonic()
        while self._running:
            msg = self.bus.recv(timeout=self.tick_s)
            now = time.monotonic()
            with self.lock:
                if msg is not None and not self.bus_off:
                    self._handle(msg)
                self.tick(now - last)
                if (self.heartbeat_enabled and self.heartbeat_ms > 0 and
                        now - self._last_hb >= self.heartbeat_ms / 1000.0):
                    self._last_hb = now
                    self._send(HB_BASE + self.node_id, [self.nmt_state])
            last = now

    def _handle(self, msg):
        if msg.is_extended_id or msg.is_remote_frame:
            return
        if msg.arbitration_id == NMT_ID and len(msg.data) >= 2:
            if msg.data[1] in (0, self.node_id):
                self._nmt(msg.data[0])
            return
        if msg.arbitration_id == SDO_RX_BASE + self.node_id:
            if self.nmt_state == NMT_STOPPED and not self.sdo_when_stopped:
                return  # CiA 301: only NMT and heartbeat in STOPPED
            self.rx_count += 1
            self._sdo(msg)
            self.on_rx_frame(msg)
            return
        self.rx_count += 1
        self.on_rx_frame(msg)

    def _nmt(self, cmd):
        if cmd == 0x01:
            self.nmt_state = NMT_OPERATIONAL
        elif cmd == 0x02:
            self.nmt_state = NMT_STOPPED
        elif cmd == 0x80:
            self.nmt_state = NMT_PRE_OPERATIONAL
        elif cmd in (0x81, 0x82):
            self.nmt_state = NMT_PRE_OPERATIONAL
            self._send_boot()

    def _abort(self, index, sub, code):
        self._send(SDO_TX_BASE + self.node_id,
                   struct.pack("<BHBI", 0x80, index, sub, code))

    def _sdo(self, msg):
        data = bytes(msg.data).ljust(8, b"\x00")
        cs = data[0]
        index, sub = struct.unpack("<HB", data[1:4])
        if cs == 0x40:  # upload (read)
            entry = self.od.entries.get((index, sub))
            if entry is None:
                self._abort(index, sub, ABORT_NO_SUBINDEX if self.od.has_index(index) else ABORT_NO_OBJECT)
                return
            if entry["rw"] == "wo":
                self._abort(index, sub, ABORT_WRITE_ONLY)
                return
            payload = pack_value(entry["value"], entry["size"], entry["signed"])
            self._send(SDO_TX_BASE + self.node_id,
                       bytes([UPLOAD_RESP[entry["size"]]]) + struct.pack("<HB", index, sub)
                       + payload.ljust(4, b"\x00"))
        elif cs in CS_SIZE:  # expedited download (write)
            size = CS_SIZE[cs]
            entry = self.od.entries.get((index, sub))
            if entry is None:
                self._abort(index, sub, ABORT_NO_SUBINDEX if self.od.has_index(index) else ABORT_NO_OBJECT)
                return
            if entry["rw"] == "ro":
                self._abort(index, sub, ABORT_READ_ONLY)
                return
            if entry["rw"] == "local":
                self._abort(index, sub, ABORT_LOCAL_CONTROL)
                return
            if entry["rw"] == "preop" and self.nmt_state != NMT_PRE_OPERATIONAL:
                self._abort(index, sub, ABORT_DEVICE_STATE)
                return
            value = unpack_value(data[4:4 + size], entry["signed"])
            if self.swallow_write(index, sub, value):
                return
            abort = self.on_write(index, sub, value)
            if abort:
                self._abort(index, sub, abort)
                return
            self._send(SDO_TX_BASE + self.node_id, struct.pack("<BHB4x", 0x60, index, sub))
        else:
            self._abort(index, sub, ABORT_GENERAL)


class SdoClient:
    """Tiny blocking SDO client used by the tests and the bench scripts."""

    def __init__(self, bus, node_id, timeout=0.5):
        self.bus = bus
        self.node_id = node_id
        self.timeout = timeout

    def _txrx(self, payload):
        self.bus.send(can.Message(arbitration_id=SDO_RX_BASE + self.node_id, data=payload,
                                  is_extended_id=False))
        end = time.monotonic() + self.timeout
        while time.monotonic() < end:
            msg = self.bus.recv(timeout=max(0.0, end - time.monotonic()))
            if msg is not None and msg.arbitration_id == SDO_TX_BASE + self.node_id:
                return bytes(msg.data)
        raise TimeoutError("no SDO response from node %d" % self.node_id)

    def read(self, index, sub, signed=False):
        resp = self._txrx(struct.pack("<BHB4x", 0x40, index, sub))
        if resp[0] == 0x80:
            raise SdoAbort(struct.unpack("<I", resp[4:8])[0], index, sub)
        size = {0x4F: 1, 0x4B: 2, 0x43: 4}[resp[0]]
        return unpack_value(resp[4:4 + size], signed)

    def write(self, index, sub, value, size, signed=False):
        payload = bytes([SIZE_CS[size]]) + struct.pack("<HB", index, sub) + \
            pack_value(value, size, signed).ljust(4, b"\x00")
        resp = self._txrx(payload)
        if resp[0] == 0x80:
            raise SdoAbort(struct.unpack("<I", resp[4:8])[0], index, sub)


class SdoAbort(Exception):
    def __init__(self, code, index, sub):
        super().__init__("SDO abort 0x%08X on %04Xh:%02X" % (code, index, sub))
        self.code = code
