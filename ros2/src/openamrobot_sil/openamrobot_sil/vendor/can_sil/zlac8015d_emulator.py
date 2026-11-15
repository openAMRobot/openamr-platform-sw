#!/usr/bin/env python3
"""Software-in-the-loop emulator of one ZLAC8015D V4.x CANopen node on SocketCAN.

Purpose: test the base-controller CAN1 code against a virtual driver before wheels move
(P-02, gate G4). Not a model of the real drive's electronics; it follows the vendor object
list as recorded in the OpenAMRobot research notes and has these deliberate simplifications:

- 6040h state machine is reduced: 0x06 -> ready, 0x07 -> switched on, 0x0F -> operation
  enabled, 0x80 -> fault reset, 0x02 -> quick stop (motor stays enabled, speed forced to 0).
- 6083h / 6084h are treated as the time in ms to ramp 0 to 1000 r/min (TODO(Evans): confirm
  the vendor definition of the ramp reference speed).
- 2000h link-loss: "communication" is any frame addressed to this node (SDO or other). Default
  0 means the drive keeps the last commanded speed forever. A value > 0 stops the wheels when
  no frame arrives for that many ms. TODO(Evans): confirm on the real drive what counts as
  communication; bench script zlac_link_loss_test.md measures it.
- 2010h write 1 saves 2000h and the ramp times to the emulated EEPROM; power_cycle() restores them.

Usage: python3 zlac8015d_emulator.py --channel vcan0 --node-id 1
As a library: see tests/test_zlac8015d.py.
"""
import argparse
import threading
import time

import can

from canopen_common import (ABORT_BAD_VALUE, ABORT_GENERAL, NMT_OPERATIONAL, CanopenNode,
                            ObjectDict)

# 603Fh fault bits (vendor list). The emulator reports a bit in the low 16 bits or the high
# 16 bits of 603Fh. It gives neither half a wheel or side meaning: the vendor Quick Start
# contradicts itself on that, so the meaning is open (TODO(Evans): settle on the real drive).
FAULT_OVER_VOLTAGE = 0x0001
FAULT_UNDER_VOLTAGE = 0x0002
FAULT_OVER_CURRENT = 0x0004
FAULT_OVERLOAD = 0x0008
FAULT_ENCODER_TOLERANCE = 0x0020
FAULT_ENCODER_ERROR = 0x0800
GLOBAL_FAULTS = FAULT_OVER_VOLTAGE | FAULT_UNDER_VOLTAGE

STATUS_READY = 0x0001
STATUS_SWITCHED_ON = 0x0002
STATUS_ENABLED = 0x0004
STATUS_FAULT = 0x0008
STATUS_VOLTAGE = 0x0010
STATUS_QUICK_STOP_N = 0x0020  # bit 5 low while quick stop is active

MAX_RPM = 1000
# TODO(Evans): which objects the real 2010h saves is not recorded; 2000h and 1017h are the
# two D-24 values and are saved here.
PERSISTENT = ((0x2000, 0), (0x1017, 0), (0x6083, 1), (0x6083, 2), (0x6084, 1), (0x6084, 2))


class Zlac8015dEmulator(CanopenNode):
    def __init__(self, bus, node_id=1, heartbeat_ms=100):
        super().__init__(bus, node_id, heartbeat_ms, ObjectDict())
        od = self.od
        od.add(0x1000, 0, 4, 0, rw="ro")  # device type, TODO(Evans): real value not recorded
        od.add(0x6040, 0, 2, 0)
        od.add(0x6041, 0, 2, 0, rw="ro")
        od.add(0x6060, 0, 1, 3, signed=True)
        od.add(0x6061, 0, 1, 3, signed=True, rw="ro")
        od.add(0x603F, 0, 4, 0, rw="ro")
        od.add(0x605A, 0, 2, 5, signed=True)
        od.add(0x6085, 0, 4, 10)
        od.add(0x1017, 0, 2, heartbeat_ms)
        for sub in (1, 2):
            od.add(0x60FF, sub, 4, 0, signed=True)
            od.add(0x606C, sub, 4, 0, signed=True, rw="ro")  # 0.1 r/min
            od.add(0x6083, sub, 4, 500)
            od.add(0x6084, sub, 4, 500)
        od.add(0x2000, 0, 2, 0)
        # 2026h:05 speed resolution. D-24 requires it to read 1 before any speed maths. Object
        # size is not recorded, TODO(Evans): confirm on the real drive.
        od.add(0x2026, 5, 2, 1, rw="ro")
        od.add(0x2010, 0, 2, 0, rw="wo")
        self.eeprom = {k: od.get(*k) for k in PERSISTENT}
        self.faults = 0
        self.rpm = [0.0, 0.0]  # actual, 606Ch sub-index 1 and 2
        self.statusword = 0
        self.enabled = False
        self.quick_stop = False
        self.last_rx = time.monotonic()
        self.link_lost = False
        self.link_loss_events = 0
        self.saves = 0
        self.nmt_state = NMT_OPERATIONAL  # the drive starts operational
        # failure injection for tests (test oracle only, the real drive is never asked)
        self.stop_write_fault = None  # None, "timeout" or "abort"
        self.speed_scale = [1.0, 1.0]  # actual speed as a fraction of the target, per sub-index
        self.dropped_writes = []

    # --- fault injection -------------------------------------------------
    # half: "low" sets the bit in bits 0 to 15 of 603Fh, "high" in bits 16 to 31. No wheel or
    # side meaning is attached to either half.
    def inject_over_current(self, half):
        self._inject(FAULT_OVER_CURRENT, half)

    def inject_encoder_error(self, half):
        self._inject(FAULT_ENCODER_ERROR, half)

    def inject_under_voltage(self):
        self._inject(FAULT_UNDER_VOLTAGE, None)

    def inject_over_voltage(self):
        self._inject(FAULT_OVER_VOLTAGE, None)

    def inject_stop_write_fault(self, mode):
        """While the drive is enabled, make it ignore ("timeout", no SDO response) or refuse
        ("abort") every write that would stop it: 60FFh = 0 and a 6040h write that is not the
        enable word. The write has no effect, so the wheels keep turning: the emulator never
        turns a stop that did not arrive into a stopped drive."""
        if mode not in (None, "timeout", "abort"):
            raise ValueError("mode must be None, 'timeout' or 'abort'")
        self.stop_write_fault = mode

    def inject_speed_scale(self, sub, factor):
        """Actual speed of 606Ch sub-index `sub` (1 or 2) becomes factor * target."""
        if sub not in (1, 2):
            raise ValueError("sub must be 1 or 2")
        self.speed_scale[sub - 1] = float(factor)

    def inject_speed_resolution(self, value):
        self.od.set(0x2026, 5, value)

    def _inject(self, bit, half):
        with self.lock:
            if half is None:
                self.faults |= bit & GLOBAL_FAULTS
            elif half == "low":
                self.faults |= bit
            elif half == "high":
                self.faults |= bit << 16
            else:
                raise ValueError("half must be 'low' or 'high'")
            self.enabled = False
            self.od.set(0x603F, 0, self.faults)

    def power_cycle(self):
        """Reset volatile state, restore saved parameters, keep nothing else."""
        with self.lock:
            for (i, s), v in self.eeprom.items():
                self.od.set(i, s, v)
            self.heartbeat_ms = self.od.get(0x1017, 0)
            for sub in (1, 2):
                self.od.set(0x60FF, sub, 0)
            self.od.set(0x6040, 0, 0)
            self.faults = 0
            self.od.set(0x603F, 0, 0)
            self.rpm = [0.0, 0.0]
            self.enabled = False
            self.quick_stop = False
            self.link_lost = False
            self.last_rx = time.monotonic()
            self.nmt_state = NMT_OPERATIONAL
            self._send_boot()

    # --- protocol ----------------------------------------------------------
    def on_rx_frame(self, msg):
        self.last_rx = time.monotonic()
        if self.link_lost:
            self.link_lost = False

    def swallow_write(self, index, sub, value):
        """Failure injection: True means no SDO response and no effect."""
        if self.stop_write_fault != "timeout" or not self._is_stop_write(index, value):
            return False
        self.dropped_writes.append((index, sub, value))
        return True

    def _is_stop_write(self, index, value):
        """A write that would stop the drive, only while it is enabled (so the enable
        sequence of a run still works and the failure comes after a good measurement)."""
        if not self.enabled:
            return False
        return (index == 0x60FF and value == 0) or (index == 0x6040 and (value & 0x0F) != 0x0F)

    def on_write(self, index, sub, value):
        od = self.od
        if self.stop_write_fault == "abort" and self._is_stop_write(index, value):
            self.dropped_writes.append((index, sub, value))
            return ABORT_GENERAL
        if index == 0x6040:
            self._controlword(value)
        elif index == 0x6060:
            if value != 3:
                return ABORT_BAD_VALUE  # only profile velocity is emulated
        elif index == 0x60FF:
            if not -MAX_RPM <= value <= MAX_RPM:
                return ABORT_BAD_VALUE
            self.link_lost = False
        elif index == 0x2000:
            if not 0 <= value <= 32767:
                return ABORT_BAD_VALUE
        elif index == 0x1017:
            self.heartbeat_ms = value  # 1017h is in ms here, TODO(Evans): settle the unit by capture
        elif index == 0x2010:
            if value == 1:
                for key in PERSISTENT:
                    self.eeprom[key] = od.get(*key)
                self.saves += 1
            return None
        od.set(index, sub, value)
        return None

    def _controlword(self, cw):
        masked = cw & 0x8F
        if masked & 0x80:
            if self.faults:
                self.faults = 0
                self.od.set(0x603F, 0, 0)
            self.enabled = False
            self.quick_stop = False
        elif (cw & 0x0F) == 0x0F:
            if not self.faults:
                self.enabled = True
                self.quick_stop = False
        elif (cw & 0x0F) in (0x06, 0x07):
            self.enabled = False
        elif (cw & 0x06) == 0x02 and not (cw & 0x04):
            self.quick_stop = True

    def target_rpm(self, side_index):
        if not self.enabled or self.faults or self.quick_stop or self.link_lost:
            return 0.0
        return float(self.od.get(0x60FF, side_index + 1)) * self.speed_scale[side_index]

    def tick(self, dt):
        timeout_ms = self.od.get(0x2000, 0)
        if (timeout_ms > 0 and not self.link_lost and
                time.monotonic() - self.last_rx > timeout_ms / 1000.0):
            self.link_lost = True
            self.link_loss_events += 1
        for i in (0, 1):
            tgt = self.target_rpm(i)
            ramp_ms = self.od.get(0x6084 if abs(tgt) < abs(self.rpm[i]) else 0x6083, i + 1)
            if self.quick_stop:
                ramp_ms = max(1, self.od.get(0x6085, 0))
            step = MAX_RPM * dt * 1000.0 / max(1, ramp_ms)
            diff = tgt - self.rpm[i]
            self.rpm[i] += max(-step, min(step, diff))
            self.od.set(0x606C, i + 1, int(round(self.rpm[i] * 10)))
        sw = STATUS_VOLTAGE
        if not self.quick_stop:
            sw |= STATUS_QUICK_STOP_N
        if self.faults:
            sw |= STATUS_FAULT
        elif self.enabled:
            sw |= STATUS_READY | STATUS_SWITCHED_ON | STATUS_ENABLED
        self.od.set(0x6041, 0, sw)
        self.od.set(0x603F, 0, self.faults)

    def state_line(self):
        with self.lock:
            return ("node %d nmt=0x%02X sw=0x%04X faults=0x%08X rpm L=%.1f R=%.1f "
                    "2000h=%d link_lost=%s bus_off=%s" % (
                        self.node_id, self.nmt_state, self.od.get(0x6041, 0), self.faults,
                        self.rpm[0], self.rpm[1], self.od.get(0x2000, 0),
                        self.link_lost, self.bus_off))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--node-id", type=int, default=1)
    ap.add_argument("--print-interval", type=float, default=1.0)
    args = ap.parse_args()
    bus = can.Bus(interface="socketcan", channel=args.channel)
    node = Zlac8015dEmulator(bus, args.node_id).start()
    cmds = {"oc-lo": lambda: node.inject_over_current("low"),
            "oc-hi": lambda: node.inject_over_current("high"),
            "uv": node.inject_under_voltage,
            "enc-lo": lambda: node.inject_encoder_error("low"),
            "enc-hi": lambda: node.inject_encoder_error("high"),
            "busoff": lambda: node.set_bus_off(True),
            "busoff-clear": lambda: node.set_bus_off(False),
            "power-cycle": node.power_cycle}

    def console():
        print("commands: " + ", ".join(cmds) + " (type and press Enter)")
        try:
            for line in iter(input, None):
                fn = cmds.get(line.strip())
                if fn:
                    fn()
        except EOFError:
            pass  # no console attached, keep running
    threading.Thread(target=console, daemon=True).start()
    try:
        while True:
            print(node.state_line(), flush=True)
            time.sleep(args.print_interval)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.stop()
        bus.shutdown()


if __name__ == "__main__":
    main()
