# SPDX-License-Identifier: MIT
# Monitor commands for the SIL Renode machine, loaded INTO Renode (IronPython 2.7) with
# "include @sil_renode_helpers.py" by openamrobot_sil/renode_fw.py. Test harness only: it
# changes how Renode runs the image and what it records, never the image.
#
#   sil_follow_systick             SysTick model frequency follows SystemCoreClock (see below)
#   sil_throttle_start ms          virtual time may not run ahead of wall time (see below)
#   sil_time_start ms              print a "SIL_TIME {json}" line every ms of virtual time
#   sil_time_report                print one "SIL_TIME {json}" line
#   sil_fw_tap "sysbus.fdcan1" "fdcan1" @file
#                                  write every frame THAT PERIPHERAL transmits to file
#   sil_hub_tap "can1" "hub1" @file
#                                  attach a listener to a CAN hub: every frame any other
#                                  member puts on the hub goes to file
#   sil_hub_send "can1" id "40 6C 60 01 00 00 00 00"
#                                  the listener of that hub sends one frame (a foreign sender)
#   sil_can_link "can1" "sysbus.fdcan1" 0|1
#                                  detach or attach a peripheral from its hub (bus loss)
#   sil_uart_hex "sysbus.usart2" "59 59 00"
#                                  bytes into a UART receiver
#   sil_can_state                  print one "SIL_CAN {json}" line: CCCR of FDCAN1, 2 and 3
#
# Tap files are candump -L lines, "(seconds) label ID#DATA". The time is Renode virtual time.
#
# sil_follow_systick is adapted from _calibrate_systick in openamr-platform-fw
# tools/ci/renode_drive_stub.py at fcf81f8e2fe2fa9000685ab233907adbfadeec2a (MIT).
from Antmicro.Renode.Core import EmulationManager
from Antmicro.Renode.Core.CAN import CANMessageFrame
from Antmicro.Renode.Core.Structure import NullRegistrationPoint
from Antmicro.Renode.Peripherals.Bus import Access, BusHookDelegate, SysbusAccessWidth
from Antmicro.Renode.Peripherals.CAN import ICAN
from Antmicro.Renode.Time import TimeInterval
from Antmicro.Renode.Tools.Network import CANHub
from System import Action, Array, Byte, UInt64
from System.Diagnostics import Stopwatch
from System.Threading import Thread
from threading import Lock

STATE = {"generation": 0, "slice_us": 5000, "sleeps": 0, "slept_ms": 0, "slices": 0,
         "throttle": False, "wall": None, "virtual0_us": 0, "min_slice_wall_us": -1,
         "listeners": {}}
# FDCAN CCCR (offset 18h). Bit 0 INIT is 1 after reset and 0 once the HAL started the peripheral.
CCCR = (("fdcan1", 0x4000A018), ("fdcan2", 0x4000A418), ("fdcan3", 0x4000D418))


def _machine():
    return list(EmulationManager.Instance.CurrentEmulation.Machines)[0]


def _virtual_us(machine):
    return int(machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds)


def _install_systick_hook(machine):
    bus = machine.SystemBus
    address = bus.GetSymbolAddress("SystemCoreClock")
    nvic = machine["sysbus.nvic"]

    def on_write(cpu, addr, width, value):
        if value != 0:
            nvic.Frequency = int(value)
    bus.AddWatchpointHook(address, SysbusAccessWidth.DoubleWord, Access.Write,
                          BusHookDelegate(on_write))


def mc_sil_follow_systick():
    # Renode's STM32H7 RCC model does not keep the PLL and divider registers, so the HAL derives
    # a SystemCoreClock that is not the board's and programs SysTick from it. To keep "1 tick =
    # 1 ms of virtual time" the SysTick model frequency follows every value the firmware writes
    # to SystemCoreClock. The clock tree is therefore not checked.
    # Renode drops the hook on a machine reset: the node calls this again after one.
    _install_systick_hook(_machine())


def _schedule_throttle(machine):
    STATE["generation"] += 1
    generation = STATE["generation"]
    watch = Stopwatch.StartNew()
    first = [True]

    def tick(_ts):
        if generation != STATE["generation"]:
            return
        STATE["slices"] += 1
        ahead_ms = STATE["slice_us"] / 1000 - watch.ElapsedMilliseconds
        if ahead_ms > 0:
            STATE["sleeps"] += 1
            STATE["slept_ms"] += ahead_ms
            Thread.Sleep(int(ahead_ms))
        # The rule by construction: do not go on before the slice length has passed.
        while watch.Elapsed.TotalMilliseconds * 1000 < STATE["slice_us"]:
            Thread.Sleep(0)
        took_us = int(watch.Elapsed.TotalMilliseconds * 1000)
        # The first slice after a start or reset includes monitor time: not a rate sample.
        if not first[0] and (STATE["min_slice_wall_us"] < 0
                             or took_us < STATE["min_slice_wall_us"]):
            STATE["min_slice_wall_us"] = took_us
        first[0] = False
        watch.Restart()
        machine.ScheduleAction(TimeInterval.FromMicroseconds(UInt64(STATE["slice_us"])),
                               Action[TimeInterval](tick))
    machine.ScheduleAction(TimeInterval.FromMicroseconds(UInt64(STATE["slice_us"])),
                           Action[TimeInterval](tick))


def mc_sil_throttle_start(slice_ms):
    # Every slice of virtual time takes at least the same slice of wall time: the emulation
    # thread sleeps at the end of a slice that ran faster. Renode may run slower than wall time
    # (it does with the image's busy idle loop), never faster. The rule holds per slice, so a
    # Renode that fell behind does not catch up later by running fast. The node calls this
    # again after a machine reset; an older chain of ticks then ends by its generation.
    machine = _machine()
    STATE["slice_us"] = int(slice_ms) * 1000
    _start_clock(machine)
    STATE["throttle"] = True
    _schedule_throttle(machine)


def _start_clock(machine):
    if STATE["wall"] is None:
        STATE["wall"] = Stopwatch.StartNew()
        STATE["virtual0_us"] = _virtual_us(machine)


def mc_sil_time_start(period_ms):
    # A SIL_TIME line every period of virtual time, with or without the throttle: the
    # micro-ROS image runs without it. It measures, it does not slow anything down.
    machine = _machine()
    _start_clock(machine)
    STATE["report_generation"] = STATE.get("report_generation", 0) + 1
    generation = STATE["report_generation"]
    period = UInt64(int(period_ms) * 1000)

    def report(_ts):
        if generation != STATE["report_generation"]:
            return
        mc_sil_time_report()
        machine.ScheduleAction(TimeInterval.FromMicroseconds(period),
                               Action[TimeInterval](report))
    machine.ScheduleAction(TimeInterval.FromMicroseconds(period), Action[TimeInterval](report))


def mc_sil_time_report():
    machine = _machine()
    wall_us = int(STATE["wall"].Elapsed.TotalMilliseconds * 1000) if STATE["wall"] else 0
    print ('SIL_TIME {"virtual_us":%d,"wall_us":%d,"slices":%d,"sleeps":%d,"slept_ms":%d,'
           '"slice_us":%d,"min_slice_wall_us":%d,"throttle":%d,"systick_hz":%d}' % (
               _virtual_us(machine) - STATE["virtual0_us"], wall_us, STATE["slices"],
               STATE["sleeps"], STATE["slept_ms"], STATE["slice_us"],
               STATE["min_slice_wall_us"], 1 if STATE["throttle"] else 0,
               int(machine["sysbus.nvic"].Frequency)))


# ---- CAN taps ---------------------------------------------------------------------------
class TapFile(object):
    """One writer per file: the taps run on the emulation thread and the monitor thread."""

    def __init__(self, path):
        self.out = open(path, "a")
        self.lock = Lock()

    def write(self, line):
        with self.lock:
            self.out.write(line)
            self.out.flush()


FILES = {}


def _tap_file(path):
    path = str(path)
    if path not in FILES:
        FILES[path] = TapFile(path)
    return FILES[path]


def _line(machine, label, frame):
    if frame.ExtendedFormat:
        ident = "%08X" % frame.Id
    else:
        ident = "%03X" % frame.Id
    data = "".join("%02X" % int(b) for b in frame.Data)
    if frame.RemoteFrame:
        data = "R"
    sep = "##0" if frame.FDFormat else "#"
    return "(%.6f) %s %s%s%s\n" % (_virtual_us(machine) / 1e6, label, ident, sep, data)


def mc_sil_fw_tap(peripheral_name, label, path):
    # Firmware side of the read-only proof: FrameSent of an FDCAN model fires for the frames
    # that peripheral transmits, and for no frame of any other bus member.
    machine = _machine()
    out = _tap_file(path)

    def on_sent(frame):
        out.write(_line(machine, label, frame))
    machine[peripheral_name].FrameSent += Action[CANMessageFrame](on_sent)


class HubListener(ICAN):
    """A bus member that records what the others send, and can send a frame itself."""

    def __init__(self, machine, label, path):
        self.machine = machine
        self.label = label
        self.out = _tap_file(path)
        self.handlers = []

    def add_FrameSent(self, handler):
        self.handlers.append(handler)

    def remove_FrameSent(self, handler):
        self.handlers.remove(handler)

    def Reset(self):
        pass

    def OnFrameReceived(self, frame):
        self.out.write(_line(self.machine, self.label, frame))

    def transmit(self, can_id, data):
        frame = CANMessageFrame(can_id, Array[Byte](data), can_id > 0x7FF, False)
        # Its own frames are in the file too, under "<label>.inject": the hub does not echo.
        self.out.write(_line(self.machine, self.label + ".inject", frame))
        for handler in list(self.handlers):
            handler(frame)


def _hub(name):
    manager = EmulationManager.Instance.CurrentEmulation.ExternalsManager
    for hub in manager.GetExternalsOfType[CANHub]():
        found, hub_name = manager.TryGetName(hub)
        if found and hub_name == name:
            return hub
    raise Exception("no CAN hub named " + name)


def mc_sil_hub_tap(hub_name, label, path):
    machine = _machine()
    listener = HubListener(machine, label, str(path))
    # The hub looks up the machine of every member, so the listener is registered as a
    # peripheral without a bus address (as the platform-fw drive stub is).
    machine.SystemBus.Register(listener, NullRegistrationPoint.Instance)
    machine.SetLocalName(listener, "silTap_" + hub_name)
    _hub(hub_name).AttachTo(listener)
    STATE["listeners"][hub_name] = listener


def _bytes(text):
    return [int(part, 16) for part in str(text).split()]


def _synced(machine, fn):
    # Run fn from the machine's time domain, as a peripheral model would. A monitor command
    # runs on another thread; writing into a UART or a CAN model from there while the CPU
    # executes is a race inside Renode, not a stimulus (docs/architecture/sil-topology.md).
    machine.ScheduleAction(TimeInterval.FromMicroseconds(UInt64(100)),
                           Action[TimeInterval](lambda _ts: fn()))


def mc_sil_hub_send(hub_name, can_id, data_hex):
    listener, data = STATE["listeners"][hub_name], _bytes(data_hex)
    _synced(_machine(), lambda: listener.transmit(int(can_id), data))


def mc_sil_can_link(hub_name, peripheral_name, on):
    peripheral = _machine()[peripheral_name]
    if int(on):
        _hub(hub_name).AttachTo(peripheral)
    else:
        _hub(hub_name).DetachFrom(peripheral)


def mc_sil_uart_hex(uart_name, data_hex):
    machine = _machine()
    uart, data = machine[uart_name], _bytes(data_hex)

    def write():
        for b in data:
            uart.WriteChar(b)
    if STATE.get("unsynced_uart"):       # only for the harness experiment in the docs
        write()
    else:
        _synced(machine, write)


def mc_sil_unsynced_uart(on):
    STATE["unsynced_uart"] = int(on)


def mc_sil_can_state():
    bus = _machine().SystemBus
    print "SIL_CAN {%s}" % ",".join(
        '"%s":{"cccr":%d}' % (name, int(bus.ReadDoubleWord(address))) for name, address in CCCR)
