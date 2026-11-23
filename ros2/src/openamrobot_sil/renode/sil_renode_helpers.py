# SPDX-License-Identifier: MIT
# Monitor commands for the SIL Renode machine, loaded INTO Renode (IronPython 2.7) with
# "include @sil_renode_helpers.py" by openamrobot_sil/renode_fw.py. Test harness only: it
# changes how Renode runs the image, never the image.
#
#   sil_follow_systick       SysTick model frequency follows SystemCoreClock (see below)
#   sil_throttle_start ms    virtual time may not run ahead of wall time (see below)
#   sil_time_report          print one "SIL_TIME {json}" line
#
# sil_follow_systick is adapted from _calibrate_systick in openamr-platform-fw
# tools/ci/renode_drive_stub.py at fcf81f8e2fe2fa9000685ab233907adbfadeec2a (MIT).
from Antmicro.Renode.Core import EmulationManager
from Antmicro.Renode.Peripherals.Bus import Access, BusHookDelegate, SysbusAccessWidth
from Antmicro.Renode.Time import TimeInterval
from System import Action, UInt64
from System.Diagnostics import Stopwatch
from System.Threading import Thread

STATE = {"generation": 0, "watch": None, "virtual_us": 0, "slice_us": 5000, "sleeps": 0,
         "slept_ms": 0, "slices": 0}


def _machine():
    return list(EmulationManager.Instance.CurrentEmulation.Machines)[0]


def mc_sil_follow_systick():
    # Renode's STM32H7 RCC model does not keep the PLL and divider registers, so the HAL derives
    # a SystemCoreClock that is not the board's and programs SysTick from it. To keep "1 tick =
    # 1 ms of virtual time" the SysTick model frequency follows every value the firmware writes
    # to SystemCoreClock. The clock tree is therefore not checked.
    machine = _machine()
    bus = machine.SystemBus
    address = bus.GetSymbolAddress("SystemCoreClock")
    nvic = machine["sysbus.nvic"]

    def on_write(cpu, addr, width, value):
        if value != 0:
            nvic.Frequency = int(value)
    bus.AddWatchpointHook(address, SysbusAccessWidth.DoubleWord, Access.Write,
                          BusHookDelegate(on_write))


def mc_sil_throttle_start(slice_ms):
    # Every slice of virtual time takes at least the same slice of wall time: the emulation
    # thread sleeps at the end of a slice that ran faster. Renode may run slower than wall time
    # (it does with the image's busy idle loop), never faster by more than one slice. Scheduled
    # actions do not survive "machine Reset", so the SIL node calls this again after a reset.
    machine = _machine()
    STATE["generation"] += 1
    generation = STATE["generation"]
    STATE["slice_us"] = int(slice_ms) * 1000
    watch = Stopwatch.StartNew()
    STATE["watch"] = watch

    def tick(_ts):
        if generation != STATE["generation"]:
            return
        STATE["virtual_us"] += STATE["slice_us"]
        STATE["slices"] += 1
        ahead_ms = STATE["slice_us"] / 1000 - watch.ElapsedMilliseconds
        if ahead_ms > 0:
            STATE["sleeps"] += 1
            STATE["slept_ms"] += ahead_ms
            Thread.Sleep(int(ahead_ms))
        watch.Restart()
        machine.ScheduleAction(TimeInterval.FromMicroseconds(UInt64(STATE["slice_us"])),
                               Action[TimeInterval](tick))
    machine.ScheduleAction(TimeInterval.FromMicroseconds(UInt64(STATE["slice_us"])),
                           Action[TimeInterval](tick))


def mc_sil_time_report():
    print('SIL_TIME {"virtual_us":%d,"slices":%d,"sleeps":%d,"slept_ms":%d,"slice_us":%d}' % (
        STATE["virtual_us"], STATE["slices"], STATE["sleeps"], STATE["slept_ms"],
        STATE["slice_us"]))
