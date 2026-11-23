# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Lift path at a low real-time factor, on the threads a SIL launch runs.

Baseline 1 phase 4 was BLOCKED: the CTL-017 model faulted while the lift moved below a
real-time factor (RTF) of about 0.55, and the hosted runner reaches 0.13 to 0.18. This test runs
the lift path as the launch does: the vendored CTL-017 emulator on its own node thread
(``CanopenNode._run``), the host stand-in on its own thread on the fed clock, a Gazebo stand-in
thread that steps physics in 1 ms of simulation time and takes 1 ms / RTF of host time per step,
and a host steady health thread for the stall check (C5). The firmware path is used end to end:
permit and RPDO1 frames on the bus, the emulator's demand moves the stand-in joint, the joint
position gives the Hall count and stop sensors.

Three clocks (docs/architecture/clock-reset-contract.md, "Three clocks"):
- simulation time: the stand-in joint, the fed /clock and every feedback freshness check;
- firmware time: every CTL-017 timer, the emulator step and its heartbeat producer, on the fed
  simulation clock in SIL;
- wall-clock supervision: the stall check of the clock coordinator only, and the
  simulation-only hold of the Gazebo lift command.

Development evidence only: no Gazebo, no ROS, no firmware binary, no hardware. Not safety
evidence.
"""
import threading
import time

import pytest

from openamrobot_sil.can_sil import ctl017_emulator as c
from openamrobot_sil.ctl017_gazebo import GazeboCtl017
from openamrobot_sil.sim_time import ClockCoordinator, FedClock
from openamrobot_sil.stm32_host import LiftHost
from test_clock_gates import PERMIT

PHYSICS_DT = 0.001         # Gazebo step, simulation seconds
JOINT_STATE_EVERY = 10     # joint state to the emulator process every 10 steps (100 Hz)
CMD_EVERY = 10             # ctl017_node on_cmd device timer, 10 ms of ROS (simulation) time
HEALTH_PERIOD_S = 0.1      # NodeClock health timer, host steady time


class ThreadedRtfLift:
    """Gazebo stand-in at a given real-time factor next to the real emulator and stand-in
    threads."""

    def __init__(self, make_bus, rtf, position_m=0.0, start_s=10.0):
        self.rtf = rtf
        self.clock = FedClock()
        self.coord = ClockCoordinator(self.clock)
        self.position_m = position_m
        self.sim_t = start_s
        self.gz_demand = 0.0
        self.reports = []
        self.emu = GazeboCtl017(make_bus(), permit=PERMIT, operational=True, clock=self.clock)
        self.emu.attach_external_plant(position_m)
        self.host = LiftHost(make_bus(), permit=PERMIT, clock=self.clock)
        self._run = False
        self._threads = []

    def start(self):
        self._run = True
        self.coord.feed(self.sim_t)        # first /clock
        self.emu.start()
        self.host.start()
        for target in (self._gazebo, self._health):
            th = threading.Thread(target=target, daemon=True)
            th.start()
            self._threads.append(th)
        return self

    def stop(self):
        self._run = False
        for th in self._threads:
            th.join(timeout=2)
        self.host.stop()
        self.emu.stop()

    def _gazebo(self):
        step = 0
        wall_per_step = PHYSICS_DT / self.rtf
        next_wall = time.monotonic()
        while self._run:
            next_wall += wall_per_step
            delay = next_wall - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            step += 1
            if step % CMD_EVERY == 0:
                self.gz_demand = self.emu.demand_velocity_m_s()
            self.position_m = min(0.350, max(0.0, self.position_m + self.gz_demand * PHYSICS_DT))
            self.sim_t += PHYSICS_DT
            self.coord.feed(self.sim_t)
            if step % JOINT_STATE_EVERY == 0:
                self.emu.set_joint_position(self.position_m)

    def _health(self):
        while self._run:
            time.sleep(HEALTH_PERIOD_S)
            report = self.coord.check()
            if report:
                self.reports.append(report)
                self.gz_demand = 0.0          # simulation-only hold (NodeClock.on_stall)

    def wait_sim(self, sim_seconds, cond=None):
        """Wait until sim_seconds of simulation time have passed or cond() holds."""
        end = self.clock.now() + sim_seconds
        wall_bound = time.monotonic() + 3.0 * sim_seconds / self.rtf + 5.0
        while self.clock.now() < end and time.monotonic() < wall_bound:
            if cond is not None and cond():
                return True
            time.sleep(0.005)
        return bool(cond()) if cond is not None else self.clock.now() >= end


@pytest.mark.parametrize('rtf', [0.1, 0.2])
def test_lift_moves_without_fault_at_low_real_time_factor(make_bus, rtf):
    """At RTF 0.1 and 0.2 the lift leaves L1000 under permit and keeps moving for 1.5 s of
    simulation time (past the 300 ms Hall start grace, several 200 ms Hall windows and the
    300 ms stall bound) with no fault and the Hall rate inside its band."""
    r = ThreadedRtfLift(make_bus, rtf).start()
    try:
        emu = r.emu
        assert r.wait_sim(2.0, lambda: emu.state == c.IDLE and emu._standstill(emu._now_ms()))
        r.host.set_permit(True)
        assert r.wait_sim(1.0, emu.permit_fresh)
        r.host.command(c.OP_MOVE_TO_STOP, 1)
        assert r.wait_sim(1.0, lambda: emu.drive_on)
        t_move = r.clock.now()
        r.wait_sim(1.5, lambda: emu.faults != 0)
        state = emu.state_dict()
        moved_mm = r.position_m * 1000.0
        assert state['faults'] == 0, (rtf, state, moved_mm, r.reports)
        assert state['state'] == c.MOVING, (rtf, state)
        # 1.5 s at 18 mm/s is 27 mm on simulation time; a host-time plant would differ by 1/RTF
        elapsed = r.clock.now() - t_move
        assert moved_mm == pytest.approx(18.0 * elapsed, rel=0.25), (moved_mm, elapsed)
        assert r.reports == []                 # no stall: the simulation never stopped
    finally:
        r.stop()


class StepLog(GazeboCtl017):
    """Records the dt of every emulator step (the firmware-time step of the node thread)."""

    def __init__(self, *a, **kw):
        self.dts = []
        super().__init__(*a, **kw)

    def tick(self, dt):
        self.dts.append(dt)
        super().tick(dt)


def heartbeat_times(r, bus, sim_seconds):
    """Simulation receipt times of the CTL-017 heartbeat frames for sim_seconds."""
    times = []
    end = r.clock.now() + sim_seconds
    wall_bound = time.monotonic() + 3.0 * sim_seconds / r.rtf + 5.0
    while r.clock.now() < end and time.monotonic() < wall_bound:
        msg = bus.recv(timeout=0.002)
        if msg is not None and msg.arbitration_id == HEARTBEAT_ID:
            times.append(r.clock.now())
    return times


HEARTBEAT_ID = 0x700 + c.NODE_ID


@pytest.mark.parametrize('rtf', [0.1, 0.2])
def test_lift_heartbeat_and_step_run_on_firmware_time_at_low_real_time_factor(make_bus, rtf):
    """Firmware time is the fed simulation clock: the CTL-017 heartbeat producer keeps its
    100 ms period in simulation time, and the emulator steps add up to the simulation time that
    passed, at RTF 0.1 and 0.2. On host time the period would be 100 ms * RTF of simulation
    time and the steps would add up to 1 / RTF times the simulation time."""
    raw = make_bus()
    r = ThreadedRtfLift(make_bus, rtf)
    r.emu = StepLog(make_bus(), permit=PERMIT, operational=True, clock=r.clock)
    r.emu.attach_external_plant(0.0)
    r.start()
    try:
        assert r.wait_sim(0.3)                       # past boot-up
        t0 = r.clock.now()
        n0 = len(r.emu.dts)
        times = heartbeat_times(r, raw, 1.0)
        t1 = r.clock.now()
        stepped = sum(r.emu.dts[n0:])
        intervals = [b - a for a, b in zip(times, times[1:])]
        assert len(intervals) >= 5, times
        intervals.sort()
        median = intervals[len(intervals) // 2]
        assert median == pytest.approx(0.100, abs=0.015), (rtf, median, intervals)
        assert stepped == pytest.approx(t1 - t0, abs=0.05), (rtf, stepped, t1 - t0)
    finally:
        r.stop()
