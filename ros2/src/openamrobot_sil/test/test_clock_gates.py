# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Gate tests D3 of the clock contract (docs/architecture/sil-topology.md section 6.3).

The CTL-017 tests run in lock-step: one loop advances the fed simulation clock, the stand-in
lift joint, the emulator and the host stand-in, with a separate manual "wall" clock for the
coordinator's stall check (C5). Nothing in them depends on how fast the runner is, except
test_lift_rate_window_does_not_depend_on_runner_speed, which slows the loop on purpose.
The drive tests run the ZLAC8015D emulator on its own thread (its tick still reads host time,
openamr-platform-fw#57) and step the host stand-in by hand. Protocol evidence only.
"""
import time

import can
import pytest

from conftest import wait_for
from openamrobot_sil.can_sil import canopen_common, ctl017_emulator as c
from openamrobot_sil.ctl017_gazebo import GazeboCtl017, PermitConfig
from openamrobot_sil.sim_time import ClockCoordinator, FedClock, ManualClock
from openamrobot_sil.stm32_host import DriveHost, LiftHost
from openamrobot_sil.zlac_gazebo import GazeboZlac8015d

PERMIT = PermitConfig(cob_id=0x410, flag_bits=8, boot_bits=16, seq_bits=16, reason_bits=8,
                      permit_value=1, timeout_ms=200, refused_result=0x80)
DT = 0.005
BOOT_UP = 0x700 + c.NODE_ID


class LockStepLift:
    """Gazebo stand-in in lock-step: fed /clock, lift joint, CTL-017 and the host stand-in."""

    def __init__(self, make_bus, position_m=0.0, start_s=10.0):
        self.make_bus = make_bus
        self.wall = ManualClock()
        self.clock = FedClock(start_s)
        self.coord = ClockCoordinator(self.clock, steady=self.wall)
        self.coord.feed(start_s)
        self.position_m = position_m
        self.gz_paused = False
        self.held = None          # list: permit frames held back (delay injection)
        self.reports = []
        self.raw = make_bus()
        self.emu = self._new_emu(operational=True)
        self.host = LiftHost(make_bus(), permit=PERMIT, clock=self.clock)
        self._hb_ms = 0
        self.wall_sleep = 0.0     # host sleep per step: slows the runner, not the simulation

    def _new_emu(self, operational):
        emu = GazeboCtl017(self.make_bus(), permit=PERMIT, operational=operational,
                           clock=self.clock)
        emu.attach_external_plant(self.position_m)
        return emu

    def restart_emulator(self):
        """Emulator process restart: a new node object, boot-up frame, nothing kept."""
        self.emu.bus.shutdown()
        self.emu = self._new_emu(operational=False)
        self.emu._send_boot()

    def pump(self):
        msg = self.emu.bus.recv(timeout=0)
        while msg is not None:
            if self.held is not None and msg.arbitration_id == PERMIT.cob_id:
                self.held.append(msg)
            else:
                with self.emu.lock:
                    self.emu._handle(msg)
            msg = self.emu.bus.recv(timeout=0)

    def release_held(self):
        held, self.held = self.held, None
        for msg in held:
            with self.emu.lock:
                self.emu._handle(msg)

    def step(self, wall_sleep=None):
        self.wall.advance(DT)
        if not self.gz_paused:
            self.position_m += self.emu.demand_velocity_m_s() * DT
            self.position_m = min(0.350, max(0.0, self.position_m))
            self.coord.feed(self.clock.now() + DT)
            self.emu.set_joint_position(self.position_m)
        report = self.coord.check()
        if report:
            self.reports.append(report)
        self.pump()
        with self.emu.lock:
            self.emu.tick(DT)
            now_ms = self.emu._now_ms()
            if now_ms - self._hb_ms >= 100 or now_ms < self._hb_ms:   # 100 ms heartbeat
                self._hb_ms = now_ms
                self.emu._send(canopen_common.HB_BASE + self.emu.node_id, [self.emu.nmt_state])
        if not self.clock.paused:   # the stand-in's loop waits on the fed clock (C1)
            self.host.follow_clock()
            self.host.step()
        wall_sleep = self.wall_sleep if wall_sleep is None else wall_sleep
        if wall_sleep:
            time.sleep(wall_sleep)

    def run(self, seconds, **kw):
        for _ in range(int(round(seconds / DT))):
            self.step(**kw)

    def until(self, cond, seconds):
        for _ in range(int(round(seconds / DT))):
            if cond():
                return True
            self.step()
        return bool(cond())

    def reset_time(self, t=0.0):
        """Gazebo reset: /clock moves backward."""
        self.coord.feed(t)

    def ready(self):
        return self.until(lambda: self.emu.state == c.IDLE and self.emu._standstill(
            self.emu._now_ms()), 2.0)

    def start_move(self, stop=2):
        assert self.ready()
        self.host.set_permit(True)
        assert self.until(self.emu.permit_fresh, 0.5)
        self.host.command(c.OP_MOVE_TO_STOP, stop)
        assert self.until(lambda: self.position_m > 0.005, 1.5)


def frames(bus, arb_id):
    out = []
    msg = bus.recv(timeout=0)
    while msg is not None:
        if msg.arbitration_id == arb_id:
            out.append(bytes(msg.data))
        msg = bus.recv(timeout=0)
    return out


@pytest.fixture
def lift(make_bus):
    return lambda **kw: LockStepLift(make_bus, **kw)


# --- (a) pause 5 s wall time, then resume ------------------------------------------------------

def test_gazebo_pause_freezes_the_lift_timers(lift):
    r = lift()
    r.start_move()
    r.gz_paused = True
    r.run(0.9)          # 0.9 s wall, below the stall bound: simulation time frozen
    assert r.emu.drive_on and r.emu.faults == 0 and r.reports == []
    assert r.emu.permit_fresh()   # no device timeout fires in the pause


def test_pause_longer_than_the_stall_bound_drops_motion_and_permit_and_nothing_rearms(lift):
    r = lift()
    r.start_move()
    r.gz_paused = True
    r.run(5.0)                                            # Gazebo paused 5 s wall time
    assert len(r.reports) == 1 and r.reports[0].startswith('simulation stalled')
    assert not r.emu.drive_on and r.emu.target_stop == 0xFF
    assert not r.emu.permit_valid and r.emu.clock_stalls == 1
    assert r.emu.faults == 0                              # a stall is never a device fault
    old_cmd = r.host.cmd
    held = r.position_m
    r.gz_paused = False
    r.step()
    assert not r.coord.clock.paused
    assert r.host.cmd is None and r.host.clock_events == 1   # the stand-in dropped its command
    r.run(1.0)
    assert r.emu.permit_fresh()                           # new permit frames after the resume
    r.raw.send(can.Message(arbitration_id=c.RPDO1, data=bytes(old_cmd), is_extended_id=False))
    r.run(1.0)  # the old command replayed is a keep-alive only
    assert not r.emu.drive_on and r.position_m == pytest.approx(held, abs=1e-6)
    assert r.emu.faults == 0
    r.host.command(c.OP_MOVE_TO_STOP, 2)                  # a new command after the resume
    assert r.until(lambda: r.position_m > held + 0.003, 1.0)


def test_a_permit_frame_received_during_the_pause_does_not_survive_the_resume(lift):
    r = lift()
    r.start_move()
    r.gz_paused = True
    r.run(1.5)
    assert not r.emu.permit_valid
    r.host.permit_silent = True
    payload = PERMIT.pack(PERMIT.permit_value, r.host.boot_id, r.host.permit_seq + 1, 0)
    r.raw.send(can.Message(arbitration_id=PERMIT.cob_id, data=payload, is_extended_id=False))
    r.step()
    assert r.emu.permit_valid                              # accepted while paused ...
    r.gz_paused = False
    r.step()
    assert not r.emu.permit_fresh()                        # ... and invalid after the resume


# --- (b) simulation time reset ----------------------------------------------------------------

def test_time_reset_is_a_power_cycle_of_the_lift_node(lift):
    r = lift()
    r.start_move()
    frames(r.raw, BOOT_UP)
    r.reset_time(0.0)
    r.step()
    assert r.emu.clock_resets == 1
    assert b'\x00' in frames(r.raw, BOOT_UP)               # boot-up frame
    assert r.emu.nmt_state == canopen_common.NMT_PRE_OPERATIONAL
    assert not r.emu.drive_on and r.emu.target_stop == 0xFF and not r.emu.permit_valid
    assert r.host.cmd is None
    r.run(1.0)                                             # host starts the node, permit returns
    assert r.emu.nmt_state == canopen_common.NMT_OPERATIONAL and r.emu.permit_fresh()
    assert not r.emu.drive_on                              # nothing re-arms by itself
    assert not r.emu.homed and r.emu.state == c.UNHOMED    # off a stop sensor: unhomed


def test_time_reset_at_a_stop_sensor_is_homed_again(lift):
    r = lift(position_m=0.0)
    assert r.ready()
    r.reset_time(0.0)
    r.run(0.2)
    assert r.emu.clock_resets == 1
    assert r.emu.homed and r.emu.position_valid and r.emu.sensor_bits() == c.IN_STOP0


# --- (c) emulator restart ---------------------------------------------------------------------

def test_emulator_restart_keeps_nothing_and_the_host_does_not_replay(lift):
    r = lift()
    r.start_move()
    r.restart_emulator()
    r.step()
    assert r.host.cmd is None                              # boot-up seen: command dropped
    assert not r.emu.permit_valid and not r.emu.drive_on
    r.run(1.5)
    assert r.emu.nmt_state == canopen_common.NMT_OPERATIONAL and r.emu.permit_fresh()
    assert not r.emu.drive_on and r.emu.target_stop == 0xFF


# --- (d) delayed CAN frames are judged by receipt time ------------------------------------------

def test_delayed_permit_frames_are_judged_by_receipt_time(lift):
    r = lift()
    assert r.ready()
    r.host.set_permit(True)
    assert r.until(r.emu.permit_fresh, 0.5)
    r.held = []                                            # permit frames delayed from here
    r.run(0.3)
    assert not r.emu.permit_fresh()                        # last receipt 0.3 s ago > 0.2 s
    r.host.command(c.OP_MOVE_TO_STOP, 1)
    r.step()
    assert r.emu.cmd_result == PERMIT.refused_result       # stale permit: command refused
    assert len(r.held) >= 5
    r.release_held()                                       # the late frames arrive now
    assert r.emu.permit_fresh()                            # fresh from their receipt time
    r.run(0.5)
    assert not r.emu.drive_on                              # the refused command is not replayed


# --- runner speed (rule C1, F02 Hall rate window) -----------------------------------------------

def test_lift_rate_window_does_not_depend_on_runner_speed(lift):
    """The loop runs at about a third of real time; on host time F02 or F07 would trip."""
    r = lift()
    r.wall_sleep = 3 * DT                  # every step: 5 ms simulation, about 15 ms wall time
    r.start_move(stop=1)
    start = r.position_m
    r.run(1.0)                             # 1 s simulation time, about 3 s wall time
    assert r.emu.faults == 0 and r.emu.drive_on
    assert r.position_m - start == pytest.approx(0.018, abs=0.003)


# --- drive: ZLAC8015D and the host stand-in ---------------------------------------------------

@pytest.fixture
def drive(make_bus):
    clock = ManualClock(10.0)
    emu = GazeboZlac8015d(make_bus(), 1, clock=clock).start()
    host = DriveHost(make_bus(), 1, 250, 0.100, 0.400, clock=clock)
    raw = make_bus()
    yield clock, emu, host, raw
    emu.stop()


def run_host(clock, host, n=10):
    for _ in range(n):
        clock.advance(0.02)
        host.follow_clock()
        host.step()


def drive_moving(clock, emu, host):
    host.set_twist(0.2, 0.0)
    run_host(clock, host)
    assert host.state()['configured'] and emu.enabled
    assert emu.state()['target_rpm'] == [19, 19]


@pytest.mark.parametrize('event', ['stall', 'reset'])
def test_drive_stall_or_reset_drops_target_and_enable_and_nothing_rearms(drive, event):
    clock, emu, host, raw = drive
    drive_moving(clock, emu, host)
    frames(raw, 0x701)
    if event == 'stall':
        clock.declare_stall()
    else:
        clock.reset(0.0)
    assert wait_for(lambda: getattr(emu, 'clock_%ss' % event) == 1, 1.0)
    assert not emu.enabled and emu.state()['target_rpm'] == [0, 0]
    assert emu.demand_rad_s() == (0.0, 0.0)
    assert b'\x00' in frames(raw, 0x701)                   # boot-up frame of the drive
    clock.declare_resume()
    run_host(clock, host)                                  # the stand-in brings the drive up again
    assert host.clock_events == 1 and host.state()['configured'] and emu.enabled
    assert emu.state()['target_rpm'] == [0, 0]             # with a zero target, not the old twist
    host.set_twist(0.2, 0.0)                               # a new command moves it
    run_host(clock, host, 2)
    assert emu.state()['target_rpm'] == [19, 19]


def test_wheel_feedback_age_is_receipt_time_on_the_fed_clock(drive):
    clock, emu, _host, _raw = drive
    emu.set_wheel_feedback(1.0, 1.0)
    clock.paused = True
    time.sleep(0.6)                                        # longer than the 0.5 s timeout, wall
    assert emu.feedback_fresh()                            # simulation time did not advance
    clock.paused = False
    clock.advance(0.6)
    assert not emu.feedback_fresh()
    emu.set_wheel_feedback(1.0, 1.0)
    clock.declare_stall()
    assert not emu.feedback_fresh()                        # nothing from before a stall


# --- host stand-in pacing (rule C1) -----------------------------------------------------------

def test_host_stand_in_paces_on_the_fed_clock(make_bus):
    clock = ManualClock(0.0)
    host = LiftHost(make_bus(), clock=clock).start()
    raw = make_bus()
    try:
        time.sleep(0.3)                                    # wall time passes, simulation does not
        assert frames(raw, 0x701) == []                    # no core heartbeat
        for _ in range(100):                               # 1 s simulation time
            clock.advance(0.01)
            time.sleep(0.004)
        beats = len(frames(raw, 0x701))
        assert 8 <= beats <= 11, beats                     # 100 ms heartbeat in simulation time
    finally:
        host.stop()
