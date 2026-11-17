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

    def __init__(self, make_bus, position_m=0.0, start_s=10.0, permit=PERMIT):
        self.make_bus = make_bus
        self.permit = permit
        self.wall = ManualClock()
        self.clock = FedClock(start_s)
        self.coord = ClockCoordinator(self.clock, steady=self.wall)
        self.coord.feed(start_s)
        self.position_m = position_m
        self.gz_paused = False
        self.duplicates_in_pause = False   # paused Gazebo still publishes its (frozen) time
        self.held = None          # list: frames held back (delay injection)
        self.hold_ids = (PERMIT.cob_id,)
        self.reports = []
        self.events = []          # what the coordinator returned for each /clock time
        self.raw = make_bus()
        self.emu = self._new_emu(operational=True)
        self.host = LiftHost(make_bus(), permit=permit, clock=self.clock)
        self._hb_ms = 0
        self.wall_sleep = 0.0     # host sleep per step: slows the runner, not the simulation

    def _new_emu(self, operational):
        emu = GazeboCtl017(self.make_bus(), permit=self.permit, operational=operational,
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
            if self.held is not None and msg.arbitration_id in self.hold_ids:
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
            self.feed(self.clock.now() + DT)
            self.emu.set_joint_position(self.position_m)
        elif self.duplicates_in_pause:
            self.feed(self.clock.now())
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

    def feed(self, t):
        event = self.coord.feed(t)
        if event:
            self.events.append(event)
        return event

    def reset_time(self, t=0.0):
        """Gazebo reset: /clock moves backward."""
        self.feed(t)

    def jump(self, seconds):
        """/clock jumps forward (missed /clock messages, a large step): no joint motion."""
        return self.feed(self.clock.now() + seconds)

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


# --- temporal discontinuities (senior review 17 Nov 2026) --------------------------------------
# What each emulator does is defined in docs/architecture/sil-topology.md section 6.3.

def move_frame(seq, stop):
    return can.Message(arbitration_id=c.RPDO1, data=bytes([seq, c.OP_MOVE_TO_STOP, stop]),
                       is_extended_id=False)


def permit_frame(r):
    payload = PERMIT.pack(PERMIT.permit_value, r.host.boot_id, r.host.permit_seq + 1, 0)
    return can.Message(arbitration_id=PERMIT.cob_id, data=payload, is_extended_id=False)


# (1) forward time jump

def test_forward_jump_expires_the_lift_permit_without_a_fault_and_nothing_restarts(lift):
    r = lift()
    r.start_move()
    assert r.jump(10.0) == 'jump'                          # /clock +10 s in one message
    assert 'jumped forward by 10.000 s' in r.coord.jump_report()
    r.step()
    assert not r.emu.drive_on and r.emu.permit_stops == 1  # the permit from before is stale
    assert r.emu.faults == 0 and r.emu.clock_resets == 0 and r.clock.generation == 0
    r.step()                       # the stand-in joint applies the zero demand one step later
    held = r.position_m
    r.run(1.0)                     # fresh permit frames, keep-alive of the old command only
    assert r.emu.permit_fresh()
    assert not r.emu.drive_on and r.position_m == pytest.approx(held, abs=1e-6)
    assert r.emu.faults == 0
    r.host.command(c.OP_MOVE_TO_STOP, 2)                   # motion needs a new command
    assert r.until(lambda: r.position_m > held + 0.003, 1.0)


def test_forward_jump_without_the_permit_gate_trips_f07_and_the_jump_is_reported(lift):
    """Device behaviour: 10 s without keep-alive is F07 (C3). It is not silent: the
    coordinator reports the jump next to the EMCY of the fault."""
    r = lift(permit=None)
    r.start_move()
    r.jump(10.0)
    r.step()
    assert r.emu.faults & (1 << (c.F07 - 1)) and c.F07 in r.emu.emcy_log
    assert not r.emu.drive_on
    assert r.events[-1] == 'jump' and r.coord.jumps == 1


def test_forward_jump_makes_wheel_feedback_stale_and_is_no_drive_power_cycle(drive):
    clock, emu, host, _raw = drive
    drive_moving(clock, emu, host)
    emu.set_wheel_feedback(2.0, 2.0)
    assert emu.feedback_fresh()
    clock.advance(10.0)
    assert not emu.feedback_fresh()                        # never fresh again by a jump
    time.sleep(0.05)
    assert emu.clock_resets == 0 and emu.clock_stalls == 0 and emu.enabled
    assert host.follow_clock() is None                     # the stand-in keeps its own twist


# (2) duplicate /clock messages

def test_duplicate_clock_messages_change_no_generation_and_stop_no_lift_move(lift):
    r = lift()
    r.start_move()
    for _ in range(200):                                   # 1 s, every time published twice
        r.feed(r.clock.now())
        r.step()
    assert r.clock.generation == 0 and r.emu.clock_resets == 0 and r.emu.clock_stalls == 0
    assert r.reports == [] and r.events == []
    assert r.emu.drive_on and r.emu.faults == 0


def test_a_paused_gazebo_republishing_its_time_is_still_a_stall(lift):
    r = lift()
    r.start_move()
    r.gz_paused = True
    r.duplicates_in_pause = True
    r.run(1.5)
    assert len(r.reports) == 1 and not r.emu.drive_on and not r.emu.permit_valid
    assert r.emu.faults == 0 and r.clock.generation == 0


# (4) first-clock startup

def test_before_the_first_clock_the_lift_node_accepts_no_command(make_bus):
    clock = FedClock()                                     # no /clock yet
    emu = GazeboCtl017(make_bus(), permit=PERMIT, operational=True, clock=clock)
    emu.attach_external_plant(0.0)
    with emu.lock:
        emu._handle(can.Message(arbitration_id=PERMIT.cob_id,
                                data=PERMIT.pack(PERMIT.permit_value, 7, 1, 0),
                                is_extended_id=False))
        emu._handle(move_frame(1, 2))
        for _ in range(400):                               # 2 s of host ticks
            emu.tick(DT)
    assert emu.state == c.INIT and not emu.drive_on        # no device timer advanced
    assert emu.cmd_result != c.R_OK and emu.faults == 0
    clock.update(5.0)                                      # first /clock
    with emu.lock:
        for _ in range(20):
            clock.update(clock.now() + DT)
            emu.tick(DT)
    assert emu.state != c.INIT and not emu.drive_on        # starts now, the command is gone


def test_host_stand_in_sends_nothing_before_the_first_clock(make_bus):
    clock = FedClock()
    raw = make_bus()
    drive_host = DriveHost(make_bus(), 1, 250, 0.100, 0.400, sdo_timeout_s=0.02,
                           clock=clock).start()
    lift_host = LiftHost(make_bus(), permit=PERMIT, clock=clock).start()
    try:
        time.sleep(0.3)
        assert frames(raw, 0x601) == [] and drive_host.sdo_errors == 0   # no drive bring-up
        assert raw.recv(timeout=0) is None                 # no frame at all
        clock.update(0.5)                                  # first /clock
        assert wait_for(lambda: drive_host.sdo_errors > 0, 1.0)   # bring-up tried now
    finally:
        drive_host.stop()
        lift_host.stop()


def test_before_the_first_clock_the_drive_sends_no_heartbeat_and_takes_no_enable(make_bus):
    clock = FedClock()
    raw = make_bus()
    emu = GazeboZlac8015d(make_bus(), 1, clock=clock).start()
    try:
        raw.send(can.Message(arbitration_id=0x601, data=bytes([0x2B, 0x40, 0x60, 0x00, 0x0F,
                                                                0x00, 0x00, 0x00]),
                             is_extended_id=False))       # SDO write 6040h = 0x000F
        time.sleep(0.3)
        heartbeats = [d for d in frames(raw, 0x701) if d != b'\x00']
        assert heartbeats == []                            # no heartbeat producer yet
        assert not emu.enabled                             # no command accepted
    finally:
        emu.stop()


def test_at_the_first_clock_the_drive_boots_then_heartbeats_and_takes_an_enable(make_bus):
    """#160: the first /clock powers the drive: boot-up frame first, then heartbeats, and
    an SDO enable sent after the first clock is accepted."""
    clock = FedClock()
    raw = make_bus()
    emu = GazeboZlac8015d(make_bus(), 1, clock=clock).start()
    try:
        time.sleep(0.1)
        assert frames(raw, 0x701) == []                    # nothing before the first clock
        clock.update(0.0)                                  # first /clock
        assert wait_for(lambda: emu.powered, 1.0)
        time.sleep(0.35)
        hb = frames(raw, 0x701)
        assert hb and hb[0] == b'\x00' and len(hb) >= 3   # boot-up, then heartbeats
        sdo = canopen_common.SdoClient(raw, 1)
        for word in (0x06, 0x07, 0x0F):
            sdo.write(0x6040, 0, word, 2)
        assert emu.enabled
        assert emu.pre_clock_drops == 0
    finally:
        emu.stop()


# (5) clock loss and recovery

def test_clock_loss_with_a_lift_command_buffered_before_the_stall_moves_nothing(lift):
    r = lift()
    assert r.ready()
    r.host.set_permit(True)
    assert r.until(r.emu.permit_fresh, 0.5)
    r.gz_paused = True                                     # /clock stops: no message at all
    r.run(0.1)
    r.host.command(c.OP_MOVE_TO_STOP, 2)                   # accepted, Gazebo frozen
    r.run(0.05)
    assert r.emu.drive_on
    start = r.position_m
    r.run(1.5)
    assert len(r.reports) == 1 and not r.emu.drive_on      # stall reported, motion dropped
    r.gz_paused = False
    r.run(1.0)
    assert r.host.cmd is None and not r.emu.drive_on and r.emu.target_stop == 0xFF
    assert r.position_m == pytest.approx(start, abs=1e-6) and r.emu.faults == 0


def test_clock_loss_through_the_coordinator_drops_the_drive_twist_and_rearms_at_zero(make_bus):
    wall = ManualClock()
    clock = FedClock()
    coord = ClockCoordinator(clock, steady=wall)
    coord.feed(10.0)
    emu = GazeboZlac8015d(make_bus(), 1, clock=clock).start()
    host = DriveHost(make_bus(), 1, 250, 0.100, 0.400, clock=clock)

    def run(n):
        for _ in range(n):
            wall.advance(0.02)
            coord.feed(clock.now() + 0.02)
            host.follow_clock()
            host.step()
    try:
        host.set_twist(0.2, 0.0)
        run(10)
        assert emu.enabled and emu.state()['target_rpm'] == [19, 19]
        wall.advance(1.5)                                  # /clock lost
        assert coord.check().startswith('simulation stalled')
        assert wait_for(lambda: emu.clock_stalls == 1, 1.0)
        assert not emu.enabled and emu.state()['target_rpm'] == [0, 0]
        run(10)                                            # /clock returns
        assert host.clock_events == 1 and host.target_v == 0.0
        assert emu.enabled and emu.state()['target_rpm'] == [0, 0]
        assert emu.demand_rad_s() == (0.0, 0.0)
    finally:
        emu.stop()


# (6) frames queued before a reset

def test_frames_queued_before_a_reset_meet_the_power_cycled_lift_node(lift):
    r = lift(position_m=0.0)                               # at L1000: homed after the reset
    assert r.ready()
    r.host.set_permit(True)
    assert r.until(r.emu.permit_fresh, 0.5)
    # In the node's receive queue at the reset: NMT start, a permit and a MOVE command.
    r.raw.send(can.Message(arbitration_id=canopen_common.NMT_ID, data=bytes([0x01, c.NODE_ID]),
                           is_extended_id=False))
    r.raw.send(permit_frame(r))
    r.raw.send(move_frame(0x55, 2))
    r.reset_time(0.0)
    r.step()
    assert r.emu.clock_resets == 1 and not r.emu.drive_on
    assert r.emu.nmt_state == canopen_common.NMT_PRE_OPERATIONAL   # power cycle, frames ignored
    assert r.emu.state in (c.INIT, c.IDLE) and r.emu.target_stop == 0xFF
    r.run(1.5)
    assert r.emu.homed and r.emu.permit_fresh()
    assert not r.emu.drive_on and r.position_m == pytest.approx(0.0, abs=1e-6)


@pytest.mark.xfail(strict=True, reason='#159: no generation on the wire, a lift command delayed across a reset is executed')  # tracked: #159
def test_a_lift_command_delayed_across_a_reset_is_rejected(lift):
    r = lift(position_m=0.0)
    assert r.ready()
    r.host.set_permit(True)
    assert r.until(r.emu.permit_fresh, 0.5)
    r.held = []
    r.hold_ids = (c.RPDO1,)
    r.raw.send(move_frame(0x55, 2))                        # sent before the reset ...
    r.step()
    assert len(r.held) == 1
    r.reset_time(0.0)
    r.run(1.5)                                             # node operational, homed, permit
    assert r.emu.homed and r.emu.permit_fresh()
    r.release_held()                                       # ... delivered after it
    r.run(0.5)
    assert not r.emu.drive_on and r.position_m == pytest.approx(0.0, abs=1e-6)


@pytest.mark.xfail(strict=True, reason='#159: ZLAC8015D processes frames queued before a reset after its power cycle')  # tracked: #159
def test_drive_frames_queued_before_a_reset_are_dropped(make_bus):
    clock = ManualClock(10.0)
    raw = make_bus()
    emu = GazeboZlac8015d(make_bus(), 1, clock=clock)      # receive queue fills, not stepped
    for frame in (bytes([0x23, 0xFF, 0x60, 0x01, 19, 0, 0, 0]),       # 60FFh:01 = 19 rpm
                  bytes([0x2B, 0x40, 0x60, 0x00, 0x0F, 0, 0, 0])):    # 6040h = 0x000F
        raw.send(can.Message(arbitration_id=0x601, data=frame, is_extended_id=False))
    clock.reset(0.0)
    emu.start()
    try:
        assert wait_for(lambda: emu.clock_resets == 1, 1.0)
        time.sleep(0.1)
        assert not emu.enabled and emu.state()['target_rpm'] == [0, 0]
    finally:
        emu.stop()
