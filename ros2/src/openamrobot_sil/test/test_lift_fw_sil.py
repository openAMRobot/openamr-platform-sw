# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Lift firmware-SIL path and clock/reset/stale-frame contract (P0-C, P0-D).

Path under test: host stand-in (the base controller CAN3 role) -> RPDO1 and permit frames on a
CAN bus -> CTL-017 emulator (vendored openamr-platform-fw model, permit gate in GazeboCtl017) ->
motor speed demand -> stand-in lift joint -> joint position -> Hall count and stop sensors ->
TPDO1/TPDO2 -> host stand-in status. The firmware path is exercised end to end; nothing sets the
plant directly. The lock-step harness of test_clock_gates.py stands in for Gazebo.

Each test names the rule of docs/architecture/clock-reset-contract.md (Proposed) it checks.
Development evidence only: no Gazebo, no Renode, no firmware, no hardware. Nothing here is
safety evidence.
"""
import time

import can
import pytest

from conftest import collect
from openamrobot_sil.can_sil import canopen_common, ctl017_emulator as c
from openamrobot_sil.ctl017_gazebo import GazeboCtl017
from openamrobot_sil.sim_time import FedClock
from openamrobot_sil.stm32_host import LiftHost
from test_clock_gates import DT, PERMIT, LockStepLift, frames, move_frame, permit_frame

BOOT_UP = canopen_common.HB_BASE + c.NODE_ID
STOP_TOL_MM = 2.0      # 0x2101:03 stop_tol_um default of the v1 dictionary
STOP_JOINT_M = (0.0, 0.175, 0.350)


def nmt_start():
    return can.Message(arbitration_id=canopen_common.NMT_ID, data=bytes([0x01, c.NODE_ID]),
                       is_extended_id=False)


def ready_with_permit(r):
    assert r.ready()
    r.host.set_permit(True)
    assert r.until(r.emu.permit_fresh, 0.5)


class BridgeLift(LockStepLift):
    """Adds a ros_gz_bridge outage: Gazebo keeps running, but neither /clock nor the joint
    state reaches the emulator process, and the lift demand does not reach Gazebo, so the Gazebo
    joint controller keeps the last demand (gz-sim JointController has no command timeout)."""

    def __init__(self, make_bus, **kw):
        super().__init__(make_bus, **kw)
        self.bridge_down = False
        self.gz_time = self.clock.now()
        self.gz_demand = 0.0

    def step(self, wall_sleep=None):
        if not self.bridge_down:
            super().step(wall_sleep)
            self.gz_time = self.clock.now()
            self.gz_demand = self.emu.demand_velocity_m_s()
            return
        self.wall.advance(DT)
        self.gz_time += DT
        self.position_m = min(0.350, max(0.0, self.position_m + self.gz_demand * DT))
        report = self.coord.check()
        if report:
            self.reports.append(report)
        self.pump()
        with self.emu.lock:
            self.emu.tick(DT)        # the emulator thread keeps running on a frozen clock
        if not self.clock.paused:
            self.host.follow_clock()
            self.host.step()

    def reconnect(self):
        self.bridge_down = False
        self.feed(self.gz_time)
        self.emu.set_joint_position(self.position_m)


@pytest.fixture
def lift(make_bus):
    return lambda **kw: LockStepLift(make_bus, **kw)


# --- R1 first clock: nothing before it (ties to #160) ---------------------------------------

def test_lift_node_thread_sends_no_frame_before_the_first_clock(make_bus):
    """R1: no boot-up, no heartbeat before the first /clock; boot-up at the first clock."""
    clock = FedClock()
    raw = make_bus()
    emu = GazeboCtl017(make_bus(), permit=PERMIT, clock=clock)
    emu.attach_external_plant(0.0)
    emu.start()
    try:
        assert collect(raw, 0.3) == []                      # 0.3 s host time, no /clock yet
        clock.update(0.0)                                   # first /clock (Gazebo starts at 0)
        for _ in range(60):
            clock.update(clock.now() + DT)
            time.sleep(DT)
        got = collect(raw, 0.1)
        boot = [bytes(m.data) for m in got if m.arbitration_id == BOOT_UP]
        assert boot and boot[0] == b'\x00'                 # boot-up first, then heartbeats
    finally:
        emu.stop()


def test_frames_received_before_the_first_clock_are_not_kept(make_bus):
    """R1: an NMT start and a permit received before the first /clock do not act after it."""
    clock = FedClock()
    emu = GazeboCtl017(make_bus(), permit=PERMIT, clock=clock)
    emu.attach_external_plant(0.0)                          # at L1000: homed at power-up
    with emu.lock:
        emu._handle(nmt_start())
        emu._handle(can.Message(arbitration_id=PERMIT.cob_id,
                                data=PERMIT.pack(PERMIT.permit_value, 7, 1, 0),
                                is_extended_id=False))
        clock.update(0.0)                                   # first /clock
        for _ in range(20):                                 # 0.1 s simulation time
            clock.update(clock.now() + DT)
            emu.tick(DT)
        emu._handle(move_frame(1, 2))
        for _ in range(20):
            clock.update(clock.now() + DT)
            emu.tick(DT)
    assert emu.nmt_state == canopen_common.NMT_PRE_OPERATIONAL
    assert not emu.permit_valid and not emu.drive_on and not emu.any_cmd


# --- R2 permit absent, then present ------------------------------------------------------------

def test_permit_absent_refuses_present_allows_and_the_refused_command_is_not_replayed(lift):
    r = lift()
    assert r.ready()
    r.host.command(c.OP_MOVE_TO_STOP, 1)                    # no permit yet (false at boot)
    r.run(1.0)
    assert r.emu.cmd_result == PERMIT.refused_result and r.emu.permit_refusals == 1
    assert not r.emu.drive_on and r.position_m == pytest.approx(0.0, abs=1e-6)
    r.host.set_permit(True)
    assert r.until(r.emu.permit_fresh, 2 * PERMIT.timeout_ms / 1000.0)
    r.run(1.0)                                              # keep-alive of the refused command
    assert not r.emu.drive_on and r.position_m == pytest.approx(0.0, abs=1e-6)
    r.host.command(c.OP_MOVE_TO_STOP, 1)                    # a new command
    assert r.until(lambda: r.emu.drive_on, 0.1)
    assert r.emu.faults == 0


# --- R3 motion to a target with feedback within tolerance -------------------------------------

@pytest.mark.parametrize('stop', [1, 2])
def test_move_reaches_the_stop_and_can_feedback_agrees_within_tolerance(lift, stop):
    r = lift()
    ready_with_permit(r)
    t0 = r.clock.now()
    r.host.command(c.OP_MOVE_TO_STOP, stop)
    assert r.until(lambda: r.emu.state == c.IDLE and r.emu._active_stop_id(
        r.emu._now_ms()) == stop, 25.0)
    travel_s = r.clock.now() - t0
    nominal_s = STOP_JOINT_M[stop] / 0.018                  # 18 mm/s no-load (P-03 item 16)
    assert nominal_s - 0.2 <= travel_s <= nominal_s + 0.6
    r.run(0.3)                                              # TPDO1/TPDO2 of the rest state
    status = r.host.state()
    assert status['active_stop'] == stop and status['cmd_result'] == c.R_OK
    assert status['faults'] == 0 and status['velocity_mm_s'] == 0.0
    stop_mm = c.STOP_POS_MM[stop]
    assert abs(status['position_mm'] - stop_mm) <= STOP_TOL_MM       # what the host reads
    assert abs(r.emu.true_mm - stop_mm) <= STOP_TOL_MM               # where the joint is
    assert r.position_m == pytest.approx(STOP_JOINT_M[stop], abs=STOP_TOL_MM / 1000.0)


@pytest.mark.parametrize('lag_steps', [0, 4, 10])
def test_joint_feedback_at_100_hz_with_transport_lag_raises_no_f02_and_arrives(lift, lag_steps):
    """F02 timing: joint states arrive at 100 Hz (every second 5 ms step) and late by up to
    50 ms; the Hall rate window and the sensor cross-check must not raise F02 or F03."""
    r = lift()
    ready_with_permit(r)
    history = []
    real_set = r.emu.set_joint_position
    r.emu.set_joint_position = lambda p: history.append(p)
    n = [0]

    def deliver():
        n[0] += 1
        if n[0] % 2 == 0 and len(history) > lag_steps:
            real_set(history[-1 - lag_steps])
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    for _ in range(int(25.0 / DT)):
        r.step()
        deliver()
        if r.emu.state == c.IDLE and r.emu._active_stop_id(r.emu._now_ms()) == 2:
            break
    assert r.emu.faults == 0, r.emu.emcy_log
    assert abs(r.emu.true_mm - c.STOP_POS_MM[2]) <= STOP_TOL_MM


# --- R4 cancel mid-motion ---------------------------------------------------------------------

def test_stop_command_mid_motion_halts_drops_the_target_and_nothing_restarts(lift):
    r = lift()
    ready_with_permit(r)
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    r.run(2.0)
    assert r.emu.drive_on and 0.02 < r.position_m < 0.05
    r.host.command(c.OP_STOP)
    r.step()
    assert not r.emu.drive_on and r.emu.target_stop == 0xFF and r.emu.cmd_result == c.R_OK
    r.run(1.0)                                              # keep-alive of STOP only
    held = r.position_m
    r.run(1.0)
    assert r.emu.state == c.IDLE and r.position_m == pytest.approx(held, abs=1e-6)
    assert r.emu.faults == 0 and r.host.state()['target_stop'] == 0xFF
    r.host.command(c.OP_MOVE_TO_STOP, 1)                    # a new command moves again
    assert r.until(lambda: r.emu._active_stop_id(r.emu._now_ms()) == 1, 15.0)
    assert r.emu.faults == 0


# --- R5 reset: new generation, frames from before it do not act -------------------------------

def test_reset_is_a_new_generation_and_frames_queued_before_it_do_not_act(lift):
    """R5: the receive queue is lost with the power cycle. The order permit, NMT start, MOVE
    would otherwise start the node in the new generation from a frame of the old one."""
    r = lift(position_m=0.0)
    ready_with_permit(r)
    gen = r.clock.generation
    frames(r.raw, BOOT_UP)
    r.raw.send(permit_frame(r))
    r.raw.send(nmt_start())
    r.raw.send(move_frame(0x55, 2))
    r.reset_time(0.0)
    r.step()
    assert r.clock.generation == gen + 1 and r.events[-1] == 'reset'
    assert r.emu.clock_resets == 1 and b'\x00' in frames(r.raw, BOOT_UP)
    assert r.emu.nmt_state == canopen_common.NMT_PRE_OPERATIONAL
    assert not r.emu.any_cmd and not r.emu.permit_valid and not r.emu.drive_on
    r.run(1.5)                                              # host starts the node again
    assert r.emu.nmt_state == canopen_common.NMT_OPERATIONAL and r.emu.permit_fresh()
    assert not r.emu.drive_on and r.position_m == pytest.approx(0.0, abs=1e-6)


def test_a_permit_frame_delayed_across_a_reset_does_not_validate_the_permit(lift):
    """R5: the base-controller stand-in reboots with the reset (new permit boot id), so a
    permit frame of the old boot that arrives late is not trusted by the gate (rule P6)."""
    r = lift(position_m=0.0)
    ready_with_permit(r)
    r.held = []
    r.run(0.06)                                             # one or more permit frames held
    old = r.held
    r.held = None
    assert old
    r.reset_time(0.0)
    r.run(1.5)
    assert r.emu.permit_fresh()
    with r.emu.lock:
        r.emu._handle(old[-1])                              # the old frame arrives now
    assert not r.emu.permit_valid
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    r.step()
    assert r.emu.cmd_result == PERMIT.refused_result and not r.emu.drive_on


# --- R6 stale frames: freshness by receipt time ------------------------------------------------

def test_permit_age_boundary_on_simulation_time(lift):
    """R6: a command is accepted with a permit received 190 ms ago and the lift halts once the
    age reaches the 200 ms SIL freshness time; past it a command is refused."""
    r = lift()
    ready_with_permit(r)
    r.host.permit_silent = True
    rx = r.emu.permit_rx_ms
    while r.emu._now_ms() - rx < 185:
        r.step()
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    r.step()
    assert r.emu.cmd_result == c.R_OK and r.emu.drive_on
    assert r.until(lambda: not r.emu.drive_on, 0.05)
    assert 200 <= r.emu._now_ms() - rx <= 205
    assert r.emu.permit_stops == 1 and r.emu.faults == 0
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    r.step()
    assert r.emu.cmd_result == PERMIT.refused_result


def test_stale_keep_alive_without_the_permit_gate_is_f07_at_the_deadline(lift):
    """R6, v1 baseline: F07 when the RPDO1 keep-alive (0x1400:05) or the core heartbeat
    (0x1016:01) is older than 300 ms, whichever expires first, on simulation time."""
    r = lift(permit=None)
    assert r.ready()
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    r.run(0.5)
    r.host.silent = True                                    # no keep-alive, no heartbeat
    r.step()                                                # the last frame sent is handled
    last = min(r.emu.last_keepalive_ms, r.emu.hbc_last_ms)
    assert r.until(lambda: r.emu.faults & (1 << (c.F07 - 1)), 1.0)
    assert 300 <= r.emu._now_ms() - last <= 305


# --- R7 repeated and delayed /clock -----------------------------------------------------------

@pytest.mark.parametrize('gap_s,stall', [(0.95, False), (1.05, True)])
def test_delayed_clock_is_a_stall_only_above_the_bound(lift, gap_s, stall):
    r = lift()
    ready_with_permit(r)
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    r.run(1.0)
    r.gz_paused = True                                      # /clock delayed on steady time
    r.run(gap_s)
    r.gz_paused = False
    r.run(0.5)
    assert len(r.reports) == (1 if stall else 0)
    assert r.emu.drive_on is (not stall) and r.emu.faults == 0
    assert r.clock.generation == 0 and r.coord.jumps == 0


def test_every_clock_time_repeated_counts_no_extra_motion_time(lift):
    r = lift()
    ready_with_permit(r)
    r.host.command(c.OP_MOVE_TO_STOP, 1)
    t0 = r.clock.now()
    for _ in range(400):                                    # 2 s, each time published twice
        r.step()
        r.feed(r.clock.now())
    assert r.clock.now() - t0 == pytest.approx(2.0, abs=1e-6)
    assert r.position_m == pytest.approx(0.036, abs=0.002)  # 2 s at 18 mm/s
    assert r.emu.drive_on and r.emu.faults == 0 and r.events == []


# --- R8 forward jump ---------------------------------------------------------------------------

def test_forward_step_below_the_jump_bound_still_expires_the_permit(lift):
    """R8: a 0.25 s step is no reported jump, yet the 200 ms permit expires in it: halt, no
    fault, no new generation, nothing restarts."""
    r = lift()
    ready_with_permit(r)
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    r.run(1.0)
    assert r.jump(0.25) is None and r.coord.jumps == 0
    r.step()
    assert not r.emu.drive_on and r.emu.permit_stops == 1 and r.emu.faults == 0
    assert r.clock.generation == 0 and r.emu.clock_resets == 0
    r.step()
    held = r.position_m
    r.run(1.0)
    assert r.emu.permit_fresh() and r.position_m == pytest.approx(held, abs=1e-6)


# --- R10 reconnection --------------------------------------------------------------------------

@pytest.mark.parametrize('outage_s', [0.5, 3.0])
def test_reconnection_after_a_bridge_restart_stops_and_feedback_stays_consistent(
        make_bus, outage_s):
    """R10: ros_gz_bridge restarts while the lift moves. Gazebo keeps running on the last
    demand (gap G4). After the reconnection the lift is halted, the Hall position agrees with
    the joint, and motion needs a new command. Below the stall bound the permit expiry halts the
    lift without a fault. Above it the stall stops the drive, and the motion Gazebo made on the
    old demand during the outage is uncommanded motion after a stop: F08, the device reaction."""
    r = BridgeLift(make_bus)
    ready_with_permit(r)
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    r.run(1.0)
    r.bridge_down = True
    r.run(outage_s)
    stalled = outage_s > 1.0
    assert len(r.reports) == (1 if stalled else 0)
    r.reconnect()
    if stalled:
        assert r.events[-1] == 'resumed' and r.coord.jumps == 1
    r.step()
    r.step()
    assert not r.emu.drive_on and r.emu.target_stop == 0xFF
    assert r.emu.faults == ((1 << (c.F08 - 1)) if stalled else 0)
    assert r.emu.position_valid
    assert abs(r.emu.position_mm() - r.emu.true_mm) <= STOP_TOL_MM
    held = r.position_m
    r.run(1.0)
    assert r.position_m == pytest.approx(held, abs=1e-6) and r.emu.permit_fresh()
    if stalled:
        r.host.command(c.OP_CLEAR_FAULT)
        r.step()
        assert r.emu.faults == 0
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    assert r.until(lambda: r.emu._active_stop_id(r.emu._now_ms()) == 2, 25.0)
    assert r.emu.faults == 0 and abs(r.emu.true_mm - c.STOP_POS_MM[2]) <= STOP_TOL_MM


def restart_host_while_moving(r):
    r.host.command(c.OP_MOVE_TO_STOP, 2)
    r.run(1.0)
    old_boot = r.host.boot_id
    r.host.silent = True                                    # process gone
    r.run(0.5)
    assert not r.emu.drive_on and r.emu.permit_stops == 1 and r.emu.faults == 0
    r.host.bus.shutdown()
    r.host = LiftHost(r.make_bus(), permit=PERMIT, clock=r.clock)
    if r.host.boot_id == old_boot:
        r.host.boot_id = old_boot ^ 1
    r.host.set_permit(True)
    held = r.position_m
    r.run(1.0)
    assert r.emu.permit_fresh() and r.host.cmd is None
    assert not r.emu.drive_on and r.position_m == pytest.approx(held, abs=1e-6)


def test_reconnection_after_a_host_stand_in_restart_needs_a_new_command(lift):
    """R10: the base-controller stand-in process restarts while the lift moves: the permit
    expires (halt, no fault), the new process has a new boot id and no command to replay."""
    r = lift()
    ready_with_permit(r)
    restart_host_while_moving(r)
    r.host.command(c.OP_MOVE_TO_STOP, 1)                    # seq 1 again, another target
    r.step()
    assert r.emu.cmd_result == c.R_SEQ_CONFLICT and not r.emu.drive_on   # refused, fail-safe
    r.host.command(c.OP_MOVE_TO_STOP, 1)                    # seq 2
    assert r.until(lambda: r.emu.drive_on, 0.1)


@pytest.mark.xfail(strict=True, reason='#159: RPDO1 has no producer boot id, the first command of a restarted producer with the last seq, op and stop is taken as a keep-alive')  # tracked: #159
def test_the_first_command_of_a_restarted_host_is_executed_even_with_the_last_seq(lift):
    r = lift()
    ready_with_permit(r)
    restart_host_while_moving(r)
    r.host.command(c.OP_MOVE_TO_STOP, 2)                    # seq 1, MOVE, 2: as before
    assert r.until(lambda: r.emu.drive_on, 0.1)


# --- the contract page names only tests that exist --------------------------------------------

def test_every_test_the_contract_names_exists():
    import os
    import re
    from conftest import HERE, REPO_ROOT
    doc = open(os.path.join(REPO_ROOT, 'docs', 'architecture', 'clock-reset-contract.md')).read()
    files = {'fw_sil': 'test_lift_fw_sil.py', 'gates': 'test_clock_gates.py',
             'sim_time': 'test_sim_time.py', 'node_clock': 'test_node_clock.py'}
    named = re.findall(r'`(fw_sil|gates|sim_time|node_clock)::(test_\w+)`', doc)
    assert len(named) >= 30
    for prefix, name in named:
        source = open(os.path.join(HERE, files[prefix])).read()
        assert re.search(r'^def %s\(' % name, source, re.M), '%s::%s' % (prefix, name)
