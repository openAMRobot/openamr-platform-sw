import struct
import time

import can
import pytest

from canopen_common import SdoClient
import ctl017_emulator as c
from conftest import collect, drain, wait_for


class Core:
    """Stand-in for the core: sends commands, core heartbeat and keep-alive, decodes TPDO1."""

    def __init__(self, bus):
        self.bus = bus
        self.seq = 0
        self.cmd = None
        self.last_sent = 0.0
        self.hb = True
        self.keepalive = True
        self.status = None
        self.emcy = []

    def send_cmd(self, op, stop=0, seq=None):
        if seq is None:
            self.seq = (self.seq + 1) & 0xFF
            seq = self.seq
        self.cmd = (seq, op, stop)
        self._tx_cmd()

    def _tx_cmd(self):
        self.bus.send(can.Message(arbitration_id=c.RPDO1, data=bytes(self.cmd), is_extended_id=False))

    def pump(self, seconds):
        end = time.monotonic() + seconds
        last_hb = 0.0
        last_ka = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if self.hb and now - last_hb >= 0.1:
                self.bus.send(can.Message(arbitration_id=0x701, data=[0x05], is_extended_id=False))
                last_hb = now
            if self.keepalive and self.cmd and now - last_ka >= 0.1:
                self._tx_cmd()
                last_ka = now
            m = self.bus.recv(timeout=0.01)
            while m is not None:
                if m.arbitration_id == c.TPDO1:
                    self.status = self.decode(m.data)
                elif m.arbitration_id == c.EMCY:
                    self.emcy.append(m.data[3])
                m = self.bus.recv(timeout=0)

    @staticmethod
    def decode(d):
        state, flags, pos, stop, last_seq, res, counter = struct.unpack("<BBHBBBB", bytes(d))
        return dict(state=state, flags=flags, pos=pos, stop=stop, last_seq=last_seq,
                    result=res, counter=counter)

    def until(self, cond, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump(0.05)
            if self.status and cond(self.status):
                return True
        return False


@pytest.fixture
def make_node(make_bus):
    nodes = []

    def factory(start=None, operational=True):
        n = c.Ctl017Emulator(make_bus(), start_pos_mm=start, operational=operational).start()
        nodes.append(n)
        return n
    yield factory
    for n in nodes:
        n.stop()


@pytest.fixture
def core(master):
    return Core(master)


def nmt_start(bus, node=c.NODE_ID):
    bus.send(can.Message(arbitration_id=0, data=[0x01, node], is_extended_id=False))


def homed_node(make_node, core, start=30.0):
    node = make_node(start=start)  # near the lower end so homing is quick (9 mm/s homing speed)
    core.pump(0.2)
    core.send_cmd(c.OP_START_HOMING)
    assert core.until(lambda s: s["state"] == c.IDLE, 25.0)
    return node


def test_heartbeat_and_initial_status(make_node, core):
    make_node()
    drain(core.bus)
    frames = collect(core.bus, 0.45)
    assert len([f for f in frames if f.arbitration_id == 0x710]) >= 4
    core.pump(0.2)
    s = core.status
    assert s["state"] == c.UNHOMED
    assert s["result"] == 0xFF and s["last_seq"] == 0
    assert s["pos"] == 0xFFFF
    assert s["flags"] & c.F_RECOVERY and not s["flags"] & c.F_POSITION_VALID
    assert s["flags"] & c.F_MOTOR_POWER


def test_status_counter_increments_by_one(make_node, core):
    make_node()
    drain(core.bus)
    counters = []
    end = time.monotonic() + 0.8
    while time.monotonic() < end:
        m = core.bus.recv(timeout=0.2)
        if m is not None and m.arbitration_id == c.TPDO1:
            counters.append(m.data[7])
    assert len(counters) >= 6
    assert all((b - a) & 0xFF == 1 for a, b in zip(counters, counters[1:]))


def test_move_rejected_before_homing(make_node, core):
    make_node()
    core.pump(0.2)
    core.send_cmd(c.OP_MOVE_TO_STOP, 1)
    core.pump(0.3)
    assert core.status["result"] == c.R_RECOVERY
    assert core.status["state"] == c.UNHOMED


def test_homing_then_moves_to_each_stop(make_node, core):
    node = homed_node(make_node, core)
    assert core.status["flags"] & c.F_POSITION_VALID
    assert not core.status["flags"] & c.F_RECOVERY
    for stop in (1, 2, 0):
        core.send_cmd(c.OP_MOVE_TO_STOP, stop)
        assert core.until(lambda s: s["state"] == c.MOVING, 2.0)
        assert not core.status["flags"] & c.F_AT_STOP
        assert core.status["stop"] == 0xFF
        assert core.until(lambda s: s["state"] == c.IDLE and s["stop"] == stop, 30.0)
        assert core.status["flags"] & c.F_AT_STOP
        assert abs(node.counted - c.STOP_POS_MM[stop]) < c.MM_PER_HALL * 2
        assert abs(node.true_mm - c.SENSOR_POS_MM[stop]) <= c.SENSOR_WINDOW_MM + c.MM_PER_HALL


def test_motor_model_speed(make_node, core):
    node = homed_node(make_node, core)
    core.send_cmd(c.OP_MOVE_TO_STOP, 1)
    t0 = time.monotonic()
    p0 = node.counted
    core.pump(2.0)
    rate = (node.counted - p0) / (time.monotonic() - t0)
    assert 15.0 < rate < 20.0  # 18 mm/s
    core.send_cmd(c.OP_STOP)
    assert core.until(lambda s: s["state"] == c.IDLE, 2.0)


def test_stop_sensors_and_head_light_getac(make_node, core):
    node = make_node(start=c.STOP_POS_MM[1])
    core.pump(0.2)
    assert core.status["state"] == c.IDLE  # powered at a sensor: valid without homing
    assert node.sensors() == 0b010
    core.bus.send(can.Message(arbitration_id=c.RPDO2, data=[1, 60, 1], is_extended_id=False))
    assert wait_for(lambda: node.getac_enable == 1 and node.head_light_on == 1
                    and node.head_light_dim == 60)


def test_repeated_command_is_keepalive_and_conflict_rejected(make_node, core):
    homed_node(make_node, core)
    core.send_cmd(c.OP_MOVE_TO_STOP, 2)
    core.pump(0.5)  # keep-alives with the same seq
    assert core.status["state"] == c.MOVING and core.status["result"] == c.R_OK
    core.send_cmd(c.OP_MOVE_TO_STOP, 0, seq=core.seq)  # same seq, different content
    core.cmd = (core.seq, c.OP_MOVE_TO_STOP, 2)  # keep-alive keeps the original
    core.pump(0.3)
    assert core.status["result"] == c.R_SEQ_CONFLICT
    assert core.status["state"] == c.MOVING
    core.send_cmd(c.OP_STOP)
    assert core.until(lambda s: s["state"] == c.IDLE, 2.0)


def test_busy_and_bad_stop_and_bad_op(make_node, core):
    homed_node(make_node, core)
    core.send_cmd(c.OP_MOVE_TO_STOP, 3)
    core.pump(0.3)
    assert core.status["result"] == c.R_BAD_STOP
    core.send_cmd(9)
    core.pump(0.3)
    assert core.status["result"] == c.R_BAD_OP
    core.send_cmd(c.OP_MOVE_TO_STOP, 2)
    core.pump(0.3)
    core.send_cmd(c.OP_MOVE_TO_STOP, 1)
    core.pump(0.3)
    assert core.status["result"] == c.R_BUSY
    core.send_cmd(c.OP_STOP)
    assert core.until(lambda s: s["state"] == c.IDLE, 2.0)


def test_move_to_current_stop_is_noop(make_node, core):
    make_node(start=c.STOP_POS_MM[0])
    core.pump(0.2)
    core.send_cmd(c.OP_MOVE_TO_STOP, 0)
    core.pump(0.3)
    assert core.status["result"] == c.R_OK and core.status["state"] == c.IDLE


def test_keepalive_loss_raises_f07_and_stops(make_node, core):
    node = homed_node(make_node, core)
    core.send_cmd(c.OP_MOVE_TO_STOP, 2)
    core.pump(0.4)
    core.keepalive = False
    assert core.until(lambda s: s["state"] == c.FAULT, 1.5)
    assert c.F07 in core.emcy
    assert node.target_stop == 0xFF
    pos = node.counted
    core.pump(0.3)
    assert node.counted == pos  # no resume
    assert wait_for(lambda: node.od.get(0x2002, 0) == 1 << (c.F07 - 1))


def test_core_heartbeat_loss_raises_f07(make_node, core):
    homed_node(make_node, core)
    core.send_cmd(c.OP_MOVE_TO_STOP, 2)
    core.pump(0.4)
    core.hb = False  # D12 alternative (a): either loss stops the node
    assert core.until(lambda s: s["state"] == c.FAULT, 1.5)
    assert c.F07 in core.emcy


def test_clear_fault_then_no_motion_until_commanded(make_node, core):
    node = homed_node(make_node, core)
    core.send_cmd(c.OP_MOVE_TO_STOP, 2)
    core.pump(0.4)
    core.keepalive = False
    assert core.until(lambda s: s["state"] == c.FAULT, 1.5)
    core.keepalive = True
    core.send_cmd(c.OP_MOVE_TO_STOP, 1)
    core.pump(0.2)
    assert core.status["result"] == c.R_FAULT
    core.send_cmd(c.OP_CLEAR_FAULT)
    assert core.until(lambda s: s["state"] == c.IDLE, 1.0)
    assert not core.status["flags"] & c.F_FAULT
    pos = node.counted
    core.pump(0.3)
    assert node.counted == pos  # clearing never moves the lift


def test_stall_raises_f03(make_node, core):
    node = make_node(start=c.STOP_POS_MM[0])
    core.pump(0.2)
    node.inject_stall()  # no Hall pulses while driving, away from any sensor
    core.send_cmd(c.OP_MOVE_TO_STOP, 1)
    assert core.until(lambda s: s["state"] == c.FAULT, 3.0)
    assert c.F03 in core.emcy


def test_hall_slip_detected_at_sensor_f02(make_node, core):
    node = homed_node(make_node, core)
    node.inject_hall_slip(-6.0)  # count is 6 mm short of the true position
    core.send_cmd(c.OP_MOVE_TO_STOP, 2)
    assert core.until(lambda s: s["state"] == c.FAULT, 40.0)
    assert c.F02 in core.emcy
    assert core.status["flags"] & c.F_RECOVERY
    assert not core.status["flags"] & c.F_POSITION_VALID


def test_two_sensors_active_is_fault(make_node, core):
    node = homed_node(make_node, core)
    node.forced_sensors = 0b101
    assert core.until(lambda s: s["state"] == c.FAULT, 1.0)
    assert c.F09 in core.emcy  # #15: F09 is the two-sensors fault
    assert core.status["flags"] & c.F_RECOVERY


def test_motor_power_absent_rejects_homing(make_node, core):
    node = make_node()
    node.set_motor_power(False)
    core.pump(0.3)
    assert not core.status["flags"] & c.F_MOTOR_POWER
    core.send_cmd(c.OP_START_HOMING)
    core.pump(0.3)
    assert core.status["result"] == c.R_NO_MOTOR_POWER
    assert core.status["state"] == c.UNHOMED


def test_motor_power_loss_during_move_halts_without_fault(make_node, core):
    node = homed_node(make_node, core)
    core.send_cmd(c.OP_MOVE_TO_STOP, 2)
    core.pump(0.5)
    node.set_motor_power(False)
    assert core.until(lambda s: s["state"] == c.IDLE, 2.0)
    assert not core.status["flags"] & c.F_FAULT
    assert core.status["flags"] & c.F_POSITION_VALID  # count tracked through E-stop (D4)


def test_node_power_cycle_requires_rehoming(make_node, core):
    node = homed_node(make_node, core)
    core.cmd = None  # a core must stop its keep-alive when the node restarts
    node.power_cycle(start_pos_mm=100.0)
    core.status = None
    core.pump(0.3)
    assert core.status is None  # PRE-OPERATIONAL: no TPDO until the core sends NMT start
    nmt_start(core.bus)
    core.pump(0.3)
    assert core.status["state"] == c.UNHOMED
    assert core.status["result"] == 0xFF
    assert core.status["flags"] & c.F_RECOVERY


def test_sdo_hall_count_and_bus_off(master, make_node, core):
    node = make_node(start=c.STOP_POS_MM[2])
    sdo = SdoClient(master, c.NODE_ID)
    assert sdo.read(0x2300, 1, signed=True) == node.hall_count
    drain(master)
    node.set_bus_off(True)
    time.sleep(0.05)
    drain(master)
    assert not [f for f in collect(master, 0.35) if f.arbitration_id in (0x710, c.TPDO1)]
