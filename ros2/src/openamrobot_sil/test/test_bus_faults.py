# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""CAN and UART fault and timing layer (models/bus_faults.py, config/models/bus_faults.yaml).

Runs on the bus of conftest.py (vcan in CI, python-can's virtual bus with SIL_TEST_BUS=virtual).
Model time is a ManualClock. Host time appears only in this test file, to wait for frames
crossing a real vcan socket; the model under test reads none. Development evidence only: the
bus physics and error states are computed, vcan carries no error frames.
"""
import ast
import json
import math
import pathlib
import re
import statistics
import time

import can
import pytest

from openamrobot_sil.canbus import NotVirtualCan
from openamrobot_sil.models import bus_faults as bf
from openamrobot_sil.sim_time import ManualClock

PKG = pathlib.Path(__file__).resolve().parents[1]
CONFIG = PKG / 'config' / 'models' / 'bus_faults.yaml'
MODULE = PKG / 'openamrobot_sil' / 'models' / 'bus_faults.py'


def frame(i=0, arb=0x601, n=8, ext=False):
    data = (i & 0xFFFFFFFF).to_bytes(4, 'little') + bytes(max(0, n - 4))
    return can.Message(arbitration_id=arb, data=data[:n], is_extended_id=ext)


def drain(bus, expect, timeout=3.0):
    """Collect frames from a raw bus until ``expect`` arrived or ``timeout`` of host time."""
    out = []
    end = time.monotonic() + timeout
    while len(out) < expect and time.monotonic() < end:
        m = bus.recv(timeout=0.05)
        if m is not None:
            out.append(m)
    # anything extra that is already there
    while True:
        m = bus.recv(timeout=0.05)
        if m is None:
            return out
        out.append(m)


def index(msg):
    return int.from_bytes(bytes(msg.data[:4]), 'little')


def run_until(fb, clock, t_end, step):
    while clock.now() < t_end - 1e-12:
        clock.advance(step)
        fb.poll()


# ---- bus physics ---------------------------------------------------------------------------

def test_frame_bits_hand_calculation():
    # classic standard: 47 + 8n bits, worst stuffing floor((34 + 8n - 1) / 4)
    assert bf.can_frame_bits(8) == (135, 0)          # 111 + 24
    assert bf.can_frame_bits(0) == (55, 0)           # 47 + 8
    assert bf.can_frame_bits(8, stuffing='none') == (111, 0)
    # classic extended: 67 + 8n bits, worst stuffing floor((54 + 8n - 1) / 4)
    assert bf.can_frame_bits(8, extended=True) == (160, 0)  # 131 + 29
    clock = ManualClock()
    m = bf.CanBusModel('drive', clock, 500000)
    assert m.frame_time_s(frame()) == pytest.approx(270e-6)  # RG-COM-009 worst case
    with pytest.raises(ValueError):
        bf.can_frame_bits(9)


def test_bus_load_matches_hand_calculation(make_bus):
    clock = ManualClock()
    model = bf.CanBusModel('drive', clock, 500000)
    tx, rx = make_bus(), make_bus()
    fb = bf.FaultyBus(tx, clock, model=model, name='drive')
    for i in range(100):
        fb.send(frame(i))
        run_until(fb, clock, (i + 1) * 0.001, 0.001)
    # 100 frames x 270 us in 0.1 s = 27.0 %
    assert model.load_percent() == pytest.approx(27.0, rel=1e-9)
    assert model.load_percent(window_s=0.05) == pytest.approx(27.0, rel=1e-6)
    assert len(drain(rx, 100)) == 100
    assert fb.metrics()['tx']['latency']['max_s'] == pytest.approx(270e-6, abs=1e-6)


def test_arbitration_priority_and_serialisation(make_bus):
    clock = ManualClock()
    model = bf.CanBusModel('drive', clock, 500000)
    tx, rx = make_bus(), make_bus()
    fb = bf.FaultyBus(tx, clock, model=model)
    clock.advance(0.0)
    # three frames ready at the same instant: lowest identifier first, one frame time apart
    with_ids = [0x581, 0x181, 0x701]
    fb._sync()
    for i, aid in enumerate(with_ids):
        fb._push(fb._tx, 0.0, frame(i, arb=aid), 0.0, None)
    fb.poll()
    assert fb.stats['tx'].frames_out == 0  # nothing finished at t = 0
    clock.advance(270e-6)
    fb.poll()
    assert fb.stats['tx'].frames_out == 1
    clock.advance(2 * 270e-6)
    fb.poll()
    got = drain(rx, 3)
    assert [m.arbitration_id for m in got] == [0x181, 0x581, 0x701]
    lat = sorted(fb.stats['tx'].latencies)
    assert lat == pytest.approx([270e-6, 540e-6, 810e-6])


def test_arbitration_delay_estimate():
    clock = ManualClock()
    m = bf.CanBusModel('drive', clock, 500000)
    est = m.arbitration_delay_estimate(frame(), load_percent=50.0)
    assert est['mean_wait_s'] == pytest.approx(0.5 * 270e-6 / (2 * 0.5))
    assert est['worst_blocking_s'] == pytest.approx(160 / 500000)
    assert math.isinf(m.arbitration_delay_estimate(frame(), load_percent=100.0)['mean_wait_s'])


def test_fd_frame_time_against_arm_estimate():
    # RG-COM-001: the arm bus model estimates about 52 us per frame at 1 / 5 Mbit/s with
    # 10 percent stuff bits. Both figures are estimates; this checks they agree.
    clock = ManualClock()
    m = bf.CanBusModel('arm_l', clock, 1000000, 5000000, fd=True, stuffing=0.10)
    msg = can.Message(arbitration_id=0x01, data=bytes(8), is_extended_id=False, is_fd=True,
                      bitrate_switch=True)
    assert m.frame_time_s(msg) == pytest.approx(52e-6, rel=0.05)
    slow = can.Message(arbitration_id=0x01, data=bytes(8), is_extended_id=False, is_fd=True)
    assert m.frame_time_s(slow) > 2 * m.frame_time_s(msg)


# ---- faults -------------------------------------------------------------------------------

def _latencies(seed, make_bus, n=600):
    clock = ManualClock()
    rule = bf.FaultRule('j', 0x600, 0x780, direction='tx', delay_s=0.002,
                        jitter={'distribution': 'uniform', 'low_s': 0.0, 'high_s': 0.001})
    fb = bf.FaultyBus(make_bus(), clock, rules=[rule], seed=seed)
    for i in range(n):
        fb.send(frame(i))
        for _ in range(40):  # 40 x 0.1 ms = 4 ms per frame, resolution 0.1 ms
            clock.advance(0.0001)
            fb.poll()
    return fb.stats['tx'].latencies, fb


def test_delay_and_jitter_statistics(make_bus):
    lat, fb = _latencies(7, make_bus)
    assert len(lat) == 600 and fb.pending()['tx'] == 0
    assert min(lat) >= 0.002 - 1e-12
    assert max(lat) <= 0.0031 + 1e-9          # 3 ms plus one poll step
    # uniform 0..1 ms plus up to one 0.1 ms poll step: mean 2.55 ms, sd about 0.29 ms
    assert statistics.mean(lat) == pytest.approx(0.00255, abs=0.00005)
    assert statistics.pstdev(lat) == pytest.approx(0.001 / math.sqrt(12), rel=0.12)
    again, _ = _latencies(7, make_bus)
    other, _ = _latencies(8, make_bus)
    assert again == lat
    assert other != lat
    assert fb.metrics()['seed'] == 7


def test_loss_rate_within_binomial_bounds(make_bus):
    clock = ManualClock()
    n, p = 5000, 0.1
    fb = bf.FaultyBus(make_bus(), clock, rules=[bf.FaultRule('loss', loss_p=p)], seed=3)
    for i in range(n):
        fb.send(frame(i))
        clock.advance(0.001)
    lost = fb.stats['tx'].lost
    sigma = math.sqrt(n * p * (1 - p))
    assert abs(lost - n * p) <= 4.5 * sigma, lost
    assert fb.stats['tx'].frames_out == n - lost
    assert fb.metrics()['tx']['lost'] == lost


def test_duplicates(make_bus):
    clock = ManualClock()
    tx, rx = make_bus(), make_bus()
    rule = bf.FaultRule('dup', duplicate_p=1.0, duplicate_gap_s=0.001)
    fb = bf.FaultyBus(tx, clock, rules=[rule])
    for i in range(20):
        fb.send(frame(i))
        run_until(fb, clock, (i + 1) * 0.01, 0.0005)
    got = drain(rx, 40)
    assert len(got) == 40
    assert [index(m) for m in got] == [i for i in range(20) for _ in (0, 1)]
    assert fb.metrics()['tx']['duplicates'] == 20


def test_reorder(make_bus):
    clock = ManualClock()
    tx, rx = make_bus(), make_bus()
    rule = bf.FaultRule('re', reorder_p=0.5, reorder_window_s=0.005)
    fb = bf.FaultyBus(tx, clock, rules=[rule], seed=11)
    n = 200
    for i in range(n):
        fb.send(frame(i))
        run_until(fb, clock, (i + 1) * 0.001, 0.0005)
    run_until(fb, clock, clock.now() + 0.01, 0.0005)
    got = [index(m) for m in drain(rx, n)]
    assert sorted(got) == list(range(n))
    assert got != list(range(n))
    inversions, high = 0, -1
    for g in got:
        if g < high:
            inversions += 1
        high = max(high, g)
    assert inversions == fb.metrics()['tx']['reordered'] > 0


def test_outages_fixed_window_and_bursts(make_bus):
    clock = ManualClock()
    fixed = bf.FaultRule('fixed', 0x100, 0x7FF, outages=[{'start_s': 0.1, 'duration_s': 0.1}])
    burst = bf.FaultRule('burst', 0x200, 0x7FF, burst={'rate_hz': 1.0, 'duration_s': 0.1})
    fb = bf.FaultyBus(make_bus(), clock, rules=[fixed, burst], seed=5)
    dropped_fixed = 0
    for i in range(30):  # 0 .. 0.29 s, set exactly (no float sum)
        clock.update(i / 100.0)
        before = fb.stats['tx'].outage_drops
        fb.send(frame(i, arb=0x100))
        if fb.stats['tx'].outage_drops > before:
            dropped_fixed += 1
            assert 0.1 <= clock.now() < 0.2
    assert dropped_fixed == 10
    base = fb.stats['tx'].outage_drops
    n = 20000  # 200 s at 10 ms: expected share 0.1 / (1 + 0.1)
    for i in range(n):
        fb.send(frame(i, arb=0x200))
        clock.advance(0.01)
    share = (fb.stats['tx'].outage_drops - base) / n
    assert 0.06 < share < 0.12, share


def test_rx_rule_delays_reception(make_bus):
    clock = ManualClock()
    tx, rxraw = make_bus(), make_bus()
    fb = bf.FaultyBus(rxraw, clock, rules=[bf.FaultRule('rx', direction='rx', delay_s=0.010)])
    tx.send(frame(1))
    assert fb.recv(timeout=1.0) is None          # arrived, not due in the receiver's clock
    assert fb.pending()['rx'] == 1
    clock.advance(0.009)
    assert fb.recv(timeout=0) is None
    clock.advance(0.001)
    got = fb.recv(timeout=0)
    assert got is not None and index(got) == 1
    assert fb.metrics()['rx']['latency']['p50_s'] == pytest.approx(0.010)


def test_paused_clock_holds_frames(make_bus):
    clock = ManualClock()
    fb = bf.FaultyBus(make_bus(), clock, rules=[bf.FaultRule('d', delay_s=0.005)])
    fb.send(frame())
    clock.paused = True
    for _ in range(10):
        clock.advance(0.01)
        fb.poll()
    assert fb.pending()['tx'] == 1
    clock.paused = False
    clock.advance(0.005)
    fb.poll()
    assert fb.pending()['tx'] == 0


# ---- error counters -----------------------------------------------------------------------

def test_error_counter_state_machine():
    clock = ManualClock()
    c = bf.CanErrorCounters(clock, 500000)
    c.tx_error(15)
    assert (c.tec, c.state) == (120, bf.ERROR_ACTIVE)
    clock.advance(1.0)
    c.tx_error()
    assert (c.tec, c.state) == (128, bf.ERROR_PASSIVE)
    c.tx_success()
    assert (c.tec, c.state) == (127, bf.ERROR_ACTIVE)
    c.rx_error(128)
    assert (c.rec, c.state) == (128, bf.ERROR_PASSIVE)
    c.rx_success()
    assert (c.rec, c.state) == (127, bf.ERROR_ACTIVE)
    c.rx_error(severe=True)
    assert c.rec == 135
    c.tec = 250
    c.tx_error()
    assert (c.tec, c.state) == (258, bf.BUS_OFF)
    c.tx_success()
    c.rx_success()
    assert c.state == bf.BUS_OFF  # only recovery leaves bus-off
    states = [(t['from'], t['to']) for t in c.transitions]
    assert states == [
        (bf.ERROR_ACTIVE, bf.ERROR_PASSIVE), (bf.ERROR_PASSIVE, bf.ERROR_ACTIVE),
        (bf.ERROR_ACTIVE, bf.ERROR_PASSIVE), (bf.ERROR_PASSIVE, bf.ERROR_ACTIVE),
        (bf.ERROR_ACTIVE, bf.ERROR_PASSIVE), (bf.ERROR_PASSIVE, bf.BUS_OFF)]
    assert c.transitions[0]['t'] == 1.0 and c.transitions[0]['generation'] == 0


def test_bus_off_manual_recovery_timing():
    clock = ManualClock()
    c = bf.CanErrorCounters(clock, 500000, recovery='manual')
    assert c.recovery_time_s == pytest.approx(128 * 11 / 500000)   # 2.816 ms
    c.force_bus_off()
    clock.advance(1.0)
    assert c.poll() == bf.BUS_OFF          # manual: waits for a request
    c.request_recovery()
    clock.advance(c.recovery_time_s - 2e-6)  # one bit short of 1408 bit times
    assert c.poll() == bf.BUS_OFF
    clock.advance(2e-6)
    assert c.poll() == bf.ERROR_ACTIVE
    assert (c.tec, c.rec, c.bus_off_count) == (0, 0, 1)
    auto = bf.CanErrorCounters(clock, 250000, recovery='auto', restart_delay_s=0.1)
    auto.force_bus_off()
    clock.advance(0.1 + auto.recovery_time_s - 1e-5)
    assert auto.poll() == bf.BUS_OFF
    clock.advance(1e-5)
    assert auto.poll() == bf.ERROR_ACTIVE


def test_tx_errors_drive_bus_off_and_cost_bus_time(make_bus):
    clock = ManualClock()
    model = bf.CanBusModel('drive', clock, 500000)
    counters = bf.CanErrorCounters(clock, 500000)
    fb = bf.FaultyBus(make_bus(), clock, rules=[bf.FaultRule('err', tx_error_p=1.0)],
                      model=model, counters=counters)
    fb.send(frame())
    run_until(fb, clock, 0.05, 0.0001)
    assert counters.state == bf.BUS_OFF and counters.tec == 256
    assert model.error_frames == 32
    assert model.busy_s == pytest.approx(32 * (135 + 23) / 500000)
    m = fb.metrics()
    assert m['tx']['frames_out'] == 0 and m['tx']['bus_off_drops'] == 1
    fb.send(frame(2))  # a bus-off node sends nothing
    assert fb.metrics()['tx']['bus_off_drops'] == 2
    assert [t['to'] for t in m['error_state']['transitions']] == [bf.ERROR_PASSIVE, bf.BUS_OFF]
    # error-passive transmitters suspend 8 bits before each of the retries 17 to 31
    passive_t = m['error_state']['transitions'][0]['t']
    off_t = m['error_state']['transitions'][1]['t']
    assert off_t - passive_t >= (15 * 8 + 16 * (135 + 23) - 158) / 500000 - 1e-4
    fb.inject_bus_off()
    fb.request_recovery()
    run_until(fb, clock, clock.now() + 0.01, 0.0005)
    assert counters.state == bf.ERROR_ACTIVE


# ---- reset (gate D3) ----------------------------------------------------------------------

def test_reset_flushes_queued_frames(make_bus):
    clock = ManualClock(start=5.0)
    tx, rx = make_bus(), make_bus()
    model = bf.CanBusModel('drive', clock, 500000)
    fb = bf.FaultyBus(tx, clock, rules=[bf.FaultRule('d', direction='tx', delay_s=1.0)],
                      model=model)
    for i in range(5):
        fb.send(frame(i))
    assert fb.pending()['tx'] == 5
    clock.reset(0.0)
    run_until(fb, clock, 3.0, 0.01)
    assert drain(rx, 1, timeout=0.3) == []
    m = fb.metrics()
    assert m['flushed_on_reset'] == 5 and m['generation'] == 1
    assert m['resets'][0]['flushed'] == 5
    fb.send(frame(9))
    run_until(fb, clock, clock.now() + 1.1, 0.01)
    got = drain(rx, 1)
    assert [index(g) for g in got] == [9]


def test_reset_flushes_frames_on_the_wire_and_rx_queue(make_bus):
    clock = ManualClock()
    a, b = make_bus(), make_bus()
    model = bf.CanBusModel('drive', clock, 500000)
    fa = bf.FaultyBus(a, clock, model=model)
    fb = bf.FaultyBus(b, clock, rules=[bf.FaultRule('rx', direction='rx', delay_s=0.5)])
    fa.send(frame(1))                      # on the wire, 270 us to go
    assert model._in_flight is not None
    a.send(frame(2))                       # raw: arrives at b, held 0.5 s by the rx rule
    assert fb.recv(timeout=1.0) is None and fb.pending()['rx'] == 1
    clock.reset(clock.now())
    clock.advance(1.0)
    fa.poll()
    assert fb.recv(timeout=0) is None
    assert fa.metrics()['flushed_on_reset'] == 1
    assert fb.metrics()['flushed_on_reset'] == 1
    assert fa.stats['tx'].frames_out == 0


# ---- interface guard (D7) -----------------------------------------------------------------

def test_refuses_anything_but_a_virtual_bus(make_bus):
    clock = ManualClock()
    with pytest.raises(NotVirtualCan):
        bf.FaultyBus(object(), clock)
    from can.interfaces import socketcan  # Linux: the SIL runs on Linux only
    fake = socketcan.SocketcanBus.__new__(socketcan.SocketcanBus)
    fake.channel = 'can0'
    with pytest.raises(NotVirtualCan):
        bf.check_virtual_bus(fake)
    fake.channel = 'vcan_drive'
    assert bf.check_virtual_bus(fake) == 'vcan_drive'
    bf.FaultyBus(make_bus(), clock)  # the conftest bus (vcan or virtual) is accepted


# ---- host time ----------------------------------------------------------------------------

def test_no_host_time_reads(monkeypatch):
    tree = ast.parse(MODULE.read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            imported.add((node.module or '').split('.')[0])
    assert not imported & {'time', 'datetime', 'threading', 'asyncio'}, imported

    def forbidden(*a, **k):
        raise AssertionError('host time read inside the model')
    for name in ('time', 'monotonic', 'perf_counter', 'time_ns', 'monotonic_ns', 'sleep'):
        monkeypatch.setattr(time, name, forbidden)
    clock = ManualClock()
    model = bf.CanBusModel('drive', clock, 500000)
    c = bf.CanErrorCounters(clock, 500000)
    c.force_bus_off()
    c.request_recovery()
    clock.advance(0.01)
    c.poll()
    model.observe(frame())
    model.metrics()
    u = bf.UartFaultModel(clock, 115200, seed=1, byte_loss_p=0.1, noise_burst_rate_hz=5,
                          noise_burst_duration_s=0.01)
    u.apply(bytes(1000))
    u.metrics()


# ---- UART -----------------------------------------------------------------------------------

def test_uart_corruption_rates():
    clock = ManualClock()
    n = 100000
    u = bf.UartFaultModel(clock, 115200, seed=2, byte_loss_p=0.01)
    out = u.apply(bytes(range(256)) * (n // 256) + bytes(n % 256))
    sigma = math.sqrt(n * 0.01 * 0.99)
    assert abs(u.stats['lost'] - n * 0.01) <= 4.5 * sigma
    assert len(out) == n - u.stats['lost']

    u = bf.UartFaultModel(clock, 115200, seed=3, framing_error_p=0.02)
    data = b'\x55' * n
    out = u.apply(data)
    assert len(out) == n
    assert out.count(0) == u.stats['framing_errors']
    assert abs(u.stats['framing_errors'] - n * 0.02) <= 4.5 * math.sqrt(n * 0.02 * 0.98)

    drop = bf.UartFaultModel(clock, 115200, seed=3, framing_error_p=0.02,
                             error_byte_policy='drop')
    assert len(drop.apply(data)) == n - u.stats['framing_errors']   # same seed, same errors
    mark = bf.UartFaultModel(clock, 115200, seed=3, framing_error_p=0.02,
                             error_byte_policy='mark')
    assert len(mark.apply(data)) == n + 2 * u.stats['framing_errors']

    p = bf.UartFaultModel(clock, 9600, parity='E', seed=4, parity_error_p=0.05)
    p.apply(bytes(20000))
    assert abs(p.stats['parity_errors'] - 1000) <= 4.5 * math.sqrt(20000 * 0.05 * 0.95)
    with pytest.raises(ValueError):
        bf.UartFaultModel(clock, 9600, parity='N', parity_error_p=0.1)

    # noise bursts of 1 ms at 10 per second: share about 0.001 / (0.1 + 0.001)
    nb = bf.UartFaultModel(clock, 115200, seed=6, noise_burst_rate_hz=10.0,
                           noise_burst_duration_s=0.001)
    clean = bytes(200000)                              # about 17.4 s of line time
    noisy = nb.apply(clean)
    changed = sum(1 for a, b in zip(clean, noisy) if a != b)
    assert changed == nb.stats['noise_corrupted']
    assert 0.006 < changed / len(clean) < 0.014, changed / len(clean)


def test_uart_baud_mismatch_threshold():
    clock = ManualClock()
    u = bf.UartFaultModel(clock, 115200)
    assert u.mismatch_tolerance == pytest.approx(0.5 / 9.5)
    data = bytes(range(256))
    assert bf.UartFaultModel(clock, 115200, baud_mismatch=0.05).apply(data) == data
    bad = bf.UartFaultModel(clock, 115200, baud_mismatch=-0.06)
    assert bad.apply(data) == bytes(256)
    assert bad.stats['baud_errors'] == 256


def test_uart_deterministic_and_reset():
    clock = ManualClock()
    kw = dict(seed=9, byte_loss_p=0.05, framing_error_p=0.01, noise_burst_rate_hz=20.0,
              noise_burst_duration_s=0.002)
    data = bytes(range(256)) * 200
    a = bf.UartFaultModel(clock, 115200, **kw).apply(data)
    b = bf.UartFaultModel(clock, 115200, **kw).apply(data)
    assert a == b
    c = bf.UartFaultModel(clock, 115200, **dict(kw, seed=10)).apply(data)
    assert a != c
    u = bf.UartFaultModel(clock, 115200, **kw)
    u.apply(data, t_start=10.0)
    clock.reset(0.0)
    u.apply(data)       # a new generation restarts the burst schedule; no error going back
    assert u.metrics()['bytes_in'] == 2 * len(data)


# ---- configuration and metrics --------------------------------------------------------------

def _records(node, path=''):
    if bf.is_parameter(node):
        yield path, node
    elif isinstance(node, dict):
        for k, v in node.items():
            yield from _records(v, '%s/%s' % (path, k))


def test_config_records_are_complete():
    cfg = bf.load_config(str(CONFIG))
    recs = list(_records({k: cfg[k] for k in ('can_buses', 'error_model', 'uarts')}))
    assert len(recs) >= 40
    confidences = {'measured', 'manufacturer_document', 'cad', 'design_decision', 'estimate',
                   'unknown'}
    for path, r in recs:
        assert set(r) == set(bf.PARAMETER_FIELDS), path
        assert r['confidence'] in confidences, path
        assert r['last_verified'] is None, path        # nothing here is measured
        assert re.fullmatch(r'MP-\d\d', r['measurement_item'] or ''), path
        if r['confidence'] == 'unknown':
            assert r['value'] is None, path
        if r['value'] is not None:
            assert r['source'], path
    # every physical leaf is a record: no bare numbers outside the profiles
    for top in ('can_buses', 'uarts', 'error_model'):
        for path, leaf in _leaves(cfg[top], top):
            assert not isinstance(leaf, (int, float)) or isinstance(leaf, bool), path


def _leaves(node, path):
    if bf.is_parameter(node):
        return
    if isinstance(node, dict):
        for k, v in node.items():
            yield from _leaves(v, '%s/%s' % (path, k))
    else:
        yield path, node


def test_config_bitrates_agree_with_sil_yaml():
    import yaml
    cfg = bf.values(bf.load_config(str(CONFIG)))
    sil = yaml.safe_load((PKG / 'config' / 'sil.yaml').read_text())
    for name in ('drive', 'bms', 'aux'):
        assert cfg['can_buses'][name]['interface'] == sil['buses'][name]['interface']
    text = (PKG / 'config' / 'sil.yaml').read_text()
    assert 'Bitrate 500 kbit/s' in text and 'Bitrate 250 kbit/s' in text
    assert cfg['can_buses']['drive']['nominal_bitrate'] == 500000
    assert cfg['can_buses']['bms']['nominal_bitrate'] == 250000
    assert cfg['can_buses']['aux']['nominal_bitrate'] == 500000


def test_config_builds_every_bus(make_bus):
    cfg = bf.load_config(str(CONFIG))
    clock = ManualClock()
    for profile in cfg['profiles']:
        for name in cfg['can_buses']:
            fb = bf.build_can(cfg, name, make_bus(), clock, profile=profile)
            assert fb.model.fd == name.startswith('arm')
            assert fb.counters.recovery == 'manual'
    deg = bf.build_can(cfg, 'drive', make_bus(), clock, profile='degraded')
    assert [r.name for r in deg.rules] == ['drive_rx_heartbeat', 'drive_tx_targets']
    assert deg.seed == 20261116


def test_config_builds_every_uart():
    cfg = bf.load_config(str(CONFIG))
    clock = ManualClock()
    bauds = {}
    for profile in cfg['profiles']:
        for name in cfg['uarts']:
            u = bf.build_uart(cfg, name, clock, profile=profile)
            bauds[name] = u.baud
            assert u.bits_per_char == 10
    assert bauds['tof_front'] == 115200 and bauds['ultrasonic_rear_left'] == 9600
    assert bauds['lidar'] == 1000000


def test_metrics_export_for_the_scenario_json(make_bus):
    cfg = bf.load_config(str(CONFIG))
    clock = ManualClock()
    fb = bf.build_can(cfg, 'drive', make_bus(), clock, profile='degraded')
    for i in range(200):
        fb.send(frame(i, arb=0x601))
        run_until(fb, clock, (i + 1) * 0.002, 0.0005)
    m = json.loads(json.dumps(fb.metrics()))
    for key in ('frames', 'bytes', 'load_percent', 'seed', 'flushed_on_reset', 'error_state'):
        assert key in m
    for key in ('p50_s', 'p95_s', 'p99_s'):
        assert m['tx']['latency'][key] is not None
    assert m['tx']['latency']['p50_s'] <= m['tx']['latency']['p95_s'] <= m['tx']['latency']['p99_s']
    assert m['frames'] == m['tx']['frames_out'] and m['bytes'] == 8 * m['frames']
    assert m['tx']['lost'] + m['tx']['frames_out'] - m['tx']['duplicates'] == 200
    assert 'transitions' in m['error_state'] and m['wire']['nominal_bitrate'] == 500000
    assert 0 < m['load_percent'] < 100
