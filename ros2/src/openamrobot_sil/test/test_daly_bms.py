"""Minimal Daly BMS emulator. The protocol is unconfirmed for the delivered unit."""
import struct
import time

import can
import pytest

from conftest import collect, wait_for
from openamrobot_sil import daly_bms
from openamrobot_sil.stm32_host import BmsHost


@pytest.fixture
def bms(make_bus):
    emu = daly_bms.DalyBmsEmulator(make_bus(), pack_voltage_v=26.4, current_a=-3.5,
                                   soc_percent=72.5).start()
    yield emu
    emu.stop()


def ask(bus, data_id, destination=0x01, source=0x40, extended=True):
    bus.send(can.Message(arbitration_id=daly_bms.daly_id(data_id, destination, source),
                         data=bytes(8), is_extended_id=extended))
    return [m for m in collect(bus, 0.15) if m.is_extended_id]


def test_identifier_layout():
    assert daly_bms.daly_id(0x90, 0x01, 0x40) == 0x18900140
    assert daly_bms.daly_id(0x90, 0x40, 0x01) == 0x18904001
    assert daly_bms.split_id(0x18904001) == (0x18, 0x90, 0x40, 0x01)


def test_soc_frame_carries_voltage_current_and_soc(make_bus, bms):
    answers = ask(make_bus(), daly_bms.ID_SOC)
    assert [m.arbitration_id for m in answers] == [0x18904001]
    assert len(answers[0].data) == 8
    assert struct.unpack('>HHHH', bytes(answers[0].data)) == (264, 264, 30000 - 35, 725)
    decoded = daly_bms.decode_response(daly_bms.ID_SOC, answers[0].data)
    assert decoded['pack_voltage_v'] == pytest.approx(26.4)
    assert decoded['current_a'] == pytest.approx(-3.5)
    assert decoded['soc_percent'] == pytest.approx(72.5)


def test_status_minmax_and_mos_frames(make_bus, bms):
    bus = make_bus()
    cells = bytes(ask(bus, daly_bms.ID_CELL_MINMAX)[0].data)
    assert struct.unpack('>HBHB', cells[:6]) == (3300, 1, 3300, 8)
    temps = bytes(ask(bus, daly_bms.ID_TEMP_MINMAX)[0].data)
    assert temps[:4] == bytes([65, 1, 65, 1])
    status = bytes(ask(bus, daly_bms.ID_STATUS)[0].data)
    assert status[0] == 8 and status[1] == 1
    mos = daly_bms.decode_response(daly_bms.ID_MOS, ask(bus, daly_bms.ID_MOS)[0].data)
    assert mos['charge_mos_on'] and mos['discharge_mos_on']
    assert mos['remaining_mah'] == int(84000 * 0.725)


def test_fault_injection_and_mos_state(make_bus, bms):
    bus = make_bus()
    clean = daly_bms.decode_response(daly_bms.ID_FAILURE, ask(bus, daly_bms.ID_FAILURE)[0].data)
    assert not (clean['any_level1'] or clean['any_level2'] or clean['any_hw_failure'])
    bms.inject_failure(0, 0x01)
    bms.inject_failure(1, 0x02)
    bms.inject_failure(4, 0x10)
    bms.set_mos(False, False)
    fault = daly_bms.decode_response(daly_bms.ID_FAILURE, ask(bus, daly_bms.ID_FAILURE)[0].data)
    assert fault['any_level1'] and fault['any_level2'] and fault['any_hw_failure']
    assert fault['failure_raw'].startswith('0102000010')
    mos = daly_bms.decode_response(daly_bms.ID_MOS, ask(bus, daly_bms.ID_MOS)[0].data)
    assert not mos['charge_mos_on'] and not mos['discharge_mos_on']
    bms.clear_failures()
    again = daly_bms.decode_response(daly_bms.ID_FAILURE, ask(bus, daly_bms.ID_FAILURE)[0].data)
    assert not again['any_level1']


def test_foreign_standard_and_unsupported_requests_get_no_answer(make_bus, bms):
    bus = make_bus()
    assert ask(bus, daly_bms.ID_SOC, destination=0x02) == []
    assert ask(bus, daly_bms.ID_SOC, source=0x41) == []
    assert ask(bus, 0x10, extended=False) == []
    assert ask(bus, 0x95) == []
    assert bms.foreign == 2 and bms.unsupported == 1


def test_host_poll_reads_values_and_sees_an_offline_bms(make_bus, bms):
    host = BmsHost(make_bus()).start()
    try:
        assert wait_for(lambda: host.state().get('soc_percent') == pytest.approx(72.5), 2.0)
        state = host.state()
        assert state['pack_voltage_v'] == pytest.approx(26.4) and state['timeouts'] == 0
        bms.set_values(soc_percent=20.0, current_a=12.0)
        assert wait_for(lambda: host.state()['soc_percent'] == pytest.approx(20.0), 2.0)
        bms.offline = True
        assert wait_for(lambda: host.state()['timeouts'] >= 3, 2.0)
        time.sleep(0.5)
        assert host.state()['age_s'] > 0.4   # no stale value is refreshed
    finally:
        host.stop()
