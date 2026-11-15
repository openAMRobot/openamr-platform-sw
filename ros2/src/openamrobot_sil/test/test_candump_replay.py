"""Replay of a candump trace in place of the drive emulator, against an SDO reader that
asks one object at a time as the image does. Protocol evidence on vcan or the virtual bus.
The trace is synthetic: it is the shape of a capture, not a capture.
"""
import os
import subprocess
import sys

from conftest import PACKAGE_DIR, collect, wait_for
from openamrobot_sil import renode_fw
from openamrobot_sil.can_sil import canopen_common
from openamrobot_sil.candump_replay import Replay

TRACE = os.path.join(PACKAGE_DIR, 'traces', 'zlac_synthetic_v1.candump')


def test_trace_file_is_what_the_generator_writes(tmp_path):
    out = tmp_path / 'trace.log'
    subprocess.run([sys.executable, '-I', os.path.join(PACKAGE_DIR, 'scripts',
                                                       'sil_make_trace.py'), str(out)],
                   check=True)
    with open(TRACE, encoding='utf-8') as f:
        assert out.read_text() == f.read()


def test_replay_never_holds_a_master_side_frame(make_bus):
    frames = renode_fw.read_candump(TRACE)
    assert sum(f['id'] == 0x601 for f in frames) == 300      # the trace has them
    replay = Replay(make_bus(), frames)
    assert replay.skipped_master_frames == 302                # 300 requests, NMT, RPDO
    assert all(ident in (0x701,) for _, ident, _ in replay.unasked)
    assert sorted(replay.answers) == [(0x2026, 5), (0x603F, 0), (0x6041, 0), (0x606C, 1),
                                      (0x606C, 2)]
    assert all(len(queue) == 60 for queue in replay.answers.values())


def test_reader_gets_the_recorded_values_in_order_and_silence_after_the_end(make_bus):
    listener = make_bus()
    replay = Replay.from_file(make_bus(), TRACE).start()
    reader = canopen_common.SdoClient(make_bus(), 1, timeout=0.04)
    try:
        values = []
        while not replay.ended and len(values) < 40:
            ch1 = reader.read(0x606C, 1, signed=True)
            ch2 = reader.read(0x606C, 2, signed=True)
            assert reader.read(0x6041, 0) == 0x0237
            values.append((ch1, ch2))
        recorded = [int.from_bytes(d[4:8], 'little', signed=True)
                    for d in (f['data'] for f in renode_fw.read_candump(TRACE)
                              if f['id'] == 0x581 and f['data'][1:4] == b'\x6c\x60\x01')]
        assert [v[0] for v in values] == recorded[:len(values)] and len(values) >= 20
        assert all(ch2 == -ch1 and ch1 != 0 for ch1, ch2 in values)
        assert max(v[0] for v in values) == 301               # the ramp was seen
        frames = collect(listener, 0.3)
        assert any(m.arbitration_id == 0x701 for m in frames)
        # No frame of the master side came from the replay (the reader's own are 601h).
        assert not [m for m in frames if m.arbitration_id in (0x000, 0x201, 0x301)]
        assert wait_for(lambda: replay.ended, timeout=8.0)
        before = replay.state()
        try:
            reader.read(0x606C, 1, signed=True)
            answered = True
        except Exception:      # the vendored client raises its own timeout type
            answered = False
        assert not answered and replay.state()['unanswered'] == before['unanswered'] + 1
        while listener.recv(timeout=0.0) is not None:     # frames queued before the end
            pass
        assert not [m for m in collect(listener, 0.4) if m.arbitration_id == 0x701]
    finally:
        replay.stop()


def test_an_object_the_trace_never_answered_gets_no_answer(make_bus):
    replay = Replay.from_file(make_bus(), TRACE).start()
    reader = canopen_common.SdoClient(make_bus(), 1, timeout=0.04)
    try:
        try:
            reader.read(0x6064, 1, signed=True)
            answered = True
        except Exception:
            answered = False
        assert not answered and replay.state()['unanswered'] == 1
    finally:
        replay.stop()
