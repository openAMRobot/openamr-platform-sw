"""MatchingSdoClient of the host test client (fixture, vcan only): answers are matched to requests.

Pure logic against a scripted bus: no vcan, no ROS. The scripted bus records every frame the
client sends and answers each request with the frames a test puts in its queue.
"""
import struct
import time

import can
import pytest

from openamrobot_sil.can_sil import canopen_common
from openamrobot_sil.stm32_host import MatchingSdoClient

NODE = 1
TX = canopen_common.SDO_TX_BASE + NODE   # 581h: node to client


def frame(cs, index, sub, value=0):
    return can.Message(arbitration_id=TX, data=struct.pack('<BHBi', cs, index, sub, value),
                       is_extended_id=False)


class ScriptedBus:
    def __init__(self):
        self.sent = []
        self.replies = []   # one list of frames per request, in order

    def send(self, msg):
        self.sent.append((msg.arbitration_id, bytes(msg.data)))
        self._pending = list(self.replies.pop(0)) if self.replies else []

    def recv(self, timeout=None):
        if self._pending:
            return self._pending.pop(0)
        time.sleep(min(timeout or 0.0, 0.01))
        return None


def test_late_download_confirm_is_discarded_not_taken_as_the_upload_answer():
    """The hosted failure (run 43736333260): a 60h confirm of 60FFh:01 arrives late, in front of
    the answer to the next request, a 606Ch:01 upload."""
    bus = ScriptedBus()
    bus.replies = [[frame(0x60, 0x60FF, 1), frame(0x43, 0x606C, 1, -190)]]
    sdo = MatchingSdoClient(bus, NODE, timeout=0.2)
    assert sdo.read(0x606C, 1, signed=True) == -190
    assert sdo.discarded == 1


def test_the_vendored_client_fails_on_the_same_sequence():
    """Control: the unmodified vendored client takes the stale confirm as the answer."""
    bus = ScriptedBus()
    bus.replies = [[frame(0x60, 0x60FF, 1), frame(0x43, 0x606C, 1, -190)]]
    sdo = canopen_common.SdoClient(bus, NODE, timeout=0.2)
    with pytest.raises(KeyError):
        sdo.read(0x606C, 1, signed=True)


def test_answer_for_another_object_or_a_duplicate_is_discarded():
    bus = ScriptedBus()
    bus.replies = [[frame(0x43, 0x606C, 2, 5), frame(0x80, 0x6041, 0, 0x06020000),
                    frame(0x60, 0x60FF, 2), frame(0x60, 0x60FF, 2)],
                   [frame(0x43, 0x606C, 1, 7)]]
    sdo = MatchingSdoClient(bus, NODE, timeout=0.2)
    sdo.write(0x60FF, 2, 19, 4, signed=True)        # 2 discarded before its own confirm
    assert sdo.discarded == 2
    assert sdo.read(0x606C, 1) == 7                  # the duplicate confirm was left in the
    assert sdo.discarded == 2                        # first request's window and is gone


def test_an_abort_of_the_outstanding_request_is_still_an_abort():
    bus = ScriptedBus()
    bus.replies = [[frame(0x80, 0x606C, 1, 0x06020000)]]
    sdo = MatchingSdoClient(bus, NODE, timeout=0.2)
    with pytest.raises(canopen_common.SdoAbort):
        sdo.read(0x606C, 1)


def test_only_stale_frames_time_out_as_before():
    bus = ScriptedBus()
    bus.replies = [[frame(0x60, 0x60FF, 1)]]
    sdo = MatchingSdoClient(bus, NODE, timeout=0.1)
    t0 = time.monotonic()
    with pytest.raises(TimeoutError):
        sdo.read(0x606C, 1)
    assert time.monotonic() - t0 >= 0.1 and sdo.discarded == 1


def test_frames_sent_are_exactly_those_of_the_vendored_client():
    """No new command and no new object: the same requests produce the same frames."""
    sent = []
    for cls in (canopen_common.SdoClient, MatchingSdoClient):
        bus = ScriptedBus()
        bus.replies = [[frame(0x60, 0x60FF, 1)], [frame(0x60, 0x6040, 0)],
                       [frame(0x43, 0x606C, 1, 0)], [frame(0x4B, 0x6041, 0, 0x237)]]
        sdo = cls(bus, NODE, timeout=0.2)
        sdo.write(0x60FF, 1, -19, 4, signed=True)
        sdo.write(0x6040, 0, 0x0F, 2)
        sdo.read(0x606C, 1, signed=True)
        sdo.read(0x6041, 0)
        sent.append(bus.sent)
    assert sent[0] == sent[1]
