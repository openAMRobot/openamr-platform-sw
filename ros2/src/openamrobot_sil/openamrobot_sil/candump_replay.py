"""Replay a candump trace into a drive bus in place of the drive emulator.

For bench captures: record the real drive bus with "candump -L can0 > file", drop the file
in, and the firmware image reads the recorded drive instead of the SIL emulator. Today the
only trace is synthetic (traces/zlac_synthetic_v1.candump, made by scripts/sil_make_trace.py).

A trace cannot be replayed blindly: the image asks for one object at a time, on its own
schedule, and takes an answer only for the object it asked for. So the replay has two parts.

  - Frames the drive sent unasked (heartbeat 700h + node, EMCY 080h + node, transmit PDOs)
    are sent at their trace time, measured from the first frame. Host wall time.
  - SDO answers (580h + node) are kept in one queue per object, in trace order. When a
    request for an object arrives on the bus, the next recorded answer for it is sent. When
    the queue of an object is empty, or the trace has ended, the request gets no answer.
  - Frames the trace holds from the master side (600h + node requests, NMT, receive PDOs)
    are never sent. The replay cannot command anything.

After the last unasked frame the replayed drive is silent, as a drive that lost power.
Test fixture. It is not a model of the drive, and a recorded answer is only as good as the
capture it came from.
"""
import threading
import time

import can

from openamrobot_sil import renode_fw


class Replay:
    def __init__(self, bus, frames, node_id=1, answer_delay_s=0.0005):
        self.bus = bus
        self.node_id = node_id
        self.answer_delay_s = answer_delay_s
        self.unasked = []            # (offset s, id, data)
        self.answers = {}            # (index, sub) -> [data, ...]
        self.skipped_master_frames = 0
        self.sent_unasked = 0
        self.answered = 0
        self.unanswered = 0
        self.requests = 0
        self.served = []             # (index, sub, data) in the order they were sent
        t0 = frames[0]['t'] if frames else 0.0
        master = {0, renode_fw.SDO_RX_BASE + node_id} | {
            base + node_id for base in renode_fw.RPDO_BASES}
        for f in frames:
            if f['extended'] or f['remote'] or f['fd'] or f['id'] in master:
                self.skipped_master_frames += 1
            elif f['id'] == renode_fw.SDO_TX_BASE + node_id:
                if len(f['data']) >= 4:
                    key = (int.from_bytes(f['data'][1:3], 'little'), f['data'][3])
                    self.answers.setdefault(key, []).append(f['data'])
            else:
                self.unasked.append((f['t'] - t0, f['id'], f['data']))
        self.duration_s = self.unasked[-1][0] if self.unasked else 0.0
        self.ended = False
        self._run = False
        self._threads = []

    @classmethod
    def from_file(cls, bus, path, node_id=1):
        return cls(bus, renode_fw.read_candump(path), node_id)

    def start(self):
        self._run = True
        self._t0 = time.monotonic()
        for target in (self._play, self._serve):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self._threads.append(thread)
        return self

    def _send(self, arbitration_id, data):
        self.bus.send(can.Message(arbitration_id=arbitration_id, data=data,
                                  is_extended_id=False))

    def _play(self):
        for offset, arbitration_id, data in self.unasked:
            while self._run and time.monotonic() - self._t0 < offset:
                time.sleep(min(0.005, max(0.0, offset - (time.monotonic() - self._t0))))
            if not self._run:
                return
            self._send(arbitration_id, data)
            self.sent_unasked += 1
        self.ended = True

    def _serve(self):
        while self._run:
            msg = self.bus.recv(timeout=0.05)
            if (msg is None or msg.is_extended_id
                    or msg.arbitration_id != renode_fw.SDO_RX_BASE + self.node_id
                    or len(msg.data) < 4):
                continue
            self.requests += 1
            key = (int.from_bytes(msg.data[1:3], 'little'), msg.data[3])
            queue = self.answers.get(key)
            if self.ended or not queue:
                self.unanswered += 1
                continue
            data = queue.pop(0)
            time.sleep(self.answer_delay_s)
            self._send(renode_fw.SDO_TX_BASE + self.node_id, data)
            self.answered += 1
            self.served.append((key[0], key[1], bytes(data)))

    def state(self):
        return {'duration_s': self.duration_s, 'ended': self.ended,
                'sent_unasked': self.sent_unasked, 'unasked_total': len(self.unasked),
                'requests': self.requests, 'answered': self.answered,
                'unanswered': self.unanswered,
                'answers_left': sum(len(q) for q in self.answers.values()),
                'skipped_master_frames': self.skipped_master_frames}

    def stop(self):
        self._run = False
        for thread in self._threads:
            thread.join(timeout=1)
