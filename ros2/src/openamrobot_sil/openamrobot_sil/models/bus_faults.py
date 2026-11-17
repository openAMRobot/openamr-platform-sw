# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""CAN and UART fault and timing layer of the SIL digital twin (W4, Proposed).

Development evidence only. Nothing here touches hardware, and nothing here is safety logic: it
delays, drops, duplicates, reorders and corrupts frames and bytes so that the emulators and the
host client can be tested against a bus that is not perfect. Linux vcan and python-can's
``virtual`` bus deliver every frame whole and at once and have no error frames, so bus load,
arbitration, error counters and bus-off are computed by a model here, not observed on a bus.

Parts:

- ``FaultyBus`` wraps a python-can bus that it is given and never opens an interface itself
  (decision D7: virtual CAN only). It refuses a bus that is not python-can's ``virtual`` bus or
  a SocketCAN bus on a channel named ``vcan*``. Per rule (arbitration ID and mask, direction)
  it adds a fixed delay, jitter, loss, duplication, reordering and outages.
- ``CanBusModel`` is the shared wire of one bus: frame length in bits (classic CAN or CAN FD,
  bit stuffing worst case, none or a fraction), serialisation and priority arbitration of the
  frames submitted through ``FaultyBus``, bus load and an arbitration delay estimate.
- ``CanErrorCounters`` is the fault confinement state machine of one node (TEC, REC,
  error-active, error-passive, bus-off and recovery).
- ``UartFaultModel`` corrupts a byte stream: byte loss, framing and parity errors, noise
  bursts and a baud rate mismatch.

Every time in this module is read from an injected clock (``now()``, ``generation``,
``paused``; openamrobot_sil.sim_time). The module reads no host time (contract rule C6: delays
are counted in the receiver's clock domain). A change of ``generation`` (a simulation reset)
flushes every queued frame (gate D3: no stale command survives a reset). Random numbers come
from ``random.Random`` streams seeded from the recorded seed and the rule name, so a run is
repeatable for a given seed and the same sequence of calls.
"""
import heapq
import math
import random

import can

from openamrobot_sil.canbus import NotVirtualCan, VCAN_PREFIX

TX = 'tx'
RX = 'rx'
DIRECTIONS = (TX, RX, 'both')

ERROR_ACTIVE = 'error_active'
ERROR_PASSIVE = 'error_passive'
BUS_OFF = 'bus_off'

JITTER_DISTRIBUTIONS = ('none', 'uniform', 'normal', 'exponential')

# Due-time tolerance (1 ns of clock time) so that float sums of clock steps do not hold a frame
# one poll longer than its due time.
EPS_S = 1e-9


# --------------------------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------------------------

def _rng(seed, *names):
    """Independent, repeatable random stream for a seed and a name path (str seeding is stable)."""
    return random.Random('%s|%s' % (seed, '|'.join(str(n) for n in names)))


def _prob(name, p):
    p = float(p)
    if not 0.0 <= p <= 1.0:
        raise ValueError('%s must be within [0, 1], got %r' % (name, p))
    return p


def _nonneg(name, v):
    v = float(v)
    if not math.isfinite(v) or v < 0.0:
        raise ValueError('%s must be finite and >= 0, got %r' % (name, v))
    return v


def percentile(values, p):
    """Nearest-rank percentile; None for an empty list."""
    if not values:
        return None
    s = sorted(values)
    k = max(0, int(math.ceil(p / 100.0 * len(s))) - 1)
    return s[min(k, len(s) - 1)]


def latency_summary(values):
    return {
        'count': len(values),
        'p50_s': percentile(values, 50),
        'p95_s': percentile(values, 95),
        'p99_s': percentile(values, 99),
        'max_s': max(values) if values else None,
    }


class BurstSchedule:
    """Outage or noise bursts with exponential gaps (rate per second of clock time).

    The schedule is drawn lazily as time advances, from its own random stream, so it does not
    depend on how many frames or bytes pass. ``reset(t)`` starts a new schedule at ``t``.
    """

    def __init__(self, rng, rate_hz, duration_s):
        self.rate_hz = _nonneg('burst rate_hz', rate_hz)
        self.duration_s = _nonneg('burst duration_s', duration_s)
        self._rng = rng
        self._onset = None
        self._origin = None
        self.count = 0

    def reset(self, t):
        self._origin = t
        self._onset = None

    def _draw(self, after):
        self._onset = after + self._rng.expovariate(self.rate_hz)
        self.count += 1

    def active(self, t):
        if self.rate_hz <= 0.0 or self.duration_s <= 0.0:
            return False
        if self._origin is None or t < self._origin:
            self.reset(t)
        if self._onset is None:
            self._draw(self._origin)
        while t >= self._onset + self.duration_s:
            self._draw(self._onset + self.duration_s)
        return self._onset <= t


# --------------------------------------------------------------------------------------------
# Rules
# --------------------------------------------------------------------------------------------

class FaultRule:
    """Faults for the frames that match an arbitration ID under a mask, in one direction.

    The first matching rule of a ``FaultyBus`` applies; a frame that matches no rule passes
    with no delay. Times are seconds of the injected clock.
    """

    def __init__(self, name, arbitration_id=0, mask=0, extended=None, direction='both',
                 delay_s=0.0, jitter=None, loss_p=0.0, duplicate_p=0.0, duplicate_gap_s=0.0,
                 reorder_p=0.0, reorder_window_s=0.0, outages=(), burst=None, tx_error_p=0.0):
        if direction not in DIRECTIONS:
            raise ValueError('direction must be one of %s, got %r' % (DIRECTIONS, direction))
        self.name = str(name)
        self.arbitration_id = int(arbitration_id)
        self.mask = int(mask)
        self.extended = None if extended is None else bool(extended)
        self.direction = direction
        self.delay_s = _nonneg('delay_s', delay_s)
        self.jitter = dict(jitter or {'distribution': 'none'})
        dist = self.jitter.get('distribution', 'none')
        if dist not in JITTER_DISTRIBUTIONS:
            raise ValueError('jitter distribution must be one of %s, got %r'
                             % (JITTER_DISTRIBUTIONS, dist))
        self.loss_p = _prob('loss_p', loss_p)
        self.duplicate_p = _prob('duplicate_p', duplicate_p)
        self.duplicate_gap_s = _nonneg('duplicate_gap_s', duplicate_gap_s)
        self.reorder_p = _prob('reorder_p', reorder_p)
        self.reorder_window_s = _nonneg('reorder_window_s', reorder_window_s)
        pairs = [(o['start_s'], o['duration_s']) if isinstance(o, dict) else tuple(o)
                 for o in outages or ()]
        self.outages = [(_nonneg('outage start', s), _nonneg('outage duration', d))
                        for s, d in pairs]
        self.burst = dict(burst) if burst else None
        self.tx_error_p = _prob('tx_error_p', tx_error_p)

    @classmethod
    def from_dict(cls, d):
        return cls(**dict(d))

    def matches(self, msg, direction):
        if self.direction != 'both' and self.direction != direction:
            return False
        if self.extended is not None and bool(msg.is_extended_id) != self.extended:
            return False
        return (msg.arbitration_id & self.mask) == (self.arbitration_id & self.mask)

    def sample_jitter(self, rng):
        j = self.jitter
        dist = j.get('distribution', 'none')
        if dist == 'none':
            return 0.0
        if dist == 'uniform':
            return rng.uniform(float(j['low_s']), float(j['high_s']))
        if dist == 'normal':
            return rng.gauss(float(j.get('mean_s', 0.0)), float(j['sigma_s']))
        return rng.expovariate(1.0 / float(j['mean_s']))  # exponential

    def in_fixed_outage(self, t):
        return any(s <= t < s + d for s, d in self.outages)


# --------------------------------------------------------------------------------------------
# Error counters (ISO 11898-1 fault confinement)
# --------------------------------------------------------------------------------------------

class CanErrorCounters:
    """TEC and REC of one CAN node and its error state.

    ISO 11898-1 fault confinement as modelled here: a transmit error adds 8 to TEC, a
    successful transmission subtracts 1; a receive error adds 1 to REC (8 for the severe
    cases), a successful reception subtracts 1 or, from above 127, sets REC to
    ``rec_passive_success_value`` (the standard allows 119 to 127). TEC or REC at 128 or more
    is error-passive; TEC at 256 or more is bus-off. A bus-off node leaves bus-off after it has
    seen 128 occurrences of 11 consecutive recessive bits, counted here as 11 bit times each of
    clock time from the start of recovery (a lower bound: on a busy bus each frame end gives one
    occurrence). ``recovery='manual'`` waits for ``request_recovery()`` (Linux restart-ms 0 and
    a controller whose software must restart it); ``'auto'`` starts recovery on entering bus-off
    after ``restart_delay_s``.

    vcan carries no error frames, so these counters change only through this model's API or the
    ``tx_error_p`` of a rule. They are modelled states, not observations of a bus.
    """

    def __init__(self, clock, bitrate, recovery='manual', restart_delay_s=0.0,
                 tx_error_increment=8, success_decrement=1, rx_error_increment=1,
                 rx_error_severe_increment=8, passive_threshold=128, bus_off_threshold=256,
                 recovery_sequences=128, recovery_sequence_bits=11, rec_passive_success_value=127,
                 name='node'):
        if recovery not in ('manual', 'auto'):
            raise ValueError('recovery must be manual or auto, got %r' % recovery)
        self.clock = clock
        self.name = name
        self.bit_time_s = 1.0 / float(bitrate)
        self.recovery = recovery
        self.restart_delay_s = _nonneg('restart_delay_s', restart_delay_s)
        self.tx_error_increment = int(tx_error_increment)
        self.success_decrement = int(success_decrement)
        self.rx_error_increment = int(rx_error_increment)
        self.rx_error_severe_increment = int(rx_error_severe_increment)
        self.passive_threshold = int(passive_threshold)
        self.bus_off_threshold = int(bus_off_threshold)
        self.recovery_sequences = int(recovery_sequences)
        self.recovery_sequence_bits = int(recovery_sequence_bits)
        self.rec_passive_success_value = int(rec_passive_success_value)
        self.tec = 0
        self.rec = 0
        self.state = ERROR_ACTIVE
        self.transitions = []
        self.bus_off_count = 0
        self._recovery_start = None

    @property
    def recovery_time_s(self):
        """Minimum time from the start of recovery to error-active."""
        return self.recovery_sequences * self.recovery_sequence_bits * self.bit_time_s

    def _set_state(self, new, cause):
        if new == self.state:
            return
        self.transitions.append({
            't': self.clock.now(), 'generation': self.clock.generation,
            'from': self.state, 'to': new, 'tec': self.tec, 'rec': self.rec, 'cause': cause})
        self.state = new
        if new == BUS_OFF:
            self.bus_off_count += 1
            self._recovery_start = None
            if self.recovery == 'auto':
                self._recovery_start = self.clock.now() + self.restart_delay_s

    def _update(self, cause):
        if self.state == BUS_OFF:
            return
        if self.tec >= self.bus_off_threshold:
            self._set_state(BUS_OFF, cause)
        elif self.tec >= self.passive_threshold or self.rec >= self.passive_threshold:
            self._set_state(ERROR_PASSIVE, cause)
        else:
            self._set_state(ERROR_ACTIVE, cause)

    def tx_error(self, n=1, cause='tx_error'):
        for _ in range(int(n)):
            if self.state == BUS_OFF:
                return
            self.tec += self.tx_error_increment
            self._update(cause)

    def tx_success(self):
        if self.state == BUS_OFF:
            return
        self.tec = max(0, self.tec - self.success_decrement)
        self._update('tx_success')

    def rx_error(self, n=1, severe=False, cause='rx_error'):
        inc = self.rx_error_severe_increment if severe else self.rx_error_increment
        for _ in range(int(n)):
            if self.state == BUS_OFF:
                return
            self.rec += inc
            self._update(cause)

    def rx_success(self):
        if self.state == BUS_OFF:
            return
        if self.rec >= self.passive_threshold:
            self.rec = self.rec_passive_success_value
        elif self.rec > 0:
            self.rec -= self.success_decrement
        self._update('rx_success')

    def force_bus_off(self, cause='injected'):
        """Bus-off injection: TEC to the bus-off threshold."""
        if self.state == BUS_OFF:
            return
        self.tec = max(self.tec, self.bus_off_threshold)
        self._update(cause)

    def request_recovery(self):
        """Start the bus-off recovery sequence (manual mode); no effect outside bus-off."""
        if self.state == BUS_OFF and self._recovery_start is None:
            self._recovery_start = self.clock.now()

    def recovery_sequences_seen(self):
        if self.state != BUS_OFF or self._recovery_start is None:
            return 0
        elapsed = self.clock.now() - self._recovery_start
        if elapsed < 0:
            return 0
        return int(elapsed / (self.recovery_sequence_bits * self.bit_time_s) + 1e-9)

    def poll(self):
        """Advance recovery with the clock. Returns the state."""
        if self.state == BUS_OFF and self.recovery_sequences_seen() >= self.recovery_sequences:
            self.tec = 0
            self.rec = 0
            self._recovery_start = None
            self._set_state(ERROR_ACTIVE, 'recovered')
        return self.state

    def metrics(self):
        return {'tec': self.tec, 'rec': self.rec, 'state': self.state,
                'bus_off_count': self.bus_off_count, 'transitions': list(self.transitions)}


# --------------------------------------------------------------------------------------------
# Bus physics
# --------------------------------------------------------------------------------------------

def _stuff_bits(stuffable_bits, stuffing):
    if stuffing == 'worst':
        return (stuffable_bits - 1) // 4
    if stuffing == 'none':
        return 0
    return int(round(float(stuffing) * stuffable_bits))


def can_frame_bits(dlc_bytes, extended=False, fd=False, brs=False, stuffing='worst'):
    """Bits of one data frame on the wire, as (nominal_rate_bits, data_rate_bits).

    Classic CAN: SOF, identifier, control, data, CRC 15 and the fixed tail (CRC and ACK
    delimiters, ACK slot, EOF 7, intermission 3): 47 + 8n bits standard, 67 + 8n extended.
    Stuffing applies from SOF to the end of the CRC sequence (34 + 8n and 54 + 8n bits); the
    worst case is floor((g + 8n - 1) / 4). An 8-byte standard frame is 135 bits worst case,
    270 us at 500 kbit/s.

    CAN FD (estimate): arbitration phase (SOF to BRS, 17 bits standard or 36 extended) and the
    tail (CRC delimiter onward, 13 bits) at the nominal rate; ESI, DLC, data, stuff count (4)
    and CRC 17 (up to 16 bytes) or 21 with 5 or 6 fixed stuff bits at the data rate when BRS is
    set. Dynamic stuffing is applied to SOF to the end of the data field.
    """
    n = int(dlc_bytes)
    if not fd:
        if n > 8:
            raise ValueError('classic CAN carries at most 8 data bytes')
        head = 34 if not extended else 54
        total = (47 if not extended else 67) + 8 * n
        total += _stuff_bits(head + 8 * n, stuffing)
        return total, 0
    arb = 17 if not extended else 36
    crc, fixed = (17, 5) if n <= 16 else (21, 6)
    data = 1 + 4 + 8 * n + 4 + crc + fixed
    stuff = _stuff_bits(arb + 1 + 4 + 8 * n, stuffing)
    tail = 13
    if brs:
        # share the dynamic stuff bits between the two phases by their bit counts
        stuff_arb = int(round(stuff * arb / float(arb + 5 + 8 * n)))
        return arb + tail + stuff_arb, data + (stuff - stuff_arb)
    return arb + tail + data + stuff, 0


class CanBusModel:
    """The shared wire of one CAN bus, timed on the injected clock.

    Frames submitted by the ``FaultyBus`` handles on this model are serialised: one frame at
    a time, and at each free moment the pending frame with the highest priority (lowest
    identifier, base ID before an extended ID with the same base) wins. A frame is delivered to
    its python-can bus at the end of its frame time. Frames that do not pass through a
    ``FaultyBus`` can be accounted with ``observe()``. Load and latency are computed values;
    vcan itself has no bit rate.
    """

    def __init__(self, name, clock, nominal_bitrate, data_bitrate=None, fd=False,
                 stuffing='worst', error_frame_bits=23, passive_suspend_bits=8,
                 load_window_s=1.0, seed=0):
        self.name = name
        self.clock = clock
        self.nominal_bitrate = float(nominal_bitrate)
        self.data_bitrate = float(data_bitrate) if data_bitrate else None
        self.fd = bool(fd)
        if self.fd and not self.data_bitrate:
            self.data_bitrate = self.nominal_bitrate
        self.stuffing = stuffing
        self.error_frame_bits = int(error_frame_bits)
        self.passive_suspend_bits = int(passive_suspend_bits)
        self.load_window_s = _nonneg('load_window_s', load_window_s)
        self.seed = seed
        self._rng = _rng(seed, name, 'wire')
        self._pending = []
        self._in_flight = None
        self._wire_free = None
        self._seq = 0
        self._generation = clock.generation
        self._intervals = []
        self.busy_s = 0.0
        self.frames = 0
        self.bits = 0
        self.error_frames = 0
        self.flushed_on_reset = 0
        self._t0 = clock.now()
        self.max_frame_time_s = 0.0

    # frame time ------------------------------------------------------------------------------
    def frame_time_s(self, msg):
        nbits, dbits = can_frame_bits(
            len(msg.data), extended=msg.is_extended_id,
            fd=self.fd and bool(getattr(msg, 'is_fd', False)),
            brs=bool(getattr(msg, 'bitrate_switch', False)), stuffing=self.stuffing)
        t = nbits / self.nominal_bitrate
        if dbits:
            t += dbits / self.data_bitrate
        return t

    def worst_frame_time_s(self):
        if self.fd:
            n, d = can_frame_bits(64, extended=True, fd=True, brs=False, stuffing=self.stuffing)
            return n / self.nominal_bitrate
        n, _ = can_frame_bits(8, extended=True, stuffing=self.stuffing)
        return n / self.nominal_bitrate

    # load ------------------------------------------------------------------------------------
    def _account(self, start, duration, bits):
        self.busy_s += duration
        self.bits += bits
        self._intervals.append((start, start + duration))
        horizon = start - max(self.load_window_s, 0.0) - duration
        while self._intervals and self._intervals[0][1] < horizon:
            self._intervals.pop(0)

    def observe(self, msg):
        """Account a frame seen on the bus that did not pass through a FaultyBus on this model."""
        self._sync()
        ft = self.frame_time_s(msg)
        self._account(self.clock.now(), ft, self._bits(msg))
        self.frames += 1

    def _bits(self, msg):
        n, d = can_frame_bits(len(msg.data), extended=msg.is_extended_id,
                              fd=self.fd and bool(getattr(msg, 'is_fd', False)),
                              brs=bool(getattr(msg, 'bitrate_switch', False)),
                              stuffing=self.stuffing)
        return n + d

    def load_percent(self, window_s=None):
        """Busy share of the wire over the last ``window_s`` seconds, or since the start."""
        now = self.clock.now()
        if window_s is None:
            elapsed = now - self._t0
            return 100.0 * self.busy_s / elapsed if elapsed > 0 else 0.0
        lo = now - window_s
        busy = sum(max(0.0, min(e, now) - max(s, lo)) for s, e in self._intervals)
        return 100.0 * busy / window_s if window_s > 0 else 0.0

    def arbitration_delay_estimate(self, msg=None, load_percent=None):
        """Waiting time before a frame wins the bus, as an estimate.

        ``mean_wait_s`` is the M/D/1 queue wait rho * C / (2 (1 - rho)) with C the frame time
        of ``msg`` (or of an 8-byte standard frame) and rho the load; it ignores priority.
        ``worst_blocking_s`` is one maximal frame already on the wire (CAN is not pre-emptive).
        Neither is a measurement.
        """
        if msg is None:
            msg = can.Message(arbitration_id=0x7FF, data=bytes(8), is_extended_id=False)
        rho = (self.load_percent() if load_percent is None else load_percent) / 100.0
        c = self.frame_time_s(msg)
        mean = math.inf if rho >= 1.0 else rho * c / (2.0 * (1.0 - rho))
        return {'load': rho, 'frame_time_s': c, 'mean_wait_s': mean,
                'worst_blocking_s': self.worst_frame_time_s()}

    # wire ------------------------------------------------------------------------------------
    @staticmethod
    def _priority(msg):
        aid = msg.arbitration_id
        if msg.is_extended_id:
            return (aid >> 18, 1, aid)
        return (aid, 0, 0)

    def _sync(self):
        if self.clock.generation != self._generation:
            self.flushed_on_reset += len(self._pending) + (1 if self._in_flight else 0)
            self._pending = []
            self._in_flight = None
            self._wire_free = None
            self._generation = self.clock.generation
            self._intervals = []

    def submit(self, owner, msg, ready_t, sent_t, rule, tx_seq=None):
        self._sync()
        self._seq += 1
        self._pending.append({'owner': owner, 'msg': msg, 'ready': ready_t, 'sent': sent_t,
                              'rule': rule, 'seq': self._seq, 'gen': self._generation,
                              'tx_seq': self._seq if tx_seq is None else tx_seq})

    def flush_owner(self, owner):
        before = len(self._pending)
        self._pending = [e for e in self._pending if e['owner'] is not owner]
        n = before - len(self._pending)
        if self._in_flight and self._in_flight[0]['owner'] is owner:
            self._in_flight = None
            n += 1
        return n

    def pump(self):
        """Run the wire up to the clock's now; deliver finished frames to their owners."""
        self._sync()
        now = self.clock.now()
        while True:
            if self._in_flight is not None:
                entry, end = self._in_flight
                if end > now + EPS_S:
                    return
                self._in_flight = None
                entry['owner']._wire_done(entry, end)
                continue
            if not self._pending:
                return
            earliest = min(e['ready'] for e in self._pending)
            t0 = earliest if self._wire_free is None else max(self._wire_free, earliest)
            if t0 > now + EPS_S:
                return
            ready = [e for e in self._pending if e['ready'] <= t0 + EPS_S]
            win = min(ready, key=lambda e: (self._priority(e['msg']), e['seq']))
            self._pending.remove(win)
            owner = win['owner']
            counters = owner.counters
            if counters is not None and counters.poll() == BUS_OFF:
                owner._dropped_bus_off(win)
                continue
            ft = self.frame_time_s(win['msg'])
            self.max_frame_time_s = max(self.max_frame_time_s, ft)
            rule = win['rule']
            if rule is not None and rule.tx_error_p > 0.0 and \
                    self._rng.random() < rule.tx_error_p:
                # error detected during the frame: frame bits plus an error frame on the wire,
                # then automatic retransmission
                dur = ft + self.error_frame_bits / self.nominal_bitrate
                self._account(t0, dur, self._bits(win['msg']) + self.error_frame_bits)
                self.error_frames += 1
                self._wire_free = t0 + dur
                if counters is not None:
                    passive = counters.state == ERROR_PASSIVE
                    counters.tx_error(cause='tx_error_injected')
                    if counters.state == BUS_OFF:
                        owner._dropped_bus_off(win)
                        continue
                    if passive:
                        self._wire_free += self.passive_suspend_bits / self.nominal_bitrate
                win['ready'] = self._wire_free
                win['retransmissions'] = win.get('retransmissions', 0) + 1
                self._pending.append(win)
                continue
            self._account(t0, ft, self._bits(win['msg']))
            self.frames += 1
            self._wire_free = t0 + ft
            self._in_flight = (win, t0 + ft)

    def metrics(self):
        return {
            'bus': self.name,
            'nominal_bitrate': self.nominal_bitrate,
            'data_bitrate': self.data_bitrate,
            'fd': self.fd,
            'stuffing': self.stuffing,
            'frames_on_wire': self.frames,
            'bits_on_wire': self.bits,
            'error_frames': self.error_frames,
            'load_percent': self.load_percent(),
            'load_percent_window': self.load_percent(self.load_window_s)
            if self.load_window_s > 0 else None,
            'load_window_s': self.load_window_s,
            'max_frame_time_s': self.max_frame_time_s,
            'flushed_on_reset': self.flushed_on_reset,
            'arbitration_estimate': self.arbitration_delay_estimate(),
        }


# --------------------------------------------------------------------------------------------
# Fault wrapper
# --------------------------------------------------------------------------------------------

def check_virtual_bus(bus):
    """Refuse a bus that is not python-can's virtual bus or SocketCAN on a ``vcan*`` channel.

    This layer never opens an interface. It only wraps one that ``canbus.open_bus`` (or a test)
    opened, and it re-checks the bus it is given (decision D7).
    """
    from can.interfaces.virtual import VirtualBus
    if isinstance(bus, VirtualBus):
        return 'virtual'
    try:
        from can.interfaces.socketcan import SocketcanBus
    except ImportError:  # pragma: no cover - non-Linux
        SocketcanBus = None
    if SocketcanBus is not None and isinstance(bus, SocketcanBus):
        channel = str(getattr(bus, 'channel', '') or '')
        if channel.startswith(VCAN_PREFIX):
            return channel
        raise NotVirtualCan('FaultyBus wraps vcan interfaces only, refused: %r' % channel)
    raise NotVirtualCan('FaultyBus wraps python-can virtual or vcan buses only, refused: %s'
                        % type(bus).__name__)


class _DirStats:
    def __init__(self):
        self.frames_in = 0
        self.frames_out = 0
        self.bytes_out = 0
        self.lost = 0
        self.outage_drops = 0
        self.duplicates = 0
        self.reordered = 0
        self.bus_off_drops = 0
        self.latencies = []
        self._max_seq = 0

    def delivered(self, msg, seq, latency):
        self.frames_out += 1
        self.bytes_out += len(msg.data)
        self.latencies.append(latency)
        if seq < self._max_seq:
            self.reordered += 1
        self._max_seq = max(self._max_seq, seq)

    def as_dict(self):
        return {
            'frames_in': self.frames_in, 'frames_out': self.frames_out,
            'bytes_out': self.bytes_out, 'lost': self.lost, 'outage_drops': self.outage_drops,
            'duplicates': self.duplicates, 'reordered': self.reordered,
            'bus_off_drops': self.bus_off_drops, 'latency': latency_summary(self.latencies)}


def _copy(msg):
    return can.Message(
        timestamp=msg.timestamp, arbitration_id=msg.arbitration_id,
        is_extended_id=msg.is_extended_id, is_remote_frame=msg.is_remote_frame,
        is_error_frame=msg.is_error_frame, channel=msg.channel, dlc=msg.dlc,
        data=bytes(msg.data), is_fd=msg.is_fd, bitrate_switch=msg.bitrate_switch,
        error_state_indicator=msg.error_state_indicator)


class FaultyBus:
    """A python-can bus with faults and timing, on the injected clock.

    ``send()`` queues the frame with its fate (lost, delayed, duplicated); ``poll()`` delivers
    the frames whose time has come (through the ``CanBusModel`` wire when one is given, so they
    also take their frame time and arbitration); ``recv()`` polls, reads what the inner bus has,
    applies the receive rules and returns the first frame that is due in this clock, or None.
    Delivery happens at the first ``poll()`` or ``recv()`` at or after the due time, so the
    latency resolution is the caller's polling period in the same clock.

    ``recv(timeout)`` passes ``timeout`` to the inner bus only when no frame is due; the wait is
    the inner bus's, not a reading of time by this layer.
    """

    def __init__(self, bus, clock, rules=(), seed=0, name='bus', model=None, counters=None,
                 observe_rx=False):
        self.interface = check_virtual_bus(bus)
        self.bus = bus
        self.clock = clock
        self.name = name
        self.seed = seed
        self.rules = [r if isinstance(r, FaultRule) else FaultRule.from_dict(r) for r in rules]
        names = [r.name for r in self.rules]
        if len(set(names)) != len(names):
            raise ValueError('rule names must be unique: %s' % names)
        self.model = model
        if model is not None and model.clock is not clock:
            raise ValueError('a FaultyBus and its CanBusModel must share one clock')
        self.counters = counters
        self.observe_rx = bool(observe_rx)
        self._rngs = {r.name: _rng(seed, name, r.name) for r in self.rules}
        self._bursts = {r.name: BurstSchedule(_rng(seed, name, r.name, 'burst'),
                                              r.burst.get('rate_hz', 0.0),
                                              r.burst.get('duration_s', 0.0))
                        for r in self.rules if r.burst}
        self._tx = []
        self._rx = []
        self._seq = 0
        self._generation = clock.generation
        self.stats = {TX: _DirStats(), RX: _DirStats()}
        self.flushed_on_reset = 0
        self.resets = []
        self.unmatched = {TX: 0, RX: 0}

    # ---- generation (gate D3) ----------------------------------------------------------------
    def _sync(self):
        gen = self.clock.generation
        if gen == self._generation:
            return
        n = len(self._tx) + len(self._rx)
        if self.model is not None:
            n += self.model.flush_owner(self)
        self._tx = []
        self._rx = []
        self.flushed_on_reset += n
        self.resets.append({'t': self.clock.now(), 'from_generation': self._generation,
                            'to_generation': gen, 'flushed': n})
        self._generation = gen
        for b in self._bursts.values():
            b.reset(self.clock.now())

    # ---- fate ------------------------------------------------------------------------------
    def _rule_for(self, msg, direction):
        for r in self.rules:
            if r.matches(msg, direction):
                return r
        self.unmatched[direction] += 1
        return None

    def _fate(self, rule, now, stats):
        """List of delays, one per copy to deliver; empty when the frame is lost."""
        if rule is None:
            return [0.0]
        rng = self._rngs[rule.name]
        burst = self._bursts.get(rule.name)
        if rule.in_fixed_outage(now) or (burst is not None and burst.active(now)):
            stats.outage_drops += 1
            return []
        if rule.loss_p and rng.random() < rule.loss_p:
            stats.lost += 1
            return []
        d = max(0.0, rule.delay_s + rule.sample_jitter(rng))
        if rule.reorder_p and rng.random() < rule.reorder_p:
            d += rng.uniform(0.0, rule.reorder_window_s)
        delays = [d]
        if rule.duplicate_p and rng.random() < rule.duplicate_p:
            stats.duplicates += 1
            delays.append(d + rule.duplicate_gap_s)
        return delays

    def _push(self, heap, due, msg, t_in, rule):
        self._seq += 1
        heapq.heappush(heap, (due, self._seq, msg, t_in, rule, self._generation))

    # ---- transmit --------------------------------------------------------------------------
    def send(self, msg, timeout=None):
        self._sync()
        now = self.clock.now()
        st = self.stats[TX]
        st.frames_in += 1
        if self.counters is not None and self.counters.poll() == BUS_OFF:
            st.bus_off_drops += 1
            return
        rule = self._rule_for(msg, TX)
        for d in self._fate(rule, now, st):
            self._push(self._tx, now + d, _copy(msg), now, rule)
        self.poll()

    def poll(self):
        """Deliver every transmit frame that is due at the clock's now."""
        self._sync()
        now = self.clock.now()
        if self.counters is not None:
            self.counters.poll()
        while self._tx and self._tx[0][0] <= now + EPS_S:
            due, seq, msg, t_in, rule, gen = heapq.heappop(self._tx)
            if gen != self._generation:
                continue
            if self.model is not None:
                self.model.submit(self, msg, due, t_in, rule, tx_seq=seq)
            else:
                self._transmit(msg, seq, t_in, now)
        if self.model is not None:
            self.model.pump()

    def _transmit(self, msg, seq, t_in, t_out):
        self.bus.send(msg)
        self.stats[TX].delivered(msg, seq, t_out - t_in)
        if self.counters is not None:
            self.counters.tx_success()

    def _wire_done(self, entry, end):
        if entry['gen'] != self._generation:
            return
        self._transmit(entry['msg'], entry['tx_seq'], entry['sent'], end)

    def _dropped_bus_off(self, entry):
        self.stats[TX].bus_off_drops += 1

    # ---- receive ---------------------------------------------------------------------------
    def _take_rx(self, now):
        while self._rx and self._rx[0][0] <= now + EPS_S:
            due, seq, msg, t_in, rule, gen = heapq.heappop(self._rx)
            if gen != self._generation:
                continue
            self.stats[RX].delivered(msg, seq, now - t_in)
            return msg
        return None

    def _ingest(self, msg):
        now = self.clock.now()
        st = self.stats[RX]
        st.frames_in += 1
        if self.counters is not None:
            if self.counters.poll() == BUS_OFF:
                st.bus_off_drops += 1
                return
            self.counters.rx_success()
        if self.model is not None and self.observe_rx:
            self.model.observe(msg)
        rule = self._rule_for(msg, RX)
        for d in self._fate(rule, now, st):
            self._push(self._rx, now + d, _copy(msg), now, rule)

    def recv(self, timeout=None):
        self.poll()
        now = self.clock.now()
        while True:
            m = self.bus.recv(timeout=0.0)
            if m is None:
                break
            self._ingest(m)
        got = self._take_rx(now)
        if got is not None or not timeout:
            return got
        m = self.bus.recv(timeout=timeout)
        self._sync()
        if m is not None:
            self._ingest(m)
        self.poll()
        return self._take_rx(self.clock.now())

    # ---- python-can surface ------------------------------------------------------------------
    @property
    def channel_info(self):
        return 'FaultyBus(%s) on %s' % (self.name, getattr(self.bus, 'channel_info', '?'))

    def pending(self):
        return {TX: len(self._tx), RX: len(self._rx)}

    def shutdown(self):
        self.bus.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.shutdown()

    # ---- fault injection API -----------------------------------------------------------------
    def inject_bus_off(self, cause='injected'):
        if self.counters is None:
            raise RuntimeError('bus-off injection needs CanErrorCounters on this FaultyBus')
        self.counters.force_bus_off(cause)

    def request_recovery(self):
        if self.counters is not None:
            self.counters.request_recovery()

    # ---- metrics (DT-12) ---------------------------------------------------------------------
    def metrics(self):
        out = {
            'bus': self.name,
            'interface': self.interface,
            'seed': self.seed,
            'generation': self._generation,
            'rules': [r.name for r in self.rules],
            'unmatched_frames': dict(self.unmatched),
            'tx': self.stats[TX].as_dict(),
            'rx': self.stats[RX].as_dict(),
            'flushed_on_reset': self.flushed_on_reset,
            'resets': list(self.resets),
        }
        out['frames'] = out['tx']['frames_out'] + out['rx']['frames_out']
        out['bytes'] = out['tx']['bytes_out'] + out['rx']['bytes_out']
        out['wire'] = self.model.metrics() if self.model is not None else None
        out['load_percent'] = out['wire']['load_percent'] if out['wire'] else None
        out['error_state'] = self.counters.metrics() if self.counters is not None else None
        return out


# --------------------------------------------------------------------------------------------
# UART
# --------------------------------------------------------------------------------------------

class UartFaultModel:
    """Faults on a UART byte stream, timed on the injected clock.

    ``apply(data)`` returns the bytes the receiver reads. Byte ``i`` is on the line at
    ``t_start + i * char_time_s`` (``t_start`` defaults to the clock's now). Per byte, in order:

    - noise burst active: the byte is received with random bit errors (a non-zero XOR mask);
    - baud mismatch: the receiver samples the stop bit ``(bits_per_char - 0.5) * |mismatch|``
      bit times off; at half a bit or more every byte is a framing error (tolerance
      0.5 / (bits_per_char - 0.5), 5.26 % for 8N1, both ends together, ideal sampling);
    - byte loss with ``byte_loss_p`` (an overrun or a dropped byte);
    - framing error with ``framing_error_p``; parity error with ``parity_error_p`` (only with
      parity configured).

    A framing or parity error byte follows ``error_byte_policy``: ``null`` reads b'\\x00'
    (Linux termios with IGNPAR and PARMRK off), ``drop`` removes it (IGNPAR), ``mark`` reads
    b'\\xff\\x00' and the byte (PARMRK). A new clock generation restarts the burst schedule.
    """

    POLICIES = ('null', 'drop', 'mark')

    def __init__(self, clock, baud, data_bits=8, parity='N', stop_bits=1, seed=0, name='uart',
                 byte_loss_p=0.0, framing_error_p=0.0, parity_error_p=0.0,
                 noise_burst_rate_hz=0.0, noise_burst_duration_s=0.0, baud_mismatch=0.0,
                 error_byte_policy='null'):
        if parity not in ('N', 'E', 'O'):
            raise ValueError('parity must be N, E or O')
        if error_byte_policy not in self.POLICIES:
            raise ValueError('error_byte_policy must be one of %s' % (self.POLICIES,))
        self.clock = clock
        self.name = name
        self.seed = seed
        self.baud = float(baud)
        self.data_bits = int(data_bits)
        self.parity = parity
        self.stop_bits = float(stop_bits)
        self.bits_per_char = 1 + self.data_bits + (0 if parity == 'N' else 1) + self.stop_bits
        self.char_time_s = self.bits_per_char / self.baud
        self.byte_loss_p = _prob('byte_loss_p', byte_loss_p)
        self.framing_error_p = _prob('framing_error_p', framing_error_p)
        self.parity_error_p = _prob('parity_error_p', parity_error_p)
        if parity == 'N' and self.parity_error_p > 0:
            raise ValueError('parity_error_p needs a parity bit (parity N has none)')
        self.baud_mismatch = float(baud_mismatch)
        self.error_byte_policy = error_byte_policy
        self._rng = _rng(seed, name, 'bytes')
        self._burst = BurstSchedule(_rng(seed, name, 'noise'), noise_burst_rate_hz,
                                    noise_burst_duration_s)
        self._generation = clock.generation
        self.stats = {'bytes_in': 0, 'bytes_out': 0, 'lost': 0, 'framing_errors': 0,
                      'parity_errors': 0, 'noise_corrupted': 0, 'baud_errors': 0}

    @property
    def mismatch_tolerance(self):
        return 0.5 / (self.bits_per_char - 0.5)

    def _error_byte(self, out, b):
        if self.error_byte_policy == 'null':
            out.append(0)
        elif self.error_byte_policy == 'mark':
            out.extend((0xFF, 0x00, b))

    def apply(self, data, t_start=None):
        if self.clock.generation != self._generation:
            self._generation = self.clock.generation
            self._burst.reset(self.clock.now())
        t0 = self.clock.now() if t_start is None else float(t_start)
        out = bytearray()
        mask_all = (1 << self.data_bits) - 1
        baud_bad = abs(self.baud_mismatch) * (self.bits_per_char - 0.5) >= 0.5
        for i, b in enumerate(bytes(data)):
            self.stats['bytes_in'] += 1
            t = t0 + i * self.char_time_s
            if self._burst.active(t):
                b ^= self._rng.randint(1, mask_all)
                self.stats['noise_corrupted'] += 1
            if baud_bad:
                self.stats['baud_errors'] += 1
                self._error_byte(out, b)
                continue
            if self.byte_loss_p and self._rng.random() < self.byte_loss_p:
                self.stats['lost'] += 1
                continue
            if self.framing_error_p and self._rng.random() < self.framing_error_p:
                self.stats['framing_errors'] += 1
                self._error_byte(out, b)
                continue
            if self.parity_error_p and self._rng.random() < self.parity_error_p:
                self.stats['parity_errors'] += 1
                self._error_byte(out, b)
                continue
            out.append(b)
        self.stats['bytes_out'] += len(out)
        return bytes(out)

    def metrics(self):
        m = dict(self.stats)
        m.update({'uart': self.name, 'seed': self.seed, 'baud': self.baud,
                  'format': '%d%s%g' % (self.data_bits, self.parity, self.stop_bits),
                  'char_time_s': self.char_time_s, 'noise_bursts': self._burst.count,
                  'baud_mismatch': self.baud_mismatch,
                  'mismatch_tolerance': self.mismatch_tolerance})
        return m


# --------------------------------------------------------------------------------------------
# Configuration (config/models/bus_faults.yaml)
# --------------------------------------------------------------------------------------------

PARAMETER_FIELDS = ('value', 'unit', 'source', 'confidence', 'uncertainty', 'hardware_revision',
                    'owner', 'test', 'last_verified', 'measurement_item')


def is_parameter(node):
    return isinstance(node, dict) and set(PARAMETER_FIELDS) <= set(node)


def values(node):
    """Strip the parameter records of a configuration subtree to their values."""
    if is_parameter(node):
        return node['value']
    if isinstance(node, dict):
        return {k: values(v) for k, v in node.items()}
    if isinstance(node, list):
        return [values(v) for v in node]
    return node


def load_config(path):
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)


def build_can(config, bus_name, bus, clock, profile='nominal', seed=None):
    """FaultyBus with its CanBusModel and CanErrorCounters for one bus of the configuration."""
    cfg = values(config['can_buses'][bus_name])
    em = values(config['error_model'])
    prof = config['profiles'][profile]
    seed = prof.get('seed', 0) if seed is None else seed
    model = CanBusModel(bus_name, clock, cfg['nominal_bitrate'], cfg.get('data_bitrate'),
                        fd=cfg['frame_format'] == 'fd', stuffing=cfg['stuffing'],
                        error_frame_bits=em['error_frame_bits'],
                        passive_suspend_bits=em['passive_suspend_bits'], seed=seed)
    recovery = cfg.get('bus_off_recovery') or em['default_bus_off_recovery']
    counters = CanErrorCounters(
        clock, cfg['nominal_bitrate'], recovery=recovery,
        tx_error_increment=em['tec_tx_error_increment'],
        success_decrement=em['success_decrement'],
        rx_error_increment=em['rec_rx_error_increment'],
        rx_error_severe_increment=em['rec_rx_error_severe_increment'],
        passive_threshold=em['error_passive_threshold'],
        bus_off_threshold=em['bus_off_threshold'],
        recovery_sequences=em['recovery_sequences'],
        recovery_sequence_bits=em['recovery_sequence_bits'],
        rec_passive_success_value=em['rec_passive_success_value'], name=bus_name)
    rules = (prof.get('can') or {}).get(bus_name, [])
    return FaultyBus(bus, clock, rules=rules, seed=seed, name=bus_name, model=model,
                     counters=counters)


def build_uart(config, uart_name, clock, profile='nominal', seed=None):
    cfg = values(config['uarts'][uart_name])
    prof = config['profiles'][profile]
    seed = prof.get('seed', 0) if seed is None else seed
    faults = dict((prof.get('uart') or {}).get(uart_name, {}))
    return UartFaultModel(clock, cfg['baud'], data_bits=cfg['data_bits'], parity=cfg['parity'],
                          stop_bits=cfg['stop_bits'], seed=seed, name=uart_name, **faults)
