# Copyright 2026 PhysicalFleet
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Per-topic statistics computed from raw (still serialized) messages.

Messages are never deserialized: rate, bandwidth, size and gaps come from the
arrival time and byte length, and the header stamp (when the type starts with a
std_msgs/Header) is read straight from the CDR bytes. That keeps the cost per
message to a few microseconds of Python, even for images and point clouds.
"""

from collections import deque
import math
import struct
import time

WINDOW_S = 10.0          # rate / jitter / bandwidth window
MAX_SAMPLES = 20000      # hard cap on per-topic memory


def stamp_reader(msg_class):
    """
    Return a function raw_bytes -> (sec, nanosec) or None, for this message type.

    Handles types whose first field is std_msgs/Header (sensor_msgs, nav_msgs,
    geometry_msgs/*Stamped ...) or builtin_interfaces/Time, and tf2_msgs/TFMessage
    style types (a sequence whose element starts with a Header).
    """
    try:
        fields = list(msg_class.get_fields_and_field_types().items())
    except Exception:
        return None
    if not fields:
        return None
    _, first = fields[0]
    if first in ('std_msgs/Header', 'builtin_interfaces/Time'):
        return lambda raw: _cdr_time(raw, 4)
    if first.startswith('sequence<') and first.endswith('>'):
        inner = first[len('sequence<'):-1].split(',')[0]
        if '/' in inner:
            try:
                import importlib
                pkg, name = inner.split('/')
                mod = importlib.import_module(pkg + '.msg')
                inner_cls = getattr(mod, name)
                inner_first = list(inner_cls.get_fields_and_field_types().values())[0]
            except Exception:
                return None
            if inner_first == 'std_msgs/Header':
                return _seq_first_time
    return None


def _endian(raw):
    # CDR encapsulation header: byte 1 odd -> little endian (CDR_LE=1, PL_CDR_LE=3,
    # CDR2_LE=7 ...), even -> big endian.
    return '<' if raw[1] & 1 else '>'


def _cdr_time(raw, off):
    if len(raw) < off + 8:
        return None
    sec, nsec = struct.unpack_from(_endian(raw) + 'iI', raw, off)
    return sec, nsec


def _seq_first_time(raw):
    if len(raw) < 16:
        return None
    (n,) = struct.unpack_from(_endian(raw) + 'I', raw, 4)
    if n == 0:
        return None
    return _cdr_time(raw, 8)


class TopicStats:
    """Rolling statistics for one topic."""

    # Subscription callbacks only append to `arrivals` / `ages` (the hot path, run for
    # every message); everything else is derived once per snapshot in compute().
    __slots__ = ('arrivals', 'ages', 'total_msgs', '_counted_t', 'rate_hist', 'learned_hz',
                 'periodic', '_learn_t')

    def __init__(self):
        self.arrivals = deque(maxlen=MAX_SAMPLES)  # (t_monotonic, size)
        self.ages = deque(maxlen=2000)            # (t_monotonic, age_s)
        self.total_msgs = 0
        self._counted_t = -1.0
        self.rate_hist = deque(maxlen=120)          # one rate value per snapshot (sparkline)
        self.learned_hz = None                      # slow EWMA of the steady rate
        self.periodic = None                        # True / False / None (unknown)
        self._learn_t = None

    def on_message(self, size, now=None, age=None):
        now = time.monotonic() if now is None else now
        self.arrivals.append((now, size))
        if age is not None:
            self.ages.append((now, age))

    @property
    def last_seen(self):
        return self.arrivals[-1][0] if self.arrivals else None

    def compute(self, now=None, window=WINDOW_S):
        """Return a dict of current statistics over the last `window` seconds."""
        now = time.monotonic() if now is None else now
        cutoff = now - window
        arr = self.arrivals
        while arr and arr[0][0] < now - window - 2.0:
            arr.popleft()
        recent = [a for a in arr if a[0] >= cutoff]
        new = 0
        for t, _ in reversed(arr):
            if t <= self._counted_t:
                break
            new += 1
        self.total_msgs += new
        if arr:
            self._counted_t = arr[-1][0]
        last = self.last_seen
        out = {'hz': 0.0, 'bw': 0.0, 'size': arr[-1][1] if arr else 0, 'jitter_ms': None,
               'max_gap_ms': None, 'msgs': self.total_msgs,
               'silent_s': None if last is None else round(now - last, 2)}
        if len(recent) >= 2:
            span = recent[-1][0] - recent[0][0]
            # Measure over the time we actually observed: up to now, but no further
            # back than the window or the first message.
            observed = min(window, now - recent[0][0]) if span > 0 else 0
            ivals = [b[0] - a[0] for a, b in zip(recent, recent[1:])]
            if span > 0:
                out['hz'] = (len(recent) - 1) / span
                # if the topic went quiet since, decay the rate toward zero
                if observed > span * 1.5 and now - recent[-1][0] > 2 * (span / (len(recent) - 1)):
                    out['hz'] = (len(recent) - 1) / observed
            out['bw'] = sum(s for _, s in recent) / max(observed, 1e-6) if observed else 0.0
            mean = sum(ivals) / len(ivals)
            if len(ivals) >= 3:
                var = sum((x - mean) ** 2 for x in ivals) / (len(ivals) - 1)
                out['jitter_ms'] = round(math.sqrt(var) * 1000.0, 2)
                cv = math.sqrt(var) / mean if mean > 0 else 99
                if len(ivals) >= 10:
                    self.periodic = cv < 0.6
            out['max_gap_ms'] = round(max(ivals) * 1000.0, 1)
        elif len(recent) == 1:
            out['bw'] = recent[0][1] / window
        ages = [a for t, a in self.ages if t >= cutoff]
        if ages:
            out['age_ms'] = round(1000.0 * sum(ages) / len(ages), 1)
            out['age_max_ms'] = round(1000.0 * max(ages), 1)
        # Learn the steady rate slowly (time constant ~2 min, whatever the snapshot rate).
        if out['hz'] > 0 and self.periodic:
            if self.learned_hz is None:
                self.learned_hz = out['hz']
            else:
                dt = min(30.0, now - self._learn_t) if self._learn_t is not None else 1.0
                self.learned_hz += (out['hz'] - self.learned_hz) * min(1.0, dt / 120.0)
            self._learn_t = now
        out['learned_hz'] = None if self.learned_hz is None else round(self.learned_hz, 2)
        out['periodic'] = self.periodic
        self.rate_hist.append(float('%.3g' % out['hz']))
        return out
