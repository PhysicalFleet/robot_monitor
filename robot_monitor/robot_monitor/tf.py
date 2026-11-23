# Copyright 2026 The Robot Monitor Authors
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
TF tree tracking from raw /tf and /tf_static messages.

Frame names are read straight from the CDR bytes of tf2_msgs/TFMessage, so even a
/tf stream of thousands of transforms per second costs little: no Python message
objects are built.
"""

from collections import deque
import struct
import time

_CDR2_IDS = {0x0006, 0x0007, 0x000a, 0x000b, 0x0010, 0x0011}


def parse_tf_frames(raw):
    """
    Return [(parent, child, stamp_sec_float), ...] from a serialized TFMessage.

    Returns None if the bytes don't look like a TFMessage.
    """
    if len(raw) < 8:
        return None
    rep = (raw[0] << 8) | raw[1]
    le = raw[1] & 1
    e = '<' if le else '>'
    max_align = 4 if rep in _CDR2_IDS else 8
    data = memoryview(raw)[4:]
    n_bytes = len(data)
    pos = 0

    def align(p, a):
        a = min(a, max_align)
        return (p + a - 1) & ~(a - 1)

    def read_u32(p):
        p = align(p, 4)
        if p + 4 > n_bytes:
            raise ValueError
        return struct.unpack_from(e + 'I', data, p)[0], p + 4

    def read_str(p):
        ln, p = read_u32(p)
        if ln > 4096 or p + ln > n_bytes:
            raise ValueError
        s = bytes(data[p:p + ln]).rstrip(b'\0').decode('utf-8', 'replace')
        return s, p + ln

    out = []
    try:
        count, pos = read_u32(pos)
        if count > 100000:
            return None
        for _ in range(count):
            pos = align(pos, 4)
            if pos + 8 > n_bytes:
                return None
            sec, nsec = struct.unpack_from(e + 'iI', data, pos)
            pos += 8
            parent, pos = read_str(pos)
            child, pos = read_str(pos)
            pos = align(pos, 8) + 7 * 8   # translation (3 doubles) + rotation (4 doubles)
            if pos > n_bytes:
                return None
            out.append((parent.lstrip('/'), child.lstrip('/'), sec + nsec * 1e-9))
    except (ValueError, struct.error):
        return None
    return out


class _Frame:
    __slots__ = ('child', 'parent', 'static', 'times', 'parents', 'last', 'stamp', 'both')

    def __init__(self, child):
        self.child = child
        self.parent = None
        self.static = False
        self.times = deque(maxlen=400)   # monotonic arrival times (dynamic frames)
        self.parents = {}                # parent -> last monotonic time seen with it
        self.last = None
        self.stamp = None
        self.both = False                # seen on /tf_static and on /tf


class TfTracker:
    """The current TF tree, per-frame rates and staleness."""

    def __init__(self):
        self.frames = {}

    def on_message(self, raw, static, now=None):
        now = time.monotonic() if now is None else now
        items = parse_tf_frames(raw)
        if not items:
            return
        for parent, child, stamp in items:
            if not child:
                continue
            f = self.frames.get(child)
            if f is None:
                if len(self.frames) > 5000:
                    continue
                f = self.frames[child] = _Frame(child)
            if static:
                if f.times and not f.static:
                    f.both = True
                f.static = True
            else:
                if f.static:
                    f.both = True
                f.times.append(now)
            f.parent = parent
            f.parents[parent] = now
            f.last = now
            f.stamp = stamp

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        for child in [c for c, f in self.frames.items()
                      if f.last is not None and now - f.last > 600 and not f.static]:
            del self.frames[child]   # nobody has published it for 10 minutes
        frames = []
        for f in self.frames.values():
            recent = [t for t in f.times if t >= now - 10.0]
            hz = 0.0
            if len(recent) >= 2 and recent[-1] > recent[0]:
                hz = (len(recent) - 1) / max(recent[-1] - recent[0], now - recent[0], 1e-6)
            period = None
            if len(f.times) >= 10:
                ts = list(f.times)[-50:]
                period = (ts[-1] - ts[0]) / (len(ts) - 1) if ts[-1] > ts[0] else None
            # Parents seen recently (dynamic) or ever (static): two at once breaks lookups.
            window = 1e9 if f.static and not f.times else 10.0
            parents = sorted(p for p, t in f.parents.items() if t >= now - window)
            frames.append({
                'frame': f.child, 'parent': f.parent, 'static': f.static and not f.times,
                'hz': round(hz, 2), 'age_s': None if f.last is None else round(now - f.last, 2),
                'period_s': None if period is None else round(period, 4),
                'parents': parents, 'both': f.both,
            })
        frames.sort(key=lambda x: x['frame'])
        children = {fr['frame'] for fr in frames}
        roots = sorted({fr['parent'] for fr in frames
                        if fr['parent'] and fr['parent'] not in children})
        return {'frames': frames, 'roots': roots}
