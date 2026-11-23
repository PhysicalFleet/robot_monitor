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
Passive DDS discovery listener: which process owns which DDS participant.

Every DDS participant (one per ROS 2 process) multicasts an SPDP announcement with its
GUID prefix and the unicast UDP ports it listens on. Matching those ports against the
sockets in /proc/net/udp gives the PID, for any DDS vendor (Cyclone DDS, Fast DDS,
Connext...). For participants on other machines it gives their IP address.

Safety: the socket is bound to the multicast group address itself (not INADDR_ANY), so
it only ever receives multicast copies; it can never take a unicast discovery packet
meant for one of the robot's own processes. It never sends anything.
"""

import os
import socket
import struct
import threading
import time

SPDP_GROUP = '239.255.0.1'
PID_SENTINEL = 0x0001
PID_PARTICIPANT_GUID = 0x0050
PID_DEFAULT_UNICAST_LOCATOR = 0x0031
PID_METATRAFFIC_UNICAST_LOCATOR = 0x0032
SPDP_WRITER = b'\x00\x01\x00\xc2'
LOCATOR_KIND_UDPV4 = 1


def spdp_port(domain):
    return 7400 + 250 * int(domain)


def parse_spdp(data):
    """
    Parse one RTPS datagram.

    Returns (guid_prefix, [(ip, port), ...]) for an SPDP participant announcement, or None.
    """
    if len(data) < 20 or data[:4] != b'RTPS':
        return None
    prefix = bytes(data[8:20])
    off = 20
    while off + 4 <= len(data):
        sid, flags = data[off], data[off + 1]
        le = flags & 1
        (length,) = struct.unpack_from('<H' if le else '>H', data, off + 2)
        body = off + 4
        end = len(data) if length == 0 else body + length
        if end > len(data):
            return None
        if sid == 0x15:  # DATA
            res = _parse_data(data, body, end, le, flags)
            if res is not None:
                guid, locs = res
                return (guid or prefix), locs
        off = end
    return None


def _parse_data(data, body, end, le, flags):
    if body + 20 > end:
        return None
    e = '<' if le else '>'
    (to_inline,) = struct.unpack_from(e + 'H', data, body + 2)
    writer_id = bytes(data[body + 8:body + 12])
    if writer_id != SPDP_WRITER:
        return None
    pos = body + 4 + to_inline
    if flags & 0x02:  # inline QoS present: skip its parameter list
        pos = _skip_params(data, pos, end, e)
        if pos is None:
            return None
    if not flags & 0x04 or pos + 4 > end:  # no serialized payload
        return None
    rep = data[pos:pos + 2]
    ple = rep in (b'\x00\x03', b'\x00\x01')  # PL_CDR_LE / CDR_LE
    pe = '<' if ple else '>'
    pos += 4
    guid, locs = None, []
    while pos + 4 <= end:
        pid, plen = struct.unpack_from(pe + 'HH', data, pos)
        pos += 4
        if pid == PID_SENTINEL:
            break
        val = data[pos:pos + plen]
        pid &= 0x3fff
        if pid == PID_PARTICIPANT_GUID and plen >= 12:
            guid = bytes(val[:12])
        elif pid in (PID_DEFAULT_UNICAST_LOCATOR, PID_METATRAFFIC_UNICAST_LOCATOR) and plen >= 24:
            kind, port = struct.unpack_from(pe + 'iI', val, 0)
            if kind == LOCATOR_KIND_UDPV4 and 0 < port < 65536:
                locs.append((socket.inet_ntoa(bytes(val[20:24])), port))
        pos += plen
    return guid, locs


def _skip_params(data, pos, end, e):
    while pos + 4 <= end:
        pid, plen = struct.unpack_from(e + 'HH', data, pos)
        pos += 4 + plen
        if pid == PID_SENTINEL:
            return pos
    return None


def udp_port_inodes():
    """Map local UDP port -> set of socket inodes, from /proc/net/udp and udp6."""
    out = {}
    for path in ('/proc/net/udp', '/proc/net/udp6'):
        try:
            with open(path) as f:
                next(f)
                for line in f:
                    parts = line.split()
                    if len(parts) < 10:
                        continue
                    port = int(parts[1].rsplit(':', 1)[1], 16)
                    out.setdefault(port, set()).add(int(parts[9]))
        except (OSError, ValueError, StopIteration):
            continue
    return out


def socket_inodes_of(pid):
    inodes = set()
    try:
        fds = os.listdir('/proc/%d/fd' % pid)
    except OSError:
        return inodes
    for fd in fds:
        try:
            link = os.readlink('/proc/%d/fd/%s' % (pid, fd))
        except OSError:
            continue
        if link.startswith('socket:['):
            inodes.add(int(link[8:-1]))
    return inodes


def local_ipv4s():
    ips = {'127.0.0.1'}
    try:
        with open('/proc/net/fib_trie') as f:
            last = None
            for line in f:
                line = line.strip()
                if line.startswith('|--'):
                    last = line[4:]
                elif '/32 host LOCAL' in line and last:
                    ips.add(last)
    except OSError:
        pass
    return ips


class SpdpListener:
    """Collects participant announcements in a background thread."""

    def __init__(self, domain=None):
        if domain is None:
            try:
                domain = int(os.environ.get('ROS_DOMAIN_ID') or 0)
            except ValueError:
                domain = 0
        self.domain = domain
        self.participants = {}   # guid prefix -> {'locs': [(ip, port)], 't': monotonic}
        self.error = None
        self._sock = None
        self._lock = threading.Lock()
        self._pid_cache = {}     # guid prefix -> pid
        self._last_scan = 0.0
        self._local_ips = local_ipv4s()

    def start(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, 'SO_REUSEPORT'):
                try:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except OSError:
                    pass
            # Bound to the group address: only multicast datagrams reach this socket.
            s.bind((SPDP_GROUP, spdp_port(self.domain)))
            joined = 0
            for ip in sorted(self._local_ips):
                try:
                    mreq = socket.inet_aton(SPDP_GROUP) + socket.inet_aton(ip)
                    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                    joined += 1
                except OSError:
                    pass
            if not joined:
                raise OSError('could not join the SPDP multicast group')
            s.settimeout(1.0)
            self._sock = s
        except OSError as e:
            self.error = str(e)
            return False
        threading.Thread(target=self._run, name='robot_monitor-spdp', daemon=True).start()
        return True

    def _run(self):
        while True:
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                res = parse_spdp(data)
            except Exception:
                res = None
            if res is None:
                continue
            prefix, locs = res
            with self._lock:
                self.participants[prefix] = {'locs': locs, 't': time.monotonic()}
                if len(self.participants) > 5000:
                    old = sorted(self.participants.items(), key=lambda kv: kv[1]['t'])
                    for k, _ in old[:1000]:
                        del self.participants[k]

    def lookup(self, prefixes, candidate_pids):
        """
        Resolve GUID prefixes to processes.

        Returns {prefix: ('pid', pid) | ('remote', ip)} for the prefixes we know about.
        candidate_pids: processes whose sockets are worth scanning (ROS processes).
        """
        with self._lock:
            known = {p: dict(self.participants[p]) for p in prefixes if p in self.participants}
        out, todo = {}, {}
        for prefix, info in known.items():
            ips = {ip for ip, _ in info['locs']}
            if ips and not ips & self._local_ips:
                out[prefix] = ('remote', sorted(ips)[0])
                continue
            pid = self._pid_cache.get(prefix)
            if pid and os.path.exists('/proc/%d' % pid):
                out[prefix] = ('pid', pid)
                continue
            todo[prefix] = {port for ip, port in info['locs'] if ip in self._local_ips}
        if todo and time.monotonic() - self._last_scan > 10.0:
            self._last_scan = time.monotonic()
            ports = udp_port_inodes()
            owners = {}
            for pid in candidate_pids:
                for ino in socket_inodes_of(pid):
                    owners[ino] = pid
            for prefix, wanted in todo.items():
                pids = {owners[ino] for port in wanted for ino in ports.get(port, ())
                        if ino in owners}
                if len(pids) == 1:
                    pid = pids.pop()
                    self._pid_cache[prefix] = pid
                    out[prefix] = ('pid', pid)
        return out
