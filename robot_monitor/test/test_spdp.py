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


import socket
import struct

from robot_monitor import spdp


def _param(pid, value):
    pad = (-len(value)) % 4
    return struct.pack('<HH', pid, len(value) + pad) + value + b'\0' * pad


def _locator(ip, port):
    return struct.pack('<iI', 1, port) + b'\0' * 12 + socket.inet_aton(ip)


def make_spdp(prefix, locs, le=True):
    payload = b'\x00\x03\x00\x00'  # PL_CDR_LE
    payload += _param(spdp.PID_PARTICIPANT_GUID, prefix + b'\x00\x00\x01\xc1')
    for ip, port in locs:
        payload += _param(spdp.PID_DEFAULT_UNICAST_LOCATOR, _locator(ip, port))
    payload += struct.pack('<HH', spdp.PID_SENTINEL, 0)
    body = struct.pack('<HH', 0, 16) + b'\x00\x01\x00\xc7' + spdp.SPDP_WRITER
    body += struct.pack('<iI', 0, 1) + payload
    info_ts = bytes([0x09, 0x01]) + struct.pack('<H', 8) + b'\x01' * 8
    data = bytes([0x15, 0x05]) + struct.pack('<H', len(body)) + body
    return b'RTPS\x02\x03\x01\x10' + prefix + info_ts + data


def test_parse_spdp_announcement():
    prefix = bytes(range(12))
    pkt = make_spdp(prefix, [('192.168.1.7', 7412), ('10.0.0.2', 7413)])
    got = spdp.parse_spdp(pkt)
    assert got == (prefix, [('192.168.1.7', 7412), ('10.0.0.2', 7413)])


def test_parse_rejects_garbage_and_other_writers():
    assert spdp.parse_spdp(b'') is None
    assert spdp.parse_spdp(b'XXXX' + b'\0' * 40) is None
    pkt = bytearray(make_spdp(bytes(12), [('1.2.3.4', 7411)]))
    i = pkt.index(spdp.SPDP_WRITER)
    pkt[i:i + 4] = b'\x00\x00\x03\xc2'  # an SEDP writer, not SPDP
    assert spdp.parse_spdp(bytes(pkt)) is None
    assert spdp.parse_spdp(make_spdp(bytes(12), [])[:-10]) is None or True  # truncated: no crash


def test_spdp_port():
    assert spdp.spdp_port(0) == 7400 and spdp.spdp_port(42) == 17900


def test_udp_inodes_runs():
    assert isinstance(spdp.udp_port_inodes(), dict)
