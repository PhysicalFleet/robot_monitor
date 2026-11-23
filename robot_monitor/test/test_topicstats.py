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

import struct

from robot_monitor.topicstats import stamp_reader, TopicStats


def test_rate_and_bandwidth():
    s = TopicStats()
    for i in range(101):
        s.on_message(1000, now=100.0 + i * 0.1)
    out = s.compute(now=110.0)
    assert abs(out['hz'] - 10.0) < 0.2
    assert 9000 < out['bw'] < 11000
    assert out['jitter_ms'] < 1.0
    assert out['periodic'] is True


def test_rate_decays_when_silent():
    s = TopicStats()
    for i in range(50):
        s.on_message(10, now=100.0 + i * 0.1)
    assert s.compute(now=104.95)['hz'] > 9
    assert s.compute(now=109.0)['hz'] < 6
    assert s.compute(now=120.0)['hz'] == 0.0


def test_event_driven_is_not_periodic():
    s = TopicStats()
    t = 100.0
    for gap in (0.1, 2.0, 0.05, 3.0, 0.2, 0.1, 4.0, 0.3, 1.5, 0.05, 2.5, 0.1):
        t += gap
        s.on_message(10, now=t)
    assert s.compute(now=t, window=30.0)['periodic'] is False


def _cdr_header(sec, nsec, le=True):
    enc = b'\x00\x01\x00\x00' if le else b'\x00\x00\x00\x00'
    return enc + struct.pack('<iI' if le else '>iI', sec, nsec) + b'\x05\x00\x00\x00base\x00'


def test_stamp_reader_header_types():
    from sensor_msgs.msg import Imu
    r = stamp_reader(Imu)
    assert r(_cdr_header(1700000000, 5)) == (1700000000, 5)
    assert r(_cdr_header(1700000000, 5, le=False)) == (1700000000, 5)


def test_stamp_reader_matches_real_serialization():
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan
    m = LaserScan()
    m.header.stamp.sec, m.header.stamp.nanosec = 1234567, 890
    m.header.frame_id = 'laser'
    assert stamp_reader(LaserScan)(serialize_message(m)) == (1234567, 890)


def test_stamp_reader_tf_message():
    from rclpy.serialization import serialize_message
    try:
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
    except ImportError:
        return
    m = TFMessage(transforms=[TransformStamped()])
    m.transforms[0].header.stamp.sec = 42
    assert stamp_reader(TFMessage)(serialize_message(m)) == (42, 0)
    assert stamp_reader(TFMessage)(serialize_message(TFMessage())) is None


def test_stamp_reader_none_for_headerless():
    from std_msgs.msg import String
    assert stamp_reader(String) is None
