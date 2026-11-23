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


from robot_monitor.health import HealthEngine
from robot_monitor.tf import parse_tf_frames, TfTracker


def _tf_bytes(pairs):
    from geometry_msgs.msg import TransformStamped
    from rclpy.serialization import serialize_message
    from tf2_msgs.msg import TFMessage
    msg = TFMessage()
    for parent, child in pairs:
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.header.stamp.sec, t.header.stamp.nanosec = 100, 500000000
        t.transform.translation.x = 1.5
        t.transform.rotation.w = 1.0
        msg.transforms.append(t)
    return serialize_message(msg)


def test_parse_real_serialization():
    raw = _tf_bytes([('odom', 'base_link'), ('/base_link', 'a_much_longer_frame_name_x')])
    assert parse_tf_frames(raw) == [('odom', 'base_link', 100.5),
                                    ('base_link', 'a_much_longer_frame_name_x', 100.5)]
    assert parse_tf_frames(_tf_bytes([])) == []


def test_parse_garbage():
    assert parse_tf_frames(b'') is None
    assert parse_tf_frames(b'\x00\x01\x00\x00\xff\xff\xff\xff') is None
    raw = _tf_bytes([('odom', 'base_link')])
    assert parse_tf_frames(raw[:30]) is None


def model(tf):
    return {'nodes': {}, 'topics': {}, 'procs': {}, 'host': {'supported': True}, 'gone': {},
            'env': {}, 'uptime': 100, 'self': {'cpu': 1}, 'log_counts': {}, 'diagnostics': [],
            'expect': None, 'tf': tf}


def test_two_parents_and_static_both():
    tr = TfTracker()
    tr.on_message(_tf_bytes([('odom', 'base_link')]), False, now=10.0)
    tr.on_message(_tf_bytes([('map', 'base_link')]), False, now=10.5)
    tr.on_message(_tf_bytes([('base_link', 'laser')]), True, now=10.5)
    tr.on_message(_tf_bytes([('base_link', 'laser')]), False, now=11.0)
    snap = tr.snapshot(now=11.0)
    f = {x['frame']: x for x in snap['frames']}
    assert f['base_link']['parents'] == ['map', 'odom']
    assert f['laser']['both'] is True
    ids = {i['id'] for i in HealthEngine().evaluate(model(snap), now=0)}
    assert 'tf:parents:base_link' in ids and 'tf:both:laser' in ids


def test_stale_and_rates_and_roots():
    tr = TfTracker()
    for i in range(50):
        tr.on_message(_tf_bytes([('odom', 'base_link')]), False, now=100.0 + i * 0.02)
    tr.on_message(_tf_bytes([('world', 'table')]), True, now=100.0)
    snap = tr.snapshot(now=101.0)
    f = {x['frame']: x for x in snap['frames']}
    assert 45 < f['base_link']['hz'] < 55 and f['table']['static']
    assert snap['roots'] == ['odom', 'world']
    eng = HealthEngine()
    late = tr.snapshot(now=110.0)
    eng.evaluate(model(late), now=0)
    ids = {i['id'] for i in eng.evaluate(model(late), now=3)}
    assert 'tf:stale:base_link' in ids
    assert 'tf:stale:table' not in ids
