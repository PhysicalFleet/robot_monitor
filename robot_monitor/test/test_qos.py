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

from robot_monitor import qos


def q(rel='reliable', dur='volatile', deadline=None, live='automatic', lease=None):
    return {'reliability': rel, 'durability': dur, 'history': 'keep_last', 'depth': 10,
            'deadline_ns': deadline, 'lifespan_ns': None, 'liveliness': live, 'lease_ns': lease}


def test_compatible_pairs():
    assert qos.check(q(), q())[0] == 'ok'
    assert qos.check(q(), q(rel='best_effort'))[0] == 'ok'
    assert qos.check(q(dur='transient_local'), q())[0] == 'ok'


def test_reliability_mismatch():
    level, reason = qos.check(q(rel='best_effort'), q(rel='reliable'))
    assert level == 'error'
    assert 'RELIABLE' in reason
    assert 'SensorDataQoS' in qos.fix_hint(None, None, reason)


def test_durability_mismatch():
    level, reason = qos.check(q(dur='volatile'), q(dur='transient_local'))
    assert level == 'error' and 'TRANSIENT_LOCAL' in reason


def test_deadline():
    assert qos.check(q(deadline=100000000), q(deadline=50000000))[0] == 'error'
    assert qos.check(q(deadline=50000000), q(deadline=100000000))[0] == 'ok'
    assert qos.check(q(), q(deadline=100000000))[0] == 'error'


def test_liveliness():
    assert qos.check(q(live='automatic'), q(live='manual_by_topic'))[0] == 'error'


def test_short():
    assert qos.short(q()) == 'reliable/volatile/keep_last(10)'
    assert qos.short(None) == '?'


def test_qos_to_dict_from_rclpy():
    from rclpy.qos import qos_profile_sensor_data
    d = qos.qos_to_dict(qos_profile_sensor_data)
    assert d['reliability'] == 'best_effort'
    assert d['durability'] == 'volatile'
    assert d['deadline_ns'] is None
