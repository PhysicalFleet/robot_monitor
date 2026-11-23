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

from robot_monitor.health import HealthEngine


def model(**kw):
    m = {'nodes': {}, 'topics': {}, 'procs': {}, 'host': {'supported': True}, 'gone': {},
         'env': {}, 'uptime': 100, 'self': {'cpu': 1}, 'log_counts': {}, 'diagnostics': [],
         'expect': None}
    m.update(kw)
    return m


def node(name, **kw):
    d = {'name': name, 'count': 1, 'pubs': [], 'subs': [], 'hidden': False, 'self': False}
    d.update(kw)
    return d


def topic(name, pubs=(), subs=(), types=('std_msgs/String',), stats=None, **kw):
    d = {'name': name, 'types': list(types), 'pubs': [{'node': p} for p in pubs],
         'subs': [{'node': s} for s in subs], 'qos_issues': [], 'stats': stats, 'latched': False}
    d.update(kw)
    return d


def ids(issues):
    return {i['id'] for i in issues}


def test_no_nodes_hint():
    out = HealthEngine().evaluate(model(env={'ROS_DOMAIN_ID': '42'}), now=0)
    assert 'discovery:none' in ids(out)
    assert '42' in [i for i in out if i['id'] == 'discovery:none'][0]['detail']


def test_duplicate_and_types():
    m = model(nodes={'/a': node('/a', count=2)},
              topics={'/x': topic('/x', ['/a'], ['/b'], types=('a/A', 'b/B'))})
    got = ids(HealthEngine().evaluate(m, now=0))
    assert 'dup:/a' in got and 'types:/x' in got


def test_dangling_with_suggestion_after_grace():
    eng = HealthEngine()
    m = model(nodes={'/a': node('/a'), '/det': node('/det')},
              topics={'/camera/image_raw': topic('/camera/image_raw', ['/a']),
                      '/camera/image_rect': topic('/camera/image_rect', [], ['/det'])})
    assert 'dangling:/camera/image_rect' not in ids(eng.evaluate(m, now=0))
    out = eng.evaluate(m, now=30)
    i = [x for x in out if x['id'] == 'dangling:/camera/image_rect'][0]
    assert i['severity'] == 'warn' and '/camera/image_raw' in i['title']
    assert i['focus'] == ['/camera/image_rect']


def test_system_topics_not_dangling():
    eng = HealthEngine()
    m = model(nodes={'/a': node('/a')}, topics={'/tf': topic('/tf', [], ['/a'])})
    eng.evaluate(m, now=0)
    assert not [i for i in eng.evaluate(m, now=100) if i['id'].startswith('dangling')]


def test_stale_only_for_periodic():
    eng = HealthEngine()
    st = {'hz': 0.0, 'learned_hz': 10.0, 'periodic': True, 'silent_s': 5.0}
    m = model(nodes={'/a': node('/a')}, topics={'/x': topic('/x', ['/a'], stats=st)})
    assert 'stale:/x' in ids(eng.evaluate(m, now=0))
    st2 = dict(st, periodic=False)
    m2 = model(nodes={'/a': node('/a')}, topics={'/x': topic('/x', ['/a'], stats=st2)})
    assert 'stale:/x' not in ids(eng.evaluate(m2, now=0))
    latched = topic('/x', ['/a'], stats=st, latched=True)
    m3 = model(nodes={'/a': node('/a')}, topics={'/x': latched})
    assert 'stale:/x' not in ids(eng.evaluate(m3, now=0))


def test_rate_drop_needs_to_hold():
    eng = HealthEngine()
    st = {'hz': 4.0, 'learned_hz': 10.0, 'periodic': True, 'silent_s': 0.1}
    m = model(nodes={'/a': node('/a')}, topics={'/x': topic('/x', ['/a'], stats=st)})
    assert 'drop:/x' not in ids(eng.evaluate(m, now=0))
    assert 'drop:/x' in ids(eng.evaluate(m, now=20))


def test_cpu_saturation_and_leak():
    eng = HealthEngine()
    p = {'pid': 5, 'key': '5:1', 'name': 'planner', 'nodes': ['/planner'], 'cpu': 99.0,
         'threads': 8, 'state': 'S', 'rss': 300 * 1048576, 'rss_slope': 5.0,
         'rss_growth_mb': 120.0, 'rss_window_s': 1200}
    m = model(nodes={'/planner': node('/planner')}, procs={5: p})
    first = ids(eng.evaluate(m, now=0))
    assert 'leak:5:1' in first and 'cpu:5:1' not in first
    assert 'cpu:5:1' in ids(eng.evaluate(m, now=40))


def test_host_rules():
    h = {'supported': True, 'throttle': {'undervoltage_now': True}, 'mem_pct': 97,
         'disks': [{'path': '/', 'pct': 98, 'free': 1000}], 'temps': [{'name': 'cpu', 'c': 90}]}
    got = ids(HealthEngine().evaluate(model(host=h), now=0))
    assert {'host:undervoltage', 'host:mem', 'host:disk:/', 'host:temp'} <= got


def test_baseline_rules():
    m = model(nodes={'/a': node('/a')}, topics={},
              expect={'nodes': ['/a', '/missing'], 'topics': {'/scan': {'hz': 10}}})
    got = ids(HealthEngine().evaluate(m, now=0))
    assert 'expect:node:/missing' in got and 'expect:node:/a' not in got
    assert 'expect:topic:/scan' in got


def test_score():
    assert HealthEngine.score([]) == 100
    assert HealthEngine.score([{'severity': 'error'}, {'severity': 'warn'}]) == 76


def test_normalize_baseline():
    from robot_monitor.monitor import normalize_baseline
    assert normalize_baseline(None) == {'nodes': [], 'topics': {}}
    b = normalize_baseline({'nodes': 'lidar', 'topics': ['scan', '/odom']})
    assert b['nodes'] == ['/lidar'] and b['topics'] == {'/scan': {}, '/odom': {}}
    b = normalize_baseline({'nodes': ['/a', 'b'], 'topics': {'/x': {'hz': '10'}, 'y': 5,
                                                             '/z': {'hz': 'fast'}}})
    assert b['nodes'] == ['/a', '/b']
    assert b['topics'] == {'/x': {'hz': 10.0}, '/y': {'hz': 5.0}, '/z': {}}


def test_timers_are_forgotten_for_vanished_things():
    eng = HealthEngine()
    p = {'pid': 9, 'key': '9:1', 'name': 'x', 'nodes': [], 'cpu': 99.0, 'threads': 1,
         'state': 'S', 'rss': 1, 'rss_slope': None, 'rss_growth_mb': 0, 'rss_window_s': 0}
    eng.evaluate(model(nodes={'/a': node('/a')}, procs={9: p}), now=0)
    assert any(k.startswith('cpu:') for k in eng._since)
    eng.evaluate(model(nodes={'/a': node('/a')}, procs={}), now=1)
    assert not any(k.startswith('cpu:') for k in eng._since)
