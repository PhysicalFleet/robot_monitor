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
"""End to end: a publisher in another process, a mismatched subscriber, the monitor and its API."""

import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

PUB = r"""
import rclpy, sys
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
rclpy.init(args=sys.argv)
n = rclpy.create_node('itest_default_name')
p = n.create_publisher(Imu, '/itest/imu', qos_profile_sensor_data)
m = Imu()
def tick():
    m.header.stamp = n.get_clock().now().to_msg()
    p.publish(m)
n.create_timer(0.05, tick)
rclpy.spin(n)
"""


@pytest.fixture(scope='module')
def system(tmp_path_factory):
    domain = str(100 + os.getpid() % 100)
    os.environ['ROS_DOMAIN_ID'] = domain
    env = dict(os.environ, ROS_DOMAIN_ID=domain)
    child = subprocess.Popen([sys.executable, '-c', PUB, '--ros-args', '-r', '__node:=itest_pub'],
                             env=env)
    rclpy.init()
    from robot_monitor.monitor import Monitor, spin_in_thread
    from robot_monitor.server import serve
    from sensor_msgs.msg import Imu
    base = str(tmp_path_factory.mktemp('ns') / 'baseline.yaml')
    mon = Monitor(node_name='itest_monitor', parameter_overrides=[
        Parameter('publish_diagnostics', value=False), Parameter('baseline_file', value=base)])
    sub_node = rclpy.create_node('itest_sub')
    sub_node.create_subscription(Imu, '/itest/imu', lambda m: None,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    diag_pub = sub_node.create_publisher(DiagnosticArray, '/diagnostics', 10)
    sub_node.create_timer(0.1, lambda: diag_pub.publish(DiagnosticArray(status=[
        DiagnosticStatus(level=DiagnosticStatus.WARN, name='itest/battery', message='low'),
        # what another Robot Monitor on the network publishes: must not be read back
        DiagnosticStatus(level=DiagnosticStatus.ERROR, name='robot_monitor: summary',
                         hardware_id='robot_monitor@other-machine', message='x')])))
    ex, _ = spin_in_thread(mon)
    from rclpy.executors import SingleThreadedExecutor
    ex2 = SingleThreadedExecutor()
    ex2.add_node(sub_node)
    import threading
    threading.Thread(target=ex2.spin, daemon=True).start()
    srv, port = serve(mon, '127.0.0.1', 0)
    deadline = time.time() + 30
    while time.time() < deadline:
        t = {x['name']: x for x in mon.snapshot.get('topics', [])}.get('/itest/imu')
        n = {x['name']: x for x in mon.snapshot.get('nodes', [])}.get('/itest_pub')
        if t and t.get('stats') and t['stats']['hz'] > 15 and t['qos_issues'] \
                and n and n.get('pid'):
            break
        time.sleep(0.5)
    yield mon, port, child
    srv.shutdown()
    child.terminate()
    child.wait(5)
    ex.shutdown()
    ex2.shutdown()
    rclpy.try_shutdown()


def test_rate_latency_and_qos(system):
    mon, _, child = system
    t = {x['name']: x for x in mon.snapshot['topics']}['/itest/imu']
    assert 15 < t['stats']['hz'] < 25
    assert t['stats']['age_ms'] is not None and t['stats']['age_ms'] < 500
    assert t['qos_issues'] and t['qos_issues'][0]['sub'] == '/itest_sub'
    assert any(i['id'].startswith('qos:/itest/imu') and i['severity'] == 'error'
               for i in mon.snapshot['issues'])


def test_feed_topics_count_their_own_messages(system):
    mon, _, _ = system
    t = {x['name']: x for x in mon.snapshot['topics']}
    assert t['/diagnostics']['stats']['hz'] > 3, t['/diagnostics']['stats']
    names = {d['name'] for d in mon.snapshot['diagnostics']}
    assert 'itest/battery' in names
    assert not any(n.startswith('robot_monitor:') for n in names), names
    assert not any('robot_monitor:' in i['title'] for i in mon.snapshot['issues'])
    if '/tf_static' in t and t['/tf_static'].get('stats'):
        assert t['/tf_static']['stats']['hz'] == 0


def test_node_mapped_to_child_process(system):
    mon, _, child = system
    n = {x['name']: x for x in mon.snapshot['nodes']}['/itest_pub']
    assert n['pid'] == child.pid, n
    assert n['cpu'] is not None


def _get(port, path, method='GET', headers=None):
    req = urllib.request.Request('http://127.0.0.1:%d%s' % (port, path), method=method,
                                 headers=headers or {})
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.status, r.headers.get('Content-Type'), r.read()


def test_http_api(system):
    mon, port, _ = system
    code, ctype, body = _get(port, '/api/snapshot')
    assert code == 200 and json.loads(body)['version']
    code, ctype, body = _get(port, '/')
    assert b'Robot Monitor' in body and 'text/html' in ctype
    assert b'ros_topic_rate_hz{topic="/itest/imu"}' in _get(port, '/metrics')[2]
    assert b'ROS 2 health report' in _get(port, '/api/report?format=md')[2]
    assert _get(port, '/static/app.js')[0] == 200
    with pytest.raises(urllib.error.HTTPError) as e:
        _get(port, '/static/../monitor.py')
    assert e.value.code == 404


def test_sse_stream(system):
    _, port, _ = system
    with urllib.request.urlopen('http://127.0.0.1:%d/api/events' % port, timeout=5) as r:
        first = r.readline()
        assert first.startswith(b'event: snapshot')


def test_latest_message_and_parameters(system):
    _, port, _ = system
    d = json.loads(_get(port, '/api/echo?topic=/itest/imu')[2])
    assert 'header' in d['message'] and d['bytes'] > 0, d
    assert 'error' in json.loads(_get(port, '/api/echo?topic=/does/not/exist')[2])
    p = json.loads(_get(port, '/api/params?node=/itest_pub')[2])
    assert any(name == 'use_sim_time' for name, _ in p['params']), p


def test_baseline_roundtrip_and_csrf(system):
    mon, port, _ = system
    with pytest.raises(urllib.error.HTTPError) as e:
        _get(port, '/api/baseline', 'POST', {'Origin': 'http://evil.example'})
    assert e.value.code == 403
    code, _, body = _get(port, '/api/baseline', 'POST')
    data = json.loads(body)['baseline']
    assert '/itest_pub' in data['nodes']
    assert os.path.exists(mon.p_baseline)
    _get(port, '/api/baseline', 'DELETE')
    assert not os.path.exists(mon.p_baseline)
