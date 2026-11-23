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

from robot_monitor.procs import Proc
from robot_monitor.resolver import Calibration, logger_name, Resolver


def fastdds_prefix(host, pid, rnd=b'\xaa\xbb', part=b'\x00\x00\x00\x01'):
    return b'\x01\x0f' + host + (pid & 0xffff).to_bytes(2, 'little') + rnd + part


class FakeTable:

    def __init__(self, procs, self_pid=1):
        self.procs = {p.pid: p for p in procs}
        self.self_pid = self_pid

    def ros_procs(self):
        return [p for p in self.procs.values() if p.is_ros]

    def get(self, pid):
        return self.procs.get(pid)


def proc(pid, cmd, ros=True, comm=None):
    p = Proc(pid, 1)
    p.cmdline = cmd
    p.comm = comm or (cmd[0].rsplit('/', 1)[-1][:15] if cmd else '')
    p.is_ros = ros
    return p


def test_calibration_fastdds():
    c = Calibration([fastdds_prefix(b'\x24\x0d', 4242)], 4242)
    assert c.ok and c.offset == 4 and c.width == 2
    assert c.decode(fastdds_prefix(b'\x24\x0d', 77)) == (True, 77)
    assert c.decode(fastdds_prefix(b'\x99\x99', 77))[0] is False


def test_host_id_changed_after_interface_change():
    # Same machine, but the robot's processes started before a VPN interface came up,
    # so their Fast DDS host id differs from ours. They must still map locally.
    me = fastdds_prefix(b'\x52\x34', 10)
    r = Resolver(log_dirs=[])
    r.calibrate([me], 10)
    table = FakeTable([proc(10, ['python3', 'mon']), proc(1560, ['/ws/lib/x/tof'])], self_pid=10)
    nodes = {'/mon': {'prefixes': {me}},
             '/tof': {'prefixes': {fastdds_prefix(b'\x5c\x93', 1560)}},
             '/far': {'prefixes': {fastdds_prefix(b'\x11\x11', 4321)}}}
    out = r.resolve(nodes, table, '/mon')
    assert out['/tof'] == {'pid': 1560, 'how': 'guid', 'remote': False}
    assert out['/far']['remote'] is True and out['/far']['pid'] is None


def test_calibration_random_prefix_fails():
    c = Calibration([bytes.fromhex('0110f2903a833843ce7ca235')], 50)
    assert not c.ok


def test_resolve_by_guid_and_remote():
    me = fastdds_prefix(b'\x24\x0d', 10)
    r = Resolver(log_dirs=[])
    r.calibrate([me], 10)
    table = FakeTable([proc(10, ['python3', 'mon']), proc(70000, ['/x/talker'])], self_pid=10)
    nodes = {
        '/mon': {'prefixes': {me}},
        '/talker': {'prefixes': {fastdds_prefix(b'\x24\x0d', 70000)}},
        '/far': {'prefixes': {fastdds_prefix(b'\x11\x11', 5)}},
    }
    out = r.resolve(nodes, table, '/mon')
    assert out['/mon']['how'] == 'self'
    assert out['/talker'] == {'pid': 70000, 'how': 'guid', 'remote': False}
    assert out['/far']['remote'] is True and out['/far']['pid'] is None


def test_resolve_by_remap_launch_group_and_name():
    r = Resolver(log_dirs=[])
    r.calibrate([bytes(12)], 1)  # cyclone-like: no pid in GUID
    p_cam = proc(20, ['/opt/ws/lib/cam/cam_node', '--ros-args', '-r', '__node:=camera',
                      '-r', '__ns:=/front'])
    p_launch = proc(21, ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'launch', 'x'])
    p_talk = proc(22, ['/usr/bin/python3', '/ws/install/demo/lib/demo/talker'])
    table = FakeTable([p_cam, p_launch, p_talk])
    shared = bytes.fromhex('0110aaaaaaaaaaaaaaaaaaaa')
    nodes = {
        '/front/camera': {'prefixes': {shared}},
        '/front/camera_helper': {'prefixes': {shared}},
        '/launch_ros_21': {'prefixes': set()},
        '/talker': {'prefixes': {bytes.fromhex('0110bbbbbbbbbbbbbbbbbbbb')}},
        '/ghost': {'prefixes': set()},
    }
    out = r.resolve(nodes, table)
    assert out['/front/camera'] == {'pid': 20, 'how': 'remap', 'remote': False}
    assert out['/front/camera_helper']['pid'] == 20
    assert out['/front/camera_helper']['how'] == 'group'
    assert out['/launch_ros_21']['pid'] == 21
    assert out['/talker'] == {'pid': 22, 'how': 'name', 'remote': False}
    assert out['/ghost']['pid'] is None


def test_resolve_by_log_file(tmp_path):
    (tmp_path / 'driver_node_31_1700000000000.log').write_text(
        '1700000000.1 [INFO] [robot.arm.driver]: started\n')
    r = Resolver(log_dirs=[str(tmp_path)])
    table = FakeTable([proc(31, ['/ws/lib/x/driver_node']), proc(32, ['/ws/lib/x/other'])])
    out = r.resolve({'/robot/arm/driver': {'prefixes': set()}}, table)
    assert out['/robot/arm/driver'] == {'pid': 31, 'how': 'log', 'remote': False}


def test_container_members():
    r = Resolver(log_dirs=[])
    table = FakeTable([proc(40, ['component_container', '--ros-args', '-r',
                                 '__node:=my_container'])])
    r.container_members = {'/talker': '/my_container'}
    out = r.resolve({'/my_container': {'prefixes': set()}, '/talker': {'prefixes': set()}}, table)
    assert out['/talker']['pid'] == 40 and out['/talker']['how'] == 'container'


def test_logger_name():
    assert logger_name('/cameras/wrist/camera') == 'cameras.wrist.camera'
    assert logger_name('/talker') == 'talker'
