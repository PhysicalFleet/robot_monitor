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

import os

from robot_monitor import procs


def test_parse_stat_with_spaces_and_parens():
    line = ('123 (my (weird) proc) S 1 123 123 0 -1 4194560 100 0 0 0 250 50 0 0 20 0 7 0 '
            '5000 100000000 2560 18446744073709551615 0 0 0 0 0 0 0 0 0 0 0 0 17 2 0 0 0 0 0')
    st = procs.parse_stat(line.encode())
    assert st['comm'] == 'my (weird) proc'
    assert st['state'] == 'S' and st['ppid'] == 1
    assert st['ticks'] == 300 and st['threads'] == 7 and st['start'] == 5000
    assert st['rss'] == 2560 * procs.PAGE_SIZE


def test_parse_stat_garbage():
    assert procs.parse_stat(b'nonsense') is None


def test_remap_args():
    p = procs.Proc(1, 1)
    p.cmdline = ['x', '--ros-args', '-r', '__node:=cam', '--remap', '__ns:=/front', '-p', 'a:=1']
    assert p.args_after_ros_args() == {'__node': 'cam', '__ns': '/front'}


def test_display_name_python_script():
    p = procs.Proc(1, 1)
    p.cmdline = ['/usr/bin/python3', '-u', '/ws/install/pkg/lib/pkg/talker', '--ros-args']
    assert p.display_name() == 'talker'


def test_live_tables():
    if not procs.IS_LINUX:
        return
    t = procs.ProcessTable()
    t.refresh()
    t.refresh()
    me = t.get(os.getpid())
    assert me is not None and me.rss > 0
    h = procs.HostMonitor().refresh()
    assert h['supported'] and h['mem_total'] > 0 and h['ncpu'] >= 1


def test_rss_slope():
    p = procs.Proc(1, 1)
    for i in range(60):
        p.rss_hist.append((i * 10.0, (100 + i * 2) * 1048576))  # +2 MB per 10 s
    assert abs(procs.rss_slope_mb_per_min(p) - 12.0) < 0.01


def test_redact_cmdline():
    from robot_monitor.monitor import redact_cmdline
    out = redact_cmdline(['node', '--token', 'abc', '--api-key=xyz', '-p', 'db_password:=hunter2',
                          '-p', 'rate:=10', '--verbose'])
    assert 'abc' not in out and 'xyz' not in out and 'hunter2' not in out
    assert 'rate:=10' in out and '--verbose' in out and '--token ***' in out
