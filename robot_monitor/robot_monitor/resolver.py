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
Map ROS nodes to the OS processes that host them.

ROS 2 has no API for "which PID runs node X". We combine several clues, from
most to least certain, and record which one was used:

  self       our own node.
  guid       the middleware embeds the PID in the DDS GUID prefix (Fast DDS does).
             We *calibrate* this at runtime against our own GUID and PID, so no
             vendor layout is hard-coded and remote hosts are told apart.
  remap      the process command line carries `-r __node:=name [-r __ns:=/ns]`
             (what launch files pass for Node(name=...)).
  container  a composition container lists its components (`_container/list_nodes`).
  launch     launch_ros names its own node `launch_ros_<pid>`.
  log        the process's ROS log file (<exe>_<pid>_<time>.log) shows the node's logger.
  name       the executable or script is named like the node, and only one process matches.
  group      another node in the same DDS participant (= same process) was resolved.
  discovery  the participant's SPDP announcement lists UDP ports owned by the process
             (any DDS vendor, e.g. Cyclone DDS), or an IP address of another machine.
"""

import glob
import os
import re

_LOGGER_RE = re.compile(rb'\] \[([A-Za-z0-9_.]+)\]: ')
_LOGFILE_RE = re.compile(r'^(?P<exe>.+)_(?P<pid>\d+)_(?P<ts>\d+)\.log$')
_LAUNCH_RE = re.compile(r'^launch_ros_(\d+)$')


def logger_name(full_name):
    """'/cameras/wrist/camera' -> 'cameras.wrist.camera' (rcl's node logger name)."""
    return full_name.lstrip('/').replace('/', '.')


def split_name(full):
    ns, _, name = full.rpartition('/')
    return (ns or '/'), name


class Calibration:
    """Where (if anywhere) the middleware puts our PID in the GUID prefix."""

    def __init__(self, own_prefixes, own_pid):
        self.offset = None
        self.width = None
        self.order = None
        self.host_key = None
        for prefix in own_prefixes:
            if self._find(prefix, own_pid):
                break

    def _find(self, prefix, pid):
        for width, fmt in ((4, 'little'), (4, 'big'), (2, 'little'), (2, 'big')):
            want = (pid & ((1 << (8 * width)) - 1)).to_bytes(width, fmt)
            for off in range(2, len(prefix) - width + 1):
                if prefix[off:off + width] == want:
                    self.offset, self.width, self.order = off, width, fmt
                    self.host_key = bytes(prefix[:off])
                    return True
        return False

    @property
    def ok(self):
        return self.offset is not None

    def decode(self, prefix):
        """Return (same_host, pid_fragment) for a GUID prefix, or (None, None)."""
        if not self.ok or len(prefix) < self.offset + self.width:
            return None, None
        same = bytes(prefix[:self.offset]) == self.host_key
        frag = int.from_bytes(prefix[self.offset:self.offset + self.width], self.order)
        return same, frag

    def describe(self):
        if not self.ok:
            return 'middleware does not embed process ids'
        return 'pid at GUID bytes %d..%d (%s-endian)' % (
            self.offset, self.offset + self.width - 1, self.order)


class Resolver:

    def __init__(self, log_dirs=None):
        self.calibration = None
        self.log_dirs = log_dirs or default_log_dirs()
        self._log_cache = {}          # (pid, start) -> set(logger names)
        self.container_members = {}   # node full name -> container full name
        self.spdp = None              # optional SpdpListener

    def calibrate(self, own_prefixes, own_pid):
        cal = Calibration(own_prefixes, own_pid)
        if cal.ok or self.calibration is None:
            self.calibration = cal

    def resolve(self, nodes, ptable, self_name=None):
        """
        nodes: {full_name: {'prefixes': set(bytes)}}.

        Returns {full_name: {'pid': int|None, 'how': str, 'remote': bool}}.
        """
        out = {}
        ros = ptable.ros_procs()
        by_pid = {p.pid: p for p in ros}

        if self_name in nodes:
            out[self_name] = {'pid': ptable.self_pid, 'how': 'self', 'remote': False}

        # 1. GUID-embedded pids (calibrated against ourselves).
        # The host part of a Fast DDS GUID is derived from the network interfaces when
        # the process starts, so processes started before an interface change (VPN up,
        # new DHCP address) carry a different host id on the same machine. A host-id
        # mismatch is therefore only a hint: discovery data and local pids decide.
        cal = self.calibration
        other_host = {}   # name -> (prefix, pid fragment) when the host id differs
        mask = (1 << (8 * cal.width)) - 1 if cal is not None and cal.ok else 0
        if cal is not None and cal.ok:
            for name, info in nodes.items():
                if name in out:
                    continue
                for prefix in info['prefixes']:
                    same, frag = cal.decode(prefix)
                    if same is None:
                        continue
                    if not same:
                        other_host.setdefault(name, (prefix, frag))
                        continue
                    cands = [p for p in ros if (p.pid & mask) == frag]
                    if not cands:  # e.g. another user's process whose maps we can't read
                        cands = [p for p in ptable.procs.values()
                                 if (p.pid & mask) == frag and p.cmdline]
                        if len(cands) == 1:
                            cands[0].is_ros = True
                            by_pid[cands[0].pid] = cands[0]
                    if len(cands) == 1:
                        out[name] = {'pid': cands[0].pid, 'how': 'guid', 'remote': False}
                        break

        # 1b. DDS discovery announcements (vendor independent): ports -> process.
        if self.spdp is not None:
            pending = {n: info for n, info in nodes.items() if n not in out and info['prefixes']}
            if pending:
                allp = {p for info in pending.values() for p in info['prefixes']}
                cands = [p.pid for p in ptable.procs.values() if p.cmdline]
                found = self.spdp.lookup(allp, cands)
                for name, info in pending.items():
                    for prefix in info['prefixes']:
                        hit = found.get(bytes(prefix))
                        if hit is None:
                            continue
                        if hit[0] == 'remote':
                            out[name] = {'pid': None, 'how': 'discovery', 'remote': True,
                                         'host_ip': hit[1]}
                        elif hit[1] in ptable.procs:
                            p = ptable.procs[hit[1]]
                            p.is_ros = True
                            by_pid[p.pid] = p
                            out[name] = {'pid': p.pid, 'how': 'discovery', 'remote': False}
                        break

        # 1c. Host id differed and discovery didn't settle it: a unique local ROS process
        # with that pid is the answer; otherwise the node runs on another machine.
        for name, (prefix, frag) in other_host.items():
            if name in out:
                continue
            cands = [p for p in ros if (p.pid & mask) == frag]
            if len(cands) == 1:
                out[name] = {'pid': cands[0].pid, 'how': 'guid', 'remote': False}
            else:
                out[name] = {'pid': None, 'how': 'guid', 'remote': True,
                             'host_key': prefix[:cal.offset].hex()}

        # 2. Command-line remaps, 3. launch_ros_<pid>, 4. composition containers.
        remap_index = {}
        for p in ros:
            a = p.args_after_ros_args()
            if '__node' in a:
                ns = a.get('__ns', '/')
                if not ns.startswith('/'):
                    ns = '/' + ns
                full = (ns.rstrip('/') + '/' + a['__node'])
                remap_index.setdefault(full, []).append(p.pid)
        for name in nodes:
            if name in out:
                continue
            pids = remap_index.get(name)
            if pids and len(pids) == 1:
                out[name] = {'pid': pids[0], 'how': 'remap', 'remote': False}
                continue
            m = _LAUNCH_RE.match(split_name(name)[1])
            if m and int(m.group(1)) in ptable.procs:
                out[name] = {'pid': int(m.group(1)), 'how': 'launch', 'remote': False}
        for member, container in self.container_members.items():
            if member in nodes and member not in out and container in out:
                c = out[container]
                out[member] = {'pid': c['pid'], 'how': 'container', 'remote': c['remote']}

        # 5. Participant groups: nodes sharing a GUID prefix live in one process.
        self._propagate_groups(nodes, out)

        # 6. ROS log files, 7. executable names, for what is still unknown.
        claimed = {v['pid'] for v in out.values() if v.get('pid')}
        unresolved = [n for n in nodes if n not in out]
        if unresolved:
            loggers = {}
            for p in ros:
                if p.pid in claimed:
                    continue
                for lg in self._loggers_for(p):
                    loggers.setdefault(lg, set()).add(p.pid)
            for name in unresolved:
                pids = loggers.get(logger_name(name))
                if pids and len(pids) == 1:
                    out[name] = {'pid': next(iter(pids)), 'how': 'log', 'remote': False}
            self._propagate_groups(nodes, out)
            claimed = {v['pid'] for v in out.values() if v.get('pid')}
            for name in [n for n in nodes if n not in out]:
                base = split_name(name)[1]
                cands = [p for p in ros if p.pid not in claimed and _name_match(p, base)]
                if len(cands) == 1:
                    out[name] = {'pid': cands[0].pid, 'how': 'name', 'remote': False}
                    claimed.add(cands[0].pid)
            self._propagate_groups(nodes, out)

        for name in nodes:
            if name not in out:
                out[name] = {'pid': None, 'how': None, 'remote': False}
            elif out[name].get('pid') and out[name]['pid'] not in by_pid \
                    and out[name]['pid'] != ptable.self_pid:
                out[name]['pid'] = None
        return out

    @staticmethod
    def _propagate_groups(nodes, out):
        groups = {}
        for name, info in nodes.items():
            for prefix in info['prefixes']:
                groups.setdefault(bytes(prefix), set()).add(name)
        for members in groups.values():
            if len(members) < 2:
                continue
            known = [out[m] for m in members if m in out and out[m].get('pid')]
            if not known:
                continue
            for m in members:
                if m not in out:
                    out[m] = {'pid': known[0]['pid'], 'how': 'group', 'remote': False}

    def _loggers_for(self, p):
        key = (p.pid, p.start)
        if key in self._log_cache:
            return self._log_cache[key]
        names = set()
        for d in self.log_dirs:
            for path in glob.glob(os.path.join(d, '*_%d_*.log' % p.pid)):
                m = _LOGFILE_RE.match(os.path.basename(path))
                if not m or int(m.group('pid')) != p.pid:
                    continue
                try:
                    size = os.path.getsize(path)
                    with open(path, 'rb') as f:
                        head = f.read(65536)
                        if size > 131072:
                            f.seek(size - 65536)
                        tail = f.read(65536)
                except OSError:
                    continue
                for chunk in (head, tail):
                    names.update(x.decode('utf-8', 'replace') for x in _LOGGER_RE.findall(chunk))
        # Only cache positive results; a fresh process may not have logged yet.
        if names:
            if len(self._log_cache) > 2000:
                self._log_cache.clear()
            self._log_cache[key] = names
        return names


def _name_match(p, base):
    if not base:
        return False
    disp = p.display_name()
    for cand in (disp, os.path.splitext(disp)[0], p.comm):
        if cand == base or cand.replace('-', '_') == base:
            return True
    return False


def default_log_dirs():
    dirs = []
    if os.environ.get('ROS_LOG_DIR'):
        dirs.append(os.environ['ROS_LOG_DIR'])
    home = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
    dirs.append(os.path.join(home, 'log'))
    return [d for d in dirs if os.path.isdir(d)]
