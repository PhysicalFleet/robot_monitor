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
Process and host facts read straight from /proc and /sys (Linux).

No third-party dependencies, so it works in any ROS environment (apt, source,
RoboStack, containers). Every read is defensive: a missing file or a process
that exits mid-read just yields less data, never an exception to the caller.
"""

from collections import deque
import glob
import os
import shutil
import socket
import subprocess
import time

IS_LINUX = os.path.isdir('/proc/self')
try:
    CLK_TCK = os.sysconf('SC_CLK_TCK')
    PAGE_SIZE = os.sysconf('SC_PAGE_SIZE')
except (ValueError, OSError, AttributeError):  # pragma: no cover - non-POSIX
    CLK_TCK, PAGE_SIZE = 100, 4096

VIRTUAL_IFACES = ('veth', 'docker', 'br-', 'tunl', 'gre', 'erspan', 'sit', 'ip6', 'ip_vti',
                  'dummy', 'virbr')
# Shared-object names that mark a process as a ROS 2 process.
_ROS_LIB_MARKERS = (b'librcl.so', b'librmw_implementation', b'_rclpy_pybind11')


def _read(path, limit=65536):
    try:
        with open(path, 'rb') as f:
            return f.read(limit)
    except OSError:
        return None


def _read_text(path):
    data = _read(path)
    return data.decode('utf-8', 'replace').strip() if data is not None else None


def parse_stat(data):
    """Parse /proc/<pid>/stat. Returns a dict or None."""
    try:
        text = data.decode('utf-8', 'replace')
        lpar, rpar = text.index('('), text.rindex(')')
        comm = text[lpar + 1:rpar]
        f = text[rpar + 2:].split()
        # f[0] is field 3 (state) in proc(5) numbering, so field N is f[N - 3].
        return {
            'comm': comm,
            'state': f[0],
            'ppid': int(f[1]),
            'ticks': int(f[11]) + int(f[12]),
            'threads': int(f[17]),
            'start': int(f[19]),
            'rss': int(f[21]) * PAGE_SIZE,
        }
    except (ValueError, IndexError, AttributeError):
        return None


def boot_time():
    data = _read_text('/proc/stat') or ''
    for line in data.splitlines():
        if line.startswith('btime '):
            return int(line.split()[1])
    return time.time() - float((_read_text('/proc/uptime') or '0').split()[0] or 0)


class Proc:
    """One live process we track (keyed by pid + start time, so pid reuse is safe)."""

    __slots__ = ('pid', 'start', 'comm', 'cmdline', 'exe', 'is_ros', 'state', 'ppid',
                 'threads', 'rss', 'cpu', 'ticks', 't', 'fds', 'rss_hist', 'cpu_hist',
                 'user', 'gpu_mem', 'children_cpu', 'classified', 'checked')

    def __init__(self, pid, start):
        self.pid, self.start = pid, start
        self.comm, self.cmdline, self.exe = '', [], ''
        self.is_ros = False
        self.state, self.ppid, self.threads, self.rss = '?', 0, 0, 0
        self.cpu, self.ticks, self.t = 0.0, None, None
        self.fds = None
        self.user = None
        self.gpu_mem = None
        self.children_cpu = 0.0
        self.classified = False
        self.checked = 0.0
        # (monotonic time, rss bytes) every ~10 s for leak detection, 1 h deep.
        self.rss_hist = deque(maxlen=360)
        self.cpu_hist = deque(maxlen=60)

    def uptime(self, btime):
        return max(0.0, time.time() - (btime + self.start / CLK_TCK))

    def args_after_ros_args(self):
        """Return ROS remap args from the command line, e.g. {'__node': 'cam'}."""
        out = {}
        cmd = self.cmdline
        for i, a in enumerate(cmd):
            if a in ('-r', '--remap') and i + 1 < len(cmd) and ':=' in cmd[i + 1]:
                k, v = cmd[i + 1].split(':=', 1)
                if k in ('__node', '__name', '__ns'):
                    out[k] = v
            elif a.startswith(('__node:=', '__name:=', '__ns:=')):  # ROS 1 style / legacy
                k, v = a.split(':=', 1)
                out[k] = v
        if '__name' in out and '__node' not in out:
            out['__node'] = out['__name']
        return out

    def display_name(self):
        """Human name: the ROS executable or script rather than 'python3'."""
        cmd = self.cmdline
        if not cmd:
            return self.comm
        base = os.path.basename(cmd[0])
        if base.startswith('python') and len(cmd) > 1:
            for a in cmd[1:]:
                if a == '-m':
                    continue
                if not a.startswith('-'):
                    return os.path.basename(a)
        return base or self.comm


class ProcessTable:
    """Samples every process cheaply; keeps rich detail for ROS processes."""

    def __init__(self):
        self.procs = {}           # pid -> Proc
        self.btime = boot_time() if IS_LINUX else time.time()
        self.self_pid = os.getpid()
        self.uid = os.getuid() if hasattr(os, 'getuid') else 0
        self._last_rss_sample = 0.0
        self._last_fd_sample = 0.0
        self._users = {}
        self._tick = 0

    def _username(self, uid):
        if uid not in self._users:
            try:
                import pwd
                self._users[uid] = pwd.getpwuid(uid).pw_name
            except Exception:
                self._users[uid] = str(uid)
        return self._users[uid]

    def refresh(self):
        if not IS_LINUX:
            return
        now = time.monotonic()
        self._tick += 1
        # Non-ROS processes are only re-read every 5th refresh: on a busy robot reading
        # every /proc/<pid>/stat each time is most of this monitor's own CPU.
        full = self._tick % 5 == 1
        up = (_read_text('/proc/uptime') or '0').split()
        uptime_ticks = float(up[0]) * CLK_TCK if up else 0.0
        seen = set()
        try:
            pids = [int(p) for p in os.listdir('/proc') if p.isdigit()]
        except OSError:
            return
        for pid in pids:
            p = self.procs.get(pid)
            if p is not None and not full and not p.is_ros and p.classified:
                seen.add(pid)
                continue
            st = parse_stat(_read('/proc/%d/stat' % pid, 4096) or b'')
            if st is None:
                continue
            seen.add(pid)
            if p is None or p.start != st['start']:
                p = Proc(pid, st['start'])
                self.procs[pid] = p
            if not p.classified and (uptime_ticks - st['start']) >= 3 * CLK_TCK:
                # Classify once the process has lived 3 s: skips short-lived helpers.
                self._fill_identity(p)
                p.classified = True
                p.checked = now
            elif (full and p.classified and not p.is_ros and p.cmdline and now - p.checked > 20
                  and (uptime_ticks - st['start']) < 600 * CLK_TCK):
                # Young non-ROS processes are re-checked: a Python node may import torch
                # for 30 s before it loads rclpy.
                p.checked = now
                p.is_ros = self._looks_ros(p)
            p.comm, p.state, p.ppid = st['comm'], st['state'], st['ppid']
            p.threads, p.rss = st['threads'], st['rss']
            if p.ticks is not None and p.t is not None and now > p.t:
                p.cpu = 100.0 * (st['ticks'] - p.ticks) / CLK_TCK / (now - p.t)
            p.ticks, p.t = st['ticks'], now
            if p.is_ros:
                p.cpu_hist.append(round(p.cpu, 1))
        for pid in list(self.procs):
            if pid not in seen:
                del self.procs[pid]
        if now - self._last_rss_sample >= 10.0:
            self._last_rss_sample = now
            for p in self.procs.values():
                if p.is_ros:
                    p.rss_hist.append((now, p.rss))
        if now - self._last_fd_sample >= 10.0:
            self._last_fd_sample = now
            for p in self.procs.values():
                if p.is_ros:
                    try:
                        p.fds = len(os.listdir('/proc/%d/fd' % p.pid))
                    except OSError:
                        p.fds = None

    def _fill_identity(self, p):
        raw = _read('/proc/%d/cmdline' % p.pid) or b''
        p.cmdline = [a.decode('utf-8', 'replace') for a in raw.split(b'\0') if a]
        try:
            p.exe = os.readlink('/proc/%d/exe' % p.pid)
        except OSError:
            p.exe = ''
        try:
            p.user = self._username(os.stat('/proc/%d' % p.pid).st_uid)
        except OSError:
            p.user = None
        p.is_ros = self._looks_ros(p)

    @staticmethod
    def _looks_ros(p):
        if not p.cmdline:
            return False  # kernel thread
        if '--ros-args' in p.cmdline:
            return True
        maps = _read('/proc/%d/maps' % p.pid, 4 * 1024 * 1024)
        if maps is not None:
            return any(m in maps for m in _ROS_LIB_MARKERS)
        # Not readable (other user): fall back to path hints.
        joined = ' '.join(p.cmdline[:3])
        return '/opt/ros/' in joined or 'ros2' in os.path.basename(p.cmdline[0])

    def ros_procs(self):
        return [p for p in self.procs.values() if p.is_ros]

    def get(self, pid):
        return self.procs.get(pid)

    def top(self, n=8):
        return sorted(self.procs.values(), key=lambda p: p.cpu, reverse=True)[:n]

    def tree_cpu(self, pid):
        """CPU of a process plus all of its descendants (e.g. a launch file)."""
        kids = {}
        for p in self.procs.values():
            kids.setdefault(p.ppid, []).append(p)
        total, stack = 0.0, [pid]
        while stack:
            cur = stack.pop()
            p = self.procs.get(cur)
            if p:
                total += p.cpu
            stack.extend(k.pid for k in kids.get(cur, ()))
        return total


def rss_slope_mb_per_min(p):
    """Least-squares slope of RSS over the history, in MB/min (None if too short)."""
    h = p.rss_hist
    if len(h) < 30:  # at least 5 minutes
        return None
    t0 = h[0][0]
    xs = [(t - t0) / 60.0 for t, _ in h]
    ys = [r / 1048576.0 for _, r in h]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    if den <= 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den


# ---------------------------------------------------------------- host ---

class HostMonitor:
    """CPU, memory, load, temperatures, power/throttle flags, disk, network, GPU."""

    def __init__(self, disk_paths=None):
        self.hostname = socket.gethostname()
        self.ncpu = os.cpu_count() or 1
        self._cpu_prev = None
        self._net_prev = None
        self.disk_paths = disk_paths or ['/']
        self.cpu_hist = deque(maxlen=120)
        self.mem_hist = deque(maxlen=120)
        self.model = self._model()
        self.os = self._os_name()
        self.kernel = os.uname().release if hasattr(os, 'uname') else ''
        self._gpu_last = 0.0
        self._gpu = None
        self._gpu_procs = {}
        self._nvsmi = shutil.which('nvidia-smi')
        self._jetson_gpu = self._find_jetson_gpu()
        self.data = {}

    @staticmethod
    def _model():
        for path in ('/proc/device-tree/model', '/sys/firmware/devicetree/base/model'):
            t = _read_text(path)
            if t:
                return t.replace('\x00', '')
        t = _read_text('/sys/class/dmi/id/product_name')
        vendor = _read_text('/sys/class/dmi/id/sys_vendor')
        if t:
            return ('%s %s' % (vendor or '', t)).strip()
        return ''

    @staticmethod
    def _os_name():
        data = _read_text('/etc/os-release') or ''
        for line in data.splitlines():
            if line.startswith('PRETTY_NAME='):
                return line.split('=', 1)[1].strip('"')
        return ''

    @staticmethod
    def _find_jetson_gpu():
        for pat in ('/sys/devices/gpu.0/load', '/sys/devices/platform/gpu.0/load',
                    '/sys/devices/platform/bus@0/*.gpu/load', '/sys/devices/platform/*.gpu/load',
                    '/sys/devices/*.gpu/load'):
            hits = glob.glob(pat)
            if hits:
                return hits[0]
        return None

    def refresh(self):
        if not IS_LINUX:
            self.data = {'hostname': self.hostname, 'ncpu': self.ncpu, 'supported': False}
            return self.data
        d = {'hostname': self.hostname, 'ncpu': self.ncpu, 'model': self.model,
             'os': self.os, 'kernel': self.kernel, 'supported': True}
        d.update(self._cpu())
        d.update(self._mem())
        la = (_read_text('/proc/loadavg') or '0 0 0').split()
        d['load'] = [float(x) for x in la[:3]]
        up = (_read_text('/proc/uptime') or '0').split()
        d['uptime'] = float(up[0]) if up else 0.0
        d['temps'] = self._temps()
        d['throttle'] = self._throttle()
        d['disks'] = self._disks()
        d['net'] = self._net()
        d['gpu'] = self._gpu_stats()
        self.cpu_hist.append(round(d.get('cpu', 0.0), 1))
        self.mem_hist.append(round(d.get('mem_pct', 0.0), 1))
        self.data = d
        return d

    def _cpu(self):
        data = _read_text('/proc/stat') or ''
        cores = []
        for line in data.splitlines():
            if line.startswith('cpu'):
                parts = line.split()
                vals = [int(x) for x in parts[1:9]]
                idle = vals[3] + vals[4]
                cores.append((parts[0], sum(vals), idle))
        out = {'cpu': 0.0, 'cpu_cores': []}
        if self._cpu_prev and len(self._cpu_prev) == len(cores):
            pcts = []
            for (name, tot, idle), (_, ptot, pidle) in zip(cores, self._cpu_prev):
                dt = tot - ptot
                pcts.append(0.0 if dt <= 0 else 100.0 * (1.0 - (idle - pidle) / dt))
            out['cpu'] = round(pcts[0], 1)
            out['cpu_cores'] = [round(x, 1) for x in pcts[1:]]
        self._cpu_prev = cores
        return out

    @staticmethod
    def _mem():
        info = {}
        for line in (_read_text('/proc/meminfo') or '').splitlines():
            k, _, v = line.partition(':')
            try:
                info[k] = int(v.split()[0]) * 1024
            except (ValueError, IndexError):
                pass
        tot = info.get('MemTotal', 0)
        avail = info.get('MemAvailable', info.get('MemFree', 0))
        stot, sfree = info.get('SwapTotal', 0), info.get('SwapFree', 0)
        return {
            'mem_total': tot, 'mem_avail': avail,
            'mem_pct': round(100.0 * (tot - avail) / tot, 1) if tot else 0.0,
            'swap_total': stot, 'swap_used': stot - sfree,
        }

    @staticmethod
    def _temps():
        out = []
        for zone in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
            t = _read_text(zone + '/temp')
            if not t:
                continue
            try:
                c = int(t) / 1000.0
            except ValueError:
                continue
            if c <= -40 or c > 150:
                continue
            out.append({'name': _read_text(zone + '/type') or os.path.basename(zone),
                        'c': round(c, 1)})
        return out

    @staticmethod
    def _throttle():
        """Raspberry Pi firmware flags: under-voltage, throttling, frequency cap."""
        raw = _read_text('/sys/devices/platform/soc/soc:firmware/get_throttled')
        if raw is None:
            # rpi_volt hwmon exposes the under-voltage alarm on newer kernels.
            for hw in glob.glob('/sys/class/hwmon/hwmon*'):
                if _read_text(hw + '/name') == 'rpi_volt':
                    alarm = _read_text(hw + '/in0_lcrit_alarm')
                    if alarm is not None:
                        return {'source': 'rpi_volt', 'undervoltage_now': alarm == '1'}
            return None
        try:
            v = int(raw, 16)
        except ValueError:
            return None
        return {
            'source': 'firmware', 'raw': hex(v),
            'undervoltage_now': bool(v & 0x1), 'freq_capped_now': bool(v & 0x2),
            'throttled_now': bool(v & 0x4), 'soft_temp_limit_now': bool(v & 0x8),
            'undervoltage_since_boot': bool(v & 0x10000),
            'throttled_since_boot': bool(v & 0x40000),
        }

    def _disks(self):
        out, seen = [], set()
        for path in self.disk_paths:
            try:
                st = os.statvfs(path)
            except OSError:
                continue
            key = (st.f_blocks, st.f_bfree)
            if key in seen:
                continue
            seen.add(key)
            total = st.f_blocks * st.f_frsize
            free = st.f_bavail * st.f_frsize
            out.append({'path': path, 'total': total, 'free': free,
                        'pct': round(100.0 * (total - free) / total, 1) if total else 0.0})
        return out

    def _net(self):
        now = time.monotonic()
        cur = {}
        for line in (_read_text('/proc/net/dev') or '').splitlines()[2:]:
            name, _, rest = line.partition(':')
            name = name.strip()
            if name == 'lo' or name.startswith(VIRTUAL_IFACES):
                continue
            f = rest.split()
            if len(f) >= 9:
                cur[name] = (int(f[0]), int(f[8]))
        out = []
        if self._net_prev:
            pt, prev = self._net_prev
            dt = max(1e-3, now - pt)
            for name, (rx, tx) in cur.items():
                if name in prev:
                    out.append({'iface': name,
                                'rx': max(0, rx - prev[name][0]) / dt,
                                'tx': max(0, tx - prev[name][1]) / dt})
        self._net_prev = (now, cur)
        return out

    def _gpu_stats(self):
        now = time.monotonic()
        if now - self._gpu_last < 5.0:
            return self._gpu
        self._gpu_last = now
        gpus = []
        if self._jetson_gpu:
            raw = _read_text(self._jetson_gpu)
            try:
                gpus.append({'name': 'Jetson GPU', 'util': int(raw) / 10.0})
            except (TypeError, ValueError):
                pass
        if self._nvsmi:
            try:
                out = subprocess.run(
                    [self._nvsmi, '--query-gpu=name,utilization.gpu,memory.used,memory.total,'
                     'temperature.gpu', '--format=csv,noheader,nounits'],
                    capture_output=True, text=True, timeout=3).stdout
                for line in out.strip().splitlines():
                    n, u, mu, mt, t = [x.strip() for x in line.split(',')]
                    gpus.append({'name': n, 'util': float(u), 'mem_used': float(mu) * 1048576,
                                 'mem_total': float(mt) * 1048576, 'temp': float(t)})
                out = subprocess.run(
                    [self._nvsmi, '--query-compute-apps=pid,used_memory',
                     '--format=csv,noheader,nounits'],
                    capture_output=True, text=True, timeout=3).stdout
                procs = {}
                for line in out.strip().splitlines():
                    pid, mem = [x.strip() for x in line.split(',')]
                    procs[int(pid)] = float(mem) * 1048576
                self._gpu_procs = procs
            except (OSError, ValueError, subprocess.SubprocessError):
                self._nvsmi = None  # don't keep paying for a broken nvidia-smi
        self._gpu = gpus or None
        return self._gpu

    def gpu_mem_of(self, pid):
        return self._gpu_procs.get(pid)
