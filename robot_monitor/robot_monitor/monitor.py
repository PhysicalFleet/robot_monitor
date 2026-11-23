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
"""
The monitor node: watches the ROS graph, topics, processes and the host.

Everything ROS-related runs in this node's executor thread (timers and
callbacks). The result of each refresh is an immutable snapshot (a dict and its
JSON bytes) that the web server and CLI read without locks.
"""

from collections import deque
import itertools
import json
import os
import re
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from . import __version__
from . import qos as qosmod
from .health import HealthEngine, is_hidden_node, is_hidden_topic
from .procs import HostMonitor, ProcessTable, rss_slope_mb_per_min
from .resolver import logger_name, Resolver
from .topicstats import stamp_reader, TopicStats

try:  # Jazzy and newer
    from rclpy.event_handler import SubscriptionEventCallbacks
except ImportError:  # Humble
    from rclpy.qos_event import SubscriptionEventCallbacks

try:
    from rosidl_runtime_py.utilities import get_message, get_service
except ImportError:  # pragma: no cover
    get_message = get_service = None

# Types that are expensive to receive: start them in sampled mode.
HEAVY_TYPES = {'sensor_msgs/msg/Image', 'sensor_msgs/msg/PointCloud2',
               'sensor_msgs/msg/PointCloud', 'stereo_msgs/msg/DisparityImage',
               'octomap_msgs/msg/Octomap', 'grid_map_msgs/msg/GridMap'}
# Topics we read through dedicated subscriptions (content, not just stats).
FEED_TOPICS = {'/rosout', '/diagnostics'}
NOT_MONITORED = {'/parameter_events'}

MONITOR_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
FEED_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)

ENV_KEYS = ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
            'ROS_STATIC_PEERS', 'ROS_LOCALHOST_ONLY', 'ROS_DISCOVERY_SERVER',
            'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_BUILTIN_TRANSPORTS', 'CYCLONEDDS_URI',
            'ZENOH_ROUTER_CONFIG_URI', 'ZENOH_SESSION_CONFIG_URI', 'ROS_LOG_DIR')


class _Watched:
    """A topic we subscribe to for statistics."""

    __slots__ = ('name', 'type', 'cls', 'stats', 'sub', 'mode', 'reader', 'frozen',
                 'window_end', 'next_on', 'reason', 'heavy', 'on_s')

    def __init__(self, name, type_name):
        self.name, self.type = name, type_name
        self.cls = None
        self.stats = TopicStats()
        self.sub = None
        self.mode = 'continuous'   # continuous | sampled | unavailable | excluded
        self.reader = None
        self.frozen = None
        self.window_end = 0.0
        self.next_on = 0.0
        self.reason = ''
        self.heavy = False
        self.on_s = 2.0


class Monitor(Node):

    def __init__(self, node_name='robot_monitor', **kwargs):
        super().__init__(node_name, **kwargs)
        p = self.declare_parameter
        self.p_period = p('refresh_period', 1.0).value
        self.p_graph_period = p('graph_period', 2.0).value
        self.p_exclude = [re.compile(x) for x in p('exclude_topics', ['']).value if x]
        self.p_monitor_topics = p('monitor_topics', True).value
        self.p_sample_heavy = p('sample_heavy', True).value
        self.p_heavy_mbps = p('heavy_bandwidth_mb', 8.0).value
        self.p_max_hz = p('max_rate_hz', 1500.0).value
        self.p_on_s = p('sample_on_s', 2.0).value
        self.p_off_s = p('sample_off_s', 8.0).value
        self.p_idle_period = p('idle_period', 3.0).value
        self.p_max_total_hz = p('max_total_rate_hz', 1000.0).value
        self._last_budget = 0.0
        self.always_active = False
        self.watchers = 0           # open dashboard streams (SSE)
        self.last_api_hit = -1e9
        self._last_snap = -1e9
        self._last_proc = -1e9
        self.p_pub_diag = p('publish_diagnostics', True).value
        self.p_baseline = os.path.expanduser(
            p('baseline_file', '~/.ros/robot_monitor/baseline.yaml').value)

        self.started = time.monotonic()
        self.host = HostMonitor(disk_paths=_disk_paths())
        self.ptable = ProcessTable()
        self.resolver = Resolver()
        if p('listen_discovery', True).value:
            from .spdp import SpdpListener
            spdp = SpdpListener()
            if spdp.start():
                self.resolver.spdp = spdp
        self.health = HealthEngine()
        self.full_name = self.get_fully_qualified_name()
        self.env = {k: os.environ.get(k, '') for k in ENV_KEYS}

        self.watched = {}           # topic -> _Watched
        self._ep_cache = {}         # topic -> {'key', 't'}: endpoint info, refetched on change
        self._ep_full_t = 0.0
        self._ep_nodes = frozenset()
        self.graph = {'nodes': {}, 'topics': {}}
        self.node_track = {}        # full name -> {'pid', 'first', 'restarts'}
        self.gone = {}
        self.ns_services = {}
        self.lifecycle = {}         # node -> state label
        self.container_members = {}
        self.ns_clients = {}
        self.ns_pending = {}
        self.diag = {}              # status name -> dict
        self.logs = deque(maxlen=3000)
        self.log_seq = 0
        self.log_lock = threading.Lock()
        self.log_times = {}         # node/logger -> deque of (t, level, msg)
        self.expect = self._load_baseline()
        self.snapshot = {}
        self.snapshot_json = b'{}'
        self.snapshot_cond = threading.Condition()
        self.issues = []
        self._diag_pub = None
        self._last_diag_pub = 0.0
        self._rosout_cls = get_message('rcl_interfaces/msg/Log') if get_message else None
        self._diag_cls = (get_message('diagnostic_msgs/msg/DiagnosticArray')
                          if get_message else None)

        self._feeds = []
        self._make_feeds()
        if self.p_pub_diag and self._diag_cls is not None:
            self._diag_pub = self.create_publisher(self._diag_cls, '/diagnostics', 10)

        self.ptable.refresh()
        self.host.refresh()
        self.create_timer(self.p_graph_period, self._graph_tick)
        self.create_timer(self.p_period, self._snapshot_tick)
        self.create_timer(0.25, self._sample_tick)
        self.create_timer(5.0, self._slow_tick)
        self._graph_tick()

    # ------------------------------------------------------------ feeds --
    def _make_feeds(self):
        from rclpy.serialization import deserialize_message
        self._deserialize = deserialize_message
        if self._rosout_cls is not None:
            w = _Watched('/rosout', 'rcl_interfaces/msg/Log')
            w.mode = 'feed'
            self.watched['/rosout'] = w
            self._feeds.append(self.create_subscription(
                self._rosout_cls, '/rosout', lambda raw: self._on_rosout(raw, w),
                FEED_QOS, raw=True, event_callbacks=_no_events()))
        if self._diag_cls is not None:
            w = _Watched('/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray')
            w.mode = 'feed'
            self.watched['/diagnostics'] = w
            self._feeds.append(self.create_subscription(
                self._diag_cls, '/diagnostics', lambda raw: self._on_diag(raw, w),
                FEED_QOS, raw=True, event_callbacks=_no_events()))

    def _on_rosout(self, raw, w):
        w.stats.arrivals.append((time.monotonic(), len(raw)))
        try:
            m = self._deserialize(raw, self._rosout_cls)
        except Exception:
            return
        if m.name == logger_name(self.full_name):
            return
        t = m.stamp.sec + m.stamp.nanosec * 1e-9
        level = int(m.level)
        with self.log_lock:
            self.log_seq += 1
            self.logs.append({'seq': self.log_seq, 't': t, 'level': level, 'logger': m.name,
                              'msg': m.msg[:2000], 'file': m.file, 'fn': m.function,
                              'line': m.line})
        if level >= 30:
            dq = self.log_times.setdefault(m.name, deque(maxlen=500))
            dq.append((time.monotonic(), level, m.msg[:300]))

    def _on_diag(self, raw, w):
        w.stats.arrivals.append((time.monotonic(), len(raw)))
        try:
            m = self._deserialize(raw, self._diag_cls)
        except Exception:
            return
        now = time.monotonic()
        for s in m.status:
            if s.hardware_id == self._hw_id():
                continue  # our own statuses
            lvl = s.level if isinstance(s.level, int) else ord(s.level)
            prev = self.diag.get(s.name)
            held_since = prev['level_since'] if prev and prev['level'] == lvl else now
            self.diag[s.name] = {
                'name': s.name, 'level': lvl, 'message': s.message,
                'hardware_id': s.hardware_id, 't': now, 'level_since': held_since,
                'values': [[kv.key, kv.value] for kv in s.values[:40]],
            }

    def _hw_id(self):
        return 'robot_monitor@' + self.host.hostname

    # ------------------------------------------------------------ graph --
    def _graph_tick(self):
        try:
            self._refresh_graph()
        except Exception as e:  # never let a graph glitch kill the monitor
            self.get_logger().warn('graph refresh failed: %r' % (e,), throttle_duration_sec=30)

    def _refresh_graph(self):
        now = time.monotonic()
        names = self.get_node_names_and_namespaces()
        counts = {}
        for name, ns in names:
            full = (ns.rstrip('/') + '/' + name) if ns != '/' else '/' + name
            counts[full] = counts.get(full, 0) + 1
        nodes = {}
        for full, c in counts.items():
            nodes[full] = {'name': full, 'count': c, 'pubs': [], 'subs': [],
                           'prefixes': set(), 'hidden': is_hidden_node(full),
                           'self': full == self.full_name}
        topics = {}
        own_prefixes = []
        node_set = frozenset(counts)
        full_refresh = (now - self._ep_full_t > 30.0) or node_set != self._ep_nodes
        if full_refresh:
            self._ep_full_t, self._ep_nodes = now, node_set
        cache = self._ep_cache
        live = set()
        for tname, types in self.get_topic_names_and_types():
            live.add(tname)
            try:  # cheap change detection: only rebuild endpoints when counts move
                key = (tuple(types), self.count_publishers(tname), self.count_subscribers(tname))
            except Exception:
                key = None
            ent = cache.get(tname)
            if ent is None or full_refresh or key is None or ent['key'] != key:
                ent = {'key': key, 't': self._fetch_topic(tname, types)}
                cache[tname] = ent
            t = ent['t']
            for kind in ('pubs', 'subs'):
                for ep in t[kind]:
                    full = ep['node']
                    if full not in nodes:  # node seen via endpoint before the node list
                        nodes[full] = {'name': full, 'count': 1, 'pubs': [], 'subs': [],
                                       'prefixes': set(), 'hidden': is_hidden_node(full),
                                       'self': full == self.full_name}
                    nodes[full][kind].append(tname)
                    pre = ep['_prefix']
                    if pre is not None:
                        nodes[full]['prefixes'].add(pre)
                        if ep['self']:
                            own_prefixes.append(pre)
            topics[tname] = t
        for tname in list(cache):
            if tname not in live:
                del cache[tname]

        cal = self.resolver.calibration
        if own_prefixes and (cal is None or not cal.ok):
            self.resolver.calibrate(own_prefixes, os.getpid())
        self.resolver.container_members = dict(self.container_members)
        res = self.resolver.resolve(nodes, self.ptable, self.full_name)
        for full, r in res.items():
            nodes[full].update(pid=r['pid'], how=r['how'], remote=r['remote'],
                               host_key=r.get('host_key'), host_ip=r.get('host_ip'))

        self._track_nodes(nodes, now)
        self.graph = {'nodes': nodes, 'topics': topics}
        if self.p_monitor_topics:
            self._manage_subscriptions(topics, now)
        try:
            self.ns_services = dict(self.get_service_names_and_types())
        except Exception:
            pass

    def _fetch_topic(self, tname, types):
        t = {'name': tname, 'types': [_short_type(x) for x in types], 'full_types': list(types),
             'pubs': [], 'subs': [], 'qos_issues': []}
        for kind, getter in (('pubs', self.get_publishers_info_by_topic),
                             ('subs', self.get_subscriptions_info_by_topic)):
            try:
                infos = getter(tname)
            except Exception:
                infos = []
            for i in infos:
                ns = i.node_namespace
                full = (ns.rstrip('/') + '/' + i.node_name) if ns != '/' else '/' + i.node_name
                gid = bytes(i.endpoint_gid)
                pre = gid[:12] if len(gid) >= 12 and gid[:12] != b'\0' * 12 else None
                t[kind].append({'node': full, 'qos': qosmod.qos_to_dict(i.qos_profile),
                                'type': _short_type(i.topic_type),
                                'self': full == self.full_name,
                                'hidden': is_hidden_node(full), '_prefix': pre})
        for sb in t['subs']:
            if sb['self']:
                continue
            for pb in t['pubs']:
                level, reason = qosmod.check(pb['qos'], sb['qos'])
                if level != 'ok' and pb['type'] == sb['type']:
                    t['qos_issues'].append({
                        'pub': pb['node'], 'sub': sb['node'], 'level': level, 'reason': reason,
                        'fix': qosmod.fix_hint(pb['qos'], sb['qos'], reason),
                        'pub_qos': qosmod.short(pb['qos']), 'sub_qos': qosmod.short(sb['qos'])})
        t['latched'] = bool(t['pubs']) and all(
            (p['qos'] or {}).get('durability') == 'transient_local' for p in t['pubs'])
        return t

    def _track_nodes(self, nodes, now):
        for full, n in nodes.items():
            tr = self.node_track.get(full)
            pid = n.get('pid')
            if tr is None:
                tr = {'pid': pid, 'first': time.time(), 'restarts': None}
                self.node_track[full] = tr
                g = self.gone.pop(full, None)
                if g is not None:
                    self._note_restart(tr, g.get('pid'), pid, now)
            elif pid and tr['pid'] and pid != tr['pid']:
                self._note_restart(tr, tr['pid'], pid, now)
            if pid:
                tr['pid'] = pid
            n['first_seen'] = tr['first']
            n['restarts'] = tr['restarts']
        for full in list(self.node_track):
            if full not in nodes:
                tr = self.node_track.pop(full)
                if not is_hidden_node(full):
                    p = self.ptable.get(tr['pid']) if tr['pid'] else None
                    self.gone[full] = {'t': now, 'pid': tr['pid'],
                                       'proc': p.display_name() if p else None,
                                       'restarts': tr['restarts']}
        for full in list(self.gone):
            if now - self.gone[full]['t'] > 3600:
                del self.gone[full]

    @staticmethod
    def _note_restart(tr, old, new, now):
        r = tr.get('restarts') or {'count': 0}
        r.update(count=r['count'] + 1, last=now, old_pid=old, new_pid=new)
        tr['restarts'] = r

    # ---------------------------------------------------- subscriptions --
    def _manage_subscriptions(self, topics, now):
        for tname, t in topics.items():
            if tname in FEED_TOPICS or tname in NOT_MONITORED:
                continue
            w = self.watched.get(tname)
            others = [p for p in t['pubs'] if not p['self']]
            if w is None:
                if not others:
                    continue
                w = _Watched(tname, t['full_types'][0])
                self.watched[tname] = w
                self._prepare(w, t)
            if w.mode in ('unavailable', 'excluded'):
                continue
            if not others and w.sub is not None:
                self._unsubscribe(w)
            elif others and w.sub is None and w.mode == 'continuous':
                self._subscribe(w)
            elif others and w.mode == 'sampled' and w.sub is None and now >= w.next_on:
                self._subscribe(w)
                w.window_end = now + w.on_s
        for tname in list(self.watched):
            if tname not in topics and tname not in FEED_TOPICS:
                w = self.watched.pop(tname)
                self._unsubscribe(w)

    def _prepare(self, w, t):
        if any(rx.search(w.name) for rx in self.p_exclude):
            w.mode, w.reason = 'excluded', 'excluded by the exclude_topics parameter'
            return
        if len(t['full_types']) > 1:
            w.reason = 'several types on one topic; measuring %s' % w.type
        try:
            w.cls = get_message(w.type)
        except Exception as e:
            w.mode = 'unavailable'
            w.reason = 'message type %s is not installed on this machine (%s)' % (
                w.type, type(e).__name__)
            return
        w.reader = stamp_reader(w.cls)
        w.heavy = w.type in HEAVY_TYPES  # checked for cost after 10 messages, not 50
        w.on_s = self.p_on_s

    def _subscribe(self, w):
        # Hot path: runs for every message, so keep it to a few local lookups.
        arr_append, age_append, reader = w.stats.arrivals.append, w.stats.ages.append, w.reader
        mono = time.monotonic
        if reader is None:
            def cb(raw):
                arr_append((mono(), len(raw)))
        else:
            if self._sim_time():
                clock = self.get_clock()

                def now_ns():
                    return clock.now().nanoseconds
            else:
                now_ns = time.time_ns  # ROS time == system time without use_sim_time

            def cb(raw):
                t = mono()
                arr_append((t, len(raw)))
                try:
                    st = reader(raw)
                except Exception:
                    return
                if st is not None and st[0] > 0:
                    age = (now_ns() - (st[0] * 1000000000 + st[1])) * 1e-9
                    if -3.0e7 < age < 3.0e7:   # else: unset or sim-time stamp
                        age_append((t, age))
        try:
            w.sub = self.create_subscription(w.cls, w.name, cb, MONITOR_QOS, raw=True,
                                             event_callbacks=_no_events())
        except Exception as e:
            w.mode, w.reason = 'unavailable', 'cannot subscribe: %s' % e

    def _sim_time(self):
        try:
            return bool(self.get_parameter('use_sim_time').value)
        except Exception:
            return False

    def _unsubscribe(self, w):
        if w.sub is not None:
            try:
                self.destroy_subscription(w.sub)
            except Exception:
                pass
            w.sub = None

    def _enforce_budget(self, now):
        """
        Keep the total received message rate under max_total_rate_hz.

        Every message costs Python time in this process, so on a robot with hundreds of
        topics the busiest ones are sampled instead of received continuously.
        """
        rates = []
        for w in self.watched.values():
            if w.mode != 'continuous' or w.sub is None:
                continue
            arr = w.stats.arrivals
            n = 0
            for t, _ in reversed(arr):
                if t < now - 3.0:
                    break
                n += 1
            rates.append((n / 3.0, w))
        total = sum(r for r, _ in rates)
        if total <= self.p_max_total_hz:
            return
        for r, w in sorted(rates, key=lambda x: -x[0]):
            if total <= self.p_max_total_hz * 0.9 or r < 5:
                break
            w.mode = 'sampled'
            w.on_s = min(10.0, max(self.p_on_s, 6.0 / r))
            w.window_end = now + w.on_s
            w.reason = ('busy system (%.0f msg/s in total): sampled %.0fs in every %.0fs to '
                        'stay under max_total_rate_hz' % (total, w.on_s, w.on_s + self.p_off_s))
            total -= r * (1.0 - w.on_s / (w.on_s + self.p_off_s))

    def _sample_tick(self):
        now = time.monotonic()
        if now - self._last_budget >= 2.0:
            self._last_budget = now
            if self.p_sample_heavy:
                self._enforce_budget(now)
        for w in list(self.watched.values()):
            if w.mode == 'sampled' and w.sub is not None and now >= w.window_end:
                arr = w.stats.arrivals
                got = sum(1 for t, _ in arr if t >= now - w.on_s)
                if got < 4 and w.on_s < 10.0:
                    w.window_end = now + 1.0  # slow topic: keep the window open a bit longer
                    w.on_s += 1.0
                    continue
                w.frozen = w.stats.compute(now, window=w.on_s)
                self._unsubscribe(w)
                w.next_on = now + self.p_off_s
            elif w.mode == 'continuous' and w.sub is not None and self.p_sample_heavy:
                # Switch to sampling if this topic turns out to be expensive.
                arr = w.stats.arrivals
                k = 10 if w.heavy else 50
                if len(arr) > k and arr[-1][0] > now - 1.0:
                    t0 = arr[-k][0]
                    dt = now - t0
                    if dt > 0:
                        hz = k / dt
                        bw = sum(s for _, s in itertools.islice(reversed(arr), k)) / dt
                        if hz > self.p_max_hz or bw > self.p_heavy_mbps * 1048576:
                            w.mode = 'sampled'
                            w.on_s = min(10.0, max(self.p_on_s, 6.0 / hz))
                            w.window_end = now + w.on_s
                            what = ('%.0f Hz' % hz if hz > self.p_max_hz
                                    else '%.1f MB/s' % (bw / 1048576))
                            w.reason = '%s: sampled %.0fs in every %.0fs to save CPU' % (
                                what, w.on_s, w.on_s + self.p_off_s)

    # -------------------------------------------- lifecycle, composition --
    def _slow_tick(self):
        try:
            self._poll_services()
        except Exception as e:
            self.get_logger().warn('service poll failed: %r' % (e,), throttle_duration_sec=60)

    def _poll_services(self):
        if get_service is None:
            return
        wanted = {}
        for srv, types in self.ns_services.items():
            if srv.endswith('/get_state') and 'lifecycle_msgs/srv/GetState' in types:
                wanted[srv] = ('lifecycle', srv[:-len('/get_state')])
            elif srv.endswith('/_container/list_nodes') and \
                    'composition_interfaces/srv/ListNodes' in types:
                wanted[srv] = ('container', srv[:-len('/_container/list_nodes')])
        for srv in list(self.ns_clients):
            if srv not in wanted:
                self.destroy_client(self.ns_clients.pop(srv)[0])
                self.ns_pending.pop(srv, None)
        for srv, (kind, node) in wanted.items():
            if srv in self.ns_pending and not self.ns_pending[srv].done():
                continue
            if srv not in self.ns_clients:
                st = get_service('lifecycle_msgs/srv/GetState' if kind == 'lifecycle'
                                 else 'composition_interfaces/srv/ListNodes')
                self.ns_clients[srv] = (self.create_client(st, srv), st)
            cli, st = self.ns_clients[srv]
            if not cli.service_is_ready():
                continue
            fut = cli.call_async(st.Request())
            self.ns_pending[srv] = fut
            fut.add_done_callback(lambda f, k=kind, n=node: self._on_service_reply(k, n, f))

    def _on_service_reply(self, kind, node, fut):
        try:
            r = fut.result()
        except Exception:
            return
        if r is None:
            return
        if kind == 'lifecycle':
            self.lifecycle[node] = r.current_state.label
        else:
            for member in r.full_node_names:
                self.container_members[member] = node

    # ------------------------------------------------------------ procs --
    def _proc_tick(self):
        try:
            self.ptable.refresh()
            self.host.refresh()
        except Exception as e:
            self.get_logger().warn('process refresh failed: %r' % (e,), throttle_duration_sec=60)

    # --------------------------------------------------------- snapshot --
    def watching(self):
        """Return True while a dashboard, API client or CLI is looking at the data."""
        return self.always_active or self.watchers > 0 or \
            time.monotonic() - self.last_api_hit < 10.0

    def _snapshot_tick(self):
        # Full refresh every refresh_period while someone watches; every idle_period
        # otherwise (still enough for /diagnostics and rate learning, at a third of the CPU).
        now = time.monotonic()
        period = self.p_period if self.watching() else max(self.p_period, self.p_idle_period)
        if now - self._last_proc >= max(2.0, period) - 0.05:
            self._last_proc = now
            self._proc_tick()
        if now - self._last_snap < period - 0.05:
            return
        self._last_snap = now
        try:
            self._build_snapshot()
        except Exception as e:
            self.get_logger().error('snapshot failed: %r' % (e,), throttle_duration_sec=30)

    def _build_snapshot(self):
        now = time.monotonic()
        nodes, topics = self.graph['nodes'], self.graph['topics']
        exp_topics = (self.expect or {}).get('topics') or {}

        # topic stats
        for tname, t in topics.items():
            w = self.watched.get(tname)
            exp = exp_topics.get(tname)
            # Topic dicts are cached between graph scans: reset every per-snapshot field.
            t['expected_hz'] = exp.get('hz') if isinstance(exp, dict) else None
            t['monitor_reason'] = ''
            t['sampled_off'] = False
            t['future_frac'] = 0
            if w is None:
                t['monitor'] = 'no publisher' if not [p for p in t['pubs'] if not p['self']] \
                    else ('not monitored' if tname in NOT_MONITORED else 'starting')
                t['stats'] = None
                continue
            t['monitor'], t['monitor_reason'] = w.mode, w.reason
            if w.mode in ('unavailable', 'excluded'):
                t['stats'] = None
                continue
            if w.mode == 'sampled':
                if w.frozen is None and w.sub is not None:
                    st = w.stats.compute(now, window=w.on_s)
                else:
                    st = dict(w.frozen) if w.frozen else None
                    if st is not None:
                        w.stats.rate_hist.append(st['hz'])
                        st['silent_s'] = None  # unknown while not subscribed
                t['sampled_off'] = w.sub is None
            else:
                st = w.stats.compute(now)
            if st is not None:
                st['hist'] = list(w.stats.rate_hist)[-60:]
                ages = [a for _, a in w.stats.ages]
                t['future_frac'] = (sum(1 for a in ages[-50:] if a < -0.05) / len(ages[-50:])) \
                    if ages else 0
                if w.reader is not None:
                    st['stamped'] = True
            t['stats'] = st

        # processes
        procs = {}
        node_pids = {}
        for full, n in nodes.items():
            if n.get('pid'):
                node_pids.setdefault(n['pid'], []).append(full)
        for pr in self.ptable.ros_procs():
            slope = rss_slope_mb_per_min(pr)
            h = pr.rss_hist
            growth = (h[-1][1] - min(r for _, r in h)) / 1048576 if len(h) > 1 else 0
            procs[pr.pid] = {
                'pid': pr.pid, 'key': '%d:%d' % (pr.pid, pr.start), 'name': pr.display_name(),
                'cmd': redact_cmdline(pr.cmdline)[:600], 'exe': pr.exe,
                'nodes': sorted(node_pids.get(pr.pid, [])), 'cpu': round(pr.cpu, 1),
                'rss': pr.rss, 'threads': pr.threads, 'fds': pr.fds, 'state': pr.state,
                'uptime': round(pr.uptime(self.ptable.btime)), 'user': pr.user,
                'ppid': pr.ppid, 'gpu_mem': self.host.gpu_mem_of(pr.pid),
                'cpu_hist': list(pr.cpu_hist)[-60:],
                'rss_slope': None if slope is None else round(slope, 2),
                'rss_growth_mb': round(growth, 1),
                'rss_window_s': (h[-1][0] - h[0][0]) if len(h) > 1 else 0,
                'self': pr.pid == self.ptable.self_pid,
            }
        for full, n in nodes.items():
            pr = procs.get(n.get('pid'))
            n['cpu'] = pr['cpu'] if pr else None
            n['rss'] = pr['rss'] if pr else None
            n['lifecycle'] = self.lifecycle.get(full)
            n['shared_process'] = bool(pr and len(pr['nodes']) > 1)
            n['container'] = self.container_members.get(full)

        me = self.ptable.get(self.ptable.self_pid)
        selfinfo = {'pid': self.ptable.self_pid, 'node': self.full_name,
                    'cpu': round(me.cpu, 1) if me else None, 'rss': me.rss if me else None,
                    'subs': sum(1 for w in self.watched.values() if w.sub is not None),
                    'version': __version__}

        # logs per node
        log_counts = {}
        cutoff = now - 300
        for lg, dq in self.log_times.items():
            while dq and dq[0][0] < cutoff:
                dq.popleft()
            errs = [x for x in dq if x[1] >= 40]
            node = self._logger_to_node(lg, nodes)
            log_counts[node] = {
                'warn5m': sum(1 for x in dq if x[1] == 30), 'error5m': len(errs),
                'fatal5m': sum(1 for x in dq if x[1] >= 50),
                'last_error': errs[-1][2] if errs else None}
        for name in [n for n, d in self.diag.items() if now - d['t'] > 600]:
            del self.diag[name]  # a status nobody has sent for 10 minutes is gone
        diag_list = []
        for d in self.diag.values():
            item = dict(d)
            item['age_s'] = round(now - d['t'], 1)
            item['held_s'] = round(now - d['level_since'], 1)
            del item['t'], item['level_since']
            diag_list.append(item)
        diag_list.sort(key=lambda d: (-d['level'] if d['level'] != 3 else -1.5, d['name']))

        model = {'nodes': nodes, 'topics': topics, 'procs': procs, 'host': self.host.data,
                 'gone': dict(self.gone), 'env': self.env,
                 'uptime': now - self.started, 'self': selfinfo, 'log_counts': log_counts,
                 'diagnostics': diag_list, 'expect': self.expect}
        issues = self.health.evaluate(model, now)
        self.issues = issues
        snap = self._serialize(model, issues, now)
        data = json.dumps(snap, separators=(',', ':'), default=_json_default).encode()
        with self.snapshot_cond:
            self.snapshot, self.snapshot_json = snap, data
            self.snapshot_cond.notify_all()
        if self._diag_pub is not None and now - self._last_diag_pub >= 2.9:
            self._last_diag_pub = now
            self._publish_diagnostics(issues, snap['score'])

    def _logger_to_node(self, lg, nodes):
        cand = '/' + lg.replace('.', '/')
        return cand if cand in nodes else lg

    def _serialize(self, m, issues, now):
        nodes = []
        for n in m['nodes'].values():
            d = {k: v for k, v in n.items() if k != 'prefixes'}
            d['pubs'] = sorted(set(n['pubs']))
            d['subs'] = sorted(set(n['subs']))
            d['services'] = sum(1 for s in self.ns_services if s.startswith(n['name'] + '/'))
            d['participant'] = sorted(p.hex() for p in n['prefixes'])[:1]
            r = d.get('restarts')
            if r:
                d['restarts'] = {'count': r['count'], 'ago_s': round(now - r['last'])}
            nodes.append(d)
        topics = []
        for t in m['topics'].values():
            d = {k: v for k, v in t.items() if k != 'full_types'}
            d['hidden'] = is_hidden_topic(t['name'])
            d['system'] = t['name'] in ('/rosout', '/parameter_events')
            for kind in ('pubs', 'subs'):
                # Compact endpoints: the dashboard only needs who and the QoS summary.
                d[kind] = [{'node': ep['node'], 'qos_short': qosmod.short(ep['qos']),
                            'self': ep['self'], 'hidden': ep['hidden']} for ep in t[kind]]
            topics.append(d)
        gone = [{'name': k, 'ago_s': round(now - v['t']), 'pid': v['pid'], 'proc': v['proc']}
                for k, v in m['gone'].items()]
        with self.log_lock:
            log_seq = self.log_seq
        return {
            'version': __version__, 'ts': time.time(), 'uptime': round(m['uptime']),
            'host': m['host'], 'self': m['self'], 'env': m['env'],
            'score': self.health.score(issues), 'issues': issues,
            'nodes': sorted(nodes, key=lambda n: n['name']),
            'topics': sorted(topics, key=lambda t: t['name']),
            'procs': sorted(m['procs'].values(), key=lambda p: -p['cpu']),
            'gone': gone, 'diagnostics': m['diagnostics'], 'log_counts': m['log_counts'],
            'log_seq': log_seq,
            'discovery_listener': ('on' if self.resolver.spdp is not None else 'off'),
            'calibration': self.resolver.calibration.describe()
            if self.resolver.calibration else 'pending',
            'baseline': {'file': self.p_baseline, 'loaded': bool(self.expect),
                         'nodes': len((self.expect or {}).get('nodes', [])),
                         'topics': len((self.expect or {}).get('topics', {}))},
            'top': [{'pid': p.pid, 'name': p.display_name(), 'cpu': round(p.cpu, 1),
                     'rss': p.rss, 'ros': p.is_ros} for p in self.ptable.top(8)],
        }

    def _publish_diagnostics(self, issues, score):
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        lvl = {'error': DiagnosticStatus.ERROR, 'warn': DiagnosticStatus.WARN,
               'info': DiagnosticStatus.OK}
        worst = max([lvl[i['severity']] for i in issues] or [DiagnosticStatus.OK],
                    key=lambda b: b if isinstance(b, int) else ord(b))
        s = DiagnosticStatus(level=worst, name='robot_monitor: summary', hardware_id=self._hw_id(),
                             message='health %d/100, %d issue(s)' % (score, len(issues)))
        n_err = sum(i['severity'] == 'error' for i in issues)
        n_warn = sum(i['severity'] == 'warn' for i in issues)
        s.values = [KeyValue(key='score', value=str(score)),
                    KeyValue(key='errors', value=str(n_err)),
                    KeyValue(key='warnings', value=str(n_warn))]
        arr.status.append(s)
        for i in issues[:40]:
            if i['severity'] == 'info':
                continue
            st = DiagnosticStatus(level=lvl[i['severity']], name='robot_monitor: ' + i['id'][:120],
                                  hardware_id=self._hw_id(), message=i['title'][:200])
            st.values = [KeyValue(key='detail', value=i['detail'][:500]),
                         KeyValue(key='fix', value=i['fix'][:500])]
            arr.status.append(st)
        self._diag_pub.publish(arr)

    # ------------------------------------------------------------ logs ---
    def logs_after(self, after=0, limit=500):
        with self.log_lock:
            items = [x for x in self.logs if x['seq'] > after]
        return items[-limit:]

    # -------------------------------------------------------- baseline ---
    def _load_baseline(self):
        try:
            import yaml
            with open(self.p_baseline) as f:
                data = normalize_baseline(yaml.safe_load(f))
            self.get_logger().info('loaded baseline %s (%d nodes, %d topics)' % (
                self.p_baseline, len(data.get('nodes', [])), len(data.get('topics', {}))))
            return data
        except FileNotFoundError:
            return None
        except Exception as e:
            self.get_logger().warn('cannot read baseline %s: %s' % (self.p_baseline, e))
            return None

    def baseline_from_now(self):
        nodes = sorted(n['name'] for n in self.graph['nodes'].values()
                       if not n['hidden'] and not n['self'])
        topics = {}
        for tname, t in self.graph['topics'].items():
            st = t.get('stats')
            if not st or is_hidden_topic(tname) or not st.get('periodic'):
                continue
            hz = st.get('learned_hz') or st.get('hz')
            if hz and hz > 0.05:
                topics[tname] = {'hz': round(hz, 2)}
        return {'nodes': nodes, 'topics': topics,
                'created': time.strftime('%Y-%m-%dT%H:%M:%S'), 'host': socket.gethostname()}

    def save_baseline(self):
        import yaml
        data = self.baseline_from_now()
        os.makedirs(os.path.dirname(self.p_baseline) or '.', exist_ok=True)
        tmp = self.p_baseline + '.tmp'
        with open(tmp, 'w') as f:
            f.write('# robot_monitor baseline: the nodes and topic rates this robot should have.\n'
                    '# Edit freely. Topics alert when slower than 80% of "hz".\n')
            yaml.safe_dump(data, f, sort_keys=False)
        os.replace(tmp, self.p_baseline)
        self.expect = data
        return data

    def clear_baseline(self):
        try:
            os.remove(self.p_baseline)
        except FileNotFoundError:
            pass
        self.expect = None


_SECRET_RE = re.compile(
    r'(pass(word|wd)?|token|secret|api[_-]?key|auth|credential|private[_-]?key)', re.IGNORECASE)


def redact_cmdline(args):
    """Join a command line for display, hiding values of secret-looking arguments."""
    out, hide_next = [], False
    for a in args:
        if hide_next:
            out.append('***')
            hide_next = False
            continue
        key, sep, val = a.partition(':=') if ':=' in a else a.partition('=')
        if sep and _SECRET_RE.search(key):
            out.append(key + sep + '***')
        elif a.startswith('-') and _SECRET_RE.search(a) and not sep:
            out.append(a)
            hide_next = True
        else:
            out.append(a)
    return ' '.join(out)


def normalize_baseline(data):
    """Validate a (possibly hand-edited) baseline into {'nodes': [...], 'topics': {...}}."""
    if not isinstance(data, dict):
        data = {}

    def fqn(n):
        n = str(n).strip()
        return n if n.startswith('/') else '/' + n
    nodes = data.get('nodes') or []
    if isinstance(nodes, (str, dict)):
        nodes = [nodes] if isinstance(nodes, str) else list(nodes)
    topics = {}
    raw = data.get('topics') or {}
    if isinstance(raw, list):
        raw = {t: {} for t in raw}
    if isinstance(raw, dict):
        for t, rule in raw.items():
            hz = None
            if isinstance(rule, dict):
                hz = rule.get('hz', rule.get('min_hz'))
            elif isinstance(rule, (int, float)):
                hz = rule
            try:
                hz = float(hz) if hz is not None else None
            except (TypeError, ValueError):
                hz = None
            topics[fqn(t)] = {'hz': hz} if hz else {}
    out = dict(data)
    out['nodes'] = sorted({fqn(n) for n in nodes if str(n).strip()})
    out['topics'] = topics
    return out


def _no_events():
    # The default QoS-event handlers cost two extra waitables per subscription that the
    # executor walks on every wake-up; a probe subscription doesn't need them.
    try:
        return SubscriptionEventCallbacks(use_default_callbacks=False)
    except TypeError:
        return SubscriptionEventCallbacks()


def make_executor():
    """Create the fastest executor this rclpy offers (EventsExecutor on Jazzy and newer)."""
    if os.environ.get('ROBOT_MONITOR_EXECUTOR', 'auto') != 'single':
        try:
            from rclpy.experimental import EventsExecutor
            return EventsExecutor()
        except Exception:
            pass
    from rclpy.executors import SingleThreadedExecutor
    return SingleThreadedExecutor()


def _short_type(t):
    return t.replace('/msg/', '/')


def _disk_paths():
    paths = ['/']
    for p in (os.path.expanduser('~'), os.environ.get('ROS_LOG_DIR'), '/var/log', '/tmp'):
        if p and os.path.isdir(p):
            paths.append(p)
    return paths


def _json_default(o):
    if isinstance(o, (set, frozenset)):
        return sorted(o)
    if isinstance(o, bytes):
        return o.hex()
    return str(o)


def spin_in_thread(node):
    """Spin a node in a daemon thread; returns (executor, thread)."""
    ex = make_executor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, name='robot_monitor-ros', daemon=True)
    th.start()
    return ex, th


__all__ = ['Monitor', 'spin_in_thread', 'rclpy']
