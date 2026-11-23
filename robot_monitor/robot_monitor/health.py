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
Health rules: turn the observed graph, topic stats and process stats into issues.

Every issue says what is wrong, why it matters and what to do, in plain words.
Rules are deliberately conservative: a false alarm costs more trust than a
missed nit, so time-based rules only fire after the condition has held for a
while, and event-driven topics are never called "stale".
"""

import difflib
import time

SEVERITY_RANK = {'error': 3, 'warn': 2, 'info': 1}
SEVERITY_WEIGHT = {'error': 18, 'warn': 6, 'info': 0}

# Topics every node creates or that are optional by design; never linted for
# "no publisher" / "no subscriber".
SYSTEM_TOPICS = {'/rosout', '/parameter_events', '/clock', '/tf', '/tf_static',
                 '/diagnostics', '/diagnostics_agg', '/robot_description',
                 '/initialpose', '/goal_pose', '/clicked_point', '/joy', '/cmd_vel'}


def is_hidden_node(full):
    base = full.rsplit('/', 1)[-1]
    return base.startswith('_') or base.startswith('launch_ros_') \
        or base.startswith('transform_listener_impl_')


def is_hidden_topic(t):
    return '/_' in t or t.startswith('/_') or t.endswith('/_action/status') \
        or '/_action/' in t


class Issue(dict):
    """A dict with a stable id so the UI can keep selection across refreshes."""

    def __init__(self, iid, severity, category, title, detail='', fix='', subjects=(),
                 since=None, focus=None):
        # subjects: everything involved (shown as links); focus: what is actually broken
        # (coloured red/yellow in the UI). Defaults to all subjects.
        super().__init__(id=iid, severity=severity, category=category, title=title,
                         detail=detail, fix=fix, subjects=list(subjects), since=since,
                         focus=list(focus) if focus is not None else list(subjects))


class HealthEngine:

    def __init__(self, cfg=None):
        self.cfg = {
            'cpu_saturated_pct': 90.0,     # % of one core, sustained
            'cpu_saturated_s': 30.0,
            'leak_mb_per_min': 1.0,
            'leak_min_growth_mb': 50.0,
            'stale_min_s': 3.0,
            'rate_drop_ratio': 0.6,
            'rate_drop_s': 15.0,
            'dangling_grace_s': 20.0,
            'temp_warn_c': 80.0,
            'temp_error_c': 85.0,
            'mem_warn_pct': 90.0,
            'mem_error_pct': 95.0,
            'disk_warn_pct': 90.0,
            'disk_error_pct': 97.0,
            'age_warn_ms': 1000.0,
        }
        if cfg:
            self.cfg.update({k: v for k, v in cfg.items() if k in self.cfg})
        self._since = {}     # condition id -> monotonic time it started
        self._first = {}     # issue id -> wall time first reported (for the UI)
        self._gone_at = {}   # issue id -> monotonic time it cleared
        self._touched = set()

    def _held(self, key, cond, now, hold):
        """Return True once `cond` has been continuously true for `hold` seconds."""
        self._touched.add(key)
        if not cond:
            self._since.pop(key, None)
            return False
        start = self._since.setdefault(key, now)
        return now - start >= hold

    def evaluate(self, model, now=None):
        now = time.monotonic() if now is None else now
        self._touched = set()
        issues = []
        add = issues.append
        self._graph_rules(model, now, add)
        self._topic_rules(model, now, add)
        self._process_rules(model, now, add)
        self._host_rules(model, now, add)
        self._feed_rules(model, now, add)
        self._tf_rules(model, now, add)
        self._expectation_rules(model, now, add)
        wall = time.time()
        live = set()
        for i in issues:
            live.add(i['id'])
            i['since'] = self._first.setdefault(i['id'], wall)
        # Remember when issues started for a while after they clear, so a flapping
        # condition keeps its original "since" instead of looking brand new.
        for k in live:
            self._gone_at.pop(k, None)
        for k in list(self._first):
            if k not in live:
                t0 = self._gone_at.setdefault(k, now)
                if now - t0 > 300:
                    del self._first[k]
                    del self._gone_at[k]
        # Forget timers of things that no longer exist (exited processes, removed topics).
        for k in [k for k in self._since if k not in self._touched]:
            del self._since[k]
        issues.sort(key=lambda i: (-SEVERITY_RANK[i['severity']], i['category'], i['title']))
        return issues

    @staticmethod
    def score(issues):
        s = 100
        for i in issues:
            s -= SEVERITY_WEIGHT[i['severity']]
        return max(0, s)

    # ------------------------------------------------------------ graph --
    def _graph_rules(self, m, now, add):
        nodes = m['nodes']
        visible = [n for n in nodes.values() if not n.get('hidden') and not n.get('self')]
        if not visible and m.get('uptime', 0) > 8:
            env = m.get('env', {})
            add(Issue('discovery:none', 'warn', 'discovery', 'No other ROS nodes are visible',
                      'This monitor runs with ROS_DOMAIN_ID=%s, RMW_IMPLEMENTATION=%s and '
                      'ROS_AUTOMATIC_DISCOVERY_RANGE=%s. Nodes in another domain, with another '
                      'middleware or a stricter discovery range are invisible to each other.' % (
                          env.get('ROS_DOMAIN_ID') or '0 (default)',
                          env.get('RMW_IMPLEMENTATION') or 'default',
                          env.get('ROS_AUTOMATIC_DISCOVERY_RANGE')
                          or env.get('ROS_LOCALHOST_ONLY') and 'LOCALHOST' or 'default'),
                      'Start the monitor with the same environment as your robot (source the '
                      'same setup file, same ROS_DOMAIN_ID and RMW). Across machines, check that '
                      'multicast works or configure a discovery server / static peers.'))
        for name, n in nodes.items():
            if n.get('count', 1) > 1:
                add(Issue('dup:' + name, 'error', 'graph',
                          '%d nodes share the name %s' % (n['count'], name),
                          'ROS 2 allows this, but parameters, services, logs and tools then '
                          'address the wrong node at random.',
                          'Give each node a unique name or namespace (Node(name=..., '
                          'namespace=...) in the launch file, or -r __node:=...).', [name]))
        expected = set((m.get('expect') or {}).get('nodes', []))
        for name, g in m.get('gone', {}).items():
            # A node in the baseline gets the stronger "expected node missing" error instead.
            if now - g['t'] < 600 and name not in expected:
                add(Issue('gone:' + name, 'warn', 'graph', 'Node %s disappeared' % name,
                          'It was running and left the graph %s ago.' % _ago(now - g['t'])
                          + (' Last process: %s (pid %s).' % (g['proc'], g['pid'])
                             if g.get('pid') else ''),
                          'Check its log (Logs tab) and its supervisor (launch, systemd, '
                          'container) for a crash.', [name]))
        for name, n in nodes.items():
            if n.get('restarts'):
                r = n['restarts']
                if now - r['last'] < 900:
                    add(Issue('restart:' + name, 'warn', 'graph',
                              'Node %s restarted %d time%s' % (
                                  name, r['count'], '' if r['count'] == 1 else 's'),
                              ('Its process changed (pid %s \u2192 %s), last %s ago.' % (
                                  r.get('old_pid'), r.get('new_pid'), _ago(now - r['last']))
                               if r.get('old_pid') and r.get('new_pid') else
                               'It left the graph and came back, last %s ago.' % _ago(
                                   now - r['last'])),
                              'A respawning node usually means a crash loop: look at its last '
                              'error log lines.', [name]))
        for name, n in nodes.items():
            st = n.get('lifecycle')
            if st and st not in ('active',) and self._held('lc:' + name, True, now, 30):
                add(Issue('lifecycle:' + name, 'info', 'graph',
                          'Lifecycle node %s is %s' % (name, st),
                          'It is not processing data until it is activated.',
                          'ros2 lifecycle set %s configure / activate, or check its '
                          'lifecycle manager.' % name, [name]))
            elif not st:
                self._since.pop('lc:' + name, None)

    # ----------------------------------------------------------- topics --
    def _topic_rules(self, m, now, add):
        topics = m['topics']
        published = [t for t, d in topics.items() if d['pubs']]
        for t, d in topics.items():
            if is_hidden_topic(t):
                continue
            if len(d['types']) > 1:
                add(Issue('types:' + t, 'error', 'topic',
                          '%s is used with %d different types' % (t, len(d['types'])),
                          'Types: %s. Endpoints with different types never connect.'
                          % ', '.join(d['types']),
                          'Make every publisher and subscriber of this topic use the same '
                          'message type, or rename one of the topics.', [t]))
            for pair in d.get('qos_issues', []):
                sev = 'error' if pair['level'] == 'error' else 'warn'
                add(Issue('qos:%s:%s:%s' % (t, pair['pub'], pair['sub']), sev, 'qos',
                          'QoS mismatch on %s: %s → %s' % (t, pair['pub'], pair['sub']),
                          'No messages flow from this publisher to this subscriber: '
                          + pair['reason'] + '.' if sev == 'error' else pair['reason'],
                          pair['fix'], [t, pair['pub'], pair['sub']], focus=[t, pair['sub']]))
            subs_real = [s for s in d['subs'] if not s.get('self') and not s.get('hidden')]
            if subs_real and not d['pubs'] and t not in SYSTEM_TOPICS:
                if self._held('dangling:' + t, True, now, self.cfg['dangling_grace_s']):
                    close = difflib.get_close_matches(t, published, n=1, cutoff=0.75)
                    who = ', '.join(sorted({s['node'] for s in subs_real}))
                    if close:
                        add(Issue('dangling:' + t, 'warn', 'topic',
                                  'Nobody publishes %s — did you mean %s?' % (t, close[0]),
                                  '%s subscribe%s to %s, which has no publisher, while %s '
                                  'exists with type %s.' % (
                                      who, 's' if len(subs_real) == 1 else '', t, close[0],
                                      ', '.join(topics[close[0]]['types'])),
                                  'Fix the topic name or add a remapping '
                                  '(-r %s:=%s).' % (t, close[0]), [t, close[0]] + sorted(
                                      {x['node'] for x in subs_real}), focus=[t]))
                    else:
                        add(Issue('dangling:' + t, 'info', 'topic',
                                  '%s has subscribers but no publisher' % t,
                                  'Waiting: %s.' % who,
                                  'Fine if the publisher starts later; otherwise check the '
                                  'name and namespace.', [t]))
            else:
                self._since.pop('dangling:' + t, None)

            s = d.get('stats')
            if not s or not d['pubs'] or d.get('latched'):
                continue
            learned = s.get('learned_hz')
            period = 1.0 / learned if learned else None
            stale_after = max(self.cfg['stale_min_s'], 5 * period) if period else None
            silent = s.get('silent_s')
            is_stale = bool(s.get('periodic') and stale_after and silent is not None
                            and silent > stale_after and not d.get('sampled_off'))
            if is_stale:
                add(Issue('stale:' + t, 'warn', 'topic', '%s stopped publishing' % t,
                          'It was steady at %.3g Hz and has been silent for %s.' % (
                              learned, _ago(silent)),
                          'Check the publisher (%s): is it stuck, blocked on hardware, or '
                          'starved of CPU?' % ', '.join(sorted({p['node'] for p in d['pubs']})),
                          [t] + [p['node'] for p in d['pubs']]))
            exp = d.get('expected_hz')
            ref = exp or learned
            drop = bool(ref and s.get('periodic') and not is_stale and s['hz'] > 0
                        and s['hz'] < ref * (0.8 if exp else self.cfg['rate_drop_ratio']))
            if self._held('drop:' + t, drop, now, self.cfg['rate_drop_s']):
                add(Issue('drop:' + t, 'warn', 'topic',
                          '%s is slow: %.3g Hz instead of %.3g Hz' % (t, s['hz'], ref),
                          ('Expected rate from your baseline.' if exp else
                           'Learned from its own steady rate.') +
                          ' Max gap %s ms, jitter %s ms.' % (
                              s.get('max_gap_ms'), s.get('jitter_ms')),
                          'Usually the publisher is CPU-bound, waiting on hardware, or its '
                          'executor is busy with other callbacks.', [t]))
            if s.get('age_ms') is not None and d.get('future_frac', 0) > 0.5:
                add(Issue('future:' + t, 'warn', 'time',
                          '%s is stamped in the future' % t,
                          "Message stamps are ahead of this machine's clock by about %s ms. "
                          'Clocks are out of sync, so TF lookups and message filters fail.'
                          % int(-s['age_ms']),
                          'Sync clocks with chrony/NTP (or PTP) on every machine; check '
                          'use_sim_time is the same everywhere.', [t]))
            elif s.get('age_ms') is not None and s['age_ms'] > self.cfg['age_warn_ms'] \
                    and s['age_ms'] < 3.6e6:
                if self._held('age:' + t, True, now, 10):
                    add(Issue('age:' + t, 'warn', 'time',
                              '%s arrives %.1f s old' % (t, s['age_ms'] / 1000.0),
                              "The header stamp is %.0f ms behind this machine's clock on "
                              'arrival (latency or clock offset).' % s['age_ms'],
                              'Check network load and clock sync between machines.', [t]))
            else:
                self._since.pop('age:' + t, None)

    # -------------------------------------------------------- processes --
    def _process_rules(self, m, now, add):
        for pid, p in m['procs'].items():
            names = ', '.join(p['nodes']) or p['name']
            hot = p['cpu'] >= self.cfg['cpu_saturated_pct']
            if self._held('cpu:%s' % pid, hot, now, self.cfg['cpu_saturated_s']):
                add(Issue('cpu:%s' % p['key'], 'warn', 'process',
                          '%s is saturating a CPU core (%d%%)' % (names, p['cpu']),
                          'If its executor is single-threaded, ~100% of one core means it cannot '
                          'keep up: callbacks queue, rates drop and latency grows.',
                          'Profile it, lower its rates, or use a MultiThreadedExecutor with '
                          'separate callback groups.', p['nodes'] or [p['name']]))
            slope = p.get('rss_slope')
            if slope is not None and slope > self.cfg['leak_mb_per_min'] \
                    and p.get('rss_growth_mb', 0) > self.cfg['leak_min_growth_mb']:
                add(Issue('leak:%s' % p['key'], 'warn', 'process',
                          'Memory of %s keeps growing (+%.1f MB/min)' % (names, slope),
                          'Resident memory grew %.0f MB in the last %s and is now %.0f MB.' % (
                              p['rss_growth_mb'], _ago(p['rss_window_s']), p['rss'] / 1048576),
                          'Look for unbounded queues, caches or message buffers (e.g. a '
                          'subscription with KEEP_ALL history).', p['nodes'] or [p['name']]))
            if p['state'] == 'Z':
                add(Issue('zombie:%s' % p['key'], 'warn', 'process',
                          '%s is a zombie process' % names,
                          'It exited but its parent never reaped it.',
                          'Restart its parent (launch / supervisor).', p['nodes'] or [p['name']]))

    # ------------------------------------------------------------- host --
    def _host_rules(self, m, now, add):
        h = m.get('host') or {}
        if not h.get('supported'):
            return
        th = h.get('throttle') or {}
        # The supply dips in bursts (motors, CPU load): keep the alarm for 2 minutes after
        # the last time it was seen so it doesn't flicker on and off.
        if th.get('undervoltage_now'):
            self._uv_last = now
        uv_recent = now - getattr(self, '_uv_last', -1e9) < 120
        if th.get('undervoltage_now') or uv_recent:
            add(Issue('host:undervoltage', 'error', 'power', 'Under-voltage right now',
                      'The board reports its supply voltage is too low. This causes random '
                      'reboots, USB devices dropping out (cameras, motor controllers) and '
                      'SD-card corruption.',
                      'Use a stronger power supply (Raspberry Pi 4/5: official 5.1 V 3 A / 5 A) '
                      'and power motors separately.', ['host']))
        elif th.get('undervoltage_since_boot'):
            add(Issue('host:undervoltage_past', 'warn', 'power',
                      'Under-voltage happened since boot',
                      'The supply dipped below the safe voltage at least once since boot '
                      '(firmware flags %s).' % th.get('raw'),
                      'Check the power supply and cable, especially under motor or CPU load.',
                      ['host']))
        if th.get('throttled_now') or th.get('freq_capped_now'):
            add(Issue('host:throttled', 'warn', 'power', 'CPU is throttled',
                      'The firmware is limiting CPU frequency (heat or power), so every node '
                      'runs slower.', 'Improve cooling and the power supply.', ['host']))
        temps = h.get('temps') or []
        if temps:
            hottest = max(temps, key=lambda t: t['c'])
            if hottest['c'] >= self.cfg['temp_error_c']:
                add(Issue('host:temp', 'error', 'thermal',
                          'Very hot: %s at %.0f °C' % (hottest['name'], hottest['c']),
                          'Throttling and shutdown are close.', 'Add cooling / airflow.',
                          ['host']))
            elif self._held('temp', hottest['c'] >= self.cfg['temp_warn_c'], now, 20):
                add(Issue('host:temp', 'warn', 'thermal',
                          'Hot: %s at %.0f °C' % (hottest['name'], hottest['c']),
                          'The CPU will throttle soon.', 'Add cooling / airflow.', ['host']))
        mem = h.get('mem_pct', 0)
        if mem >= self.cfg['mem_error_pct']:
            add(Issue('host:mem', 'error', 'memory', 'Memory almost exhausted (%.0f%%)' % mem,
                      'The kernel will start killing processes (OOM killer).',
                      'Find the biggest process in the Nodes tab; add swap/zram as a buffer.',
                      ['host']))
        elif self._held('mem', mem >= self.cfg['mem_warn_pct'], now, 20):
            add(Issue('host:mem', 'warn', 'memory', 'Memory is low (%.0f%% used)' % mem,
                      'Little headroom left before the OOM killer.',
                      'Check which processes grow in the Nodes tab.', ['host']))
        for dsk in h.get('disks') or []:
            if dsk['pct'] >= self.cfg['disk_error_pct']:
                sev = 'error'
            elif dsk['pct'] >= self.cfg['disk_warn_pct']:
                sev = 'warn'
            else:
                continue
            add(Issue('host:disk:' + dsk['path'], sev, 'disk',
                      'Disk %s is %.0f%% full' % (dsk['path'], dsk['pct']),
                      '%s left. Full disks break logging, rosbag recording and parameter files.'
                      % _bytes(dsk['free']),
                      'Delete old bags and logs (~/.ros/log, ros2 bag files).', ['host']))
        self_cpu = m.get('self', {}).get('cpu', 0)
        if self._held('selfcpu', self_cpu > 25, now, 30):
            add(Issue('self:cpu', 'info', 'monitor', 'This monitor uses %.0f%% CPU' % self_cpu,
                      'Usually because of many high-rate topics.',
                      'Exclude topics with the exclude_topics parameter or raise sample_heavy.',
                      ['self']))

    # ----------------------------------------------- logs & diagnostics --
    def _feed_rules(self, m, now, add):
        for name, c in (m.get('log_counts') or {}).items():
            if c.get('error5m'):
                add(Issue('log:' + name, 'warn' if c.get('fatal5m') else 'info', 'logs',
                          '%s logged %d error%s in 5 min' % (
                              name, c['error5m'], '' if c['error5m'] == 1 else 's'),
                          'Last: %s' % c.get('last_error', ''),
                          'Open the Logs tab filtered on this node.', [name]))
        for d in m.get('diagnostics') or []:
            lvl = d['level']
            if lvl in (2, 3) or (lvl == 1 and d.get('held_s', 0) > 10):
                sev = {1: 'warn', 2: 'error', 3: 'warn'}[lvl]
                what = {1: 'warns', 2: 'reports an error', 3: 'is stale'}[lvl]
                src = d.get('hardware_id') or 'a node'
                add(Issue('diag:' + d['name'], sev, 'diagnostics',
                          '%s %s' % (d['name'], what),
                          '%s (reported on /diagnostics by %s)' % (
                              d.get('message') or 'no message', src), '',
                          ['diag:' + d['name']]))

    # --------------------------------------------------------------- tf --
    def _tf_rules(self, m, now, add):
        tf = m.get('tf') or {}
        frames = tf.get('frames') or []
        for f in frames:
            if len(f['parents']) > 1:
                add(Issue('tf:parents:' + f['frame'], 'error', 'tf',
                          'TF frame %s has %d parents: %s' % (
                              f['frame'], len(f['parents']), ', '.join(f['parents'])),
                          'Two publishers attach the same child frame to different parents, so '
                          'transform lookups flip between them or fail.',
                          'Make exactly one node publish %s (often a duplicate '
                          'robot_state_publisher, static_transform_publisher or localization '
                          'node).' % f['frame'], ['tf:' + f['frame']]))
            elif f.get('both'):
                add(Issue('tf:both:' + f['frame'], 'warn', 'tf',
                          'TF frame %s is on both /tf and /tf_static' % f['frame'],
                          'The same transform is published as static and as dynamic.',
                          'Publish it on one of the two: /tf_static for fixed transforms, /tf '
                          'for moving ones.', ['tf:' + f['frame']]))
            period = f.get('period_s')
            stale = bool(not f['static'] and period and f['age_s'] is not None
                         and f['age_s'] > max(1.0, 5 * period))
            if self._held('tfstale:' + f['frame'], stale, now, 2):
                add(Issue('tf:stale:' + f['frame'], 'warn', 'tf',
                          'Transform %s \u2192 %s stopped updating' % (f['parent'], f['frame']),
                          'It was published every %.0f ms and the last update was %s ago. Lookups '
                          'that need a recent transform will time out.' % (
                              period * 1000, _ago(f['age_s'])),
                          'Check the node that publishes this transform (odometry, localization, '
                          'joint states for robot_state_publisher).', ['tf:' + f['frame']]))
        roots = tf.get('roots') or []
        if len(roots) > 1 and self._held('tf:roots', True, now, 20):
            add(Issue('tf:roots', 'info', 'tf',
                      'The TF tree is split into %d separate trees' % len(roots),
                      'Roots: %s. Frames in different trees cannot be transformed into each other.'
                      % ', '.join(roots[:6]),
                      'Fine if intended; otherwise a static transform is missing between them.',
                      ['tf:' + r for r in roots[:6]]))

    # ----------------------------------------------------- expectations --
    def _expectation_rules(self, m, now, add):
        exp = m.get('expect') or {}
        nodes = m['nodes']
        for name in exp.get('nodes', []):
            if name not in nodes:
                add(Issue('expect:node:' + name, 'error', 'baseline',
                          'Expected node %s is missing' % name,
                          'It is in your baseline but not in the graph.',
                          'Start it, or update the baseline if it was removed on purpose.',
                          [name]))
        for t, rule in (exp.get('topics') or {}).items():
            d = m['topics'].get(t)
            if d is None or not d['pubs']:
                add(Issue('expect:topic:' + t, 'error', 'baseline',
                          'Expected topic %s has no publisher' % t,
                          'It is in your baseline but nothing publishes it.',
                          'Start its publisher or update the baseline.', [t]))


def _ago(s):
    s = max(0, int(s))
    if s < 90:
        return '%d s' % s
    if s < 5400:
        return '%d min' % (s // 60)
    return '%.1f h' % (s / 3600.0)


def _bytes(n):
    for unit in ('B', 'KB', 'MB', 'GB', 'TB'):
        if n < 1024 or unit == 'TB':
            return '%.1f %s' % (n, unit) if unit != 'B' else '%d B' % n
        n /= 1024.0
