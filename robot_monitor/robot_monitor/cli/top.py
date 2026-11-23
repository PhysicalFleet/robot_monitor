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
"""`ros2 monitor top`: htop-style terminal view (curses, standard library only)."""


def _b(n):
    if n is None:
        return '-'
    for unit in ('B', 'K', 'M', 'G'):
        if n < 1024 or unit == 'G':
            return '%d%s' % (n, unit) if unit in ('B', 'K') else '%.1f%s' % (n, unit)
        n /= 1024.0


def frame(snap, view='nodes', width=120):
    """Build the lines of one screen; a pure function used by curses and --once."""
    h = snap.get('host') or {}
    issues = snap.get('issues') or []
    temps = h.get('temps') or []
    hot = max((t['c'] for t in temps), default=None)
    lines = []
    head = ' Robot Monitor  %s  health %s/100  cpu %s%%  mem %s%%%s  load %s' % (
        h.get('hostname', '?'), snap.get('score', '-'), h.get('cpu', '-'), h.get('mem_pct', '-'),
        ('  %.0f°C' % hot) if hot is not None else '',
        ' '.join('%.2f' % x for x in h.get('load', [])))
    th = h.get('throttle') or {}
    if th.get('undervoltage_now'):
        head += '  UNDER-VOLTAGE'
    lines.append(('head', head[:width]))
    lines.append(('dim', ' [n]odes  [t]opics  [i]ssues  [q]uit'[:width]))
    lines.append(('', ''))
    if view == 'nodes':
        lines.append(('bold', ' %-44s %7s %6s %8s %5s %5s  %s' % (
            'NODE', 'PID', 'CPU%', 'MEM', 'PUB', 'SUB', 'HOW')))
        nodes = [n for n in snap.get('nodes', []) if not n.get('hidden')]
        nodes.sort(key=lambda n: -(n.get('cpu') or -1))
        for n in nodes:
            pid = n.get('pid') or ('remote' if n.get('remote') else '?')
            lines.append(('self' if n.get('self') else '', ' %-44s %7s %6s %8s %5d %5d  %s' % (
                n['name'][:44], pid, '-' if n.get('cpu') is None else '%.1f' % n['cpu'],
                _b(n.get('rss')), len(n['pubs']), len(n['subs']), n.get('how') or '')))
    elif view == 'topics':
        lines.append(('bold', ' %-44s %9s %10s %9s %7s  %s' % (
            'TOPIC', 'HZ', 'BW/s', 'AGE ms', 'P/S', 'TYPE')))
        for t in snap.get('topics', []):
            if t.get('hidden'):
                continue
            st = t.get('stats') or {}
            lines.append(('', ' %-44s %9s %10s %9s %7s  %s' % (
                t['name'][:44], '%.3g' % st['hz'] if st else '-',
                _b(st.get('bw')) if st else '-',
                '%.0f' % st['age_ms'] if st.get('age_ms') is not None else '-',
                '%d/%d' % (len(t['pubs']), len(t['subs'])), ', '.join(t['types'])[:40])))
    lines.append(('', ''))
    lines.append(('bold', ' ISSUES (%d)' % len(issues)))
    for i in issues[:(40 if view == 'issues' else 8)]:
        lines.append((i['severity'], ' %-5s %s' % (i['severity'].upper(), i['title'])[:width]))
        if view == 'issues' and i.get('fix'):
            lines.append(('dim', '       fix: ' + i['fix'][:width - 12]))
    return lines


def run_top(node, once=False):
    if once:
        from .command import wait_settled
        wait_settled(node, 5.0)
        for _, text in frame(node.snapshot):
            print(text)
        return 0
    import curses

    def loop(scr):
        curses.curs_set(0)
        curses.use_default_colors()
        colors = {}
        for i, (k, c) in enumerate((('error', curses.COLOR_RED), ('warn', curses.COLOR_YELLOW),
                                    ('info', curses.COLOR_CYAN), ('head', curses.COLOR_GREEN),
                                    ('self', curses.COLOR_BLUE)), start=1):
            curses.init_pair(i, c, -1)
            colors[k] = curses.color_pair(i)
        colors['bold'] = curses.A_BOLD
        colors['dim'] = curses.A_DIM
        colors['head'] |= curses.A_BOLD
        scr.timeout(1000)
        view = 'nodes'
        while True:
            hgt, wid = scr.getmaxyx()
            scr.erase()
            for y, (style, text) in enumerate(frame(node.snapshot or {}, view, wid - 1)[:hgt - 1]):
                try:
                    scr.addstr(y, 0, text[:wid - 1], colors.get(style, 0))
                except curses.error:
                    pass
            scr.refresh()
            ch = scr.getch()
            if ch in (ord('q'), 27):
                return
            view = {ord('n'): 'nodes', ord('t'): 'topics', ord('i'): 'issues'}.get(ch, view)

    curses.wrapper(loop)
    return 0
