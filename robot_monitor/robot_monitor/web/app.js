/* Robot Monitor dashboard. Plain JS, no build step, no network access beyond this robot. */
'use strict';

const $ = (id) => document.getElementById(id);
const S = {
  snap: null, tab: 'overview', sel: null, filter: '', live: false,
  sort: { nodes: ['cpu', -1], topics: ['name', 1], procs: ['cpu', -1] },
  showHidden: false, topicsView: 'all', echo: {}, params: {},
  logs: [], logSeq: 0, logLevels: { 10: false, 20: true, 30: true, 40: true, 50: true },
  logPaused: false, logNode: '',
  graph: { pos: new Map(), sig: '', view: { x: 40, y: 40, k: 1 }, showSystem: false, focus: null },
};
const TABS = [
  ['overview', 'Overview'], ['nodes', 'Nodes'], ['topics', 'Topics'], ['graph', 'Graph'],
  ['tf', 'TF'], ['logs', 'Logs'], ['diagnostics', 'Diagnostics'],
];
const LVL = { 10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL' };

// ------------------------------------------------------------------ utils --
function esc(s) {
  return String(s == null ? '' : s).replace(/[&<>"']/g, (c) => (
    { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}
function fmtBytes(n, per) {
  if (n == null) return '–';
  const u = ['B', 'KB', 'MB', 'GB', 'TB']; let i = 0;
  while (Math.abs(n) >= 1024 && i < u.length - 1) { n /= 1024; i++; }
  return (i === 0 ? Math.round(n) : n.toFixed(n < 10 ? 1 : 0)) + ' ' + u[i] + (per ? '/s' : '');
}
function fmtHz(x) {
  if (x == null) return '–';
  if (x === 0) return '0';
  if (x >= 100) return x.toFixed(0);
  if (x >= 10) return x.toFixed(1);
  return x.toFixed(2);
}
function fmtDur(s) {
  if (s == null) return '–';
  s = Math.max(0, Math.round(s));
  if (s < 60) return s + 's';
  if (s < 3600) return Math.floor(s / 60) + 'm ' + (s % 60) + 's';
  if (s < 86400) return Math.floor(s / 3600) + 'h ' + Math.floor((s % 3600) / 60) + 'm';
  return Math.floor(s / 86400) + 'd ' + Math.floor((s % 86400) / 3600) + 'h';
}
function ago(ts) { return fmtDur(Date.now() / 1000 - ts) + ' ago'; }
function splitName(full) {
  const i = full.lastIndexOf('/');
  return [full.slice(0, i + 1), full.slice(i + 1)];
}
function nameHtml(full) {
  const [ns, b] = splitName(full);
  return '<span class="name mono"><span class="ns">' + esc(ns) + '</span><span class="b">' + esc(b) + '</span></span>';
}
function spark(vals, w, h, cls) {
  w = w || 70; h = h || 18;
  if (!vals || vals.length < 2) return '<svg class="' + (cls || 'spark') + '" viewBox="0 0 ' + w + ' ' + h + '"></svg>';
  const max = Math.max(...vals, 1e-9), n = vals.length;
  const pts = vals.map((v, i) => [(i / (n - 1)) * w, h - 1 - (v / max) * (h - 3)]);
  const line = pts.map((p) => p[0].toFixed(1) + ',' + p[1].toFixed(1)).join(' ');
  const area = 'M0,' + h + ' L' + line.split(' ').join(' L') + ' L' + w + ',' + h + ' Z';
  return '<svg class="' + (cls || 'spark') + '" viewBox="0 0 ' + w + ' ' + h + '" preserveAspectRatio="none"><path d="' + area + '"/><polyline points="' + line + '"/></svg>';
}
function bigChart(vals, unit) {
  const w = 520, h = 90;
  if (!vals || vals.length < 2) return '<div class="muted">Collecting data…</div>';
  const max = Math.max(...vals, 1e-9) * 1.15, n = vals.length;
  const pts = vals.map((v, i) => [(i / (n - 1)) * w, h - 12 - (v / max) * (h - 22)]);
  const line = pts.map((p) => p[0].toFixed(1) + ',' + p[1].toFixed(1)).join(' ');
  const area = 'M0,' + (h - 12) + ' L' + line.split(' ').join(' L') + ' L' + w + ',' + (h - 12) + ' Z';
  return '<svg class="bigchart" viewBox="0 0 ' + w + ' ' + h + '" preserveAspectRatio="none">' +
    '<line x1="0" x2="' + w + '" y1="' + (h - 12) + '" y2="' + (h - 12) + '"/>' +
    '<path d="' + area + '"/><polyline points="' + line + '"/>' +
    '<text x="2" y="10">' + esc(fmtHz(max / 1.15)) + ' ' + esc(unit) + ' max</text>' +
    '<text x="2" y="' + (h - 1) + '">' + n + 's ago</text><text x="' + (w - 22) + '" y="' + (h - 1) + '">now</text></svg>';
}
function toast(msg) {
  const t = $('toast'); t.textContent = msg; t.classList.add('show');
  clearTimeout(toast._t); toast._t = setTimeout(() => t.classList.remove('show'), 1800);
}
function copyText(text) {
  if (navigator.clipboard && window.isSecureContext) {
    return navigator.clipboard.writeText(text).then(() => toast('Copied'));
  }
  const ta = document.createElement('textarea');
  ta.value = text; ta.style.position = 'fixed'; ta.style.opacity = '0';
  document.body.appendChild(ta); ta.select();
  try { document.execCommand('copy'); toast('Copied'); } catch (e) { toast('Copy failed: select and copy manually'); }
  document.body.removeChild(ta);
  return Promise.resolve();
}
function sevOf(subject) { return (S.idx && S.idx.get(subject)) || null; }
function cpuCell(cpu) {
  if (cpu == null) return '<span class="faint">–</span>';
  const cls = cpu >= 85 ? 'hi' : cpu >= 50 ? 'mid' : '';
  return '<span class="cpu"><span class="num">' + cpu.toFixed(1) + '%</span><span class="barbg"><i class="' + cls + '" style="width:' + Math.min(100, cpu) + '%"></i></span></span>';
}
function matches(text) {
  if (!S.filter) return true;
  return text.toLowerCase().includes(S.filter.toLowerCase());
}
function sortRows(rows, key, dir, get) {
  return rows.sort((a, b) => {
    const x = get(a, key), y = get(b, key);
    if (x == null && y == null) return 0;
    if (x == null) return 1;
    if (y == null) return -1;
    return (x < y ? -1 : x > y ? 1 : 0) * dir;
  });
}
function th(table, key, label, right) {
  const [k, d] = S.sort[table];
  const arrow = k === key ? '<span class="arrow">' + (d > 0 ? ' ↑' : ' ↓') + '</span>' : '';
  return '<th class="' + (right ? 'r' : '') + '" data-sort="' + table + ':' + key + '">' + label + arrow + '</th>';
}

// --------------------------------------------------------------- connect --
function connect() {
  if (window.EventSource) {
    const es = new EventSource('/api/events');
    es.addEventListener('snapshot', (e) => { setLive(true); onSnap(JSON.parse(e.data)); });
    es.onerror = () => { setLive(false); };
    fetch('/api/snapshot').then((r) => r.json()).then((s) => { if (s.version) { setLive(true); onSnap(s); } }).catch(() => {});
  } else {
    const poll = () => fetch('/api/snapshot').then((r) => r.json()).then((s) => { setLive(true); onSnap(s); })
      .catch(() => setLive(false)).finally(() => setTimeout(poll, 1000));
    poll();
  }
}
function setLive(on) {
  if (S.live === on) return;
  S.live = on;
  $('live').classList.toggle('off', !on);
  $('live').lastChild.textContent = on ? 'live' : 'reconnecting';
}
function onSnap(s) {
  if (!s || !s.version) return;
  S.snap = s;
  const idx = new Map(), rank = { error: 3, warn: 2, info: 1 };
  for (const i of s.issues) {
    for (const sub of (i.focus || i.subjects)) {
      const cur = idx.get(sub);
      if (!cur || rank[i.severity] > rank[cur]) idx.set(sub, i.severity);
    }
  }
  S.idx = idx;
  if (s.log_seq > S.logSeq) fetchLogs();
  render();
}
function fetchLogs() {
  if (fetchLogs.busy) return;
  fetchLogs.busy = true;
  fetch('/api/logs?after=' + S.logSeq).then((r) => r.json()).then((items) => {
    if (items.length) {
      S.logs.push(...items);
      if (S.logs.length > 3000) S.logs.splice(0, S.logs.length - 3000);
      S.logSeq = items[items.length - 1].seq;
      if (S.tab === 'logs' && !S.logPaused) renderMain();
    }
  }).catch(() => {}).finally(() => { fetchLogs.busy = false; });
}

// ---------------------------------------------------------------- render --
function render() {
  if (!S.snap) return;
  renderTop(); renderHost(); renderTabs(); renderMain(); renderDrawer();
  const me = S.snap.self || {};
  $('foot-ver').textContent = 'Robot Monitor ' + S.snap.version;
  $('foot-self').textContent = me.cpu != null ? 'This monitor: ' + me.cpu.toFixed(1) + '% CPU, ' + fmtBytes(me.rss) + ', ' + me.subs + ' subscriptions' : '';
}

function renderTop() {
  const s = S.snap, h = s.host || {}, e = s.env || {};
  const chips = [
    '<span class="chip"><b>' + esc(h.hostname || '?') + '</b></span>',
    e.ROS_DISTRO ? '<span class="chip">ROS 2 <b>' + esc(e.ROS_DISTRO) + '</b></span>' : '',
    '<span class="chip">RMW <b>' + esc((e.RMW_IMPLEMENTATION || 'default').replace(/^rmw_|_cpp$/g, '')) + '</b></span>',
    '<span class="chip">domain <b>' + esc(e.ROS_DOMAIN_ID || '0') + '</b></span>',
  ];
  if (s.baseline && s.baseline.loaded) chips.push('<span class="chip">baseline <b>' + s.baseline.nodes + ' nodes · ' + s.baseline.topics + ' topics</b></span>');
  $('chips').innerHTML = chips.join('');
  const sc = s.score, col = sc >= 90 ? 'var(--ok)' : sc >= 60 ? 'var(--warn)' : 'var(--err)';
  const c = 2 * Math.PI * 14, off = c * (1 - sc / 100);
  $('score').innerHTML = '<svg viewBox="0 0 34 34"><circle cx="17" cy="17" r="14" fill="none" stroke="var(--line)" stroke-width="3.5"/>' +
    '<circle cx="17" cy="17" r="14" fill="none" stroke="' + col + '" stroke-width="3.5" stroke-linecap="round" stroke-dasharray="' + c.toFixed(1) + '" stroke-dashoffset="' + off.toFixed(1) + '" transform="rotate(-90 17 17)"/></svg>' +
    '<div><div class="val" style="color:' + col + '">' + sc + '</div><div class="lbl">health</div></div>';
  document.title = (s.issues.some((i) => i.severity === 'error') ? '● ' : '') + 'Robot Monitor · ' + (h.hostname || '');
}

function renderHost() {
  const h = S.snap.host || {};
  if (!h.supported) {
    $('host').innerHTML = '<div class="card metric"><div class="k">Host</div><div class="v">' + esc(h.hostname || '') + '</div><div class="s">Process and host metrics need Linux</div></div>';
    return;
  }
  const cards = [];
  const cores = (h.cpu_cores || []).map((v) => '<span style="height:' + Math.max(8, v) + '%;background:' + (v > 85 ? 'var(--err)' : v > 50 ? 'var(--warn)' : 'var(--accent)') + '"></span>').join('');
  cards.push(card('CPU', h.cpu.toFixed(0) + '%', 'load ' + (h.load || []).map((x) => x.toFixed(2)).join(' ') + ' · ' + h.ncpu + ' cores', h.cpu > 90 ? 'meh' : '', '<div class="cores">' + cores + '</div>'));
  cards.push(card('Memory', h.mem_pct.toFixed(0) + '%', fmtBytes(h.mem_total - h.mem_avail) + ' of ' + fmtBytes(h.mem_total) + (h.swap_used > 1048576 ? ' · swap ' + fmtBytes(h.swap_used) : ''), h.mem_pct >= 95 ? 'bad' : h.mem_pct >= 90 ? 'meh' : '', spark(S.snap._memHist = pushHist('mem', h.mem_pct), 90, 30)));
  const temps = h.temps || [];
  if (temps.length) {
    const hot = temps.reduce((a, b) => (b.c > a.c ? b : a));
    cards.push(card('Temperature', hot.c.toFixed(0) + ' °C', hot.name + (temps.length > 1 ? ' · ' + temps.length + ' sensors' : ''), hot.c >= 85 ? 'bad' : hot.c >= 80 ? 'meh' : '', spark(pushHist('temp', hot.c), 90, 30)));
  }
  const th = h.throttle;
  if (th) {
    const bad = th.undervoltage_now, past = th.undervoltage_since_boot || th.throttled_since_boot;
    cards.push(card('Power', bad ? 'Under-voltage' : th.throttled_now ? 'Throttled' : 'OK',
      bad ? 'supply too weak right now' : past ? 'issues since boot (' + (th.raw || '') + ')' : 'no under-voltage', bad ? 'bad' : past ? 'meh' : ''));
  }
  const disks = h.disks || [];
  if (disks.length) {
    const worst = disks.reduce((a, b) => (b.pct > a.pct ? b : a));
    cards.push(card('Disk', worst.pct.toFixed(0) + '%', worst.path + ' · ' + fmtBytes(worst.free) + ' free', worst.pct >= 97 ? 'bad' : worst.pct >= 90 ? 'meh' : ''));
  }
  const net = h.net || [];
  if (net.length) {
    const rx = net.reduce((a, b) => a + b.rx, 0), tx = net.reduce((a, b) => a + b.tx, 0);
    cards.push(card('Network', '↓ ' + fmtBytes(rx, true), '↑ ' + fmtBytes(tx, true) + ' · ' + net.map((n) => n.iface).join(', '), '', spark(pushHist('net', rx + tx), 90, 30)));
  }
  for (const g of h.gpu || []) {
    cards.push(card('GPU', g.util.toFixed(0) + '%', g.name + (g.mem_total ? ' · ' + fmtBytes(g.mem_used) + ' / ' + fmtBytes(g.mem_total) : ''), '', spark(pushHist('gpu' + g.name, g.util), 90, 30)));
  }
  const nodes = S.snap.nodes.filter((n) => !n.hidden && !n.self).length;
  const topics = S.snap.topics.filter((t) => !t.hidden && !t.system).length;
  cards.push(card('ROS graph', nodes + ' nodes', topics + ' topics · ' + S.snap.procs.filter((p) => !p.self).length + ' ROS processes', ''));
  $('host').innerHTML = cards.join('');
  $('host').querySelectorAll('.metric').forEach((el) => el.addEventListener('click', () => select('host', 'host')));
}
const HIST = {};
function pushHist(key, v) {
  const a = HIST[key] || (HIST[key] = []);
  if (S.snap.ts !== HIST[key + ':ts']) { a.push(v); HIST[key + ':ts'] = S.snap.ts; }
  if (a.length > 120) a.shift();
  return a;
}
function card(k, v, s, cls, extra) {
  return '<div class="card metric ' + (cls || '') + '" style="cursor:pointer"><div class="k">' + esc(k) + '</div><div class="v num">' + esc(v) + '</div><div class="s">' + esc(s) + '</div>' + (extra || '') + '</div>';
}

function renderTabs() {
  const s = S.snap;
  const errs = s.issues.filter((i) => i.severity === 'error').length;
  const warns = s.issues.filter((i) => i.severity === 'warn').length;
  const counts = {
    overview: errs ? '<span class="count err">' + errs + '</span>' : warns ? '<span class="count warn">' + warns + '</span>' : '',
    nodes: '<span class="count">' + s.nodes.filter((n) => S.showHidden || !n.hidden).length + '</span>',
    topics: '<span class="count">' + s.topics.filter((t) => S.showHidden || !t.hidden).length + '</span>',
    logs: (() => { const e = Object.values(s.log_counts || {}).reduce((a, c) => a + (c.error5m || 0), 0); return e ? '<span class="count err">' + e + '</span>' : ''; })(),
    diagnostics: (() => { const bad = (s.diagnostics || []).filter((d) => d.level === 2).length, w = (s.diagnostics || []).filter((d) => d.level === 1).length; return bad ? '<span class="count err">' + bad + '</span>' : w ? '<span class="count warn">' + w + '</span>' : (s.diagnostics || []).length ? '<span class="count">' + s.diagnostics.length + '</span>' : ''; })(),
    graph: '',
    tf: (() => { const f = ((s.tf || {}).frames || []).length; const bad = s.issues.some((i) => i.category === 'tf' && i.severity !== 'info'); return f ? '<span class="count' + (bad ? ' warn' : '') + '">' + f + '</span>' : ''; })(),
  };
  const html = TABS.map(([k, label]) => '<a href="#/' + k + '" class="' + (S.tab === k ? 'on' : '') + '">' + label + ' ' + (counts[k] || '') + '</a>').join('');
  if ($('tabs').innerHTML !== html) $('tabs').innerHTML = html;
}

function renderMain() {
  const m = $('main');
  const keepScroll = window.scrollY;
  const focused = document.activeElement && document.activeElement.id === 'q' ? document.activeElement.selectionStart : null;
  if (S.tab === 'graph') { renderGraph(m); return; }
  let html = '';
  if (S.tab === 'overview') html = viewOverview();
  else if (S.tab === 'nodes') html = viewNodes();
  else if (S.tab === 'topics') html = viewTopics();
  else if (S.tab === 'logs') html = viewLogs();
  else if (S.tab === 'tf') html = viewTf();
  else if (S.tab === 'diagnostics') html = viewDiagnostics();
  if (html === S.lastMain && m.dataset.tab === S.tab) return;
  if (userIsSelecting(m)) return;  // don't wipe a selection the user is making
  S.lastMain = html;
  m.innerHTML = html;
  m.dataset.tab = S.tab;
  const q = $('q');
  if (q && focused != null) { q.focus(); q.setSelectionRange(focused, focused); }
  window.scrollTo(0, keepScroll);
}

function searchBox(ph) {
  return '<label class="search"><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="11" cy="11" r="7"/><path d="m20 20-3.5-3.5"/></svg><input id="q" placeholder="' + esc(ph) + '" value="' + esc(S.filter) + '" autocomplete="off" spellcheck="false"><kbd>/</kbd></label>';
}

// -------------------------------------------------------------- overview --
function issueHtml(i) {
  const subj = i.subjects.filter((x) => x).map((x) => '<a class="subj" data-subj="' + esc(x) + '">' + esc(x.replace(/^(diag|tf):/, '')) + '</a>').join('');
  return '<div class="issue ' + i.severity + '"><div class="bar"></div><div class="body">' +
    '<div class="t"><span class="sev">' + (i.severity === 'warn' ? 'warning' : i.severity) + '</span><span>' + esc(i.title) + '</span><span class="when">' + ago(i.since) + '</span></div>' +
    (i.detail ? '<div class="d">' + esc(i.detail) + '</div>' : '') +
    (i.fix ? '<div class="f"><b>Fix</b>' + esc(i.fix) + '</div>' : '') +
    (subj ? '<div class="subjects">' + subj + '</div>' : '') + '</div></div>';
}
function viewOverview() {
  const s = S.snap;
  const issues = s.issues;
  const errs = issues.filter((i) => i.severity === 'error').length;
  const warns = issues.filter((i) => i.severity === 'warn').length;
  const visNodes = s.nodes.filter((n) => !n.hidden && !n.self);
  const live = s.topics.filter((t) => t.stats && t.stats.hz > 0 && !t.hidden).length;
  let h = '<div class="stat-row">' +
    stat(errs, 'errors', errs ? 'var(--err)' : '') + stat(warns, 'warnings', warns ? 'var(--warn)' : '') +
    stat(visNodes.length, 'nodes') + stat(live, 'topics with data') + '</div>';
  h += '<div class="grid2"><div>';
  h += '<div class="section"><h2>Issues</h2>';
  if (!issues.length) {
    h += '<div class="card allgood"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><path d="m7.5 12.5 3 3 6-6.5"/></svg><b>All clear</b>No problems found in ' + visNodes.length + ' nodes and ' + s.topics.length + ' topics.' + (s.uptime < 30 ? ' Rules that need history (stale topics, leaks) arm over the first minutes.' : '') + '</div>';
  } else {
    h += issues.map(issueHtml).join('');
  }
  h += '</div></div><div>';
  // top processes
  const procs = s.procs.slice().sort((a, b) => b.cpu - a.cpu).slice(0, 7);
  h += '<div class="section"><h2>Busiest processes</h2><div class="card"><table class="mini">' +
    procs.map((p) => '<tr data-proc="' + p.pid + '"><td>' + (p.nodes.length ? nameHtml(p.nodes[0]) + (p.nodes.length > 1 ? ' <span class="faint">+' + (p.nodes.length - 1) + '</span>' : '') : '<span class="mono">' + esc(p.name) + '</span>' + (p.self ? ' <span class="tag">robot_monitor</span>' : '')) + '</td><td class="r">' + cpuCell(p.cpu) + '</td><td class="r num muted">' + fmtBytes(p.rss) + '</td></tr>').join('') +
    (procs.length ? '' : '<tr><td class="muted">No ROS processes on this machine</td></tr>') + '</table></div></div>';
  // busiest topics
  const tops = s.topics.filter((t) => t.stats && !t.hidden).sort((a, b) => (b.stats.bw || 0) - (a.stats.bw || 0)).slice(0, 7);
  h += '<div class="section"><h2>Busiest topics</h2><div class="card"><table class="mini">' +
    tops.map((t) => '<tr data-topic="' + esc(t.name) + '"><td>' + nameHtml(t.name) + '</td><td class="r num">' + fmtHz(t.stats.hz) + ' Hz</td><td class="r num muted">' + fmtBytes(t.stats.bw, true) + '</td></tr>').join('') +
    (tops.length ? '' : '<tr><td class="muted">No topic data yet</td></tr>') + '</table></div></div>';
  // events
  const ev = [];
  for (const g of s.gone || []) ev.push([g.ago_s, '<span class="tag warn">left</span> ' + nameHtml(g.name)]);
  for (const n of s.nodes) if (n.restarts) ev.push([n.restarts.ago_s, '<span class="tag warn">restarted ×' + n.restarts.count + '</span> ' + nameHtml(n.name)]);
  if (ev.length) {
    ev.sort((a, b) => a[0] - b[0]);
    h += '<div class="section"><h2>Recent events</h2><div class="card"><table class="mini">' + ev.slice(0, 8).map((e) => '<tr><td>' + e[1] + '</td><td class="r muted">' + fmtDur(e[0]) + ' ago</td></tr>').join('') + '</table></div></div>';
  }
  h += '</div></div>';
  return h;
}
function stat(v, k, col) {
  return '<div class="card stat"><div class="v num" style="' + (col ? 'color:' + col : '') + '">' + v + '</div><div class="k">' + esc(k) + '</div></div>';
}

// ----------------------------------------------------------------- nodes --
function nodeStatus(n) {
  const sev = sevOf(n.name);
  if (sev) return sev;
  if (n.lifecycle && n.lifecycle !== 'active') return 'idle';
  return 'ok';
}
function viewNodes() {
  const s = S.snap;
  let rows = s.nodes.filter((n) => (S.showHidden || (!n.hidden)) && matches(n.name + ' ' + (n.pid || '')));
  const [k, d] = S.sort.nodes;
  const procs = new Map(s.procs.map((p) => [p.pid, p]));
  rows = sortRows(rows, k, d, (n, key) => {
    if (key === 'pubs' || key === 'subs') return n[key].length;
    if (key === 'uptime') return procs.get(n.pid) ? procs.get(n.pid).uptime : null;
    return n[key];
  });
  let h = '<div class="toolbar">' + searchBox('Filter nodes…') +
    '<label class="tog"><input type="checkbox" id="showHidden" ' + (S.showHidden ? 'checked' : '') + '> show hidden &amp; tooling nodes</label>' +
    '<span class="spacer"></span><span class="faint">process mapping: ' + esc(s.calibration) + '</span></div>';
  h += '<div class="tablewrap"><table class="t"><thead><tr><th style="width:18px"></th>' + th('nodes', 'name', 'Node') + th('nodes', 'pid', 'PID', 1) +
    th('nodes', 'cpu', 'CPU', 1) + '<th>CPU history</th>' + th('nodes', 'rss', 'Memory', 1) + th('nodes', 'pubs', 'Pubs', 1) + th('nodes', 'subs', 'Subs', 1) +
    th('nodes', 'uptime', 'Uptime', 1) + '<th>Notes</th></tr></thead><tbody>';
  for (const n of rows) {
    const p = procs.get(n.pid);
    const notes = [];
    if (n.self) notes.push('<span class="tag">this monitor</span>');
    if (n.count > 1) notes.push('<span class="tag err">×' + n.count + ' same name</span>');
    if (n.lifecycle) notes.push('<span class="tag ' + (n.lifecycle === 'active' ? 'ok' : 'info') + '">' + esc(n.lifecycle) + '</span>');
    if (n.restarts) notes.push('<span class="tag warn">restarted ×' + n.restarts.count + '</span>');
    if (n.container) notes.push('<span class="tag">in ' + esc(n.container) + '</span>');
    else if (n.shared_process) notes.push('<span class="tag">shared process</span>');
    if (n.remote) notes.push('<span class="tag info">' + (n.host_ip ? 'on ' + esc(n.host_ip) : 'other machine') + '</span>');
    const pid = n.pid ? '<span title="found via ' + esc(n.how) + '">' + n.pid + '</span>' : (n.remote ? '<span class="faint">remote</span>' : '<span class="faint" title="Could not map this node to a local process">?</span>');
    h += '<tr data-node="' + esc(n.name) + '" class="' + (S.sel && S.sel.id === n.name ? 'sel' : '') + '"><td><span class="dot ' + nodeStatus(n) + '"></span></td><td>' + nameHtml(n.name) + '</td><td class="r num mono">' + pid + '</td><td class="r">' + cpuCell(n.cpu) + '</td><td>' + (p ? spark(p.cpu_hist) : '') +
      '</td><td class="r num">' + fmtBytes(n.rss) + '</td><td class="r num">' + n.pubs.length + '</td><td class="r num">' + n.subs.length + '</td><td class="r num muted">' + (p ? fmtDur(p.uptime) : '–') + '</td><td>' + notes.join(' ') + '</td></tr>';
  }
  if (!rows.length) h += '<tr><td colspan="10" class="empty">No nodes match.</td></tr>';
  h += '</tbody></table></div>';
  return h;
}

// ---------------------------------------------------------------- topics --
function topicState(t) {
  const sev = sevOf(t.name);
  if (sev) return [sev, sev === 'error' ? 'problem' : sev === 'warn' ? 'check' : 'note'];
  if (t.system) return ['idle', 'system'];
  if (!t.pubs.length) return ['idle', 'no publisher'];
  if (t.latched) return ['ok', 'latched'];
  if (t.monitor === 'unavailable') return ['idle', 'type not installed'];
  if (t.monitor === 'excluded') return ['idle', 'excluded'];
  if (t.stats && t.stats.hz > 0) return ['ok', 'ok'];
  return ['idle', 'no data'];
}
function viewTopics() {
  const s = S.snap;
  let rows = s.topics.filter((t) => (S.showHidden || (!t.hidden)) && matches(t.name + ' ' + t.types.join(' ')));
  if (S.topicsView === 'problems') rows = rows.filter((t) => sevOf(t.name));
  if (S.topicsView === 'active') rows = rows.filter((t) => t.stats && t.stats.hz > 0);
  const [k, d] = S.sort.topics;
  rows = sortRows(rows, k, d, (t, key) => {
    const st = t.stats || {};
    if (key === 'hz' || key === 'bw' || key === 'size' || key === 'age_ms') return st[key];
    if (key === 'type') return t.types[0];
    if (key === 'conn') return t.pubs.length + t.subs.length;
    return t[key];
  });
  let h = '<div class="toolbar">' + searchBox('Filter topics or types…') +
    '<span class="seg">' + ['all', 'active', 'problems'].map((v) => '<button data-tview="' + v + '" class="' + (S.topicsView === v ? 'on' : '') + '">' + v + '</button>').join('') + '</span>' +
    '<label class="tog"><input type="checkbox" id="showHidden" ' + (S.showHidden ? 'checked' : '') + '> hidden topics</label></div>';
  h += '<div class="tablewrap"><table class="t"><thead><tr><th style="width:18px"></th>' + th('topics', 'name', 'Topic') + th('topics', 'type', 'Type') +
    th('topics', 'hz', 'Rate', 1) + '<th>Rate history</th>' + th('topics', 'bw', 'Bandwidth', 1) + th('topics', 'size', 'Msg size', 1) + th('topics', 'age_ms', 'Latency', 1) +
    th('topics', 'conn', 'Pub → Sub', 1) + '<th>Status</th></tr></thead><tbody>';
  for (const t of rows) {
    const st = t.stats;
    const [sev, label] = topicState(t);
    const pubs = t.pubs.filter((p) => !p.self).length, subs = t.subs.filter((p) => !p.self).length;
    const tagcls = sev === 'error' ? 'err' : sev === 'warn' ? 'warn' : sev === 'ok' ? 'ok' : sev === 'info' ? 'info' : '';
    let extra = '';
    if (t.monitor === 'sampled') extra = ' <span class="tag" title="' + esc(t.monitor_reason || '') + '">sampled</span>';
    h += '<tr data-topic="' + esc(t.name) + '" class="' + (S.sel && S.sel.id === t.name ? 'sel' : '') + '"><td><span class="dot ' + (sev === 'ok' ? '' : sev) + '"></span></td><td>' + nameHtml(t.name) + '</td><td class="mono muted">' + esc(t.types.join(', ')) + '</td>' +
      '<td class="r num">' + (st ? fmtHz(st.hz) + ' Hz' : '<span class="faint">–</span>') + '</td><td>' + (st ? spark(st.hist) : '') + '</td>' +
      '<td class="r num">' + (st ? fmtBytes(st.bw, true) : '–') + '</td><td class="r num muted">' + (st && st.size ? fmtBytes(st.size) : '–') + '</td>' +
      '<td class="r num">' + (st && st.age_ms != null ? latency(st.age_ms) : '<span class="faint">–</span>') + '</td>' +
      '<td class="r num">' + pubs + ' → ' + subs + '</td><td><span class="tag ' + tagcls + '">' + esc(label) + '</span>' + extra + '</td></tr>';
  }
  if (!rows.length) h += '<tr><td colspan="10" class="empty">No topics match.</td></tr>';
  h += '</tbody></table></div>';
  return h;
}
function latency(ms) {
  if (ms < 0) return '<span style="color:var(--warn)" title="stamped in the future: clock offset">' + ms.toFixed(0) + ' ms</span>';
  if (ms >= 1000) return (ms / 1000).toFixed(1) + ' s';
  return ms.toFixed(ms < 10 ? 1 : 0) + ' ms';
}

// ------------------------------------------------------------------ logs --
function viewLogs() {
  const nodes = [...new Set(S.logs.map((l) => l.logger))].sort();
  let rows = S.logs.filter((l) => S.logLevels[l.level >= 50 ? 50 : l.level] !== false && (!S.logNode || l.logger === S.logNode) && matches(l.logger + ' ' + l.msg));
  let h = '<div class="toolbar">' + searchBox('Search logs…') +
    '<span class="seg">' + [10, 20, 30, 40, 50].map((l) => '<button data-lvl="' + l + '" class="' + (S.logLevels[l] ? 'on' : '') + '">' + LVL[l] + '</button>').join('') + '</span>' +
    '<select id="lognode" class="btn"><option value="">all nodes</option>' + nodes.map((n) => '<option ' + (n === S.logNode ? 'selected' : '') + '>' + esc(n) + '</option>').join('') + '</select>' +
    '<button class="btn" id="logpause">' + (S.logPaused ? '▶ Resume' : '❚❚ Pause') + '</button>' +
    '<span class="spacer"></span><span class="faint">' + rows.length + ' of ' + S.logs.length + ' lines from /rosout since this monitor started</span></div>';
  rows = rows.slice(-600).reverse();
  h += '<div class="tablewrap logs">';
  if (!rows.length) h += '<div class="empty">No log lines yet. Nodes log to /rosout; DEBUG is hidden by default.</div>';
  for (const l of rows) {
    const d = new Date(l.t * 1000);
    const lv = l.level >= 50 ? 50 : l.level;
    h += '<div class="row lvl' + lv + '"><span class="faint">' + d.toLocaleTimeString([], { hour12: false }) + '.' + String(d.getMilliseconds()).padStart(3, '0') + '</span><span class="lv lv' + lv + '">' + (LVL[lv] || l.level) + '</span><span class="lg" title="' + esc(l.logger) + '">' + esc(l.logger) + '</span><span class="msg">' + esc(l.msg) + '</span></div>';
  }
  h += '</div>';
  return h;
}

// -------------------------------------------------------------------- tf --
function viewTf() {
  const tf = S.snap.tf || { frames: [], roots: [] };
  let h = '<div class="toolbar">' + searchBox('Filter frames…') + '<span class="spacer"></span><span class="faint">from /tf and /tf_static</span></div>';
  if (!tf.frames.length) return h + '<div class="card empty">No transforms seen on /tf or /tf_static yet. Nodes such as robot_state_publisher, static_transform_publisher, odometry and localization publish them.</div>';
  const byFrame = new Map(tf.frames.map((f) => [f.frame, f]));
  const kids = new Map();
  for (const f of tf.frames) {
    const p = f.parent || '';
    if (!kids.has(p)) kids.set(p, []);
    kids.get(p).push(f.frame);
  }
  const roots = [...new Set(tf.frames.map((f) => f.parent).filter((p) => p && !byFrame.has(p)))].sort();
  const seen = new Set();
  const rows = [];
  const walk = (frame, depth) => {
    if (seen.has(frame) || depth > 40) return;
    seen.add(frame);
    rows.push([frame, depth]);
    for (const c of (kids.get(frame) || []).sort()) walk(c, depth + 1);
  };
  for (const r of roots) walk(r, 0);
  for (const f of tf.frames) if (!seen.has(f.frame)) walk(f.frame, 0);   // cycles
  h += '<div class="tablewrap"><table class="t"><thead><tr><th style="width:18px"></th><th>Frame</th><th>Parent</th><th class="r">Rate</th><th class="r">Last update</th><th>Notes</th></tr></thead><tbody>';
  for (const [frame, depth] of rows) {
    const f = byFrame.get(frame);
    if (!matches(frame + ' ' + (f ? f.parent : ''))) continue;
    const sev = sevOf('tf:' + frame);
    const notes = [];
    if (!f) notes.push('<span class="tag">root</span>');
    else {
      if (f.static) notes.push('<span class="tag">static</span>');
      if (f.parents.length > 1) notes.push('<span class="tag err">' + f.parents.length + ' parents</span>');
      if (f.both) notes.push('<span class="tag warn">static and dynamic</span>');
    }
    h += '<tr><td><span class="dot ' + (sev || (f && !f.static && f.hz === 0 ? 'idle' : '')) + '"></span></td>' +
      '<td class="mono" style="padding-left:' + (12 + depth * 18) + 'px">' + (depth ? '<span class="faint">└ </span>' : '') + '<b>' + esc(frame) + '</b></td>' +
      '<td class="mono muted">' + esc(f ? f.parent : '') + '</td>' +
      '<td class="r num">' + (f ? (f.static ? '<span class="faint">static</span>' : fmtHz(f.hz) + ' Hz') : '') + '</td>' +
      '<td class="r num muted">' + (f && f.age_s != null && !f.static ? fmtDur(f.age_s) + ' ago' : '') + '</td><td>' + notes.join(' ') + '</td></tr>';
  }
  h += '</tbody></table></div>';
  return h;
}

// ----------------------------------------------------------- diagnostics --
function viewDiagnostics() {
  const ds = (S.snap.diagnostics || []).filter((d) => matches(d.name + ' ' + d.message + ' ' + d.hardware_id));
  let h = '<div class="toolbar">' + searchBox('Filter diagnostics…') + '<span class="spacer"></span><span class="faint">from /diagnostics (diagnostic_updater, aggregators, drivers)</span></div>';
  if (!ds.length) return h + '<div class="card empty">No /diagnostics messages seen. Drivers that use diagnostic_updater show up here automatically.</div>';
  const groups = new Map();
  for (const d of ds) {
    const g = d.name.includes('/') ? d.name.split('/')[0] : d.name.includes(':') ? d.name.split(':')[0] : 'other';
    if (!groups.has(g)) groups.set(g, []);
    groups.get(g).push(d);
  }
  const L = { 0: ['OK', 'ok'], 1: ['WARN', 'warn'], 2: ['ERROR', 'err'], 3: ['STALE', 'info'] };
  for (const [g, items] of groups) {
    h += '<div class="diag-group"><h2>' + esc(g) + '</h2><div class="card">';
    for (const d of items) {
      const [lab, cls] = L[d.level] || ['?', ''];
      h += '<div class="diag" data-diag="' + esc(d.name) + '"><span class="tag ' + cls + '">' + lab + '</span><span class="mono">' + esc(d.name) + '</span><span class="muted">' + esc(d.message) + '</span><span class="faint">' + esc(d.hardware_id) + ' · ' + fmtDur(d.age_s) + ' ago</span>' +
        '<div class="vals">' + d.values.map((kv) => '<span>' + esc(kv[0]) + '</span><span>' + esc(kv[1]) + '</span>').join('') + '</div></div>';
    }
    h += '</div></div>';
  }
  return h;
}

// ---------------------------------------------------------------- drawer --
function select(type, id) {
  S.sel = id ? { type, id } : null;
  S.forceDrawer = true;
  S.lastMain = null;
  renderDrawer();
  if (S.tab !== 'graph') renderMain(); else updateGraphStyles();
}
function userIsSelecting(el) {
  const sel = window.getSelection && window.getSelection();
  if (S.mouseDown) return true;
  return !!(sel && !sel.isCollapsed && el.contains(sel.anchorNode));
}
document.addEventListener('mousedown', () => { S.mouseDown = true; });
document.addEventListener('mouseup', () => { S.mouseDown = false; });
function renderDrawer() {
  const open = !!(S.sel && S.snap);
  $('drawer').classList.toggle('open', open);
  $('scrim').classList.toggle('open', open && window.innerWidth < 900);
  if (!open) return;
  const { type, id } = S.sel;
  const body = $('d-body');
  if (!S.forceDrawer && userIsSelecting($('drawer'))) return;
  S.forceDrawer = false;
  const st = body.scrollTop;
  if (type === 'node') drawerNode(id);
  else if (type === 'topic') drawerTopic(id);
  else if (type === 'proc') drawerProc(Number(id));
  else drawerHost();
  body.scrollTop = st;
}
function relatedIssues(subjects) {
  const set = new Set(subjects);
  const items = S.snap.issues.filter((i) => i.subjects.some((x) => set.has(x)));
  return items.length ? '<h3>Issues</h3>' + items.map(issueHtml).join('') : '';
}
function kv(pairs) {
  return '<div class="kv">' + pairs.filter((p) => p && p[1] != null && p[1] !== '').map((p) => '<div class="k">' + esc(p[0]) + '</div><div class="v">' + p[1] + '</div>').join('') + '</div>';
}
function topicLink(name) {
  const t = S.snap.topics.find((x) => x.name === name);
  const hz = t && t.stats ? fmtHz(t.stats.hz) + ' Hz' : '';
  const sev = sevOf(name);
  return '<div class="ep"><span class="dot ' + (sev || (t && t.stats && t.stats.hz > 0 ? '' : 'idle')) + '"></span><a class="grow mono" data-topic="' + esc(name) + '" href="#">' + esc(name) + '</a><span class="muted num">' + hz + '</span></div>';
}
const HOW = {
  self: 'this monitor', guid: 'process id embedded in the DDS GUID (exact)', remap: 'command line remap (__node:=)',
  container: 'listed by its component container', launch: 'launch_ros node name', log: 'matched its ROS log file',
  name: 'executable named like the node', group: 'shares a DDS participant with a mapped node',
  discovery: 'its DDS discovery announcement (UDP ports owned by the process)',
};
function drawerNode(name) {
  const s = S.snap, n = s.nodes.find((x) => x.name === name);
  $('d-title').textContent = name;
  if (!n) { $('d-sub').textContent = 'no longer in the graph'; $('d-body').innerHTML = '<div class="empty">This node left the graph.</div>'; return; }
  const p = s.procs.find((x) => x.pid === n.pid);
  $('d-sub').textContent = n.lifecycle ? 'lifecycle node · ' + n.lifecycle : (n.remote ? 'running on another machine' : 'node');
  let h = kv([
    ['Process', p ? '<span class="mono">' + esc(p.name) + '</span> · pid ' + p.pid : n.remote ? 'on another machine' + (n.host_ip ? ' (' + esc(n.host_ip) + ')' : '') : '<span class="muted">not mapped to a local process</span>'],
    ['Mapped by', n.how ? esc(HOW[n.how] || n.how) : null],
    ['CPU', p ? cpuCell(p.cpu) : null], ['Memory', p ? fmtBytes(p.rss) + (p.rss_slope != null ? ' <span class="muted">(' + (p.rss_slope >= 0 ? '+' : '') + p.rss_slope.toFixed(2) + ' MB/min)</span>' : '') : null],
    ['GPU memory', p && p.gpu_mem ? fmtBytes(p.gpu_mem) : null],
    ['Threads', p ? p.threads : null], ['Open files', p && p.fds != null ? p.fds : null], ['Uptime', p ? fmtDur(p.uptime) : null],
    ['Runs as', p ? esc(p.user || '') : null], ['Restarts', n.restarts ? n.restarts.count + ' (last ' + fmtDur(n.restarts.ago_s) + ' ago)' : null],
    ['Same process as', p && p.nodes.length > 1 ? p.nodes.filter((x) => x !== name).map((x) => '<a class="subj" data-subj="' + esc(x) + '">' + esc(x) + '</a>').join(' ') : null],
    ['Container', n.container ? esc(n.container) : null], ['Services', n.services],
  ]);
  h += relatedIssues([name]);
  if (p) h += '<h3>CPU (last minute)</h3>' + bigChart(p.cpu_hist, '%');
  const sysT = (x) => x === '/rosout' || x === '/parameter_events';
  const pubs = n.pubs.filter((x) => !sysT(x)), subs = n.subs.filter((x) => !sysT(x));
  if (pubs.length) h += '<h3>Publishes (' + pubs.length + ')</h3><div class="eplist">' + pubs.map(topicLink).join('') + '</div>';
  if (subs.length) h += '<h3>Subscribes (' + subs.length + ')</h3><div class="eplist">' + subs.map(topicLink).join('') + '</div>';
  if ((n.actions || []).length) h += '<h3>Actions (' + n.actions.length + ')</h3><div class="eplist">' + n.actions.map((a) => '<div class="ep"><span class="grow mono">' + esc(a) + '</span></div>').join('') + '</div>';
  if ((n.service_names || []).length) h += '<h3>Services (' + n.service_names.length + ')</h3><div class="eplist">' + n.service_names.map((a) => '<div class="ep"><span class="grow mono">' + esc(a) + '</span></div>').join('') + '</div>';
  h += '<h3>Parameters</h3>';
  const pr = S.params[name];
  if (!pr) h += '<button class="btn" data-params="' + esc(name) + '">Show parameters</button>';
  else if (pr.loading) h += '<div class="muted">Asking the node…</div>';
  else if (pr.error) h += '<div class="muted">' + esc(pr.error) + '</div> <button class="btn" data-params="' + esc(name) + '">Try again</button>';
  else h += '<div class="muted" style="margin-bottom:6px">' + pr.params.length + ' parameters, read ' + ago(pr.at) + ' <button class="btn" data-params="' + esc(name) + '">Refresh</button></div>' + (pr.params.length ? '<div class="eplist params">' + pr.params.map((kv) => '<div class="ep"><span class="pname mono">' + esc(kv[0]) + '</span><span class="pval mono">' + esc(JSON.stringify(kv[1])) + '</span></div>').join('') + '</div>' : '');
  if (p) h += '<h3>Command</h3><pre class="cmd">' + esc(p.cmd) + '</pre>';
  const lg = name.replace(/^\//, '').replace(/\//g, '.');
  const logs = S.logs.filter((l) => l.logger === lg).slice(-15).reverse();
  if (logs.length) h += '<h3>Recent logs</h3><div class="eplist logs">' + logs.map((l) => '<div class="ep"><span class="lv lv' + l.level + '">' + (LVL[l.level] || l.level) + '</span><span class="grow msg" style="white-space:normal">' + esc(l.msg) + '</span></div>').join('') + '</div>';
  h += '<h3>Try</h3><pre class="cmd">ros2 node info ' + esc(name) + '\nros2 param list ' + esc(name) + '</pre>';
  $('d-body').innerHTML = h;
}
function drawerTopic(name) {
  const s = S.snap, t = s.topics.find((x) => x.name === name);
  $('d-title').textContent = name;
  if (!t) { $('d-sub').textContent = 'gone'; $('d-body').innerHTML = '<div class="empty">This topic no longer exists.</div>'; return; }
  $('d-sub').textContent = t.types.join(', ');
  const st = t.stats;
  let h = '';
  if (st) {
    h += kv([
      ['Rate', '<b class="num">' + fmtHz(st.hz) + ' Hz</b>' + (st.learned_hz ? ' <span class="muted">(normally ' + fmtHz(st.learned_hz) + ' Hz)</span>' : '') + (t.expected_hz ? ' <span class="muted">· baseline ' + fmtHz(t.expected_hz) + ' Hz</span>' : '')],
      ['Bandwidth', fmtBytes(st.bw, true)], ['Message size', st.size ? fmtBytes(st.size) : null],
      ['Jitter', st.jitter_ms != null ? st.jitter_ms + ' ms' : null], ['Max gap', st.max_gap_ms != null ? st.max_gap_ms + ' ms' : null],
      ['Latency', st.age_ms != null ? latency(st.age_ms) + ' avg · ' + latency(st.age_max_ms) + ' max <span class="muted">(header stamp → arrival)</span>' : (st.stamped ? 'no valid stamps' : '<span class="muted">type has no header</span>')],
      ['Pattern', st.periodic === true ? 'periodic' : st.periodic === false ? 'event-driven (no staleness alerts)' : null],
      ['Messages seen', st.msgs], ['Monitoring', esc(t.monitor) + (t.monitor_reason ? ' <span class="muted">· ' + esc(t.monitor_reason) + '</span>' : '')],
    ]);
    h += '<h3>Rate (last minute)</h3>' + bigChart(st.hist, 'Hz');
  } else {
    h += kv([['Monitoring', esc(t.monitor || '') + (t.monitor_reason ? ' <span class="muted">· ' + esc(t.monitor_reason) + '</span>' : '')], ['Latched', t.latched ? 'yes (transient local)' : null]]);
  }
  h += relatedIssues([name]);
  for (const q of t.qos_issues || []) {
    h += '<div class="issue ' + (q.level === 'error' ? 'error' : 'warn') + '"><div class="bar"></div><div class="body"><div class="t"><span class="sev">QoS</span><span class="mono">' + esc(q.pub) + ' → ' + esc(q.sub) + '</span></div><div class="d">offered <span class="mono">' + esc(q.pub_qos) + '</span>, requested <span class="mono">' + esc(q.sub_qos) + '</span></div></div></div>';
  }
  const ep = (e) => '<div class="ep"><span class="dot ' + (sevOf(e.node) || '') + '"></span><a class="grow mono" data-node="' + esc(e.node) + '" href="#">' + esc(e.node) + '</a><span class="mono faint">' + esc(e.qos_short) + '</span></div>';
  const pubs = t.pubs.filter((e) => !e.self), subs = t.subs.filter((e) => !e.self);
  h += '<h3>Publishers (' + pubs.length + ')</h3>' + (pubs.length ? '<div class="eplist">' + pubs.map(ep).join('') + '</div>' : '<div class="muted">none</div>');
  h += '<h3>Subscribers (' + subs.length + ')</h3>' + (subs.length ? '<div class="eplist">' + subs.map(ep).join('') + '</div>' : '<div class="muted">none</div>');
  h += '<h3>Latest message</h3>';
  const ec = S.echo[name];
  if (!ec) h += '<button class="btn" data-echo="' + esc(name) + '">Show the latest message</button>';
  else if (ec.loading) h += '<div class="muted">Waiting for a message…</div>';
  else if (ec.error) h += '<div class="muted">' + esc(ec.error) + '</div> <button class="btn" data-echo="' + esc(name) + '">Try again</button>';
  else h += '<div class="muted" style="margin-bottom:6px">' + fmtBytes(ec.bytes) + ', received ' + ago(ec.at) + ' <button class="btn" data-echo="' + esc(name) + '">Refresh</button></div><pre class="cmd msg">' + esc(JSON.stringify(ec.message, null, 2)) + '</pre>';
  h += '<h3>Try</h3><pre class="cmd">ros2 topic echo ' + esc(name) + ' --once\nros2 topic info ' + esc(name) + ' --verbose</pre>';
  $('d-body').innerHTML = h;
}
function loadEcho(name) {
  S.echo[name] = { loading: true }; S.forceDrawer = true; renderDrawer();
  fetch('/api/echo?topic=' + encodeURIComponent(name)).then((r) => r.json()).then((d) => {
    S.echo[name] = d.error ? { error: d.error } : { message: d.message, bytes: d.bytes, at: Date.now() / 1000 };
  }).catch((e) => { S.echo[name] = { error: String(e) }; }).finally(() => { S.forceDrawer = true; renderDrawer(); });
}
function loadParams(node) {
  S.params[node] = { loading: true }; S.forceDrawer = true; renderDrawer();
  fetch('/api/params?node=' + encodeURIComponent(node)).then((r) => r.json()).then((d) => {
    S.params[node] = d.error ? { error: d.error } : { params: d.params, at: Date.now() / 1000 };
  }).catch((e) => { S.params[node] = { error: String(e) }; }).finally(() => { S.forceDrawer = true; renderDrawer(); });
}
function drawerProc(pid) {
  const p = S.snap.procs.find((x) => x.pid === pid);
  if (p && p.nodes.length) return drawerNode(p.nodes[0]);
  $('d-title').textContent = p ? p.name : 'process ' + pid;
  $('d-sub').textContent = 'ROS process without a node';
  if (!p) { $('d-body').innerHTML = '<div class="empty">Process exited.</div>'; return; }
  $('d-body').innerHTML = kv([['PID', p.pid], ['CPU', cpuCell(p.cpu)], ['Memory', fmtBytes(p.rss)], ['Threads', p.threads], ['Uptime', fmtDur(p.uptime)], ['Runs as', esc(p.user || '')]]) +
    '<h3>CPU (last minute)</h3>' + bigChart(p.cpu_hist, '%') + '<h3>Command</h3><pre class="cmd">' + esc(p.cmd) + '</pre>';
}
function drawerHost() {
  const h = S.snap.host || {}, e = S.snap.env || {};
  $('d-title').textContent = h.hostname || 'host';
  $('d-sub').textContent = [h.model, h.os].filter(Boolean).join(' · ');
  let out = kv([
    ['Kernel', esc(h.kernel)], ['CPUs', h.ncpu], ['Uptime', fmtDur(h.uptime)],
    ['Memory', h.mem_total ? fmtBytes(h.mem_total - h.mem_avail) + ' / ' + fmtBytes(h.mem_total) : null],
    ['Swap', h.swap_total ? fmtBytes(h.swap_used) + ' / ' + fmtBytes(h.swap_total) : 'none'],
  ]);
  out += relatedIssues(['host']);
  if ((h.temps || []).length) out += '<h3>Temperatures</h3>' + kv(h.temps.map((t) => [t.name, t.c.toFixed(1) + ' °C']));
  if (h.throttle) out += '<h3>Power flags</h3>' + kv(Object.entries(h.throttle).map(([k, v]) => [k.replace(/_/g, ' '), esc(String(v))]));
  if ((h.disks || []).length) out += '<h3>Disks</h3>' + kv(h.disks.map((d) => [d.path, d.pct.toFixed(0) + '% used · ' + fmtBytes(d.free) + ' free of ' + fmtBytes(d.total)]));
  out += '<h3>Top processes (all users)</h3><div class="eplist">' + (S.snap.top || []).map((p) => '<div class="ep"><span class="grow mono">' + esc(p.name) + ' <span class="faint">' + p.pid + '</span>' + (p.ros ? ' <span class="tag">ROS</span>' : '') + '</span><span class="num">' + p.cpu.toFixed(1) + '%</span><span class="num muted">' + fmtBytes(p.rss) + '</span></div>').join('') + '</div>';
  out += '<h3>ROS environment</h3>' + kv(Object.entries(e).filter(([, v]) => v).map(([k, v]) => [k, '<span class="mono">' + esc(v) + '</span>']));
  $('d-body').innerHTML = out;
}

// ----------------------------------------------------------------- graph --
function graphData() {
  const s = S.snap, G = S.graph;
  const hiddenNode = (n) => n.self || (!S.showHidden && n.hidden);
  const skipTopic = (t) => (!G.showSystem && (t.system || ['/tf', '/tf_static', '/diagnostics', '/clock'].includes(t.name))) || (!S.showHidden && t.hidden);
  const nodes = new Map(), edges = [];
  for (const n of s.nodes) if (!hiddenNode(n) && matches(n.name)) nodes.set('n:' + n.name, { id: 'n:' + n.name, kind: 'node', name: n.name, ref: n });
  const qosBad = new Set();
  for (const t of s.topics) {
    if (skipTopic(t)) continue;
    for (const q of t.qos_issues || []) if (q.level === 'error') qosBad.add(t.name + '|' + q.pub + '|' + q.sub);
    const pubs = t.pubs.filter((e) => nodes.has('n:' + e.node)), subs = t.subs.filter((e) => nodes.has('n:' + e.node));
    if (!pubs.length && !subs.length) continue;
    const tid = 't:' + t.name;
    nodes.set(tid, { id: tid, kind: 'topic', name: t.name, ref: t });
    for (const p of pubs) edges.push({ a: 'n:' + p.node, b: tid, bad: false, pub: p.node, topic: t.name });
    for (const sb of subs) {
      const bad = t.pubs.some((p) => qosBad.has(t.name + '|' + p.node + '|' + sb.node));
      edges.push({ a: tid, b: 'n:' + sb.node, bad, topic: t.name });
    }
  }
  return { nodes, edges };
}
function layoutGraph(data) {
  // Layered layout (Sugiyama style): publishers flow left to right, barycentric ordering
  // inside each column, hard spacing so nothing overlaps.
  const G = S.graph, N = [...data.nodes.values()];
  const rank = new Map(N.map((n) => [n.id, n.kind === 'node' ? 0 : 1]));
  for (let it = 0; it < 16; it++) {
    let changed = false;
    for (const e of data.edges) {
      const r = rank.get(e.a) + 1;
      if (r > rank.get(e.b) && r < 30) { rank.set(e.b, r); changed = true; }
    }
    if (!changed) break;
  }
  const nb = new Map(N.map((n) => [n.id, []]));
  for (const e of data.edges) { nb.get(e.a).push(e.b); nb.get(e.b).push(e.a); }
  const cols = new Map();
  for (const n of N) {
    // wide enough for the name and for the second line (CPU · memory, or Hz · type)
    const sub = n.kind === 'node' ? 18 : 12 + ((n.ref.types && n.ref.types[0]) || '').length;
    n.w = Math.max(80, n.name.length * (n.kind === 'node' ? 7.4 : 6.7), sub * 5.9) + 26;
    n.h = n.kind === 'node' ? 40 : 32;
    const r = rank.get(n.id);
    if (!cols.has(r)) cols.set(r, []);
    cols.get(r).push(n);
  }
  const order = [...cols.keys()].sort((a, b) => a - b);
  let x = 0;
  for (const r of order) {
    const col = cols.get(r), w = Math.max(...col.map((n) => n.w));
    for (const n of col) n.x = x + w / 2;
    x += w + 70;
    col.forEach((n, i) => { const old = G.pos.get(n.id); n.y = old ? old.y : i * 52; n.fixed = old && old.fixed; if (n.fixed) n.x = old.x; });
  }
  const idx = new Map(N.map((n) => [n.id, n]));
  const pack = (col) => {
    col.sort((a, b) => a.y - b.y);
    const mean = col.reduce((s, n) => s + n.y, 0) / col.length;
    for (let i = 1; i < col.length; i++) {
      const need = col[i - 1].y + (col[i - 1].h + col[i].h) / 2 + 14;
      if (col[i].y < need) col[i].y = need;
    }
    const shift = mean - col.reduce((s, n) => s + n.y, 0) / col.length;
    for (const n of col) n.y += shift;
  };
  for (let it = 0; it < 24; it++) {
    const seq = it % 2 ? order.slice().reverse() : order;
    for (const r of seq) {
      const col = cols.get(r);
      for (const n of col) {
        if (n.fixed) continue;
        const ys = nb.get(n.id).map((id) => idx.get(id).y);
        if (ys.length) n.y = n.y * 0.3 + 0.7 * (ys.reduce((a, b) => a + b, 0) / ys.length);
      }
      pack(col);
    }
  }
  // Close empty horizontal bands (disconnected parts drift apart).
  const sorted = N.slice().sort((a, b) => (a.y - a.h / 2) - (b.y - b.h / 2));
  let reach = -Infinity, shift = 0;
  for (const n of sorted) {
    const top = n.y - n.h / 2 - shift;
    if (reach > -Infinity && top - reach > 36) shift += top - reach - 36;
    n.y -= shift;
    reach = Math.max(reach, n.y + n.h / 2);
  }
  for (const n of N) G.pos.set(n.id, { x: n.x, y: n.y, fixed: n.fixed });
  return idx;
}
function renderGraph(m) {
  const G = S.graph;
  const data = graphData();
  const sig = [...data.nodes.keys()].sort().join(',') + '#' + data.edges.map((e) => e.a + '>' + e.b).sort().join(',');
  if (m.dataset.tab !== 'graph') {
    m.dataset.tab = 'graph';
    m.innerHTML = '<div class="toolbar">' + searchBox('Filter nodes…') +
      '<label class="tog"><input type="checkbox" id="gsys" ' + (G.showSystem ? 'checked' : '') + '> /tf, /diagnostics, /clock</label>' +
      '<label class="tog"><input type="checkbox" id="showHidden" ' + (S.showHidden ? 'checked' : '') + '> hidden</label>' +
      '<span class="spacer"></span><button class="btn" id="gfit">Fit</button><button class="btn" id="grelayout">Re-layout</button></div>' +
      '<div class="graphwrap" id="gwrap"><svg id="gsvg"><defs><marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="var(--line-2)"/></marker><marker id="arrbad" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="var(--err)"/></marker></defs><g id="gview"><g id="gedges"></g><g id="gnodes"></g></g></svg>' +
      '<div class="legend"><span><i style="background:var(--panel-2)"></i>node</span><span><i style="background:color-mix(in srgb,var(--accent) 12%,var(--panel))"></i>topic</span><span><i style="border-color:var(--err);border-style:dashed"></i>QoS mismatch: no data flows</span><span>drag to pan · scroll to zoom · click for details</span></div></div>';
    G.sig = '';
    bindGraph();
  }
  if (sig !== G.sig) {
    G.sig = sig;
    G.data = data;
    G.idx = layoutGraph(data);
    drawGraph();
    if (!G.fitted) { fitGraph(); G.fitted = true; }
  } else {
    G.data = data;
    for (const n of data.nodes.values()) { const o = G.idx.get(n.id); if (o) o.ref = n.ref; }
  }
  updateGraphStyles();
}
function drawGraph() {
  const G = S.graph, idx = G.idx;
  let eh = '';
  for (const e of G.data.edges) {
    const a = idx.get(e.a), b = idx.get(e.b);
    if (!a || !b) continue;
    const x1 = a.x + a.w / 2, y1 = a.y, x2 = b.x - b.w / 2, y2 = b.y;
    const back = x2 < x1;
    const d = back
      ? 'M' + x1 + ',' + y1 + ' C' + (x1 + 80) + ',' + (y1 - 60) + ' ' + (x2 - 80) + ',' + (y2 - 60) + ' ' + x2 + ',' + y2
      : 'M' + x1 + ',' + y1 + ' C' + (x1 + (x2 - x1) / 2) + ',' + y1 + ' ' + (x1 + (x2 - x1) / 2) + ',' + y2 + ' ' + x2 + ',' + y2;
    eh += '<path class="g-edge' + (e.bad ? ' bad' : '') + '" data-a="' + esc(e.a) + '" data-b="' + esc(e.b) + '" d="' + d + '" marker-end="url(#' + (e.bad ? 'arrbad' : 'arr') + ')"/>';
  }
  $('gedges').innerHTML = eh;
  let nh = '';
  for (const n of idx.values()) {
    const x = n.x - n.w / 2, y = n.y - n.h / 2;
    if (n.kind === 'node') {
      nh += '<g class="g-node" data-id="' + esc(n.id) + '" transform="translate(' + x.toFixed(1) + ',' + y.toFixed(1) + ')"><rect width="' + n.w + '" height="' + n.h + '" rx="8"/><text x="11" y="16">' + esc(n.name) + '</text><text class="cpu" x="11" y="30"></text></g>';
    } else {
      nh += '<g class="g-topic" data-id="' + esc(n.id) + '" transform="translate(' + x.toFixed(1) + ',' + y.toFixed(1) + ')"><rect width="' + n.w + '" height="' + n.h + '" rx="15"/><text x="12" y="13">' + esc(n.name) + '</text><text class="hz" x="12" y="24"></text></g>';
    }
  }
  $('gnodes').innerHTML = nh;
  applyView();
}
function updateGraphStyles() {
  const G = S.graph;
  if (!G.idx || !$('gnodes')) return;
  const sel = S.sel ? (S.sel.type === 'node' ? 'n:' : 't:') + S.sel.id : null;
  const near = new Set();
  if (sel && G.idx.has(sel)) {
    near.add(sel);
    for (const e of G.data.edges) { if (e.a === sel) near.add(e.b); if (e.b === sel) near.add(e.a); }
  }
  $('gnodes').querySelectorAll('g').forEach((g) => {
    const n = G.idx.get(g.dataset.id);
    if (!n) return;
    const ref = (G.data.nodes.get(n.id) || n).ref;
    const sev = sevOf(n.name);
    const base = n.kind === 'node' ? 'g-node' : 'g-topic';
    let cls = base + (sev === 'error' || sev === 'warn' ? ' ' + sev : '');
    if (n.kind === 'topic' && !sev && !(ref.stats && ref.stats.hz > 0) && !ref.latched) cls += ' idle';
    if (sel && !near.has(n.id)) cls += ' g-dim';
    g.setAttribute('class', cls);
    const sub = g.querySelectorAll('text')[1];
    if (n.kind === 'node') sub.textContent = ref.cpu != null ? ref.cpu.toFixed(0) + '% CPU · ' + fmtBytes(ref.rss) : (ref.remote ? 'other machine' : '');
    else sub.textContent = ref.stats ? fmtHz(ref.stats.hz) + ' Hz · ' + ref.types[0] : ref.types[0];
  });
  $('gedges').querySelectorAll('path').forEach((p) => {
    const dim = sel && !(p.dataset.a === sel || p.dataset.b === sel);
    p.classList.toggle('g-dim', !!dim);
    p.classList.toggle('flow', !!sel && !dim && !p.classList.contains('bad'));
  });
}
function applyView() {
  const v = S.graph.view;
  const g = $('gview');
  if (g) g.setAttribute('transform', 'translate(' + v.x + ',' + v.y + ') scale(' + v.k + ')');
}
function fitGraph() {
  const G = S.graph, wrap = $('gwrap');
  if (!G.idx || !G.idx.size || !wrap) return;
  let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const n of G.idx.values()) { x0 = Math.min(x0, n.x - n.w / 2); x1 = Math.max(x1, n.x + n.w / 2); y0 = Math.min(y0, n.y - n.h / 2); y1 = Math.max(y1, n.y + n.h / 2); }
  const W = wrap.clientWidth, H = wrap.clientHeight;
  const k = Math.min(1.4, Math.min((W - 60) / (x1 - x0 || 1), (H - 80) / (y1 - y0 || 1)));
  G.view = { k, x: (W - (x1 - x0) * k) / 2 - x0 * k, y: (H - 30 - (y1 - y0) * k) / 2 - y0 * k };
  applyView();
}
function bindGraph() {
  const svg = $('gsvg');
  let drag = null;
  svg.addEventListener('mousedown', (e) => {
    const g = e.target.closest('g[data-id]');
    if (g) {
      const n = S.graph.idx.get(g.dataset.id);
      drag = { node: n, sx: e.clientX, sy: e.clientY, ox: n.x, oy: n.y, moved: false };
    } else {
      drag = { pan: true, sx: e.clientX, sy: e.clientY, ox: S.graph.view.x, oy: S.graph.view.y, moved: false };
      svg.classList.add('drag');
    }
  });
  window.addEventListener('mousemove', (e) => {
    if (!drag) return;
    const dx = e.clientX - drag.sx, dy = e.clientY - drag.sy;
    if (Math.abs(dx) + Math.abs(dy) > 3) drag.moved = true;
    if (drag.pan) { S.graph.view.x = drag.ox + dx; S.graph.view.y = drag.oy + dy; applyView(); } else if (drag.moved) {
      const k = S.graph.view.k;
      drag.node.x = drag.ox + dx / k; drag.node.y = drag.oy + dy / k; drag.node.fixed = true;
      S.graph.pos.set(drag.node.id, { x: drag.node.x, y: drag.node.y, fixed: true });
      drawGraph(); updateGraphStyles();
    }
  });
  window.addEventListener('mouseup', (e) => {
    if (!drag) return;
    svg.classList.remove('drag');
    if (!drag.moved) {
      if (drag.node) select(drag.node.kind === 'node' ? 'node' : 'topic', drag.node.name);
      else select(null, null);
    }
    drag = null;
  });
  svg.addEventListener('wheel', (e) => {
    e.preventDefault();
    const v = S.graph.view, r = svg.getBoundingClientRect();
    const mx = e.clientX - r.left, my = e.clientY - r.top;
    const k2 = Math.max(0.15, Math.min(3, v.k * Math.exp(-e.deltaY * 0.0015)));
    v.x = mx - (mx - v.x) * (k2 / v.k); v.y = my - (my - v.y) * (k2 / v.k); v.k = k2;
    applyView();
  }, { passive: false });
}

// ---------------------------------------------------------------- modals --
function modal(html) { $('modal-box').innerHTML = html; $('modal').classList.add('open'); }
function closeModal() { $('modal').classList.remove('open'); }
function openReport() {
  fetch('/api/report?format=md').then((r) => r.text()).then((text) => {
    modal('<h4>Share a health report</h4><p>A Markdown summary of this robot\'s ROS 2 graph and its problems. Paste it into a GitHub issue, Discourse or chat when asking for help.</p><pre class="cmd" id="reptext">' + esc(text) + '</pre><div class="row"><button class="btn" data-close>Close</button><a class="btn" download="robot_monitor-report.md" href="data:text/markdown;charset=utf-8,' + encodeURIComponent(text) + '">Download .md</a><button class="btn primary" id="repcopy">Copy</button></div>');
    $('repcopy').onclick = () => copyText(text);
  });
}
function openBaseline() {
  const b = S.snap.baseline || {};
  fetch('/api/baseline/preview').then((r) => r.json()).then((p) => {
    const nT = Object.keys(p.topics || {}).length;
    modal('<h4>Baseline: what "normal" looks like</h4><p>Save the ' + p.nodes.length + ' nodes and the rates of ' + nT + ' steady topics running now. From then on Robot Monitor raises an error when an expected node is missing or a topic falls below 80% of its normal rate, even after a restart.</p>' +
      (b.loaded ? '<p>A baseline is active (' + b.nodes + ' nodes, ' + b.topics + ' topics). Saving replaces it.</p>' : '') +
      '<p class="faint mono">' + esc(b.file || '') + '</p><p class="faint">Use the same file in CI: <span class="mono">ros2 monitor check --baseline ' + esc(b.file || 'baseline.yaml') + '</span></p>' +
      '<div class="row">' + (b.loaded ? '<button class="btn" id="blclear">Remove baseline</button>' : '') + '<button class="btn" data-close>Cancel</button><button class="btn primary" id="blsave">Save baseline</button></div>');
    $('blsave').onclick = () => fetch('/api/baseline', { method: 'POST' }).then((r) => r.json()).then(() => { closeModal(); toast('Baseline saved'); });
    if ($('blclear')) $('blclear').onclick = () => fetch('/api/baseline', { method: 'DELETE' }).then(() => { closeModal(); toast('Baseline removed'); });
  });
}
function openAbout() {
  modal('<h4>Robot Monitor</h4><p>Free, open-source (Apache-2.0) health monitoring for ROS 2. No account, no cloud, no telemetry: everything runs on this robot.</p>' +
    '<p>Bug reports, ideas and pull requests are welcome on GitHub.</p>' +
    '<div class="row"><a class="btn" href="https://github.com/PhysicalFleet/robot_monitor" target="_blank" rel="noopener">GitHub</a><button class="btn primary" data-close>Close</button></div>');
}

// ---------------------------------------------------------------- events --
function route() {
  const h = location.hash.replace(/^#\/?/, '');
  const tab = h.split('?')[0];
  S.tab = TABS.some((t) => t[0] === tab) ? tab : 'overview';
  if (S.tab !== 'graph') delete $('main').dataset.tab;
  render();
}
window.addEventListener('hashchange', () => {
  if (!S.keepFilter) S.filter = '';  // a filter typed on one tab shouldn't hide rows on another
  S.keepFilter = false;
  $('main').dataset.tab = '';
  route();
});
document.addEventListener('click', (e) => {
  const t = e.target;
  if (t.closest('[data-close]')) { closeModal(); return; }
  if (t === $('modal')) { closeModal(); return; }
  const sortEl = t.closest('th[data-sort]');
  if (sortEl) {
    const [tb, key] = sortEl.dataset.sort.split(':');
    const cur = S.sort[tb];
    S.sort[tb] = [key, cur[0] === key ? -cur[1] : (key === 'name' || key === 'type' ? 1 : -1)];
    renderMain(); return;
  }
  const subj = t.closest('[data-subj]');
  if (subj) {
    e.preventDefault();
    const x = subj.dataset.subj;
    if (x === 'host' || x === 'self') select('host', 'host');
    else if (x.startsWith('tf:')) { S.filter = x.slice(3); S.keepFilter = true; location.hash = '#/tf'; }
    else if (x.startsWith('diag:')) { S.filter = x.slice(5); S.keepFilter = true; location.hash = '#/diagnostics'; }
    else if (S.snap.topics.some((tp) => tp.name === x)) select('topic', x);
    else select('node', x);
    return;
  }
  const ech = t.closest('[data-echo]');
  if (ech) { loadEcho(ech.dataset.echo); return; }
  const prm = t.closest('[data-params]');
  if (prm) { loadParams(prm.dataset.params); return; }
  const n = t.closest('[data-node]');
  if (n) { e.preventDefault(); select('node', n.dataset.node); return; }
  const tp = t.closest('[data-topic]');
  if (tp) { e.preventDefault(); select('topic', tp.dataset.topic); return; }
  const pr = t.closest('[data-proc]');
  if (pr) { select('proc', pr.dataset.proc); return; }
  const tv = t.closest('[data-tview]');
  if (tv) { S.topicsView = tv.dataset.tview; renderMain(); return; }
  const lv = t.closest('[data-lvl]');
  if (lv) { S.logLevels[lv.dataset.lvl] = !S.logLevels[lv.dataset.lvl]; renderMain(); return; }
  const dg = t.closest('.diag');
  if (dg) { dg.classList.toggle('open'); return; }
  if (t.id === 'logpause') { S.logPaused = !S.logPaused; renderMain(); return; }
  if (t.id === 'gfit') { fitGraph(); return; }
  if (t.id === 'grelayout') { S.graph.pos.clear(); S.graph.sig = ''; S.graph.fitted = false; renderGraph($('main')); return; }
});
document.addEventListener('input', (e) => {
  if (e.target.id === 'q') {
    S.filter = e.target.value;
    if (S.tab === 'graph') { S.graph.sig = ''; renderGraph($('main')); } else renderMain();
  }
});
document.addEventListener('change', (e) => {
  if (e.target.id === 'showHidden') { S.showHidden = e.target.checked; S.graph.sig = ''; render(); }
  if (e.target.id === 'gsys') { S.graph.showSystem = e.target.checked; S.graph.sig = ''; renderGraph($('main')); }
  if (e.target.id === 'lognode') { S.logNode = e.target.value; renderMain(); }
});
document.addEventListener('keydown', (e) => {
  if (e.key === 'Escape') { if ($('modal').classList.contains('open')) closeModal(); else select(null, null); return; }
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.metaKey || e.ctrlKey) return;
  if (e.key === '/') { const q = $('q'); if (q) { e.preventDefault(); q.focus(); } return; }
  const i = '1234567'.indexOf(e.key);
  if (i >= 0) location.hash = '#/' + TABS[i][0];
});
$('d-close').onclick = () => select(null, null);
$('scrim').onclick = () => select(null, null);
$('btn-report').onclick = openReport;
$('btn-baseline').onclick = openBaseline;
$('about-link').onclick = (e) => { e.preventDefault(); openAbout(); };
$('btn-theme').onclick = () => {
  const cur = document.documentElement.dataset.theme || (matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark');
  const next = cur === 'light' ? 'dark' : 'light';
  document.documentElement.dataset.theme = next;
  try { localStorage.setItem('ns-theme', next); } catch (err) { /* private mode */ }
};
route();
connect();
