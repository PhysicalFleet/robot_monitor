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
Built-in web server: the dashboard, a JSON API, Server-Sent Events, Prometheus.

Standard library only, so the UI works offline on a robot with no internet and
no extra packages. Read-only by design: the only write is saving the baseline
file of this monitor itself; nothing here changes the robot.
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
import threading
import time
from urllib.parse import parse_qs, urlparse

from . import report

_lock = threading.Lock()
STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def make_handler(monitor, allow_write=True):

    class Handler(BaseHTTPRequestHandler):
        server_version = 'robot_monitor'
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):  # keep the robot's console clean
            pass

        def _send(self, code, body, ctype='application/json', extra=None):
            if isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            path = u.path
            if path in ('/', '/index.html'):
                return self._static('index.html')
            if path.startswith('/static/'):
                return self._static(path[len('/static/'):])
            if path.startswith('/api/') or path == '/metrics':
                monitor.last_api_hit = time.monotonic()
            if path == '/api/snapshot':
                return self._send(200, monitor.snapshot_json)
            if path == '/api/events':
                return self._events()
            if path == '/api/logs':
                after = int((q.get('after') or ['0'])[0] or 0)
                return self._send(200, json.dumps(monitor.logs_after(after)))
            if path == '/api/report':
                fmt = (q.get('format') or ['md'])[0]
                text = report.render(monitor.snapshot, fmt=fmt)
                return self._send(200, text, 'text/markdown; charset=utf-8' if fmt == 'md'
                                  else 'text/plain; charset=utf-8')
            if path == '/api/echo':
                topic = (q.get('topic') or [''])[0]
                return self._send(200, json.dumps(monitor.request('echo', topic), default=str))
            if path == '/api/params':
                node = (q.get('node') or [''])[0]
                return self._send(200, json.dumps(monitor.request('params', node), default=str))
            if path == '/api/baseline/preview':
                return self._send(200, json.dumps(monitor.baseline_from_now()))
            if path == '/metrics':
                return self._send(200, report.prometheus(monitor.snapshot),
                                  'text/plain; version=0.0.4')
            if path == '/healthz':
                return self._send(200, '{"ok":true}')
            self._send(404, '{"error":"not found"}')

        def do_POST(self):
            if not self._same_origin():
                return self._send(403, '{"error":"cross-origin request refused"}')
            if self.path == '/api/baseline' and allow_write:
                data = monitor.save_baseline()
                return self._send(200, json.dumps({'ok': True, 'file': monitor.p_baseline,
                                                   'baseline': data}))
            self._send(404, '{"error":"not found"}')

        def do_DELETE(self):
            if not self._same_origin():
                return self._send(403, '{"error":"cross-origin request refused"}')
            if self.path == '/api/baseline' and allow_write:
                monitor.clear_baseline()
                return self._send(200, '{"ok":true}')
            self._send(404, '{"error":"not found"}')

        def _same_origin(self):
            # Block drive-by POSTs from other web pages (CSRF); allow curl.
            origin = self.headers.get('Origin')
            if not origin:
                return True
            host = self.headers.get('Host', '')
            return urlparse(origin).netloc == host

        def _static(self, rel):
            full = os.path.normpath(os.path.join(STATIC, rel))
            if not full.startswith(STATIC) or not os.path.isfile(full):
                return self._send(404, 'not found', 'text/plain')
            ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
            if ctype.startswith('text/') or ctype in ('application/javascript',):
                ctype += '; charset=utf-8'
            with open(full, 'rb') as f:
                self._send(200, f.read(), ctype)

        def _events(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            last = None
            with _lock:
                monitor.watchers += 1
            try:
                while True:
                    with monitor.snapshot_cond:
                        monitor.snapshot_cond.wait(timeout=15)
                        data = monitor.snapshot_json
                    if data is last:
                        self.wfile.write(b': keepalive\n\n')
                    else:
                        self.wfile.write(b'event: snapshot\ndata: ' + data + b'\n\n')
                        last = data
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            finally:
                with _lock:
                    monitor.watchers -= 1

    return Handler


def serve(monitor, host='0.0.0.0', port=9191, allow_write=True):
    """Start the server in a daemon thread. Returns (server, bound_port)."""
    last_err = None
    for p in ([port] if port else []) + ([port + i for i in range(1, 10)] if port else [0]):
        try:
            srv = _Server((host, p), make_handler(monitor, allow_write))
            break
        except OSError as e:
            last_err = e
    else:
        raise last_err
    th = threading.Thread(target=srv.serve_forever, name='robot_monitor-http', daemon=True)
    th.start()
    return srv, srv.server_address[1]


def urls(host, port):
    if host not in ('0.0.0.0', '::', ''):
        return ['http://%s:%d' % (host, port)]
    out = ['http://localhost:%d' % port]
    for ip in _local_ips():
        out.append('http://%s:%d' % (ip, port))
    return out


def _local_ips():
    import socket
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))  # no packet is sent
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith('127.') and ip not in ips:
                ips.append(ip)
    except OSError:
        pass
    return ips[:4]
