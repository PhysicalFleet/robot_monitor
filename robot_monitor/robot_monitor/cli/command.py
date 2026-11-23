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
"""`ros2 monitor` command and its verbs: ui (default), check, top."""

import json
import os
import sys
import time

from ros2cli.command import add_subparsers_on_demand, CommandExtension
from ros2cli.verb import VerbExtension


class MonitorCommand(CommandExtension):
    """Live health of a ROS 2 system: nodes, topics, QoS, processes, host."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        add_subparsers_on_demand(parser, cli_name, '_verb', 'robot_monitor.verb', required=False)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            return UiVerb().main(args=_Defaults())
        return getattr(args, '_verb').main(args=args)


class _Defaults:
    host = '0.0.0.0'
    port = 9191


def _start(duration_hint=None, publish_diagnostics=False, baseline=None, extra=None):
    import rclpy
    from rclpy.parameter import Parameter

    from ..monitor import Monitor, spin_in_thread
    rclpy.init()
    overrides = [Parameter('publish_diagnostics', value=publish_diagnostics)]
    if baseline:
        overrides.append(Parameter('baseline_file', value=os.path.abspath(baseline)))
    for k, v in (extra or {}).items():
        overrides.append(Parameter(k, value=v))
    node = Monitor(node_name='_robot_monitor_cli_%d' % os.getpid(), parameter_overrides=overrides)
    node.always_active = True
    ex, th = spin_in_thread(node)
    return rclpy, node, ex


def _stop(rclpy, node, ex):
    try:
        ex.shutdown(timeout_sec=1.0)
    except Exception:
        pass
    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.try_shutdown()


class UiVerb(VerbExtension):
    """Start the web dashboard (default when no verb is given)."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('--port', type=int, default=9191, help='HTTP port (default 9191)')
        parser.add_argument('--host', default='0.0.0.0',
                            help='address to listen on (default: all; 127.0.0.1 = local only)')

    def main(self, *, args):
        from ..server import serve, urls
        rclpy, node, ex = _start(publish_diagnostics=True)
        try:
            _, port = serve(node, args.host, args.port)
            print('Robot Monitor dashboard (Ctrl-C to stop):')
            for u in urls(args.host, port):
                print('  ' + u)
            while True:
                time.sleep(3600)
        except KeyboardInterrupt:
            pass
        finally:
            _stop(rclpy, node, ex)
        return 0


class CheckVerb(VerbExtension):
    """Observe the system for a few seconds and report problems (CI friendly)."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('-d', '--duration', type=float, default=6.0,
                            help='seconds to observe before reporting (default 6)')
        parser.add_argument('--format', choices=('text', 'md', 'json'), default='text',
                            help='output format (md is handy for GitHub issues)')
        parser.add_argument('--fail-on', choices=('error', 'warn', 'never'), default='error',
                            help='exit with status 1 when issues of this severity exist')
        parser.add_argument('--baseline', metavar='FILE',
                            help='expected nodes/topic rates (YAML, as saved from the dashboard)')
        parser.add_argument('--expect-nodes', nargs='*', default=[], metavar='NODE',
                            help='fail if any of these nodes is missing')

    def main(self, *, args):
        rclpy, node, ex = _start(baseline=args.baseline)
        try:
            if args.expect_nodes:
                exp = dict(node.expect or {})
                exp['nodes'] = sorted(set(exp.get('nodes', [])) | {
                    n if n.startswith('/') else '/' + n for n in args.expect_nodes})
                node.expect = exp
            _progress(node, args.duration, args.format != 'text' or not sys.stderr.isatty())
            snap = node.snapshot
        except KeyboardInterrupt:
            snap = node.snapshot
        finally:
            _stop(rclpy, node, ex)
        from .. import report
        if args.format == 'json':
            print(json.dumps(snap, indent=1, default=str))
        else:
            print(report.render(snap, fmt=args.format))
        sev = {i['severity'] for i in snap.get('issues', [])}
        if args.fail_on == 'error' and 'error' in sev:
            return 1
        if args.fail_on == 'warn' and sev & {'error', 'warn'}:
            return 1
        return 0


def wait_settled(node, duration, tick=None):
    """
    Observe for `duration` s, and also until DDS discovery has settled.

    New participants can take several seconds to see every node on a busy machine, so
    we keep going (up to 15 s more) until the node and topic counts stop changing for 3 s.
    """
    start = time.monotonic()
    last_sig, stable_since = None, start
    while True:
        now = time.monotonic()
        g = node.graph
        sig = (len(g.get('nodes', {})), len(g.get('topics', {})))
        if sig != last_sig:
            last_sig, stable_since = sig, now
        elapsed = now - start
        if elapsed >= duration and now - stable_since >= 3.0:
            break
        if elapsed >= duration + 15.0:
            break
        if tick:
            tick(max(0.0, duration - elapsed))
        time.sleep(0.25)
    time.sleep(min(1.2, node.p_period + 0.2))  # let one more snapshot include the last graph


def _progress(node, duration, quiet):
    def tick(left):
        if not quiet:
            sys.stderr.write('\rrobot_monitor: observing the ROS graph... %2ds '
                             % int(left + 0.99))
            sys.stderr.flush()
    wait_settled(node, duration, tick)
    if not quiet:
        sys.stderr.write('\r' + ' ' * 50 + '\r')


class TopVerb(VerbExtension):
    """Live terminal view of nodes by CPU, topic rates and issues."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('--once', action='store_true',
                            help='print one frame after a few seconds and exit')

    def main(self, *, args):
        from .top import run_top
        rclpy, node, ex = _start()
        try:
            return run_top(node, once=args.once)
        except KeyboardInterrupt:
            return 0
        finally:
            _stop(rclpy, node, ex)
