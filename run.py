#!/usr/bin/env python3
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
Run Robot Monitor straight from a git clone: no build, no install.

    source /opt/ros/$ROS_DISTRO/setup.bash
    python3 run.py            # web dashboard on http://localhost:9191
    python3 run.py demo       # the same, next to a demo robot full of bugs
    python3 run.py check      # one-shot report in the terminal (CI friendly)
    python3 run.py top        # live terminal view
"""

import argparse
import os
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.join(HERE, 'robot_monitor')
sys.path.insert(0, PKG_DIR)


def _need_ros():
    try:
        import rclpy  # noqa: F401
    except ImportError:
        sys.exit('Robot Monitor needs a ROS 2 environment. Source it first, for example:\n'
                 '    source /opt/ros/jazzy/setup.bash\n'
                 'then run this command again.')


def _verb(name):
    from robot_monitor.cli import command as c
    return {'ui': c.UiVerb, 'check': c.CheckVerb, 'top': c.TopVerb}[name]()


def _run_verb(name, argv):
    verb = _verb(name)
    parser = argparse.ArgumentParser(prog='run.py ' + name, description=verb.__doc__)
    verb.add_arguments(parser, 'run.py')
    return verb.main(args=parser.parse_args(argv))


def _demo(argv):
    from robot_monitor.demo import DEMO_PROCESSES
    env = dict(os.environ, PYTHONPATH=PKG_DIR + os.pathsep + os.environ.get('PYTHONPATH', ''))
    devnull = subprocess.DEVNULL
    logdir = tempfile.mkdtemp(prefix='robot_monitor_demo_')
    logs = [open(os.path.join(logdir, '%02d_%s.log' % (i, r)), 'wb')
            for i, r in enumerate(DEMO_PROCESSES)]
    procs = [subprocess.Popen([sys.executable, '-m', 'robot_monitor.demo', '--role', role],
                              env=env, stdout=devnull, stderr=log)
             for role, log in zip(DEMO_PROCESSES, logs)]
    print('Started a demo robot (%d processes) with typical bugs.' % len(procs))

    def reap():
        # Collect demo processes that exit (no zombies) and say why, instead of failing silently.
        while True:
            for role, p, log in zip(DEMO_PROCESSES, procs, logs):
                if p.poll() is not None and not getattr(p, 'reported', False):
                    p.reported = True
                    with open(log.name, 'rb') as f:
                        err = f.read()[-4000:].decode('utf-8', 'replace').strip().splitlines()
                    if p.returncode not in (0, -2, -15):
                        print('demo process %r exited (%s): %s' % (
                            role, p.returncode, err[-1] if err else 'no output'))
            time.sleep(2)
    threading.Thread(target=reap, daemon=True).start()
    try:
        return _run_verb('ui', argv)
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()


def main(argv=None):
    """Dispatch to ui (default), check, top or demo."""
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] in ('-h', '--help', 'help'):
        print(__doc__.strip())
        return 0
    cmd, rest = ('ui', argv) if not argv or argv[0].startswith('-') else (argv[0], argv[1:])
    if cmd not in ('ui', 'check', 'top', 'demo'):
        print('unknown command %r\n\n%s' % (cmd, __doc__.strip()))
        return 2
    _need_ros()
    if cmd == 'demo':
        return _demo(rest)
    return _run_verb(cmd, rest)


if __name__ == '__main__':
    sys.exit(main() or 0)
